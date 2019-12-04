#include "Strategy.h"
#include "../common/Helper.h"
#include "../simulation/Simulator.h"
#include "../model/Unit.hpp"
#include "../mathcalc/MathHelper.h"
#include <map>
#include "RunawayDirection.h"
#include <climits>
#include <cmath>
#include <algorithm>


inline bool operator<(const Bullet& lhs, const Bullet& rhs)
{
	return lhs.position.x < rhs.position.x;
}

double Strategy::getShootEnemyProbability(const Unit& me, const Unit& enemy, const Game& game, double spread,
	Debug* debug) const
{
	if (me.weapon == nullptr) return 0;
	if (me.weapon->lastAngle == nullptr) return 1;

	const auto bulletCenterPos = Vec2Double(me.position.x, me.position.y + me.size.y / 2);
	const auto deltaAngle = spread / ANGLE_SPLIT_COUNT;

	const auto xLeft = enemy.position.x - enemy.size.x / 2;
	const auto xRight = enemy.position.x + enemy.size.x / 2;
	const auto yDown = enemy.position.y;
	const auto yUp = enemy.position.y + enemy.size.y;

	auto shootingCount = 0;

	const auto bulletVelocityLength = (*me.weapon).params.bullet.speed;
	const auto halfBulletSize = me.weapon->params.bullet.size / 2;

	for (auto i = -ANGLE_SPLIT_COUNT; i <= ANGLE_SPLIT_COUNT; ++i)
	{		
		const auto angle = *me.weapon->lastAngle + deltaAngle * i;	
		auto bulletVelocity = Vec2Double(bulletVelocityLength * cos(angle), bulletVelocityLength * sin(angle));

		Vec2Double shootingCrossPoint;
		Vec2Double bulletCornerPoint;
		const auto isShooting = Simulator::getBulletRectangleFirstCrossPoint(bulletCenterPos, bulletVelocity, halfBulletSize,
			xLeft, yDown, xRight, yUp, shootingCrossPoint, bulletCornerPoint);		

		if (isShooting)
		{
			const auto bulletSimulation = Simulator::getBulletSimulation(bulletCenterPos, bulletVelocity, halfBulletSize, game);							

			if (MathHelper::getVectorLength2(
				bulletCornerPoint.x, bulletCornerPoint.y, shootingCrossPoint.x, shootingCrossPoint.y) <
				MathHelper::getVectorLength2(
					bulletSimulation.bulletCrossCorner.x, bulletSimulation.bulletCrossCorner.y,
					bulletSimulation.targetCrossPoint.x, bulletSimulation.targetCrossPoint.y))
			{
				shootingCount++;
				/*if (debug != nullptr) {
					(*debug).draw(CustomData::Line(
						vec2DoubleToVec2Float(bulletCenterPos),
						vec2DoubleToVec2Float(cp),
						0.1, ColorFloat(100, 100, 100, 0.5)));
				}*/
			}
		}
	}

	return shootingCount * 1.0 / (2 * ANGLE_SPLIT_COUNT + 1.0);
}

std::map<Bullet, BulletSimulation> Strategy::getEnemyBulletsSimulation(const Game& game, int meId)
{
	std::map<Bullet, BulletSimulation> simulations;
	for (const auto& bullet: game.bullets)
	{
		if (bullet.playerId == meId) continue;
		const auto simulation = Simulator::getBulletSimulation(bullet.position, bullet.velocity, bullet.size / 2, game);
		simulations[bullet] = simulation;
	}
	return simulations;
}

std::map<Bullet, BulletSimulation> Strategy::getShootMeBullets(const Unit& me,
	const std::map<Bullet, BulletSimulation>& enemyBulletsSimulations, const Game& game) const
{
	std::map<Bullet, BulletSimulation> shootMeBullets;
	for (const auto& bullet:game.bullets)
	{
		if (bullet.playerId == me.playerId) continue;
		Vec2Double shootingCrossPoint;
		Vec2Double bulletCornerPoint;
		const auto isShooting = Simulator::getBulletRectangleFirstCrossPoint(
			bullet.position, bullet.velocity, bullet.size/2,
			me.position.x - me.size.x/2, me.position.y, me.position.x + me.size.x/2, me.position.y + me.size.y, 
			shootingCrossPoint, bulletCornerPoint);
		if (!isShooting) continue;

		
		const auto bulletSimulation = enemyBulletsSimulations.at(bullet);
		if (MathHelper::getVectorLength2(
			bulletCornerPoint.x, bulletCornerPoint.y, shootingCrossPoint.x, shootingCrossPoint.y) <
			MathHelper::getVectorLength2(
				bulletSimulation.bulletCrossCorner.x, bulletSimulation.bulletCrossCorner.y, bulletSimulation.targetCrossPoint.x, bulletSimulation.targetCrossPoint.y))
		{
			const auto shootMeTime = MathHelper::getVectorLength(
				bulletCornerPoint.x, bulletCornerPoint.y, shootingCrossPoint.x, shootingCrossPoint.y) /
				MathHelper::getVectorLength(bullet.velocity);
			shootMeBullets[bullet] = BulletSimulation(shootingCrossPoint, bulletCornerPoint, shootMeTime);
		}
	}
	return shootMeBullets;
	
}


//TODO: упускаем случаи (например, пуля летит вверх, я иду вправо к ней)
bool Strategy::isBulletMoveCrossUnitMove(
	const Vec2Double& unitPos0, const Vec2Double& unitPos1,
	const Vec2Double& bulletPos0, const Vec2Double& bulletPos1,
	const Vec2Double& unitSize, double halfBulletSize)
{
	const auto unitLeftDown0 = Vec2Double(unitPos0.x - unitSize.x / 2, unitPos0.y);
	const auto unitRightDown0 = Vec2Double(unitPos0.x + unitSize.x / 2, unitPos0.y);
	const auto unitLeftUp0 = Vec2Double(unitPos0.x - unitSize.x / 2, unitPos0.y + unitSize.y);
	const auto unitRightUp0 = Vec2Double(unitPos0.x + unitSize.x / 2, unitPos0.y + unitSize.y);

	const auto unitLeftDown1 = Vec2Double(unitPos1.x - unitSize.x / 2, unitPos1.y);
	const auto unitRightDown1 = Vec2Double(unitPos1.x + unitSize.x / 2, unitPos1.y);
	const auto unitLeftUp1 = Vec2Double(unitPos1.x - unitSize.x / 2, unitPos1.y + unitSize.y);
	const auto unitRightUp1 = Vec2Double(unitPos1.x + unitSize.x / 2, unitPos1.y + unitSize.y);

	const Segment unitSegments[] = {
		Segment(unitLeftDown0, unitLeftUp0),
		Segment(unitLeftUp0, unitRightUp0),
		Segment(unitRightUp0, unitRightDown0),
		Segment(unitRightDown0, unitLeftDown0),

		Segment(unitLeftDown1, unitLeftUp1),
		Segment(unitLeftUp1, unitRightUp1),
		Segment(unitRightUp1, unitRightDown1),
		Segment(unitRightDown1, unitLeftDown1),

		Segment(unitLeftDown0, unitLeftDown1),
		Segment(unitLeftUp0, unitLeftUp1),
		Segment(unitRightUp0, unitRightUp1),
		Segment(unitRightDown0, unitRightUp1)
	};

	const auto bulletLeftDown0 = Vec2Double(bulletPos0.x - halfBulletSize, bulletPos0.y - halfBulletSize);
	const auto bulletLeftUp0 = Vec2Double(bulletPos0.x - halfBulletSize, bulletPos0.y + halfBulletSize);
	const auto bulletRightUp0 = Vec2Double(bulletPos0.x + halfBulletSize, bulletPos0.y + halfBulletSize);
	const auto bulletRightDown0 = Vec2Double(bulletPos0.x + halfBulletSize, bulletPos0.y - halfBulletSize);

	const auto bulletLeftDown1 = Vec2Double(bulletPos1.x - halfBulletSize, bulletPos1.y - halfBulletSize);
	const auto bulletLeftUp1 = Vec2Double(bulletPos1.x - halfBulletSize, bulletPos1.y + halfBulletSize);
	const auto bulletRightUp1 = Vec2Double(bulletPos1.x + halfBulletSize, bulletPos1.y + halfBulletSize);
	const auto bulletRightDown1 = Vec2Double(bulletPos1.x + halfBulletSize, bulletPos1.y - halfBulletSize);

	const Segment bulletSegments[] = {
		Segment(bulletLeftDown0, bulletLeftDown1),
		Segment(bulletLeftUp0, bulletLeftUp1),
		Segment(bulletRightUp0, bulletRightUp1),
		Segment(bulletRightDown0, bulletRightDown1)
	};

	for (const auto& us : unitSegments)
	{
		for (const auto& bs : bulletSegments)
		{
			const auto cross = MathHelper::areSegmentsCross(us, bs);
			if (cross) return true;
		}
	}
	return false;
}



std::tuple<RunawayDirection, int, int> Strategy::getRunawayAction(
	const Unit& me, 
	const std::map<Bullet, BulletSimulation>& shootingMeBullets,
	const std::map<Bullet, BulletSimulation>& enemyBulletsSimulations,
	const Game& game)
{
	if (shootingMeBullets.empty())
	{
		return std::make_tuple(GoNONE, -1, -1);
	}
	double minShootMeTime = INT_MAX;
	for (const auto& smb : shootingMeBullets)
	{
		if (smb.second.targetCrossTime < minShootMeTime)
		{
			minShootMeTime = smb.second.targetCrossTime;
		}
	}
	const auto minShootMeTick = static_cast<int>(ceil(minShootMeTime * game.properties.ticksPerSecond));

	double maxShootWallTime = 0;
	for (const auto& item : enemyBulletsSimulations)
	{
		if (item.second.targetCrossTime > maxShootWallTime)
		{
			maxShootWallTime = item.second.targetCrossTime;
		}
	}
	const int maxShootWallTick = static_cast<int>(ceil(maxShootWallTime * game.properties.ticksPerSecond));

	const bool isOnLadder = Simulator::isUnitOnLadder(me.position, me.size, game);
	const bool isOnPlatform = Simulator::isUnitOnPlatform(me.position, me.size, game);

	
	UnitAction action;
	action.jump = false;
	action.jumpDown = false;
	action.velocity = false;

	

	for (int startGoTick = minShootMeTick - 1; startGoTick >= 0; startGoTick--)
	{	
		
	/*	auto killGoUpTick = INT_MAX;
		auto killGoLeftTick = INT_MAX;
		auto killGoRightTick = INT_MAX;
		auto killGoDownTick = INT_MAX;*/

		for (int stopGoTick = startGoTick + 1; stopGoTick < maxShootWallTick; ++stopGoTick)
		{
			
			
			auto canGoUp = true;
			auto canGoLeft = true;
			auto canGoRight = true;
			auto canGoDown = true;

			//auto unitMoveTime = (stopGoTick - startGoTick) / game.properties.ticksPerSecond;
						

			for (const auto& bullet : game.bullets)
			{
				if (bullet.playerId == me.playerId) continue;

				const auto bulletSimulation = enemyBulletsSimulations.at(bullet);

				const auto halfBulletSize = bullet.size / 2;
				auto jumpUnitPosition = me.position;
				auto fallUnitPosition = me.position;
				auto goLeftUnitPosition = me.position;
				auto goRightUnitPosition = me.position;
				
				const auto shootWallTick = static_cast<int>(ceil(bulletSimulation.targetCrossTime * game.properties.ticksPerSecond));
				auto bulletPosition = bullet.position;
				
				for (int tick = 1; tick <= shootWallTick; ++tick)
				{
					const auto bulletTime = tick / game.properties.ticksPerSecond;
					Vec2Double newBulletPosition;
					auto exists = Simulator::getBulletInTimePosition(bullet, bulletTime, bulletSimulation, game, newBulletPosition);
					if (!exists)
					{
						//TODO: проверить случай, когда пуля исчезнет в этот тик
					}				

					//jump
					if (tick > stopGoTick)
					{
						action.jump = false;
					}
					else if (tick > startGoTick)
					{
						action.jump = true;
					}
					const auto newJumpUnitPosition = Simulator::getUnitNextTickPosition(
						jumpUnitPosition, me.size, action, game);

					action.jump = false;

					if (isBulletMoveCrossUnitMove(
						jumpUnitPosition, newJumpUnitPosition, bulletPosition, newBulletPosition, me.size, halfBulletSize))
					{
						canGoUp = false;
					}

					jumpUnitPosition = newJumpUnitPosition;

					//fall
					if (tick > stopGoTick)
					{
						action.jumpDown = false;
					}
					else if (tick > startGoTick)
					{
						action.jumpDown = true;
					}
					const auto newFallUnitPosition = Simulator::getUnitNextTickPosition(
						fallUnitPosition, me.size, action, game);

					action.jumpDown = false;

					if (isBulletMoveCrossUnitMove(
						fallUnitPosition, newFallUnitPosition, bulletPosition, newBulletPosition, me.size, halfBulletSize))
					{
						canGoDown = false;
					}
					fallUnitPosition = newFallUnitPosition;

					//left
					if (tick > stopGoTick)
					{
						action.velocity = 0;
					}
					else if (tick > startGoTick)
					{
						action.velocity = -INT_MAX;
					}
					const auto newGoLeftUnitPosition = Simulator::getUnitNextTickPosition(
						goLeftUnitPosition, me.size, action, game);

					action.velocity = 0;

					if (isBulletMoveCrossUnitMove(
						goLeftUnitPosition, newGoLeftUnitPosition, bulletPosition, newBulletPosition, me.size, halfBulletSize))
					{
						canGoLeft = false;
					}
					goLeftUnitPosition = newGoLeftUnitPosition;

					//right
					if (tick > stopGoTick)
					{
						action.velocity = 0;
					}
					else if (tick > startGoTick)
					{
						action.velocity = INT_MAX;
					}
					const auto newGoRightUnitPosition = Simulator::getUnitNextTickPosition(
						goRightUnitPosition, me.size, action, game);

					action.velocity = 0;

					if (isBulletMoveCrossUnitMove(
						goRightUnitPosition, newGoRightUnitPosition, bulletPosition, newBulletPosition, me.size, halfBulletSize))
					{
						canGoRight = false;
					}
					goRightUnitPosition = newGoRightUnitPosition;
					
					
					bulletPosition = newBulletPosition;	
					if (!canGoUp && !canGoDown && !canGoLeft && !canGoRight) break;
				}

				
				

				/*if (stopGoTick > killGoUpTick)
				{
					canGoUp = false;
				}
				else if (canGoUp && isBulletShootJumpingUnit(
					bullet, me, startGoTick, stopGoTick, enemyBulletsShootWallTimes.at(bullet), game, killGoUpTick))
				{
					canGoUp = false;
				}

				if (stopGoTick > killGoRightTick)
				{
					canGoRight = false;
				}
				else if (canGoRight && isBulletShootGoSideUnit(
					bullet, me, startGoTick, stopGoTick, enemyBulletsShootWallTimes.at(bullet), 1, game,
					killGoRightTick))
				{
					canGoRight = false;
				}

				if (stopGoTick > killGoLeftTick)
				{
					canGoLeft = false;
				}
				else if (canGoLeft && isBulletShootGoSideUnit(
					bullet, me, startGoTick, stopGoTick, enemyBulletsShootWallTimes.at(bullet), -1, game,
					killGoLeftTick))
				{
					canGoLeft = false;
				}

				if (!isOnLadder && !isOnPlatform || stopGoTick > killGoDownTick)
				{
					canGoDown = false;
				}
				else if (canGoDown && isBulletShootFallingUnit(
					bullet, me, startGoTick, isOnLadder ? stopGoTick : -1, enemyBulletsShootWallTimes.at(bullet), game,
					killGoDownTick))
				{
					canGoDown = false;
				}

				if (!canGoUp && !canGoLeft && !canGoRight && !canGoDown) break;*/

				if (!canGoUp && !canGoDown && !canGoLeft && !canGoRight) break;
			}


			if (canGoUp)
			{
				return std::make_tuple(GoUP, startGoTick, stopGoTick);
			}
			if (canGoDown)
			{
				return std::make_tuple(GoDOWN, startGoTick, stopGoTick);
			}			
			if (canGoLeft)
			{
				return std::make_tuple(GoLEFT, startGoTick, stopGoTick);
			}
			if (canGoRight)
			{
				return std::make_tuple(GoRIGHT, startGoTick, stopGoTick);
			}
		}
	}

	return std::make_tuple(GoNONE, -1, -1); //нет пуль или нет шансов спастись
}



bool Strategy::isSafeMove(
	const Unit& unit, const UnitAction& action, const std::map<Bullet, BulletSimulation>& enemyBulletsSimulations, const Game& game)
{
	//TODO: учесть не только позицию в следующий тик, но и возможность избежать пули потом

	const auto time = 1.0 / game.properties.ticksPerSecond;
	for (const auto& bullet : game.bullets)
	{
		if (bullet.playerId == unit.playerId) continue;
	

		const auto unitInTimePosition = Simulator::getUnitNextTickPosition(unit.position, unit.size, action, game);

		Vec2Double bulletInTimePosition;
		const auto exists = Simulator::getBulletInTimePosition(bullet, time, enemyBulletsSimulations.at(bullet), game, bulletInTimePosition);
		if (!exists) continue;
		
		const auto cross = isBulletMoveCrossUnitMove(
			unit.position, unitInTimePosition,
			bullet.position, bulletInTimePosition,
			unit.size, bullet.size / 2.0);
		if (cross) return false;
	}
	return true;
}

int Strategy::getRunawayDirection() const
{
	return runaway_direction_;
}

int Strategy::getStopRunawayTick() const
{
	return stop_runaway_tick_;
}

void Strategy::setRunaway(RunawayDirection runaway_direction, int sjt)
{
	runaway_direction_ = runaway_direction;
	stop_runaway_tick_ = sjt;
}

void Strategy::decreaseStopRunawayTick()
{
	if (stop_runaway_tick_ >= 0) stop_runaway_tick_--;
}


