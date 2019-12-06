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

			if (MathHelper::getVectorLength2(bulletCornerPoint, shootingCrossPoint) <
				MathHelper::getVectorLength2(bulletSimulation.bulletCrossCorner, bulletSimulation.targetCrossPoint))
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
		const auto bulletSimulation = enemyBulletsSimulations.at(bullet);
		
		Vec2Double shootingCrossPoint;
		Vec2Double bulletCornerPoint;
		auto isShooting = Simulator::getBulletRectangleFirstCrossPoint(
			bullet.position, bullet.velocity, bullet.size/2,
			me.position.x - me.size.x/2, me.position.y, me.position.x + me.size.x/2, me.position.y + me.size.y, 
			shootingCrossPoint, bulletCornerPoint);

		if (isShooting)
		{
			if (MathHelper::getVectorLength2(
				bulletCornerPoint, shootingCrossPoint) <
				MathHelper::getVectorLength2(
					bulletSimulation.bulletCrossCorner, bulletSimulation.targetCrossPoint))
			{
				const auto shootMeTime = MathHelper::getVectorLength(bulletCornerPoint, shootingCrossPoint) /
					MathHelper::getVectorLength(bullet.velocity);
				shootMeBullets[bullet] = BulletSimulation(shootingCrossPoint, bulletCornerPoint, shootMeTime);
			}
		}
		else//проверяем взрыв
		{
			const auto bulletCrossWallCenter = Vec2Double(
				bullet.position.x + bullet.velocity.x * bulletSimulation.targetCrossTime,
				bullet.position.y + bullet.velocity.y * bulletSimulation.targetCrossTime);
			if(isBulletExplosionShootUnit(bullet, bulletCrossWallCenter, me.position, me.size))
			{
				shootMeBullets[bullet] = BulletSimulation(
					bulletSimulation.targetCrossPoint, bulletSimulation.bulletCrossCorner, bulletSimulation.targetCrossTime);
			}
		}	
		
	}
	return shootMeBullets;
	
}


bool Strategy::isBulletMoveCrossUnitMove(
	const Vec2Double& unitPos, const Vec2Double& newUnitPos, const Vec2Double& unitSize,
	const Vec2Double& bulletPos, const Vec2Double& newBulletPos, double halfBulletSize)
{
	Vec2Double unitPolygon[6];
	Simulator::getPolygon(unitPos, newUnitPos, unitSize, unitPolygon);

	Vec2Double bulletPolygon[6];
	Simulator::getPolygon(bulletPos, newBulletPos, halfBulletSize, bulletPolygon);

	Segment unitSegments[6];
	for (int i = 0; i < 6; ++i)
	{
		const int endIndex = i < 5 ? i + 1 : 0;
		unitSegments[i] = Segment(unitPolygon[i], unitPolygon[endIndex]);
	}

	Segment bulletSegments[6];
	for (int i = 0; i < 6; ++i)
	{
		const int endIndex = i < 5 ? i + 1 : 0;
		bulletSegments[i] = Segment(bulletPolygon[i], bulletPolygon[endIndex]);
	}	

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
	const auto tickTime = 1.0 / game.properties.ticksPerSecond;
	
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
			
			
			auto canGoUp = me.jumpState.canJump &&
				game.level.tiles[size_t(me.position.x - me.size.x / 2)][size_t(me.position.y + me.size.y + TOLERANCE)] != WALL &&
				game.level.tiles[size_t(me.position.x + me.size.x / 2)][size_t(me.position.y + me.size.y + TOLERANCE)] != WALL;
			auto canGoLeft =
				game.level.tiles[size_t(me.position.x - me.size.x / 2 - TOLERANCE)][size_t(me.position.y)] != WALL &&
				game.level.tiles[size_t(me.position.x - me.size.x / 2 - TOLERANCE)][size_t(me.position.y + me.size.y)] != WALL;
			auto canGoRight = 
				game.level.tiles[size_t(me.position.x + me.size.x / 2 + TOLERANCE)][size_t(me.position.y)] != WALL &&
				game.level.tiles[size_t(me.position.x + me.size.x / 2 + TOLERANCE)][size_t(me.position.y + me.size.y)] != WALL;
			auto canGoDown = !Simulator::isUnitOnWall(me.position, me.size, game);

			//auto unitMoveTime = (stopGoTick - startGoTick) / game.properties.ticksPerSecond;
						

			for (const auto& bullet : game.bullets)
			{
				if (bullet.playerId == me.playerId) continue;

				const auto bulletSimulation = enemyBulletsSimulations.at(bullet);

				const auto bulletCrossWallCenter = Vec2Double(
					bullet.position.x + bullet.velocity.x * bulletSimulation.targetCrossTime,
					bullet.position.y + bullet.velocity.y * bulletSimulation.targetCrossTime
				);

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
					auto unitTime = tickTime;

					
					auto exists = Simulator::getBulletInTimePosition(bullet, bulletTime, bulletSimulation, game, newBulletPosition);
					if (!exists)
					{
						unitTime = bulletSimulation.targetCrossTime - (bulletTime - tickTime);
					}				

					//jump
					if (canGoUp) {
						if (tick > stopGoTick)
						{
							action.jump = false;
						}
						else if (tick > startGoTick)
						{
							action.jump = true;
						}
						const auto newJumpUnitPosition = Simulator::getUnitInTimePosition(
							jumpUnitPosition, me.size, action, unitTime, game);

						action.jump = false;

						if (isBulletMoveCrossUnitMove(
							jumpUnitPosition,
							newJumpUnitPosition,
							me.size,
							bulletPosition,
							newBulletPosition,
							halfBulletSize))
						{
							canGoUp = false;
						}
						else if (!exists && isBulletExplosionShootUnit(bullet, bulletCrossWallCenter, newJumpUnitPosition, me.size))
						{
							//пуля ударится о стену. надо проверить взрыв
							canGoUp = false;
						}
						jumpUnitPosition = newJumpUnitPosition;
					}

					//fall
					if (canGoDown) {
						if (tick > stopGoTick)
						{
							action.jumpDown = false;
						}
						else if (tick > startGoTick)
						{
							action.jumpDown = true;
						}
						const auto newFallUnitPosition = Simulator::getUnitInTimePosition(
							fallUnitPosition, me.size, action, unitTime, game);

						action.jumpDown = false;

						if (isBulletMoveCrossUnitMove(
							fallUnitPosition,
							newFallUnitPosition,
							me.size,
							bulletPosition,
							newBulletPosition,
							halfBulletSize))
						{
							canGoDown = false;
						}
						else if (!exists && isBulletExplosionShootUnit(bullet, bulletCrossWallCenter, newFallUnitPosition, me.size))
						{
							//пуля ударится о стену. надо проверить взрыв
							canGoDown = false;
						}
						fallUnitPosition = newFallUnitPosition;
					}

					//left
					if (canGoLeft)
					{
						if (tick > stopGoTick)
						{
							action.velocity = 0;
						}
						else if (tick > startGoTick)
						{
							action.velocity = -INT_MAX;
						}
						const auto newGoLeftUnitPosition = Simulator::getUnitInTimePosition(
							goLeftUnitPosition, me.size, action, unitTime, game);

						action.velocity = 0;

						if (isBulletMoveCrossUnitMove(
							goLeftUnitPosition,
							newGoLeftUnitPosition,
							me.size,
							bulletPosition,
							newBulletPosition,
							halfBulletSize))
						{
							canGoLeft = false;
						}
						else if (!exists && isBulletExplosionShootUnit(bullet, bulletCrossWallCenter, newGoLeftUnitPosition, me.size))
						{
							//пуля ударится о стену. надо проверить взрыв
							canGoLeft = false;
						}
						goLeftUnitPosition = newGoLeftUnitPosition;
					}

					//right
					if (canGoRight)
					{
						if (tick > stopGoTick)
						{
							action.velocity = 0;
						}
						else if (tick > startGoTick)
						{
							action.velocity = INT_MAX;
						}
						const auto newGoRightUnitPosition = Simulator::getUnitInTimePosition(
							goRightUnitPosition, me.size, action, unitTime, game);

						action.velocity = 0;

						if (isBulletMoveCrossUnitMove(
							goRightUnitPosition,
							newGoRightUnitPosition,
							me.size,
							bulletPosition,
							newBulletPosition,
							halfBulletSize))
						{
							canGoRight = false;
						}
						else if (!exists && isBulletExplosionShootUnit(bullet, bulletCrossWallCenter, newGoRightUnitPosition, me.size))
						{
							//пуля ударится о стену. надо проверить взрыв
							canGoRight = false;
						}
						goRightUnitPosition = newGoRightUnitPosition;
					}
					
					
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

	const auto tickTime = 1.0 / game.properties.ticksPerSecond;
	for (const auto& bullet : game.bullets)
	{
		if (bullet.playerId == unit.playerId) continue;

		double unitTime = tickTime;

		const auto bulletSimulation = enemyBulletsSimulations.at(bullet);
		Vec2Double bulletInTimePosition;
		const auto exists = Simulator::getBulletInTimePosition(
			bullet, tickTime, bulletSimulation, game, bulletInTimePosition);
		if (!exists)
		{
			unitTime = bulletSimulation.targetCrossTime;
		}

		const auto unitInTimePosition = Simulator::getUnitInTimePosition(unit.position, unit.size, action, unitTime, game);			
		
		const auto cross = isBulletMoveCrossUnitMove(
			unit.position,
			unitInTimePosition,
			unit.size,
			bullet.position,
			bulletInTimePosition,
			bullet.size / 2.0);
		if (cross)
		{
			return false;
		}
		if (!exists)
		{
			const auto bulletCrossWallCenter = Vec2Double(
				bullet.position.x + bullet.velocity.x * bulletSimulation.targetCrossTime,
				bullet.position.y + bullet.velocity.y * bulletSimulation.targetCrossTime);
			if (isBulletExplosionShootUnit(bullet, bulletCrossWallCenter, unitInTimePosition, unit.size))
			{
				return false;
			}
		}
	}
	return true;
}

bool Strategy::isBulletExplosionShootUnit(const Bullet& bullet, const Vec2Double& bulletCrossWallCenter,
	const Vec2Double& unitPosition, const Vec2Double& unitSize)
{
	if (bullet.explosionParams == nullptr) return false;

	const auto radius = bullet.explosionParams->radius;

	const auto unitLeft = unitPosition.x - unitSize.x / 2;
	const auto unitRight = unitPosition.x + unitSize.x / 2;
	const auto unitBottom = unitPosition.y;
	const auto unitTop = unitPosition.y + unitSize.y;

	if (abs(bulletCrossWallCenter.x - unitLeft) <= radius && abs(bulletCrossWallCenter.y - unitBottom) <= radius) return true;
	if (abs(bulletCrossWallCenter.x - unitLeft) <= radius && abs(bulletCrossWallCenter.y - unitTop) <= radius) return true;
	if (abs(bulletCrossWallCenter.x - unitRight) <= radius && abs(bulletCrossWallCenter.y - unitTop) <= radius) return true;
	if (abs(bulletCrossWallCenter.x - unitRight) <= radius && abs(bulletCrossWallCenter.y - unitBottom) <= radius) return true;
	return false;
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


