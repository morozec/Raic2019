#include "Strategy.h"
#include "../common/Helper.h"
#include "../simulation/Simulator.h"
#include "../model/Unit.hpp"
#include "../mathcalc/MathHelper.h"
#include <map>
#include "ShootMeBullet.h"
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

	const auto x1 = enemy.position.x - enemy.size.x / 2;
	const auto x2 = enemy.position.x + enemy.size.x / 2;
	const auto y1 = enemy.position.y;
	const auto y2 = enemy.position.y + enemy.size.y;

	auto shootingCount = 0;

	for (auto i = -ANGLE_SPLIT_COUNT; i <= ANGLE_SPLIT_COUNT; ++i)
	{
		auto isShooting = false;
		double minCrossDist2 = INT_MAX;
		Vec2Double cp = Vec2Double(0, 0);

		const auto bulletVelocityLength = (*me.weapon).params.bullet.speed;

		const auto angle = *me.weapon->lastAngle + deltaAngle * i;
		const auto bulletPos1 = Vec2Double(
			bulletCenterPos.x - me.weapon->params.bullet.size / 2,
			bulletCenterPos.y - me.weapon->params.bullet.size / 2);
		auto bulletVelocity = Vec2Double(bulletVelocityLength * cos(angle), bulletVelocityLength * sin(angle));
		auto shootEnemyCrossPoint = Simulator::get_shoot_me_bullet_cross_point(
			x1, y1, x2, y2, bulletPos1, bulletVelocity, game);

		if (shootEnemyCrossPoint.hasCrossPoint)
		{
			isShooting = true;
			cp = shootEnemyCrossPoint.crossPoint;
			if (shootEnemyCrossPoint.crossPointDist2 < minCrossDist2)
			{
				minCrossDist2 = shootEnemyCrossPoint.crossPointDist2;
			}
		}


		const auto bulletPos2 = Vec2Double(
			bulletCenterPos.x - me.weapon->params.bullet.size / 2,
			bulletCenterPos.y + me.weapon->params.bullet.size / 2);
		shootEnemyCrossPoint = Simulator::get_shoot_me_bullet_cross_point(
			x1, y1, x2, y2, bulletPos2, bulletVelocity, game);

		if (shootEnemyCrossPoint.hasCrossPoint)
		{
			isShooting = true;
			cp = shootEnemyCrossPoint.crossPoint;
			if (shootEnemyCrossPoint.crossPointDist2 < minCrossDist2)
			{
				minCrossDist2 = shootEnemyCrossPoint.crossPointDist2;
			}
		}


		const auto bulletPos3 = Vec2Double(
			bulletCenterPos.x + me.weapon->params.bullet.size / 2,
			bulletCenterPos.y - me.weapon->params.bullet.size / 2);
		shootEnemyCrossPoint = Simulator::get_shoot_me_bullet_cross_point(
			x1, y1, x2, y2, bulletPos3, bulletVelocity, game);

		if (shootEnemyCrossPoint.hasCrossPoint)
		{
			isShooting = true;
			cp = shootEnemyCrossPoint.crossPoint;
			if (shootEnemyCrossPoint.crossPointDist2 < minCrossDist2)
			{
				minCrossDist2 = shootEnemyCrossPoint.crossPointDist2;
			}
		}

		const auto bulletPos4 = Vec2Double(
			bulletCenterPos.x + me.weapon->params.bullet.size / 2,
			bulletCenterPos.y + me.weapon->params.bullet.size / 2);
		shootEnemyCrossPoint = Simulator::get_shoot_me_bullet_cross_point(
			x1, y1, x2, y2, bulletPos4, bulletVelocity, game);

		if (shootEnemyCrossPoint.hasCrossPoint)
		{
			isShooting = true;
			cp = shootEnemyCrossPoint.crossPoint;
			if (shootEnemyCrossPoint.crossPointDist2 < minCrossDist2)
			{
				minCrossDist2 = shootEnemyCrossPoint.crossPointDist2;
			}
		}


		if (isShooting)
		{
			const auto dist = sqrt(minCrossDist2);
			const auto time = dist / bulletVelocityLength;

			const auto isCrossWall =
				Simulator::isBulletCrossWall(bulletPos1, bulletVelocity, time, game) ||
				Simulator::isBulletCrossWall(bulletPos2, bulletVelocity, time, game) ||
				Simulator::isBulletCrossWall(bulletPos3, bulletVelocity, time, game) ||
				Simulator::isBulletCrossWall(bulletPos4, bulletVelocity, time, game);

			if (!isCrossWall)
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


std::map<Bullet, double> Strategy::getEnemyBulletsShootWallTimes(const Game& game, int meId)
{
	const auto maxX = game.level.tiles.size() * TILE_SIZE;
	const auto maxY = game.level.tiles[0].size() * TILE_SIZE;

	std::map<Bullet, double> result;
	for (const auto& bullet : game.bullets)
	{
		if (bullet.playerId == meId) continue;

		const auto crossWallPoint = Simulator::getBulletCrossWallPoint(bullet, maxX, maxY, game);
		const auto wallDist = MathHelper::getVectorLength(
			Vec2Double(bullet.position.x - crossWallPoint.x, bullet.position.y - crossWallPoint.y));

		const auto bulletVelocity = MathHelper::getVectorLength(bullet.velocity);

		const double shootWallTime = wallDist / bulletVelocity;
		result[bullet] = shootWallTime;
	}

	return result;
}



int Strategy::getShootMeBulletTick(const Unit& me, const Bullet& bullet, const Game& game)
{
	auto x1 = me.position.x - me.size.x / 2;
	auto x2 = me.position.x + me.size.x / 2;
	auto y1 = me.position.y;
	auto y2 = me.position.y + me.size.y;

	if (bullet.velocity.x > 0 && bullet.position.x - bullet.size / 2 > x2) return -1;
	if (bullet.velocity.x < 0 && bullet.position.x + bullet.size / 2 < x1) return -1;
	if (bullet.velocity.y > 0 && bullet.position.y - bullet.size / 2 > y2) return -1;
	if (bullet.velocity.y < 0 && bullet.position.y + bullet.size / 2 < y1) return -1;


	double minCrossDist2 = INT_MAX;
	const Vec2Double* minCrossDist2Point = nullptr;

	const auto bulletPosition1 = Vec2Double(bullet.position.x - bullet.size / 2, bullet.position.y - bullet.size / 2);
	const auto smbcp1 = Simulator::get_shoot_me_bullet_cross_point(x1, y1, x2, y2, bulletPosition1, bullet.velocity, game);
	if (smbcp1.hasCrossPoint)
	{
		if (smbcp1.crossPointDist2 < minCrossDist2)
		{
			minCrossDist2 = smbcp1.crossPointDist2;
			minCrossDist2Point = &smbcp1.crossPoint;
		}
	}

	const auto bulletPosition2 = Vec2Double(bullet.position.x + bullet.size / 2, bullet.position.y - bullet.size / 2);
	const auto smbcp2 = Simulator::get_shoot_me_bullet_cross_point(x1, y1, x2, y2, bulletPosition2, bullet.velocity, game);
	if (smbcp2.hasCrossPoint)
	{
		if (smbcp2.crossPointDist2 < minCrossDist2)
		{
			minCrossDist2 = smbcp2.crossPointDist2;
			minCrossDist2Point = &smbcp2.crossPoint;
		}
	}

	const auto bulletPosition3 = Vec2Double(bullet.position.x + bullet.size / 2, bullet.position.y + bullet.size / 2);
	const auto smbcp3 = Simulator::get_shoot_me_bullet_cross_point(x1, y1, x2, y2, bulletPosition3, bullet.velocity, game);
	if (smbcp3.hasCrossPoint)
	{
		if (smbcp3.crossPointDist2 < minCrossDist2)
		{
			minCrossDist2 = smbcp3.crossPointDist2;
			minCrossDist2Point = &smbcp3.crossPoint;
		}
	}


	const auto bulletPosition4 = Vec2Double(bullet.position.x + bullet.size / 2, bullet.position.y + bullet.size / 2);
	const auto smbcp4 = Simulator::get_shoot_me_bullet_cross_point(x1, y1, x2, y2, bulletPosition4, bullet.velocity, game);
	if (smbcp4.hasCrossPoint)
	{
		if (smbcp4.crossPointDist2 < minCrossDist2)
		{
			minCrossDist2 = smbcp4.crossPointDist2;
			minCrossDist2Point = &smbcp4.crossPoint;
		}
	}

	if (minCrossDist2Point != nullptr)
	{
		auto bulletVelocityLength = MathHelper::getVectorLength(bullet.velocity);
		auto minCrossDist = sqrt(minCrossDist2);

		const auto time = minCrossDist / bulletVelocityLength;
		const auto isCrossWall =
			Simulator::isBulletCrossWall(bulletPosition1, bullet.velocity, time, game) ||
			Simulator::isBulletCrossWall(bulletPosition2, bullet.velocity, time, game) ||
			Simulator::isBulletCrossWall(bulletPosition3, bullet.velocity, time, game) ||
			Simulator::isBulletCrossWall(bulletPosition4, bullet.velocity, time, game);

		if (isCrossWall) return -1;

		int shootMeTick = static_cast<int>(ceil(minCrossDist / bulletVelocityLength * game.properties.ticksPerSecond));
		return shootMeTick;
	}


	return -1;
}


std::vector<ShootMeBullet> Strategy::getShootMeBullets(const Unit& unit, const Game& game)
{
	std::vector<ShootMeBullet> result;
	for (const auto& bullet : game.bullets)
	{
		if (bullet.playerId == unit.playerId) continue;
		const auto smbt = getShootMeBulletTick(unit, bullet, game);
		if (smbt == -1) continue;
		result.emplace_back(ShootMeBullet(bullet, smbt));
	}

	return result;
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
	const Unit& me, const std::vector<ShootMeBullet>& shootingMeBullets,
	const std::map<Bullet, double>& enemyBulletsShootWallTimes,
	const Game& game)
{
	if (shootingMeBullets.empty())
	{
		return std::make_tuple(GoNONE, -1, -1);
	}
	int minShootMeTick = INT_MAX;
	for (const auto& smb : shootingMeBullets)
	{
		if (smb.shootMeTick < minShootMeTick)
		{
			minShootMeTick = smb.shootMeTick;
		}
	}

	double maxShootWallTime = 0;
	for (const auto& item : enemyBulletsShootWallTimes)
	{
		if (item.second > maxShootWallTime)
		{
			maxShootWallTime = item.second;
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
		
		auto killGoUpTick = INT_MAX;
		auto killGoLeftTick = INT_MAX;
		auto killGoRightTick = INT_MAX;
		auto killGoDownTick = INT_MAX;

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

				const auto halfBulletSize = bullet.size / 2;
				auto jumpUnitPosition = me.position;
				auto fallUnitPosition = me.position;
				auto goLeftUnitPosition = me.position;
				auto goRightUnitPosition = me.position;
				
				const auto shootWallTick = static_cast<int>(ceil(enemyBulletsShootWallTimes.at(bullet) * game.properties.ticksPerSecond));
				auto bulletPosition = bullet.position;
				
				for (int tick = 1; tick <= shootWallTick; ++tick)
				{
					const auto bulletTime = tick / game.properties.ticksPerSecond;
					auto newBulletPosition = Simulator::getBulletInTimePosition(bullet, bulletTime, game);

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



bool Strategy::isSafeMove(const Unit& unit, const UnitAction& action, const std::map<Bullet, double>& enemyBulletShootWallTimes, const Game& game)
{
	//TODO: учесть удар пули о стену
	for (const auto& bullet : game.bullets)
	{
		if (bullet.playerId == unit.playerId) continue;
		const auto time = 1.0 / game.properties.ticksPerSecond;

		const auto unitInTimePosition = Simulator::getUnitNextTickPosition(unit.position, unit.size, action, game);
		const auto bulletInTimePosition = Simulator::getBulletInTimePosition(bullet, time, game);
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


