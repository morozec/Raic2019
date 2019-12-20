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
#include <map>
#include <set>


inline bool operator<(const Bullet& lhs, const Bullet& rhs)
{
	return lhs.position.x < rhs.position.x;
}

double Strategy::getShootEnemyProbability(const Unit& me, const Unit& enemy, const Game& game, double spread)
{
	if (me.weapon == nullptr) return 0;
	if (me.weapon->lastAngle == nullptr) return 1;
	return getShootEnemyProbability(
		me.position, me.size, enemy.position, enemy.size, *me.weapon, spread, *(me.weapon->lastAngle), game);
}

double Strategy::getShootEnemyProbability(
	const Vec2Double& mePosition, const Vec2Double& meSize,
	const Vec2Double& enemyPosition, const Vec2Double& enemySize,
	const Weapon& weapon, double spread, double shootingAngle,
	const Game& game)
{

	const auto bulletCenterPos = Vec2Double(mePosition.x, mePosition.y + meSize.y / 2);
	const auto deltaAngle = spread / ANGLE_SPLIT_COUNT;

	const auto xLeft = enemyPosition.x - enemySize.x / 2;
	const auto xRight = enemyPosition.x + enemySize.x / 2;
	const auto yDown = enemyPosition.y;
	const auto yUp = enemyPosition.y + enemySize.y;

	auto shootingCount = 0;

	const auto bulletVelocityLength = weapon.params.bullet.speed;
	const auto halfBulletSize = weapon.params.bullet.size / 2;

	for (auto i = -ANGLE_SPLIT_COUNT; i <= ANGLE_SPLIT_COUNT; ++i)
	{
		const auto angle = shootingAngle + deltaAngle * i;
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
			}
		}
	}

	return shootingCount * 1.0 / (2 * ANGLE_SPLIT_COUNT + 1.0);
}

double Strategy::getShootEnemyProbability(
	const Vec2Double& meShootingPosition, const Vec2Double& meSize, 
	double shootingAngle, double spread, 
	const WeaponParams& weaponParams,
	const std::vector<Vec2Double>& enemyPositions, const Vec2Double& enemySize, 
	const Game& game)
{
	const auto tickTime = 1.0 / game.properties.ticksPerSecond;
	const auto deltaAngle = spread / ANGLE_SPLIT_COUNT;
	const auto halfBulletSize = weaponParams.bullet.size / 2;
	const auto startBulletPosition = Vec2Double(meShootingPosition.x, meShootingPosition.y + meSize.y / 2);
	
	auto shootingCount = 0;
	
	for (auto i = -ANGLE_SPLIT_COUNT; i <= ANGLE_SPLIT_COUNT; ++i)
	{
		const auto angle = shootingAngle + deltaAngle * i;
		auto bulletVelocity = Vec2Double(weaponParams.bullet.speed * cos(angle), weaponParams.bullet.speed * sin(angle));

		const auto bulletSimulation = Simulator::getBulletSimulation(
			startBulletPosition, bulletVelocity, halfBulletSize, game);
		const auto bulletPositions = Simulator::getBulletPositions(
			startBulletPosition, bulletVelocity, bulletSimulation.targetCrossTime, game);
		
		const auto shootWallTick = static_cast<size_t>(ceil(bulletSimulation.targetCrossTime * game.properties.ticksPerSecond));



		
		
		
		
		for (size_t j = 0; j < shootWallTick; ++j)
		{
			const auto& bp0 = bulletPositions.at(j);
			const auto& bp1 = bulletPositions.at(j < shootWallTick - 1 ? j + 1 : -1);

			const auto& ep0 = j < enemyPositions.size() ? enemyPositions[j] : enemyPositions.back();
			const auto& ep1 = j + 1 < enemyPositions.size() ? enemyPositions[j + 1] : enemyPositions.back();

			
			if (j == shootWallTick - 1) // тик удара о стену
			{
				const auto thisTickTime = bulletSimulation.targetCrossTime - j / game.properties.ticksPerSecond;
				const auto thisTickPart = thisTickTime / tickTime;
				const auto shootWallEp1 = ep0 + (ep1 - ep0) * thisTickPart;

				if (isBulletExplosionShootUnit(
					weaponParams.explosion, bulletPositions.at(-1), shootWallEp1, enemySize))
				{
					shootingCount++;
					break;
				}

				if (isBulletMoveCrossUnitMove(
					ep0, shootWallEp1, enemySize, bp0, bp1, halfBulletSize))
				{
					shootingCount++;
					break;
				}
			}		
			
			else
			{
				if (isBulletMoveCrossUnitMove(
					ep0, ep1, enemySize, bp0, bp1, halfBulletSize))
				{
					shootingCount++;
					break;
				}
			}
				
			
		}
	}
	return shootingCount * 1.0 / (ANGLE_SPLIT_COUNT * 2 + 1);
}

std::map<Bullet, BulletSimulation> Strategy::getEnemyBulletsSimulation(const Game& game, int mePlayerId)
{
	std::map<Bullet, BulletSimulation> simulations;
	for (const auto& bullet: game.bullets)
	{
		if (bullet.playerId == mePlayerId && bullet.explosionParams == nullptr) continue;
		auto simulation = Simulator::getBulletSimulation(bullet.position, bullet.velocity, bullet.size / 2, game);

		if (bullet.playerId == mePlayerId)//проверим пулю на столкновение с врагом
		{
			for (const auto& unit: game.units)
			{
				if (unit.playerId == mePlayerId) continue;

				Vec2Double crossPoint;
				Vec2Double bulletCorner;
				const auto isShooting = Simulator::getBulletRectangleFirstCrossPoint(
					bullet.position, bullet.velocity, bullet.size / 2,
					unit.position.x - unit.size.x / 2, unit.position.y, unit.position.x + unit.size.x / 2, unit.position.y + unit.size.y,
					crossPoint, bulletCorner);
				if (!isShooting) continue; //промахнемся
				
				if (MathHelper::getVectorLength2(bulletCorner, crossPoint) >
					MathHelper::getVectorLength2(simulation.bulletCrossCorner, simulation.targetCrossPoint))
					continue; //пуля раньше ударится в стену

				simulation.targetCrossPoint = crossPoint;
				simulation.bulletCrossCorner = bulletCorner;
				simulation.targetCrossTime = MathHelper::getVectorLength(bulletCorner, crossPoint) /
					MathHelper::getVectorLength(bullet.velocity);
			}
		}
		
		simulation.bulletPositions = Simulator::getBulletPositions(bullet.position, bullet.velocity, simulation.targetCrossTime, game);
		simulations[bullet] = simulation;
	}
	return simulations;
}

std::map<Bullet, int> Strategy::getShootMeBullets(
	const Vec2Double& mePosition, const Vec2Double& meSize, const JumpState& meJumpState, int mePlayerId, int meUnitId,
	const std::map<Bullet, BulletSimulation>& enemyBulletsSimulations, int addTicks,
	const Game& game) const
{
	if (addTicks != 0 && addTicks != 1) throw std::runtime_error("addTicks is not 0 or 1");
	
	const auto tickTime = 1.0 / game.properties.ticksPerSecond;
	std::map<Bullet, int> shootMeBullets;
	std::map<int, Vec2Double> mePositions;
	std::map<int, JumpState> meJumpStates;
	
	mePositions[0] = mePosition;
	meJumpStates[0] = meJumpState;

	UnitAction action;
	action.velocity = 0;
	action.jump = false;
	action.jumpDown = false;

	for (const auto& item:enemyBulletsSimulations)
	{
		const auto& bullet = item.first;
		const auto& bulletSimulation = item.second;
		
		std::map<int, Vec2Double> bulletPositions;
		bulletPositions[0] = bullet.position;		
	
		const auto bulletCrossWallTime = bulletSimulation.targetCrossTime;
		const int bulletCrossWallTick = static_cast<int>(ceil(bulletCrossWallTime*game.properties.ticksPerSecond));

		if (addTicks == 1 && bulletCrossWallTick <= 1)
		{
			//пуля ударилась в тик 0-1. ее мы учли заранее
			continue;
		}
		
		for (int tick = 1; tick <= bulletCrossWallTick - addTicks; ++tick)
		{
			Vec2Double bulletInTimePosition;
			const bool exists = Simulator::getBulletInTimePosition(
				bullet.position, bullet.velocity, (tick+addTicks)* tickTime, 
				bulletSimulation.targetCrossTime, game, bulletInTimePosition);
						

			if (exists)
			{
				bulletPositions[tick] = bulletInTimePosition;
				
				Vec2Double unitInTimePosition;
				if (mePositions.count(tick) > 0) unitInTimePosition = mePositions[tick];//уже посчитали для другой пули
				else
				{
					auto unitInTimeJumpState = meJumpStates.at(tick - 1);
					unitInTimePosition =
						Simulator::getUnitInTimePosition(
							mePositions.at(tick - 1), meSize, meUnitId, action, tickTime, unitInTimeJumpState, game);
					mePositions[tick] = unitInTimePosition;
					meJumpStates[tick] = unitInTimeJumpState;
				}
				const bool isShooting = bullet.playerId != mePlayerId && isBulletMoveCrossUnitMove(
					mePositions.at(tick - 1), unitInTimePosition, meSize,
					bulletPositions[tick - 1], bulletInTimePosition, bullet.size / 2.0);
				if (isShooting)
				{
					shootMeBullets[bullet] = tick;
					break;
				}
			}
			else
			{
				
				const auto thisTickBulletTime = bulletSimulation.targetCrossTime - (tick + addTicks - 1) * tickTime;

				auto unitInTimeJumpState = meJumpStates.at(tick - 1);
				const auto unitInTimePosition =
					Simulator::getUnitInTimePosition(
						mePositions.at(tick - 1), meSize, meUnitId, action, thisTickBulletTime, unitInTimeJumpState, game);
				auto isShooting = bullet.playerId != mePlayerId && isBulletMoveCrossUnitMove(
					mePositions.at(tick - 1), unitInTimePosition, meSize,
					bulletPositions[tick - 1], bulletInTimePosition, bullet.size / 2.0);
				if (!isShooting)
				{
					const auto bulletCrossWallCenter = Vec2Double(
						bullet.position.x + bullet.velocity.x * bulletSimulation.targetCrossTime,
						bullet.position.y + bullet.velocity.y * bulletSimulation.targetCrossTime);
					if (isBulletExplosionShootUnit(bullet.explosionParams, bulletCrossWallCenter, unitInTimePosition, meSize))
					{
						isShooting = true;
					}
				}				

				if (isShooting)
				{
					shootMeBullets[bullet] = tick;
					break;
				}				
			}			
		}
				
	}
	return shootMeBullets;
	
}


bool Strategy::isBulletMoveCrossUnitMove(
	const Vec2Double& unitPos, const Vec2Double& newUnitPos, const Vec2Double& unitSize,
	const Vec2Double& bulletPos, const Vec2Double& newBulletPos, double halfBulletSize)
{	
	Vec2Double bulletPolygon[6];
	Simulator::getPolygon(bulletPos, newBulletPos, halfBulletSize, bulletPolygon);

	Segment bulletSegments[6];
	for (int i = 0; i < 6; ++i)
	{
		const int endIndex = i < 5 ? i + 1 : 0;
		bulletSegments[i] = Segment(bulletPolygon[i], bulletPolygon[endIndex]);
	}

	const auto isStaticUnit = std ::abs(unitPos.x - newUnitPos.x) < TOLERANCE && 
		std ::abs(unitPos.y - newUnitPos.y) < TOLERANCE;	

	if (isStaticUnit)
	{
		Segment unitSegments[4];
		unitSegments[0] = Segment(
			{ unitPos.x - unitSize.x / 2, unitPos.y }, 
			{ unitPos.x - unitSize.x / 2,unitPos.y + unitSize.y });
		unitSegments[1] = Segment(
			{ unitPos.x - unitSize.x / 2, unitPos.y + unitSize.y },
			{ unitPos.x + unitSize.x / 2,unitPos.y + unitSize.y });
		unitSegments[2] = Segment(
			{ unitPos.x + unitSize.x / 2, unitPos.y + unitSize.y },
			{ unitPos.x + unitSize.x / 2,unitPos.y });
		unitSegments[3] = Segment(
			{ unitPos.x + unitSize.x / 2, unitPos.y },
			{ unitPos.x - unitSize.x / 2,unitPos.y });
		
		for (const auto& us : unitSegments)
		{
			for (const auto& bs : bulletSegments)
			{
				const auto cross = MathHelper::areSegmentsCross(us, bs);
				if (cross) return true;
			}
		}
	}else
	{
		Vec2Double unitPolygon[6];
		Simulator::getPolygon(unitPos, newUnitPos, unitSize, unitPolygon);

		Segment unitSegments[6];
		for (int i = 0; i < 6; ++i)
		{
			const int endIndex = i < 5 ? i + 1 : 0;
			unitSegments[i] = Segment(unitPolygon[i], unitPolygon[endIndex]);
		}

		for (const auto& us : unitSegments)
		{
			for (const auto& bs : bulletSegments)
			{
				const auto cross = MathHelper::areSegmentsCross(us, bs);
				if (cross) return true;
			}
		}
	}	
	
	return false;
}


std::tuple<RunawayDirection, int, int, int> Strategy::getRunawayAction(
	const Vec2Double& unitPosition, const Vec2Double& unitSize, int unitPlayerId, int unitId,
	const JumpState& jumpState,
	const std::map<Bullet, int>& shootingMeBullets,	
	const std::map<Bullet, BulletSimulation>& enemyBulletsSimulations, int addTicks,
	bool checkUp, bool checkDown, bool checkLeft, bool checkRight,
	const Game& game) const
{
	if (addTicks != 0 && addTicks != 1) throw std::runtime_error("addTicks is not 0 or 1");
	
	if (shootingMeBullets.empty())
	{
		return std::make_tuple(GoNONE, -1, -1, 0);
	}

	int minShootMeDamage = 0;
	int maxFirstShootMeTick = INT_MAX;
	for (const auto& item: shootingMeBullets)
	{
		minShootMeDamage += item.first.damage;
		if (item.second < maxFirstShootMeTick) maxFirstShootMeTick = item.second;
	}

	auto bestRunAwayDirection = GoNONE;
	auto bestStartGoTick = -1;
	auto bestStopGoTick = -1;
	
	const auto tickTime = 1.0 / game.properties.ticksPerSecond;
	
	auto minShootMeTick = INT_MAX;
	for (const auto& smb : shootingMeBullets)
	{
		if (smb.second < minShootMeTick)
		{
			minShootMeTick = smb.second;
		}
	}
	

	double maxShootWallTime = 0;
	for (const auto& item : enemyBulletsSimulations)
	{
		if (item.second.targetCrossTime > maxShootWallTime)
		{
			maxShootWallTime = item.second.targetCrossTime;
		}
	}
	int maxShootWallTick = static_cast<int>(ceil(maxShootWallTime * game.properties.ticksPerSecond));
	maxShootWallTick -= addTicks;

	
	UnitAction action;
	action.jump = false;
	action.jumpDown = false;
	action.velocity = false;

	std::map<int, int> beforeStartGoUpDamage;
	std::map<int, int> beforeStartGoDownDamage;
	std::map<int, int> beforeStartGoLeftDamage;
	std::map<int, int> beforeStartGoRightDamage;

	std::map<int, std::map<int, int>> beforeStopGoUpDamage;
	std::map<int, std::map<int, int>> beforeStopGoDownDamage;
	std::map<int, std::map<int, int>> beforeStopGoLeftDamage;
	std::map<int, std::map<int, int>> beforeStopGoRightDamage;
		

	for (int startGoTick = minShootMeTick - 1; startGoTick >= 0; startGoTick--)
	{	
		
	/*	auto killGoUpTick = INT_MAX;
		auto killGoLeftTick = INT_MAX;
		auto killGoRightTick = INT_MAX;
		auto killGoDownTick = INT_MAX;*/

		for (int stopGoTick = startGoTick + 1; stopGoTick < maxShootWallTick; ++stopGoTick)
		{			
			auto canGoUp = checkUp;
			auto upDamage = 0;
			int upFirstMeTick = INT_MAX;
			int upShootMeCount = 0;
			
			auto canGoLeft = checkLeft;
			auto leftDamage = 0;
			int leftFirstMeTick = INT_MAX;
			int leftShootMeCount = 0;
			
			auto canGoRight = checkRight;
			auto rightDamage = 0;
			int rightFirstMeTick = INT_MAX;
			int rightShootMeCount = 0;
						
			auto canGoDown = checkDown;
			auto downDamage = 0;
			int downFirstMeTick = INT_MAX;
			int downShootMeCount = 0;

			auto jumpUnitPosition = unitPosition;
			auto fallUnitPosition = unitPosition;
			auto goLeftUnitPosition = unitPosition;
			auto goRightUnitPosition = unitPosition;

			auto goUpJumpState = jumpState;
			auto goDownJumpState = jumpState;
			auto goLeftJumpState = jumpState;
			auto goRightJumpState = jumpState;

			std::map<Bullet, bool> gotUpBullets;
			std::map<Bullet, bool> gotDownBullets;
			std::map<Bullet, bool> gotLeftBullets;
			std::map<Bullet, bool> gotRightBullets;

			for (const auto& item : enemyBulletsSimulations)
			{
				const auto& bullet = item.first;				
				gotUpBullets[bullet] = false;
				gotDownBullets[bullet] = false;
				gotLeftBullets[bullet] = false;
				gotRightBullets[bullet] = false;
			}

			for (int tick = 1; tick <= maxShootWallTick; ++tick)
			{
				auto thisTickUpDamage = 0;
				auto thisTickDownDamage = 0;
				auto thisTickLeftDamage = 0;
				auto thisTickRightDamage = 0;
				
				
				if (tick > stopGoTick)
				{
					action.jump = false;
				}
				else if (tick > startGoTick)
				{
					action.jump = true;
				}
				
				auto thisTickGoUpJumpState = goUpJumpState;
				const auto thisTickJumpUnitPosition = Simulator::getUnitInTimePosition(
					jumpUnitPosition, unitSize, unitId, action, tickTime, thisTickGoUpJumpState, game);
				action.jump = false;				
				
				//fall
				if (tick > stopGoTick)
				{
					action.jumpDown = false;
				}
				else if (tick > startGoTick)
				{
					action.jumpDown = true;
				}

				auto thisTickGoDownJumpState = goDownJumpState;
				const auto thisTickFallUnitPosition = Simulator::getUnitInTimePosition(
					fallUnitPosition, unitSize, unitId, action, tickTime, thisTickGoDownJumpState, game);
				action.jumpDown = false;

				//left
				if (tick > stopGoTick)
				{
					action.velocity = 0;
				}
				else if (tick > startGoTick)
				{
					action.velocity = -INT_MAX;
				}
				auto thisTickGoLeftJumpState = goLeftJumpState;
				const auto thisTickGoLeftUnitPosition = Simulator::getUnitInTimePosition(
					goLeftUnitPosition, unitSize, unitId, action, tickTime, thisTickGoLeftJumpState, game);
				action.velocity = 0;

				//right
				if (tick > stopGoTick)
				{
					action.velocity = 0;
				}
				else if (tick > startGoTick)
				{
					action.velocity = INT_MAX;
				}
				auto thisTickGoRightJumpState = goRightJumpState;
				const auto thisTickGoRightUnitPosition = Simulator::getUnitInTimePosition(
					goRightUnitPosition, unitSize, unitId, action, tickTime, thisTickGoRightJumpState, game);
				action.velocity = 0;

				if (tick <= startGoTick && 
					beforeStartGoUpDamage.count(tick) > 0 && 
					beforeStartGoDownDamage.count(tick) > 0 && 
					beforeStartGoLeftDamage.count(tick) > 0 && 
					beforeStartGoRightDamage.count(tick) > 0)
				{
					thisTickUpDamage = beforeStartGoUpDamage[tick];
					thisTickDownDamage = beforeStartGoDownDamage[tick];
					thisTickLeftDamage = beforeStartGoLeftDamage[tick];
					thisTickRightDamage = beforeStartGoRightDamage[tick];
				}
				else if (tick <= stopGoTick && tick > startGoTick && 
					beforeStopGoUpDamage.count(tick) > 0 && beforeStopGoUpDamage[tick].count(startGoTick) > 0 &&
					beforeStopGoDownDamage.count(tick) > 0 && beforeStopGoDownDamage[tick].count(startGoTick) > 0 &&
					beforeStopGoLeftDamage.count(tick) > 0 && beforeStopGoLeftDamage[tick].count(startGoTick) > 0 &&
					beforeStopGoRightDamage.count(tick) > 0 && beforeStopGoRightDamage[tick].count(startGoTick) > 0)
				{
					thisTickUpDamage = beforeStopGoUpDamage[tick][startGoTick];
					thisTickDownDamage = beforeStopGoDownDamage[tick][startGoTick];
					thisTickLeftDamage = beforeStopGoLeftDamage[tick][startGoTick];
					thisTickRightDamage = beforeStopGoRightDamage[tick][startGoTick];
				}
				else
				{
					//const auto bulletTime = (tick + addTicks) / game.properties.ticksPerSecond;
					for (const auto& item : enemyBulletsSimulations)
					{
						const auto& bullet = item.first;
						const auto& bulletSimulation = item.second;
						
						const auto bulletCrossWallCenter = Vec2Double(
							bullet.position.x + bullet.velocity.x * bulletSimulation.targetCrossTime,
							bullet.position.y + bullet.velocity.y * bulletSimulation.targetCrossTime
						);
						const auto halfBulletSize = bullet.size / 2;

						auto shootWallTick = static_cast<int>(ceil(bulletSimulation.targetCrossTime * game.properties.ticksPerSecond));
						if (shootWallTick < tick + addTicks) continue;//ударилась в стену раньше

						
						const auto bulletExists = bulletSimulation.bulletPositions.count(tick + addTicks) > 0;
						const auto notExistsUnitTime = bulletSimulation.targetCrossTime - (tick + addTicks - 1) / game.properties.ticksPerSecond;
						const auto& prevTickBulletPosition = bulletSimulation.bulletPositions.at(tick + addTicks - 1);
						const auto& thisTickBulletPosition = bulletSimulation.bulletPositions.at( bulletExists ? tick + addTicks : -1);

						//jump
						if (canGoUp && !gotUpBullets[bullet]) {
							/*if (!thisTickCanJump)
							{
								action.jump = false;
							}
							else
							{*/
								if (tick > stopGoTick)
								{
									action.jump = false;
								}
								else if (tick > startGoTick)
								{
									action.jump = true;
								}
							//}

							auto newGoUpJumpState = goUpJumpState;
							const auto newJumpUnitPosition = bulletExists ? thisTickJumpUnitPosition :
								Simulator::getUnitInTimePosition(
									jumpUnitPosition,
									unitSize,
									unitId,
									action,
									notExistsUnitTime,
									newGoUpJumpState,
									game);

							action.jump = false;

							if (bullet.playerId != unitPlayerId && isBulletMoveCrossUnitMove(
								jumpUnitPosition,
								newJumpUnitPosition,
								unitSize,
								prevTickBulletPosition,
								thisTickBulletPosition,
								halfBulletSize) ||
								!bulletExists && isBulletExplosionShootUnit(bullet.explosionParams, bulletCrossWallCenter, newJumpUnitPosition, unitSize))
							{
								thisTickUpDamage += bullet.damage;
								gotUpBullets[bullet] = true;
							}							
						}


						//fall
						if (canGoDown && !gotDownBullets[bullet]) {
							if (tick > stopGoTick)
							{
								action.jumpDown = false;
							}
							else if (tick > startGoTick)
							{
								action.jumpDown = true;
							}

							auto newGoDownJumpState = goDownJumpState;
							const auto newFallUnitPosition = bulletExists ? thisTickFallUnitPosition :
								Simulator::getUnitInTimePosition(
									fallUnitPosition,
									unitSize,
									unitId,
									action,
									notExistsUnitTime,
									newGoDownJumpState,
									game);

							action.jumpDown = false;

							if (bullet.playerId != unitPlayerId && isBulletMoveCrossUnitMove(
								fallUnitPosition,
								newFallUnitPosition,
								unitSize,
								prevTickBulletPosition,
								thisTickBulletPosition,
								halfBulletSize) ||
								!bulletExists && isBulletExplosionShootUnit(bullet.explosionParams, bulletCrossWallCenter, newFallUnitPosition, unitSize))
							{
								thisTickDownDamage += bullet.damage;
								gotDownBullets[bullet] = true;
							}
						}

						//left
						if (canGoLeft && !gotLeftBullets[bullet])
						{
							if (tick > stopGoTick)
							{
								action.velocity = 0;
							}
							else if (tick > startGoTick)
							{
								action.velocity = -INT_MAX;
							}

							auto newGoLeftJumpState = goLeftJumpState;
							const auto newGoLeftUnitPosition = bulletExists ? thisTickGoLeftUnitPosition :
								Simulator::getUnitInTimePosition(
									goLeftUnitPosition,
									unitSize,
									unitId,
									action,
									notExistsUnitTime,
									newGoLeftJumpState,
									game);

							action.velocity = 0;

							if (bullet.playerId != unitPlayerId && isBulletMoveCrossUnitMove(
								goLeftUnitPosition,
								newGoLeftUnitPosition,
								unitSize,
								prevTickBulletPosition,
								thisTickBulletPosition,
								halfBulletSize) ||
								!bulletExists && isBulletExplosionShootUnit(bullet.explosionParams, bulletCrossWallCenter, newGoLeftUnitPosition, unitSize))
							{
								thisTickLeftDamage += bullet.damage;
								gotLeftBullets[bullet] = true;
							}
						}

						//right
						if (canGoRight && !gotRightBullets[bullet])
						{
							if (tick > stopGoTick)
							{
								action.velocity = 0;
							}
							else if (tick > startGoTick)
							{
								action.velocity = INT_MAX;
							}

							auto newGoRightJumpState = goRightJumpState;
							const auto newGoRightUnitPosition = bulletExists ? thisTickGoRightUnitPosition :
								Simulator::getUnitInTimePosition(
									goRightUnitPosition,
									unitSize,
									unitId,
									action,
									notExistsUnitTime,
									newGoRightJumpState,
									game);

							action.velocity = 0;

							if (bullet.playerId != unitPlayerId && isBulletMoveCrossUnitMove(
								goRightUnitPosition,
								newGoRightUnitPosition,
								unitSize,
								prevTickBulletPosition,
								thisTickBulletPosition,
								halfBulletSize) ||
								!bulletExists && isBulletExplosionShootUnit(bullet.explosionParams, bulletCrossWallCenter, newGoRightUnitPosition, unitSize))
							{
								thisTickRightDamage += bullet.damage;
								gotRightBullets[bullet] = true;
							}
						}

					}
					if (tick <= startGoTick) {
						beforeStartGoUpDamage[tick] = thisTickUpDamage;
						beforeStartGoDownDamage[tick] = thisTickDownDamage;
						beforeStartGoLeftDamage[tick] = thisTickLeftDamage;
						beforeStartGoRightDamage[tick] = thisTickRightDamage;
					}
					else if (tick <= stopGoTick && tick > startGoTick)
					{
						beforeStopGoUpDamage[tick][startGoTick] = thisTickUpDamage;
						beforeStopGoDownDamage[tick][startGoTick] = thisTickDownDamage;
						beforeStopGoLeftDamage[tick][startGoTick] = thisTickLeftDamage;
						beforeStopGoRightDamage[tick][startGoTick] = thisTickRightDamage;
					}
					
				}
				jumpUnitPosition = thisTickJumpUnitPosition;
				fallUnitPosition = thisTickFallUnitPosition;
				goLeftUnitPosition = thisTickGoLeftUnitPosition;
				goRightUnitPosition = thisTickGoRightUnitPosition;

				goUpJumpState = thisTickGoUpJumpState;
				goDownJumpState = thisTickGoDownJumpState;
				goLeftJumpState = thisTickGoLeftJumpState;
				goRightJumpState = thisTickGoRightJumpState;

				upDamage += thisTickUpDamage;
				if (thisTickUpDamage > 0) {
					upShootMeCount++;
					if (tick < upFirstMeTick) upFirstMeTick = tick;
				}
					
				
				downDamage += thisTickDownDamage;
				if (thisTickDownDamage > 0) {
					downShootMeCount++;
					if (tick < downFirstMeTick) downFirstMeTick = tick;
				}
				
				leftDamage += thisTickLeftDamage;
				if (thisTickLeftDamage > 0) {
					leftShootMeCount++;
					if (tick < leftFirstMeTick) leftFirstMeTick = tick;
				}
				
				rightDamage += thisTickRightDamage;
				if (thisTickRightDamage > 0)
				{
					rightShootMeCount++;
					if (tick < rightFirstMeTick) rightFirstMeTick = tick;
				} 

				if ((!canGoUp || upDamage >= minShootMeDamage) &&
					(!canGoDown || downDamage >= minShootMeDamage) &&
					(!canGoLeft || leftDamage >= minShootMeDamage) &&
					(!canGoRight || rightDamage >= minShootMeDamage))
					break;
			}

			if (canGoUp && upDamage == 0)
				return std::make_tuple(GoUP, startGoTick, stopGoTick, 0);
			if (canGoDown && downDamage == 0)
				return std::make_tuple(GoDOWN, startGoTick, stopGoTick, 0);
			if (canGoLeft && leftDamage == 0)
				return std::make_tuple(GoLEFT, startGoTick, stopGoTick, 0);
			if (canGoRight && rightDamage == 0)
				return std::make_tuple(GoRIGHT, startGoTick, stopGoTick, 0);

			if (canGoUp && (upDamage < minShootMeDamage || 
				upDamage == minShootMeDamage && upFirstMeTick > maxFirstShootMeTick && upShootMeCount >= 2))
			{
				minShootMeDamage = upDamage;
				bestRunAwayDirection = GoUP;
				bestStartGoTick = startGoTick;
				bestStopGoTick = stopGoTick;
				maxFirstShootMeTick = upFirstMeTick;
			}
			if (canGoDown && (downDamage < minShootMeDamage || 
				downDamage == minShootMeDamage && downFirstMeTick > maxFirstShootMeTick && downShootMeCount >= 2))
			{
				minShootMeDamage = downDamage;
				bestRunAwayDirection = GoDOWN;
				bestStartGoTick = startGoTick;
				bestStopGoTick = stopGoTick;
				maxFirstShootMeTick = downFirstMeTick;
			}
			if (canGoLeft && (leftDamage < minShootMeDamage || 
				leftDamage == minShootMeDamage && leftFirstMeTick > maxFirstShootMeTick && leftShootMeCount >= 2))
			{
				minShootMeDamage = leftDamage;
				bestRunAwayDirection = GoLEFT;
				bestStartGoTick = startGoTick;
				bestStopGoTick = stopGoTick;
				maxFirstShootMeTick = leftFirstMeTick;
			}
			if (canGoRight && (rightDamage < minShootMeDamage || 
				rightDamage == minShootMeDamage && rightFirstMeTick > maxFirstShootMeTick && rightShootMeCount >= 2))
			{
				minShootMeDamage = rightDamage;
				bestRunAwayDirection = GoRIGHT;
				bestStartGoTick = startGoTick;
				bestStopGoTick = stopGoTick;
				maxFirstShootMeTick = rightFirstMeTick;
			}					
		
		}
	}

	return std::make_tuple(bestRunAwayDirection, bestStartGoTick, bestStopGoTick, minShootMeDamage); 
}


std::set<Bullet> Strategy::isSafeMove(
	const Unit& unit, const UnitAction& action, const std::map<Bullet, BulletSimulation>& enemyBulletsSimulations, const Game& game)
{
	std::set<Bullet> thisTickShootMeBullets;
	const auto tickTime = 1.0 / game.properties.ticksPerSecond;
	for (const auto& item : enemyBulletsSimulations)
	{
		const auto& bullet = item.first;
		const auto& bulletSimulation = item.second;
		
		double unitTime = tickTime;
		
		Vec2Double bulletInTimePosition;
		const auto exists = Simulator::getBulletInTimePosition(
			bullet.position, bullet.velocity, tickTime, bulletSimulation.targetCrossTime, game, bulletInTimePosition);
		if (!exists)
		{
			unitTime = bulletSimulation.targetCrossTime;
		}
		auto jumpState = unit.jumpState;
		const auto unitInTimePosition = Simulator::getUnitInTimePosition(
			unit.position, unit.size, unit.id, action, unitTime, jumpState, game);			
		
		const auto cross = bullet.playerId != unit.playerId && isBulletMoveCrossUnitMove(
			unit.position,
			unitInTimePosition,
			unit.size,
			bullet.position,
			bulletInTimePosition,
			bullet.size / 2.0);
		if (cross)
		{
			thisTickShootMeBullets.insert(bullet);
			continue;
		}
		if (!exists)
		{
			const auto bulletCrossWallCenter = Vec2Double(
				bullet.position.x + bullet.velocity.x * bulletSimulation.targetCrossTime,
				bullet.position.y + bullet.velocity.y * bulletSimulation.targetCrossTime);
			if (isBulletExplosionShootUnit(bullet.explosionParams, bulletCrossWallCenter, unitInTimePosition, unit.size))
			{
				thisTickShootMeBullets.insert(bullet);
				continue;
			}
		}
	}
	return thisTickShootMeBullets;
}

bool Strategy::isBulletExplosionShootUnit(
	const std::shared_ptr<ExplosionParams>& explosionParams, const Vec2Double& bulletCrossWallCenter,
	const Vec2Double& unitPosition, const Vec2Double& unitSize)
{
	if (explosionParams == nullptr) return false;

	const auto radius = explosionParams->radius;

	const auto unitLeft = unitPosition.x - unitSize.x / 2;
	const auto unitRight = unitPosition.x + unitSize.x / 2;
	const auto unitBottom = unitPosition.y;
	const auto unitTop = unitPosition.y + unitSize.y;

	if (std::abs(bulletCrossWallCenter.x - unitLeft) <= radius && std::abs(bulletCrossWallCenter.y - unitBottom) <= radius) return true;
	if (std::abs(bulletCrossWallCenter.x - unitLeft) <= radius && std::abs(bulletCrossWallCenter.y - unitTop) <= radius) return true;
	if (std::abs(bulletCrossWallCenter.x - unitRight) <= radius && std::abs(bulletCrossWallCenter.y - unitTop) <= radius) return true;
	if (std::abs(bulletCrossWallCenter.x - unitRight) <= radius && std::abs(bulletCrossWallCenter.y - unitBottom) <= radius) return true;
	return false;
}


int Strategy::getRunawayDirection(int id) const
{
	return runaway_directions_.at(id);
}

int Strategy::getStopRunawayTick(int id) const
{
	return stop_runaway_ticks_.at(id);
}

void Strategy::setRunaway(int id, RunawayDirection runaway_direction, int srt)
{
	runaway_directions_[id] = runaway_direction;
	stop_runaway_ticks_[id] = srt;
}

void Strategy::decreaseStopRunawayTick(int id)
{
	if (stop_runaway_ticks_.at(id) >= 0) stop_runaway_ticks_[id]--;
}

size_t Strategy::getStartedJumpY(int id) const
{
	return startedJumpYs_.at(id);
}

void Strategy::setStartedJumpY(int id,size_t newStartedJumpY)
{
	startedJumpYs_[id] = newStartedJumpY;
}


