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

std::map<Bullet, BulletSimulation> Strategy::getEnemyBulletsSimulation(const Game& game, int mePlayerId)
{
	std::map<Bullet, BulletSimulation> simulations;
	const auto tickTime = 1.0 / game.properties.ticksPerSecond;
	for (const auto& bullet: game.bullets)
	{
		if (bullet.playerId == mePlayerId && bullet.explosionParams == nullptr) continue;
		auto simulation = Simulator::getBulletSimulation(bullet.position, bullet.velocity, bullet.size / 2, game);

		if (bullet.playerId == mePlayerId)//�������� ���� �� ������������ � ������
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
				if (!isShooting) continue; //�����������
				
				if (MathHelper::getVectorLength2(bulletCorner, crossPoint) >
					MathHelper::getVectorLength2(simulation.bulletCrossCorner, simulation.targetCrossPoint))
					continue; //���� ������ �������� � �����

				simulation.targetCrossPoint = crossPoint;
				simulation.bulletCrossCorner = bulletCorner;
				simulation.targetCrossTime = MathHelper::getVectorLength(bulletCorner, crossPoint) /
					MathHelper::getVectorLength(bullet.velocity);
			}
		}
		
		const auto shootWallTick = static_cast<int>(ceil(simulation.targetCrossTime * game.properties.ticksPerSecond));
		std::map<int, Vec2Double> bulletPositions;
		bulletPositions[0] = bullet.position;

		for (int i = 1; i <= shootWallTick; ++i)
		{
			Vec2Double position;
			const auto exists = Simulator::getBulletInTimePosition(bullet, tickTime * i, simulation, game, position);
			bulletPositions[exists ? i : -1] = position;
		}
		
		simulation.bulletPositions = bulletPositions;
		simulations[bullet] = simulation;
	}
	return simulations;
}

std::map<Bullet, int> Strategy::getShootMeBullets(
	const Unit& me,
	const std::map<Bullet, BulletSimulation>& enemyBulletsSimulations, int addTicks, const UnitAction& unitAction,
	const Game& game) const
{
	if (addTicks != 0 && addTicks != 1) throw std::runtime_error("addTicks is not 0 or 1");
	
	const auto tickTime = 1.0 / game.properties.ticksPerSecond;
	std::map<Bullet, int> shootMeBullets;
	std::map<int, Vec2Double> mePositions;
	std::map<int, JumpState> meJumpStates;
	auto jumpState = me.jumpState;
	
	mePositions[0] = addTicks == 0 ? me.position : Simulator::getUnitInTimePosition(
		me.position, me.size, unitAction, tickTime, jumpState, game);
	meJumpStates[0] = jumpState;

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
			//���� ��������� � ��� 0-1. �� �� ���� �������
			continue;
		}
		
		for (int tick = 1; tick <= bulletCrossWallTick - addTicks; ++tick)
		{
			Vec2Double bulletInTimePosition;
			const bool exists = Simulator::getBulletInTimePosition(
				bullet, (tick+addTicks)* tickTime, bulletSimulation, game, bulletInTimePosition);
						

			if (exists)
			{
				bulletPositions[tick] = bulletInTimePosition;
				
				Vec2Double unitInTimePosition;
				if (mePositions.count(tick) > 0) unitInTimePosition = mePositions[tick];//��� ��������� ��� ������ ����
				else
				{
					auto unitInTimeJumpState = meJumpStates.at(tick - 1);
					unitInTimePosition =
						Simulator::getUnitInTimePosition(
							mePositions.at(tick - 1), me.size, action, tickTime, unitInTimeJumpState, game);
					mePositions[tick] = unitInTimePosition;
					meJumpStates[tick] = unitInTimeJumpState;
				}
				const bool isShooting = bullet.playerId != me.playerId && isBulletMoveCrossUnitMove(
					mePositions.at(tick - 1), unitInTimePosition, me.size,
					bulletPositions[tick - 1], bulletInTimePosition, bullet.size / 2.0);
				if (isShooting)
				{
					shootMeBullets[bullet] = tick;
					break;
				}
			}
			else
			{
				
				const auto thisTickBulletTime = bulletSimulation.targetCrossTime - (tick - 1) * tickTime;

				auto unitInTimeJumpState = meJumpStates.at(tick - 1);
				const auto unitInTimePosition =
					Simulator::getUnitInTimePosition(
						mePositions.at(tick - 1), me.size, action, thisTickBulletTime, unitInTimeJumpState, game);
				auto isShooting = bullet.playerId != me.playerId && isBulletMoveCrossUnitMove(
					mePositions.at(tick - 1), unitInTimePosition, me.size,
					bulletPositions[tick - 1], bulletInTimePosition, bullet.size / 2.0);
				if (!isShooting)
				{
					const auto bulletCrossWallCenter = Vec2Double(
						bullet.position.x + bullet.velocity.x * bulletSimulation.targetCrossTime,
						bullet.position.y + bullet.velocity.y * bulletSimulation.targetCrossTime);
					if (isBulletExplosionShootUnit(bullet, bulletCrossWallCenter, unitInTimePosition, me.size))
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


//TODO: ������ ������������ ����� ������
std::tuple<RunawayDirection, int, int, int> Strategy::getRunawayAction(
	const Vec2Double& unitPosition, const Vec2Double& unitSize, int unitPlayerId,
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
	for (const auto& item: shootingMeBullets)
	{
		minShootMeDamage += item.first.damage;
	}

	auto bestRunAwayDirection = NoWAY;
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
			bool startedJump = false;
			
			auto canGoLeft = checkLeft;
			auto leftDamage = 0;
			
			auto canGoRight = checkRight;
			auto rightDamage = 0;
			
			auto canGoDown = checkDown;
			auto downDamage = 0;

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
				
				//jump
				//const auto thisTickCanJump = canJump && startGoTick == 0 ||
				//	!Simulator::isUnitOnAir(jumpUnitPosition, unitSize, game) ||
				//	startedJump; //TODO: ������ jumpState
				//
				//if (!thisTickCanJump)
				//{
				//	action.jump = false;
				//}
				//else
				//{
					if (tick > stopGoTick)
					{
						action.jump = false;
					}
					else if (tick > startGoTick)
					{
						action.jump = true;
						startedJump = true;
					}
				//}
				auto thisTickGoUpJumpState = goUpJumpState;
				const auto thisTickJumpUnitPosition = Simulator::getUnitInTimePosition(
					jumpUnitPosition, unitSize, action, tickTime, thisTickGoUpJumpState, game);
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
					fallUnitPosition, unitSize, action, tickTime, thisTickGoDownJumpState, game);
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
					goLeftUnitPosition, unitSize, action, tickTime, thisTickGoLeftJumpState, game);
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
					goRightUnitPosition, unitSize, action, tickTime, thisTickGoRightJumpState, game);
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
						if (shootWallTick < tick + addTicks) continue;//��������� � ����� ������

						
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
									startedJump = true;
								}
							//}

							auto newGoUpJumpState = goUpJumpState;
							const auto newJumpUnitPosition = bulletExists ? thisTickJumpUnitPosition :
								Simulator::getUnitInTimePosition(
									jumpUnitPosition,
									unitSize,
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
								halfBulletSize))
							{
								//canGoUp = false;
								thisTickUpDamage += bullet.damage;
								gotUpBullets[bullet] = true;
							}
							else if (!bulletExists && isBulletExplosionShootUnit(bullet, bulletCrossWallCenter, newJumpUnitPosition, unitSize))
							{
								//���� �������� � �����. ���� ��������� �����
								//canGoUp = false;
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
								halfBulletSize))
							{
								//canGoDown = false;
								thisTickDownDamage += bullet.damage;
								gotDownBullets[bullet] = true;
							}
							else if (!bulletExists && isBulletExplosionShootUnit(bullet, bulletCrossWallCenter, newFallUnitPosition, unitSize))
							{
								//���� �������� � �����. ���� ��������� �����
								//canGoDown = false;
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
								halfBulletSize))
							{
								//canGoLeft = false;
								thisTickLeftDamage += bullet.damage;
								gotLeftBullets[bullet] = true;
							}
							else if (!bulletExists && isBulletExplosionShootUnit(bullet, bulletCrossWallCenter, newGoLeftUnitPosition, unitSize))
							{
								//���� �������� � �����. ���� ��������� �����
								//canGoLeft = false;
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
								halfBulletSize))
							{
								//canGoRight = false;
								thisTickRightDamage += bullet.damage;
								gotRightBullets[bullet] = true;
							}
							else if (!bulletExists && isBulletExplosionShootUnit(bullet, bulletCrossWallCenter, newGoRightUnitPosition, unitSize))
							{
								//���� �������� � �����. ���� ��������� �����
								//canGoRight = false;
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
				downDamage += thisTickDownDamage;
				leftDamage += thisTickLeftDamage;
				rightDamage += thisTickRightDamage;

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

			if (canGoUp && upDamage < minShootMeDamage)
			{
				minShootMeDamage = upDamage;
				bestRunAwayDirection = GoUP;
				bestStartGoTick = startGoTick;
				bestStopGoTick = stopGoTick;
			}
			if (canGoDown && downDamage < minShootMeDamage)
			{
				minShootMeDamage = downDamage;
				bestRunAwayDirection = GoDOWN;
				bestStartGoTick = startGoTick;
				bestStopGoTick = stopGoTick;
			}
			if (canGoLeft && leftDamage < minShootMeDamage)
			{
				minShootMeDamage = leftDamage;
				bestRunAwayDirection = GoLEFT;
				bestStartGoTick = startGoTick;
				bestStopGoTick = stopGoTick;
			}
			if (canGoRight && rightDamage < minShootMeDamage)
			{
				minShootMeDamage = rightDamage;
				bestRunAwayDirection = GoRIGHT;
				bestStartGoTick = startGoTick;
				bestStopGoTick = stopGoTick;
			}

						
			////TODO: �� ������������ ����� ��� ������ ����
			//for (const auto& bullet : game.bullets)
			//{
			//	if (bullet.playerId == unitPlayerId) continue;

			//	bool gotUpBullet = false;
			//	bool gotDownBullet = false;
			//	bool gotLeftBullet = false;
			//	bool gotRightBullet = false;

			//	const auto bulletSimulation = enemyBulletsSimulations.at(bullet);

			//	const auto bulletCrossWallCenter = Vec2Double(
			//		bullet.position.x + bullet.velocity.x * bulletSimulation.targetCrossTime,
			//		bullet.position.y + bullet.velocity.y * bulletSimulation.targetCrossTime
			//	);

			//	const auto halfBulletSize = bullet.size / 2;
			//	auto jumpUnitPosition = unitPosition;
			//	auto fallUnitPosition = unitPosition;
			//	auto goLeftUnitPosition = unitPosition;
			//	auto goRightUnitPosition = unitPosition;
			//	
			//	auto shootWallTick = static_cast<int>(ceil(bulletSimulation.targetCrossTime * game.properties.ticksPerSecond));
			//	shootWallTick -= addTicks;
			//	if (shootWallTick == 0) continue;//��������� � ����� �� ������� ���
			//	
			//	auto bulletPosition = bullet.position;
			//	bool startedJump = false;
			//	
			//	for (int tick = 1; tick <= shootWallTick; ++tick)
			//	{
			//		const auto bulletTime = (tick + addTicks) / game.properties.ticksPerSecond;
			//		Vec2Double newBulletPosition;
			//		auto unitTime = tickTime;

			//		
			//		auto exists = Simulator::getBulletInTimePosition(
			//			bullet, bulletTime, bulletSimulation, game, newBulletPosition);
			//		if (!exists)
			//		{
			//			unitTime = bulletSimulation.targetCrossTime - (bulletTime - tickTime);
			//		}				

			//		//jump
			//		if (canGoUp && !gotUpBullet) {
			//			const auto thisTickCanJump = canJump && startGoTick == 0 ||
			//				!Simulator::isUnitOnAir(jumpUnitPosition, unitSize, game) || 
			//				startedJump;
			//			if (!thisTickCanJump)
			//			{
			//				action.jump = false;
			//			}else
			//			{
			//				if (tick > stopGoTick)
			//				{
			//					action.jump = false;
			//				}
			//				else if (tick > startGoTick)
			//				{
			//					action.jump = true;
			//					startedJump = true;
			//				}
			//			}
			//			
			//			const auto newJumpUnitPosition = Simulator::getUnitInTimePosition(
			//				jumpUnitPosition, unitSize, action, unitTime, game);

			//			action.jump = false;

			//			if (isBulletMoveCrossUnitMove(
			//				jumpUnitPosition,
			//				newJumpUnitPosition,
			//				unitSize,
			//				bulletPosition,
			//				newBulletPosition,
			//				halfBulletSize))
			//			{
			//				//canGoUp = false;
			//				upDamage += bullet.damage;
			//				gotUpBullet = true;
			//			}
			//			else if (!exists && isBulletExplosionShootUnit(bullet, bulletCrossWallCenter, newJumpUnitPosition, unitSize))
			//			{
			//				//���� �������� � �����. ���� ��������� �����
			//				//canGoUp = false;
			//				upDamage += bullet.damage;
			//				gotUpBullet = true;
			//			}
			//			jumpUnitPosition = newJumpUnitPosition;
			//		}

			//		//fall
			//		if (canGoDown && !gotDownBullet) {
			//			if (tick > stopGoTick)
			//			{
			//				action.jumpDown = false;
			//			}
			//			else if (tick > startGoTick)
			//			{
			//				action.jumpDown = true;
			//			}
			//			const auto newFallUnitPosition = Simulator::getUnitInTimePosition(
			//				fallUnitPosition, unitSize, action, unitTime, game);

			//			action.jumpDown = false;

			//			if (isBulletMoveCrossUnitMove(
			//				fallUnitPosition,
			//				newFallUnitPosition,
			//				unitSize,
			//				bulletPosition,
			//				newBulletPosition,
			//				halfBulletSize))
			//			{
			//				//canGoDown = false;
			//				downDamage += bullet.damage;
			//				gotDownBullet = true;
			//			}
			//			else if (!exists && isBulletExplosionShootUnit(bullet, bulletCrossWallCenter, newFallUnitPosition, unitSize))
			//			{
			//				//���� �������� � �����. ���� ��������� �����
			//				//canGoDown = false;
			//				downDamage += bullet.damage;
			//				gotDownBullet = true;
			//			}
			//			fallUnitPosition = newFallUnitPosition;
			//		}

			//		//left
			//		if (canGoLeft && !gotLeftBullet)
			//		{
			//			if (tick > stopGoTick)
			//			{
			//				action.velocity = 0;
			//			}
			//			else if (tick > startGoTick)
			//			{
			//				action.velocity = -INT_MAX;
			//			}
			//			const auto newGoLeftUnitPosition = Simulator::getUnitInTimePosition(
			//				goLeftUnitPosition, unitSize, action, unitTime, game);

			//			action.velocity = 0;

			//			if (isBulletMoveCrossUnitMove(
			//				goLeftUnitPosition,
			//				newGoLeftUnitPosition,
			//				unitSize,
			//				bulletPosition,
			//				newBulletPosition,
			//				halfBulletSize))
			//			{
			//				//canGoLeft = false;
			//				leftDamage += bullet.damage;
			//				gotLeftBullet = true;
			//			}
			//			else if (!exists && isBulletExplosionShootUnit(bullet, bulletCrossWallCenter, newGoLeftUnitPosition, unitSize))
			//			{
			//				//���� �������� � �����. ���� ��������� �����
			//				//canGoLeft = false;
			//				leftDamage += bullet.damage;
			//				gotLeftBullet = true;
			//			}
			//			goLeftUnitPosition = newGoLeftUnitPosition;
			//		}

			//		//right
			//		if (canGoRight && !gotRightBullet)
			//		{
			//			if (tick > stopGoTick)
			//			{
			//				action.velocity = 0;
			//			}
			//			else if (tick > startGoTick)
			//			{
			//				action.velocity = INT_MAX;
			//			}
			//			const auto newGoRightUnitPosition = Simulator::getUnitInTimePosition(
			//				goRightUnitPosition, unitSize, action, unitTime, game);

			//			action.velocity = 0;

			//			if (isBulletMoveCrossUnitMove(
			//				goRightUnitPosition,
			//				newGoRightUnitPosition,
			//				unitSize,
			//				bulletPosition,
			//				newBulletPosition,
			//				halfBulletSize))
			//			{
			//				//canGoRight = false;
			//				rightDamage += bullet.damage;
			//				gotRightBullet = true;
			//			}
			//			else if (!exists && isBulletExplosionShootUnit(bullet, bulletCrossWallCenter, newGoRightUnitPosition, unitSize))
			//			{
			//				//���� �������� � �����. ���� ��������� �����
			//				//canGoRight = false;
			//				rightDamage += bullet.damage;
			//				gotRightBullet = true;
			//			}
			//			goRightUnitPosition = newGoRightUnitPosition;
			//		}
			//		
			//		
			//		bulletPosition = newBulletPosition;						
			//		/*if (!canGoUp && !canGoDown && !canGoLeft && !canGoRight) break;*/
			//		if ((!canGoUp || upDamage >= minShootMeDamage) && 
			//			(!canGoDown || downDamage >= minShootMeDamage) &&
			//			(!canGoLeft && leftDamage >= minShootMeDamage) && 
			//			(!canGoRight && rightDamage >= minShootMeDamage))
			//			break;
			//	}

			//	/*if (!canGoUp && !canGoDown && !canGoLeft && !canGoRight) break;*/
			//	if ((!canGoUp || upDamage >= minShootMeDamage) &&
			//		(!canGoDown || downDamage >= minShootMeDamage) &&
			//		(!canGoLeft && leftDamage >= minShootMeDamage) &&
			//		(!canGoRight && rightDamage >= minShootMeDamage))
			//		break;
			//}

			//if (canGoUp && upDamage == 0) 
			//	return std::make_tuple(GoUP, startGoTick, stopGoTick, 0);
			//if (canGoDown && downDamage == 0) 
			//	return std::make_tuple(GoDOWN, startGoTick, stopGoTick, 0);
			//if (canGoLeft && leftDamage == 0) 
			//	return std::make_tuple(GoLEFT, startGoTick, stopGoTick, 0);
			//if (canGoRight && rightDamage == 0) 
			//	return std::make_tuple(GoRIGHT, startGoTick, stopGoTick, 0);

			//if (canGoUp && upDamage < minShootMeDamage)
			//{
			//	minShootMeDamage = upDamage;
			//	bestRunAwayDirection = GoUP;
			//	bestStartGoTick = startGoTick;
			//	bestStopGoTick = stopGoTick;
			//}
			//if (canGoDown && downDamage < minShootMeDamage)
			//{
			//	minShootMeDamage = downDamage;
			//	bestRunAwayDirection = GoDOWN;
			//	bestStartGoTick = startGoTick;
			//	bestStopGoTick = stopGoTick;
			//}
			//if (canGoLeft && leftDamage < minShootMeDamage)
			//{
			//	minShootMeDamage = leftDamage;
			//	bestRunAwayDirection = GoLEFT;
			//	bestStartGoTick = startGoTick;
			//	bestStopGoTick = stopGoTick;
			//}
			//if (canGoRight && rightDamage < minShootMeDamage)
			//{
			//	minShootMeDamage = rightDamage;
			//	bestRunAwayDirection = GoRIGHT;
			//	bestStartGoTick = startGoTick;
			//	bestStopGoTick = stopGoTick;
			//}





			

			/*if (curDamage == 0) {
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
			else if (curDamage < minShootMeDamage)
			{
				minShootMeDamage = curDamage;
				bestStartGoTick = startGoTick;
				bestStopGoTick = stopGoTick;
			}*/
		}
	}

	return std::make_tuple(bestRunAwayDirection, bestStartGoTick, bestStopGoTick, minShootMeDamage); 
}



bool Strategy::isSafeMove(
	const Unit& unit, const UnitAction& action, const std::map<Bullet, BulletSimulation>& enemyBulletsSimulations, const Game& game)
{
	const auto tickTime = 1.0 / game.properties.ticksPerSecond;
	for (const auto& item : enemyBulletsSimulations)
	{
		const auto& bullet = item.first;
		const auto& bulletSimulation = item.second;
		
		double unitTime = tickTime;
		
		Vec2Double bulletInTimePosition;
		const auto exists = Simulator::getBulletInTimePosition(
			bullet, tickTime, bulletSimulation, game, bulletInTimePosition);
		if (!exists)
		{
			unitTime = bulletSimulation.targetCrossTime;
		}
		auto jumpState = unit.jumpState;
		const auto unitInTimePosition = Simulator::getUnitInTimePosition(unit.position, unit.size, action, unitTime, jumpState, game);			
		
		const auto cross = bullet.playerId != unit.playerId && isBulletMoveCrossUnitMove(
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

size_t Strategy::getStartedJumpY() const
{
	return startedJumpY_;
}

void Strategy::setStartedJumpY(size_t newStartedJumpY)
{
	startedJumpY_ = newStartedJumpY;
}


