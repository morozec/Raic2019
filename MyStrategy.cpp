#include "MyStrategy.hpp"
#include <utility>
#include <climits>
#include <map>
#include <tuple>
#include <sstream>
#include <cmath>
#include "common/Helper.h"
#include "mathcalc/MathHelper.h"
#include "debug/DebugHelper.h"
#include "simulation/Simulator.h"
#include <iostream>
#include <algorithm>

using namespace std;


MyStrategy::MyStrategy()
{	
}


inline bool operator<(const Bullet& lhs, const Bullet& rhs)
{
	return lhs.position.x < rhs.position.x;
}

void setJumpAndJumpDown(const Unit& unit, const Vec2Double& targetPosition, const Game& game,
	bool considerYs,
	UnitAction& action, Strategy& strategy)
{
	const auto isLeftWall = targetPosition.x < unit.position.x - 1 &&
		game.level.tiles[size_t(unit.position.x - 1)][size_t(unit.position.y)] == WALL;
	const auto isRightWall = targetPosition.x > unit.position.x + 1 &&
		game.level.tiles[size_t(unit.position.x + 1)][size_t(unit.position.y)] == WALL;
	const auto isSameColumnHigher = targetPosition.y > unit.position.y && targetPosition.x >= unit.position.x - 1 && targetPosition.x <= unit.position.x + 1;
	const auto needJump = considerYs && targetPosition.y > unit.position.y || isLeftWall || isRightWall || isSameColumnHigher;

	const auto bottomTile = game.level.tiles[size_t(unit.position.x)][size_t(unit.position.y - 1)];
	if (strategy.getStartedJumpY() != 0 &&
		bottomTile != EMPTY &&
		size_t(unit.position.y) != strategy.getStartedJumpY())
	{
		action.jump = false;
		action.jumpDown = false;
		strategy.setStartedJumpY(0);
	}
	else if (needJump)
	{
		if (isLeftWall && unit.jumpState.canJump && unit.jumpState.canCancel)
		{
			auto wallHeight = 0;
			while (game.level.tiles[size_t(unit.position.x - 1)][size_t(unit.position.y + wallHeight)] == WALL)
			{
				wallHeight++;
			}
			const auto maxJumpHeight = unit.jumpState.speed * unit.jumpState.maxTime;
			if (maxJumpHeight < wallHeight)
			{
				strategy.setStartedJumpY(static_cast<int>(trunc(unit.position.y)));
			}
		}

		else if (isRightWall && unit.jumpState.canJump && unit.jumpState.canCancel)
		{
			auto wallHeight = 0;
			while (game.level.tiles[size_t(unit.position.x + 1)][size_t(unit.position.y + wallHeight)] == WALL)
			{
				wallHeight++;
			}
			const auto maxJumpHeight = unit.jumpState.speed * unit.jumpState.maxTime;
			if (maxJumpHeight < wallHeight)
			{
				strategy.setStartedJumpY(size_t(unit.position.y));
			}
		}
		else if (isSameColumnHigher)
		{
			const auto jumpHeight = targetPosition.y - unit.position.y;			
			const auto maxJumpHeight = unit.jumpState.speed * unit.jumpState.maxTime;
			if (maxJumpHeight < jumpHeight)
			{
				strategy.setStartedJumpY(size_t(unit.position.y));
			}
		}
		
		action.jump = true;
		action.jumpDown = false;
	}
	else
	{
		action.jump = false;
		action.jumpDown = considerYs && targetPosition.y < unit.position.y;
	}	
}

void setMoveToWeaponAction(const Unit& unit, const Vec2Double& weaponPosition, const Game& game,
	UnitAction& action, Strategy& strategy)
{
	setJumpAndJumpDown(unit, weaponPosition, game, true, action, strategy);
	
	if (abs(weaponPosition.x - unit.position.x) < TOLERANCE)
		action.velocity = 0;
	else
		action.velocity = weaponPosition.x > unit.position.x ? INT_MAX : -INT_MAX;

	action.shoot = false;
	action.reload = false;
	action.swapWeapon = false;
	action.plantMine = false;
}

void setMoveToEnemyAction(
	const Unit& unit, const Vec2Double& enemyPosition, bool needGo, const Game& game, 
	UnitAction& action, Strategy& strategy)
{	
	if (needGo)
	{
		action.velocity = enemyPosition.x > unit.position.x ? INT_MAX : -INT_MAX;
		setJumpAndJumpDown(unit, enemyPosition, game, false, action, strategy);		
	}
	else
	{
		action.velocity = 0;
		action.jump = false;
		action.jumpDown = false;
	}
}

void setShootingAction(const Unit& me, const Unit& enemy, const Game& game, UnitAction& action)
{
	const auto tickTime = 1.0 / game.properties.ticksPerSecond;
	const auto movingTime = me.weapon->fireTimer != nullptr ? *(me.weapon->fireTimer) : 0;
	
	const auto isEnemyOnAir = Simulator::isUnitOnAir(enemy.position, enemy.size, game);
	UnitAction enemyAction;
	if (!enemy.jumpState.canJump && !enemy.jumpState.canCancel || //������
		enemy.jumpState.canJump && !enemy.jumpState.canCancel || //������� �� ������
		!isEnemyOnAir) // ����� �� ����� 
	{
		enemyAction.jump = false;
		enemyAction.jumpDown = false;
		enemyAction.velocity = 0;
	}
	else if (enemy.jumpState.canJump && enemy.jumpState.canCancel && isEnemyOnAir) //�������
	{
		enemyAction.jump = true;
		enemyAction.jumpDown = false;
		enemyAction.velocity = 0;
	}
	else throw runtime_error("unknown enemy position");
	

	const auto fullMovingTicks = static_cast<int>(movingTime / tickTime);
	auto meShootingPosition = me.position;//�������, ������ ���������� �������
	auto meJumpState = me.jumpState;

	auto enemyShootingPosition = enemy.position;
	auto enemyJumpState = enemy.jumpState;
	
	for (int i = 0; i < fullMovingTicks; ++i)
	{
		meShootingPosition = Simulator::getUnitInTimePosition(
			meShootingPosition, me.size, action, tickTime, meJumpState, game);
		enemyShootingPosition = Simulator::getUnitInTimePosition(
			enemyShootingPosition, enemy.size, enemyAction, tickTime, enemyJumpState, game);
	}
	const auto timeLeft = movingTime - fullMovingTicks / game.properties.ticksPerSecond;
	meShootingPosition = Simulator::getUnitInTimePosition(
		meShootingPosition, me.size, action, timeLeft, meJumpState, game);
	enemyShootingPosition = Simulator::getUnitInTimePosition(
		enemyShootingPosition, enemy.size, enemyAction, timeLeft, enemyJumpState, game);


	/*if (me.weapon->fireTimer != nullptr && *(me.weapon->fireTimer) > tickTime)
	{
		action.aim = enemyShootingPosition - meShootingPosition;
		action.shoot = false;		
		return;
	}*/


	const auto shootingTick = static_cast<int>(ceil(movingTime / tickTime));
	const auto thisTickShootingTime = shootingTick / game.properties.ticksPerSecond - movingTime;

	Vec2Double targetPosition;
	if (!Simulator::isUnitOnAir(enemy.position, enemy.size, game))
	{
		targetPosition = enemy.position;
		action.shoot = false;
	}
	else
	{
		if (!enemyJumpState.canJump && !enemyJumpState.canCancel || //������
			enemyJumpState.canJump && !enemyJumpState.canCancel)   //������� �� ������
		{
			auto isShoot = false;	
			
			const auto bulletPos0 = Vec2Double(meShootingPosition.x, meShootingPosition.y + me.size.y / 2);
			const auto enemyPos0 = enemyShootingPosition;
			const auto enemyPos1 = Simulator::getUnitInTimePosition(enemyPos0, enemy.size, enemyAction, thisTickShootingTime, enemyJumpState, game);
						
			const auto maxBulletVelocity = me.weapon->params.bullet.speed;
			const auto halfBulletSize = me.weapon->params.bullet.size / 2;

			int fallingTicks = 0;
			map<int, Vec2Double> enemyPositions;
			enemyPositions[0] = enemyPos1;
			
			while (!enemyJumpState.canJump) // ����� ������ ��������� !onAir ��-�� ������� �� ������
			{
				const auto nextPos = Simulator::getUnitInTimePosition(
					enemyPositions[fallingTicks], enemy.size, enemyAction, tickTime, enemyJumpState, game);
				enemyPositions[++fallingTicks] = nextPos;
			}

			for (int i = 0; i <= fallingTicks; ++i)
			{
				const auto targetEnemyPos = enemyPositions[i];
				const auto shootingVector = targetEnemyPos - bulletPos0;

				Vec2Double bulletVelocity;
				if (abs(shootingVector.x) > TOLERANCE) {
					const auto angle = atan2(shootingVector.y, shootingVector.x);
					bulletVelocity = { maxBulletVelocity * cos(angle), maxBulletVelocity*sin(angle) };
				}
				else
				{
					bulletVelocity = { 0, shootingVector.y > 0 ? maxBulletVelocity : -maxBulletVelocity };
				}

				//��������� ����������� �� ���� 0-1
				const auto bulletPos1 = bulletPos0 + bulletVelocity * thisTickShootingTime;
				isShoot = Strategy::isBulletMoveCrossUnitMove(
					enemyPos0, enemyPos1, enemy.size, bulletPos0, bulletPos1, halfBulletSize);

				if (isShoot)
				{
					targetPosition = targetEnemyPos;
					break;
				}

				const auto bulletSimulation = Simulator::getBulletSimulation(bulletPos1, bulletVelocity, halfBulletSize, game);
				const auto bulletPositions = Simulator::getBulletPositions(
					bulletPos1, bulletVelocity, bulletSimulation.targetCrossTime, game);				
				
				const auto shootWallTick = static_cast<int>(ceil(bulletSimulation.targetCrossTime * game.properties.ticksPerSecond));

				//TODO: ��������� ���������� �� ������ �� ����������� � ������

				for (int j = 0; j < std::min(fallingTicks, shootWallTick); ++j)
				{
					const auto bp0 = bulletPositions.at(j);
					const auto bp1 = bulletPositions.at(j < shootWallTick - 1 ? j + 1 : -1);

					const auto ep0 = enemyPositions[j];
					const auto ep1 = enemyPositions[j + 1];
					
					isShoot = Strategy::isBulletMoveCrossUnitMove(
						ep0, ep1, enemy.size, bp0, bp1, halfBulletSize);
					if (isShoot) break;
				}

				if (isShoot)
				{
					targetPosition = targetEnemyPos;
					break;
				}
			}

			if (isShoot)
			{
				action.shoot = true;
			}
			else
			{				
				targetPosition = enemy.position;
				action.shoot = false;
			}
		}
		else //�������
		{
			//TODO: ��������� ������� ������ � � ������
			targetPosition = enemy.position;
			action.shoot = false;
		}
	}

	//TODO: ���������� �������� ��� ���� �����
	//TODO: �� �������� ���������� � ������ ������ ����
	
	action.aim = targetPosition - meShootingPosition;
	//action.shoot = true;
	//action.shoot = Strategy::getShootEnemyProbability(me, enemy, game, me.weapon->spread) >= SHOOTING_PROBABILITY;
}

UnitAction MyStrategy::getAction(const Unit& unit, const Game& game,
                                 Debug& debug)
{
	/*
	Vec2Double crossPointCur;
	double minDist2Cur = INT_MAX;
	const auto hasWallCross = Simulator::getBulletPointRectangleFirstCrossPoint(
		{ 34.816666665661, 20 },
		{ 14.7163169141329,  -47.7852489423546 },
		38, 7, 39, 8, crossPointCur, minDist2Cur);*/

	/*const auto bbCross = Simulator::getBulletBorderCross(
		{ 7.2000000000002604, 17.098333334333333 }, 
		{ 20.000000000000000, 0.0000000000000000000000000
		},
		game);*/

	
	const Unit* nearestEnemy = nullptr;
	for (const Unit& other : game.units)
	{
		if (other.playerId != unit.playerId)
		{
			if (nearestEnemy == nullptr ||
				MathHelper::getVectorLength2(unit.position, other.position) <
				MathHelper::getVectorLength2(unit.position, nearestEnemy->position))
			{
				nearestEnemy = &other;
			}
		}
	}
	const LootBox* nearestWeapon = nullptr;
	for (const LootBox& lootBox : game.lootBoxes)
	{
		if (std::dynamic_pointer_cast<Item::Weapon>(lootBox.item))
		{
			if (nearestWeapon == nullptr ||
				MathHelper::getVectorLength2(unit.position, lootBox.position) <
				MathHelper::getVectorLength2(unit.position, nearestWeapon->position))
			{
				nearestWeapon = &lootBox;
			}
		}
	}
	
	UnitAction action;
	action.aim = Vec2Double(0, 0);	
	action.reload = false;
	action.swapWeapon = false;
	action.plantMine = false;
	action.shoot = false;

	if (unit.weapon == nullptr)
	{
		if (nearestWeapon != nullptr) {
			setMoveToWeaponAction(unit, nearestWeapon->position, game, action, strategy_);
		}
		return action;
	}

	if (nearestEnemy == nullptr) return action;
	
	const auto tickTime = 1.0 / game.properties.ticksPerSecond;
	auto needGo = false;
	//auto needShoot = false;

	if (nearestEnemy != nullptr)
	{		
		needGo = Strategy::getShootEnemyProbability(unit, *nearestEnemy, game, unit.weapon->params.minSpread) <
			WALKING_PROBABILITY;
		/*needShoot = strategy_.getShootEnemyProbability(unit, *nearestEnemy, game, unit.weapon->spread, &debug) >=
			SHOOTING_PROBABILITY;	*/	
	}
	//action.shoot = needShoot;

	const auto enemyBulletsSimulation = strategy_.getEnemyBulletsSimulation(game, unit.playerId);	

	drawBullets(debug, game, enemyBulletsSimulation, unit.playerId);
	drawShootingSector(debug, unit, game);
	
	if (strategy_.getStopRunawayTick() == 0)
	{
		strategy_.setRunaway(GoNONE, -1);
	}
	else if (strategy_.getStopRunawayTick() > 0)
	{
		const auto runawayDirection = strategy_.getRunawayDirection();
		if (runawayDirection == GoUP)
		{
			action.jump = true;
			action.jumpDown = false;
			action.velocity = 0;
		}
		else if (runawayDirection == GoDOWN)
		{
			action.jump = false;
			action.jumpDown = true;
			action.velocity = 0;
		}
		else if (runawayDirection == GoLEFT)
		{
			action.jump = false;
			action.jumpDown = false;
			action.velocity = -INT_MAX;
		}
		else if (runawayDirection == GoRIGHT)
		{
			action.jump = false;
			action.jumpDown = false;
			action.velocity = INT_MAX;
		}else
		{
			throw runtime_error("unknown runawayDirection");
		}
		strategy_.decreaseStopRunawayTick();

		setShootingAction(unit, *nearestEnemy, game, action);
		return action;
	}

	setMoveToEnemyAction(unit, nearestEnemy->position, needGo, game, action, strategy_);

	tuple<RunawayDirection, int, int, int> runawayAction;
	auto isSafeMove = strategy_.isSafeMove(unit, action, enemyBulletsSimulation, game);
	

	if (isSafeMove)
	{
		auto jumpState = unit.jumpState;
		const auto actionUnitPosition = Simulator::getUnitInTimePosition(
			unit.position, unit.size, action, tickTime, jumpState, game);
		const auto actionShootMeBullets = strategy_.getShootMeBullets(unit, enemyBulletsSimulation, 1, action, game);
		
		runawayAction = strategy_.getRunawayAction(
			actionUnitPosition, unit.size, unit.playerId, jumpState,
			actionShootMeBullets, enemyBulletsSimulation, 1,
			true, true, true, true,
			game);	

		const auto minDamage = std::get<3>(runawayAction);
		
		if (minDamage == 0)
		{
			debug.draw(CustomData::Log(
				to_string(std::get<0>(runawayAction)) + " " +
				to_string(std::get<1>(runawayAction) + 1) + " " +
				to_string(std::get<2>(runawayAction) + 1) + " " +
				to_string(std::get<3>(runawayAction)) + "\n"));

			setShootingAction(unit, *nearestEnemy, game, action);
			return action;
			//
			//isSafeMove = true;
			//if (runawayDirection != GoNONE)//����������� ����� �� 1, �.�. ������ runawayAction ��� �� ��� ������
			//{
			//	std::get<1>(runawayAction) += 1;
			//	std::get<2>(runawayAction) += 1;
			//}
			
		}				
	}

	//���� �� ����� ������. ������� ������
	
	bool checkUp = true;
	bool checkDown = true;
	bool checkLeft = true;
	bool checkRight = true;

	if (action.jump) checkUp = false;
	else if (action.jumpDown) checkDown = false;
	else if (action.velocity < -TOLERANCE) checkLeft = false;
	else if (action.velocity > TOLERANCE) checkRight = false;

	// �������� ��� ������ ��������, �� ����� ������
	action.jump = false;
	action.jumpDown = false;
	action.velocity = 0;

	const auto shootMeBullets = strategy_.getShootMeBullets(unit, enemyBulletsSimulation, 0, action, game);
	runawayAction = strategy_.getRunawayAction(
		unit.position, unit.size, unit.playerId, unit.jumpState,
		shootMeBullets, enemyBulletsSimulation, 0,
		checkUp, checkDown, checkLeft, checkRight,
		game);
	
	
	debug.draw(CustomData::Log(
		to_string(std::get<0>(runawayAction)) + " " +
		to_string(std::get<1>(runawayAction)) + " " +
		to_string(std::get<2>(runawayAction)) + " " +
		to_string(std::get<3>(runawayAction)) + "\n"));

	const auto runawayDirection = std::get<0>(runawayAction);
	const auto startRunawayTick = std::get<1>(runawayAction);
	const auto stopRunawayTick = std::get<2>(runawayAction);

	if (startRunawayTick == 0)
	{		
		strategy_.setRunaway(runawayDirection, stopRunawayTick - 1);

		if (runawayDirection == GoUP)
		{
			action.jump = true;
			action.jumpDown = false;
			action.velocity = 0;
		}
		else if (runawayDirection == GoDOWN)
		{
			action.jump = false;
			action.jumpDown = true;
			action.velocity = 0;
		}
		else if (runawayDirection == GoLEFT)
		{
			action.jump = false;
			action.jumpDown = false;
			action.velocity = -INT_MAX;
		}
		else if (runawayDirection == GoRIGHT)
		{
			action.jump = false;
			action.jumpDown = false;
			action.velocity = INT_MAX;
		}		
		else 
		{
			throw runtime_error("unknown runawayDirection 2");
		}
	}	
	
	cout << game.currentTick << ": " << action.jump << " " << action.jumpDown << " " << action.velocity << "\n";

	setShootingAction(unit, *nearestEnemy, game, action);
	return action;
}

