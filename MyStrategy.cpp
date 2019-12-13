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
#include <corecrt_math_defines.h>

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
	if (!enemy.jumpState.canJump && !enemy.jumpState.canCancel || //падает
		enemy.jumpState.canJump && !enemy.jumpState.canCancel || //прыгает на батуте
		!isEnemyOnAir) // стоит на земле 
	{
		enemyAction.jump = false;
		enemyAction.jumpDown = false;
		enemyAction.velocity = 0;
	}
	else if (enemy.jumpState.canJump && enemy.jumpState.canCancel && isEnemyOnAir) //прыгает
	{
		enemyAction.jump = true;
		enemyAction.jumpDown = false;
		enemyAction.velocity = 0;
	}
	else throw runtime_error("unknown enemy position");
	

	const auto fullMovingTicks = static_cast<int>(movingTime / tickTime);
	auto meShootingPosition = me.position;//позиция, откуда произойдет выстрел
	auto meJumpState = me.jumpState;

	auto enemyShootingPosition = enemy.position;//позиция, где будет враг в момент моего выстрела
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

	const auto canSimulateMore = abs(action.velocity) > TOLERANCE || Simulator::isUnitOnAir(meShootingPosition, me.size, game);
		
	
	const auto isFalling = !enemyJumpState.canJump && !enemyJumpState.canCancel;  //падает
	const auto isJumping = enemyJumpState.canJump && enemyJumpState.canCancel;  //прыгает
	const auto isJumpPadJumping = enemyJumpState.canJump && !enemyJumpState.canCancel;  //прыгает на батуте
			
	const auto enemyPos1 = Simulator::getUnitInTimePosition(enemyShootingPosition, enemy.size, enemyAction, thisTickShootingTime, enemyJumpState, game);

	int fallingTicks = 0;
	map<int, Vec2Double> enemyPositions;
	enemyPositions[0] = enemyPos1;			

	bool isFallingWhileJumpPadJumping = false;
	bool isJumpPadJumpingWhileFalling = false;
	while (Simulator::isUnitOnAir(enemyPositions[fallingTicks], enemy.size, game))
	{
		const auto nextPos = Simulator::getUnitInTimePosition(
			enemyPositions[fallingTicks], enemy.size, enemyAction, tickTime, enemyJumpState, game);
		enemyPositions[++fallingTicks] = nextPos;

		if (isFallingWhileJumpPadJumping && enemyJumpState.canJump && !enemyJumpState.canCancel) break;//новый цикл прыжка на батуте
		if (isJumpPadJumpingWhileFalling && !enemyJumpState.canJump && !enemyJumpState.canCancel) break;//новый цикл падения при прыжке на батуте

		if (isJumpPadJumping && !enemyJumpState.canJump && !enemyJumpState.canCancel) 
			isFallingWhileJumpPadJumping = true;//перешли из прыжка на батуте в падение
		if (isFalling && enemyJumpState.canJump && !enemyJumpState.canCancel)
			isJumpPadJumpingWhileFalling = true;//перешли из падения в прыжок на батуте
	}


	auto maxShootingProbability = 0.0;
	double okShootingAngle = 0;
	int addShootingSimulations = 0;
	
	const int MAX_ADD_SHOOTING_SIMULATIONS = 100;
	const double OK_SHOOTING_PROBABILITY = 0.85;

	while (addShootingSimulations < MAX_ADD_SHOOTING_SIMULATIONS && maxShootingProbability < OK_SHOOTING_PROBABILITY)
	{
		double minAngle = INT_MAX;
		double maxAngle = -INT_MAX;
		for (const auto& ep : enemyPositions)
		{
			const auto shootingVector = ep.second - meShootingPosition;
			double shootingAngle;
			if (abs(shootingVector.x) > TOLERANCE) {
				shootingAngle = atan2(shootingVector.y, shootingVector.x);
			}
			else
			{
				shootingAngle = shootingVector.y > 0 ? M_PI : -M_PI;
			}
			if (shootingAngle < minAngle) minAngle = shootingAngle;
			if (shootingAngle > maxAngle) maxAngle = shootingAngle;
		}

		const int directionsCount = min(1, static_cast<int>((maxAngle - minAngle) / M_PI * 20));
		const double deltaAngle = (maxAngle - minAngle) / directionsCount;

		for (int i = 0; i < directionsCount; ++i)
		{
			const auto shootingAngle = minAngle + i * deltaAngle;
			double spread;
			if (me.weapon->lastAngle == nullptr)
			{
				spread = me.weapon->spread;
			}
			else
			{
				spread = me.weapon->spread + abs(*(me.weapon->lastAngle) - shootingAngle);
				spread = min(spread, me.weapon->params.maxSpread);
				spread = max(me.weapon->params.minSpread, spread - me.weapon->params.aimSpeed*(movingTime + tickTime * addShootingSimulations));
			}

			const auto probability = Strategy::getShootEnemyProbability(
				meShootingPosition,
				me.size,
				shootingAngle,
				spread,
				me.weapon->params.bullet,
				thisTickShootingTime,
				enemyShootingPosition,
				enemyPositions,
				enemy.size,
				enemyAction,
				enemyJumpState,
				addShootingSimulations,
				game);
			if (probability > maxShootingProbability)
			{
				maxShootingProbability = probability;
				okShootingAngle = shootingAngle;
			}
		}

		if (maxShootingProbability > OK_SHOOTING_PROBABILITY) break;
		if (!canSimulateMore) break;

		meShootingPosition = Simulator::getUnitInTimePosition(
			meShootingPosition, me.size, action, tickTime, meJumpState, game);
		addShootingSimulations++;
	}					

	Vec2Double targetPosition;
	if (maxShootingProbability >= OK_SHOOTING_PROBABILITY)
	{
		targetPosition = meShootingPosition + Vec2Double(cos(okShootingAngle), sin(okShootingAngle));				
		action.shoot = abs(movingTime) <TOLERANCE && addShootingSimulations == 0;
	}
	else
	{				
		targetPosition = enemyShootingPosition;
		action.shoot = false;
	}		
		

	//TODO: ракетницей стрелять под ноги врагу
	//TODO: не стрелять ракетницей с риском задеть себя
	
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
		needGo = game.currentTick >= 100 ? false : Strategy::getShootEnemyProbability(unit, *nearestEnemy, game, unit.weapon->params.minSpread) <
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
			//if (runawayDirection != GoNONE)//увеличиваем время на 1, т.к. расчет runawayAction был на тик вперед
			//{
			//	std::get<1>(runawayAction) += 1;
			//	std::get<2>(runawayAction) += 1;
			//}
			
		}				
	}

	//идти на врага нельзя. пробуем стоять
	
	bool checkUp = true;
	bool checkDown = true;
	bool checkLeft = true;
	bool checkRight = true;

	if (action.jump) checkUp = false;
	else if (action.jumpDown) checkDown = false;
	else if (action.velocity < -TOLERANCE) checkLeft = false;
	else if (action.velocity > TOLERANCE) checkRight = false;

	// выжидаем тик начала движения, не делая ничего
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

