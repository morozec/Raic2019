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

vector<Vec2Double> getActionPositions(const Unit& unit, const UnitAction& action, int startTick, int stopTick, const Game& game)
{
	if (startTick < 0 || stopTick < 0) throw runtime_error("getActionPositions tick is negative");
	
	const auto tickTime = 1 / game.properties.ticksPerSecond;
	vector<Vec2Double> positions;

	positions.emplace_back(unit.position);
	auto jumpState = unit.jumpState;

	UnitAction waitAction;
	waitAction.jump = false;
	waitAction.jumpDown = false;
	waitAction.velocity = 0;
	for (int i = 0; i < startTick; ++i)
	{
		const auto nextPos = Simulator::getUnitInTimePosition(
			positions.back(), unit.size, waitAction, tickTime, jumpState, game);
		positions.emplace_back(nextPos);
	}

	for (int i = 0; i < stopTick - startTick; ++i)
	{
		const auto nextPos = Simulator::getUnitInTimePosition(
			positions.back(), unit.size, action, tickTime, jumpState, game);
		positions.emplace_back(nextPos);
	}
	
	return positions;
}


void setJumpAndJumpDown(const Vec2Double& unitPosition, const JumpState& unitJumpState, 
	const Vec2Double& targetPosition, const Game& game,
	bool considerYs,
	UnitAction& action, size_t& startedJumpY)
{

	//TODO: выставить startedJumpY в стратегии
	const auto isLeftWall = targetPosition.x < unitPosition.x - 1 &&
		game.level.tiles[size_t(unitPosition.x - 1)][size_t(unitPosition.y)] == WALL;
	const auto isRightWall = targetPosition.x > unitPosition.x + 1 &&
		game.level.tiles[size_t(unitPosition.x + 1)][size_t(unitPosition.y)] == WALL;
	const auto isSameColumnHigher = 
		targetPosition.y > unitPosition.y && targetPosition.x >= unitPosition.x - 1 && targetPosition.x <= unitPosition.x + 1;
	const auto needJump = considerYs && targetPosition.y > unitPosition.y || isLeftWall || isRightWall || isSameColumnHigher;

	const auto bottomTile = game.level.tiles[size_t(unitPosition.x)][size_t(unitPosition.y - 1)];
	if (startedJumpY != 0 &&
		bottomTile != EMPTY &&
		size_t(unitPosition.y) != startedJumpY)
	{
		action.jump = false;
		action.jumpDown = false;
		startedJumpY = 0;
	}
	else if (needJump)
	{
		if (isLeftWall && unitJumpState.canJump && unitJumpState.canCancel)
		{
			auto wallHeight = 0;
			while (game.level.tiles[size_t(unitPosition.x - 1)][size_t(unitPosition.y + wallHeight)] == WALL)
			{
				wallHeight++;
			}
			const auto maxJumpHeight = unitJumpState.speed * unitJumpState.maxTime;
			if (maxJumpHeight < wallHeight)
			{
				startedJumpY = size_t(unitPosition.y);
			}
		}

		else if (isRightWall && unitJumpState.canJump && unitJumpState.canCancel)
		{
			auto wallHeight = 0;
			while (game.level.tiles[size_t(unitPosition.x + 1)][size_t(unitPosition.y + wallHeight)] == WALL)
			{
				wallHeight++;
			}
			const auto maxJumpHeight = unitJumpState.speed * unitJumpState.maxTime;
			if (maxJumpHeight < wallHeight)
			{
				startedJumpY = size_t(unitPosition.y);
			}
		}
		else if (isSameColumnHigher)
		{
			const auto jumpHeight = targetPosition.y - unitPosition.y;			
			const auto maxJumpHeight = unitJumpState.speed * unitJumpState.maxTime;
			if (maxJumpHeight < jumpHeight)
			{
				startedJumpY = size_t(unitPosition.y);
			}
		}
		
		action.jump = true;
		action.jumpDown = false;
	}
	else
	{
		action.jump = false;
		action.jumpDown = considerYs && targetPosition.y < unitPosition.y;
	}	
}

void setMoveToWeaponAction(const Unit& unit, const Vec2Double& weaponPosition, const Game& game,
	UnitAction& action, Strategy& strategy)
{
	auto startedJumpY = strategy.getStartedJumpY();
	setJumpAndJumpDown(
		unit.position, unit.jumpState, weaponPosition, game, true, action, startedJumpY);
	strategy.setStartedJumpY(startedJumpY);
	
	if (abs(weaponPosition.x - unit.position.x) < TOLERANCE)
		action.velocity = 0;
	else
		action.velocity = weaponPosition.x > unit.position.x ? INT_MAX : -INT_MAX;

	action.shoot = false;
	action.reload = false;
	action.swapWeapon = false;
	action.plantMine = false;
}

//void setMoveToEnemyAction(
//	const Unit& unit, const Vec2Double& enemyPosition, bool needGo, const Game& game, 
//	UnitAction& action, Strategy& strategy)
//{	
//	if (needGo)
//	{
//		action.velocity = enemyPosition.x > unit.position.x ? INT_MAX : -INT_MAX;
//		setJumpAndJumpDown(unit, enemyPosition, game, false, action, strategy);		
//	}
//	else
//	{
//		action.velocity = 0;
//		action.jump = false;
//		action.jumpDown = false;
//	}
//}

vector<Vec2Double> getSimplePositions(
	const Vec2Double& unitPosition, const Vec2Double& unitSize, JumpState& unitJumpState, const Game& game)
{
	const auto tickTime = 1 / game.properties.ticksPerSecond;
	
	vector<Vec2Double> positions;
	positions.emplace_back(unitPosition);

	const auto isOnAir = Simulator::isUnitOnAir(unitPosition, unitSize, game);
	const auto isFalling = !unitJumpState.canJump && !unitJumpState.canCancel;
	const auto isJumping = isOnAir && unitJumpState.canJump && unitJumpState.canCancel;
	const auto isJumpPadJumping = unitJumpState.canJump && !unitJumpState.canCancel;
	
	UnitAction action;
	if (isFalling || //падает
		isJumpPadJumping || //прыгает на батуте
		!isOnAir) // стоит на земле 
	{
		action.jump = false;
		action.jumpDown = false;
		action.velocity = 0;
	}
	else if (isJumping) //прыгает
	{
		action.jump = true;
		action.jumpDown = false;
		action.velocity = 0;
	}
	else throw runtime_error("unknown enemy position");

	auto jumpState = unitJumpState;

	bool isFallingWhileJumpPadJumping = false;
	bool isJumpPadJumpingWhileFalling = false;
	while (Simulator::isUnitOnAir(positions.back(), unitSize, game))
	{
		const auto nextPos = Simulator::getUnitInTimePosition(
			positions.back(), unitSize, action, tickTime, jumpState, game);
		if (positions.size() == 1) unitJumpState = jumpState;//обновл€ем jumpState после тика 1 TODO
		
		positions.emplace_back(nextPos);

		if (isFallingWhileJumpPadJumping && jumpState.canJump && !jumpState.canCancel) break;//новый цикл прыжка на батуте
		if (isJumpPadJumpingWhileFalling && !jumpState.canJump && !jumpState.canCancel) break;//новый цикл падени€ при прыжке на батуте

		if (isJumpPadJumping && !jumpState.canJump && !jumpState.canCancel)
			isFallingWhileJumpPadJumping = true;//перешли из прыжка на батуте в падение
		if (isFalling && jumpState.canJump && !jumpState.canCancel)
			isJumpPadJumpingWhileFalling = true;//перешли из падени€ в прыжок на батуте
	}

	return positions;
}

vector<vector<Vec2Double>> getSimplePositionsSimulations(const Unit& enemy, const Game& game)
{
	vector<vector<Vec2Double>> enemyPositions;

	auto lastEnemyPosition = enemy.position;
	auto lastEnemyJumpState = enemy.jumpState;
	auto curEnemyPositions = getSimplePositions(lastEnemyPosition, enemy.size, lastEnemyJumpState, game);
	enemyPositions.emplace_back(curEnemyPositions);

	int counter = 0;
	while (counter < MAX_SIMULATIONS)
	{
		lastEnemyPosition = curEnemyPositions.size() == 1 ? curEnemyPositions[0] : curEnemyPositions[1];
		curEnemyPositions = getSimplePositions(lastEnemyPosition, enemy.size, lastEnemyJumpState, game);
		enemyPositions.emplace_back(curEnemyPositions);

		counter++;
	}

	return enemyPositions;
}


bool areAllPositionsShooting(const Vec2Double& mePosition, const vector<Vec2Double>& enemyPositions)
{
	//TODO
	return true;
}


void getAttackingData(
	const Unit& me,	
	vector<Vec2Double>& mePositions, 
	vector<JumpState>& meJumpStates, 
	vector<UnitAction>& meActions, 
	const vector<vector<Vec2Double>>& enemyPositions,
	size_t startJumpY,
	const Game& game)
{
	const auto tickTime = 1 / game.properties.ticksPerSecond;
	
	auto lastMePosition = me.position;
	auto lastMeJumpState = me.jumpState;
	auto lastStartJumpY = startJumpY;
	mePositions.emplace_back(lastMePosition);	
	meJumpStates.emplace_back(lastMeJumpState);	

	int counter = 0;
	while (counter < MAX_SIMULATIONS)
	{
		const auto curEnemyPositions = enemyPositions[counter];
		if (areAllPositionsShooting(lastMePosition, curEnemyPositions)) return;
		
		UnitAction action;
		action.velocity = curEnemyPositions[0].x > lastMePosition.x ? INT_MAX : -INT_MAX;
		setJumpAndJumpDown(
			lastMePosition, lastMeJumpState, curEnemyPositions[0], game, false, action, lastStartJumpY);
				
		lastMePosition = Simulator::getUnitInTimePosition(lastMePosition, me.size, action, tickTime, lastMeJumpState, game);
		meActions.emplace_back(action);
		mePositions.emplace_back(lastMePosition);		
		meJumpStates.emplace_back(lastMeJumpState);
		
		counter++;
	}
}


void setShootingAction(
	const Unit& me, const vector<Vec2Double>& mePositions, 
	const Vec2Double& enemySize, const vector<vector<Vec2Double>>& enemyPositions,
	const Game& game, UnitAction& action)
{
	const auto tickTime = 1.0 / game.properties.ticksPerSecond;
	auto movingTime = me.weapon->fireTimer != nullptr ? *(me.weapon->fireTimer) : 0;
	if (abs(movingTime) < TOLERANCE) movingTime = 0;
	

	const auto canShootingTick = static_cast<size_t>(ceil(movingTime / tickTime));

	//const auto canSimulateMore = abs(action.velocity) > TOLERANCE || Simulator::isUnitOnAir(meShootingPosition, me.size, game);
		
	
	auto maxShootingProbability = 0.0;
	double okShootingAngle = 0;
	int addShootingSimulations = 0;
		
	while (canShootingTick + addShootingSimulations < MAX_SIMULATIONS)
	{
		const auto shootingTick = canShootingTick + addShootingSimulations;
		Vec2Double meShootingPosition;//позици€, откуда произойдет выстрел
		if (shootingTick >= mePositions.size())
		{
			if (addShootingSimulations > 0) break;//стоим на месте. нет смысла симулировать дальше
			meShootingPosition = mePositions.back();
		}
		else
		{
			meShootingPosition = mePositions[shootingTick];
		}
		
		auto enemyShootingPositions = //позиции врага, начина€ с тика выстрела
			enemyPositions[shootingTick];
		double minAngle = INT_MAX;
		double maxAngle = -INT_MAX;
		for (const auto& ep : enemyShootingPositions)
		{
			const auto shootingVector = ep - meShootingPosition;
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

		const int directionsCount = max(1, static_cast<int>((maxAngle - minAngle) / M_PI * 20));
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
				spread = max(
					me.weapon->params.minSpread, 
					spread - me.weapon->params.aimSpeed*(shootingTick / game.properties.ticksPerSecond));
			}

			const auto probability = Strategy::getShootEnemyProbability(
				meShootingPosition,
				me.size,
				shootingAngle,
				spread,
				me.weapon->params.bullet,
				enemyShootingPositions,
				enemySize,
				game);
			if (probability > maxShootingProbability)
			{
				maxShootingProbability = probability;
				okShootingAngle = shootingAngle;
			}
		}

		if (maxShootingProbability >= OK_SHOOTING_PROBABILITY) break;
		addShootingSimulations++;
	}					

	if (maxShootingProbability >= OK_SHOOTING_PROBABILITY)
	{
		action.shoot = canShootingTick == 0 && addShootingSimulations == 0;
		action.aim = Vec2Double(cos(okShootingAngle), sin(okShootingAngle));
	}
	else
	{				
		action.shoot = false;
		action.aim = enemyPositions[canShootingTick][0] - mePositions[canShootingTick];
	}				

	//TODO: ракетницей стрел€ть под ноги врагу
	//TODO: не стрел€ть ракетницей с риском задеть себ€	
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
	
	/*const auto tickTime = 1.0 / game.properties.ticksPerSecond;
	auto needGo = false;
	auto needShoot = false;

	if (nearestEnemy != nullptr)
	{		
		needGo = game.currentTick >= 97 ? false : Strategy::getShootEnemyProbability(unit, *nearestEnemy, game, unit.weapon->params.minSpread) <
			WALKING_PROBABILITY;
		needShoot = strategy_.getShootEnemyProbability(unit, *nearestEnemy, game, unit.weapon->spread, &debug) >=
			SHOOTING_PROBABILITY;		
	}
	action.shoot = needShoot;*/

	const auto enemyBulletsSimulation = strategy_.getEnemyBulletsSimulation(game, unit.playerId);
	const auto enemyPositions = getSimplePositionsSimulations(*nearestEnemy, game);

	drawBullets(debug, game, enemyBulletsSimulation, unit.playerId);
	drawShootingSector(debug, unit, game);
	const auto curStopRunawayTick = strategy_.getStopRunawayTick();
	
	if (curStopRunawayTick == 0)
	{
		strategy_.setRunaway(GoNONE, -1);
	}
	else if (curStopRunawayTick > 0)
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

		const auto mePositions = getActionPositions(unit, action, 0, curStopRunawayTick, game);
		setShootingAction(unit, mePositions, nearestEnemy->size, enemyPositions, game, action);

		strategy_.decreaseStopRunawayTick();
		return action;
	}

	//setMoveToEnemyAction(unit, nearestEnemy->position, needGo, game, action, strategy_);
	vector<Vec2Double> meAttackingPositions;
	vector<JumpState> meAttackingJumpStates;
	vector<UnitAction> meAttackingActions;	
	getAttackingData(
		unit, 
		meAttackingPositions, meAttackingJumpStates, meAttackingActions,
		enemyPositions, strategy_.getStartedJumpY(), game);

	tuple<RunawayDirection, int, int, int> runawayAction;
	auto isSafeMove = strategy_.isSafeMove(unit, action, enemyBulletsSimulation, game);	

	if (isSafeMove)
	{
		auto jumpState = unit.jumpState;

		const auto meAttackPosition = meAttackingPositions.size() == 1 ? meAttackingPositions[0] : meAttackingPositions[1];
		const auto meAttackingJumpState = meAttackingJumpStates.size() == 1 ? meAttackingJumpStates[0] : meAttackingJumpStates[1];
		const auto actionShootMeBullets = strategy_.getShootMeBullets(
			meAttackPosition, unit.size, meAttackingJumpState,
			unit.playerId,
			enemyBulletsSimulation, 1, game);
		
		runawayAction = strategy_.getRunawayAction(
			meAttackPosition, unit.size, unit.playerId, jumpState,
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

			//TODO: продлить meAttackingPositions
			setShootingAction(unit, meAttackingPositions, nearestEnemy->size, enemyPositions, game, action);
			return action;			
		}				
	}

	//идти на врага нельз€. пробуем сто€ть
	
	bool checkUp = true;
	bool checkDown = true;
	bool checkLeft = true;
	bool checkRight = true;

	if (action.jump) checkUp = false;
	else if (action.jumpDown) checkDown = false;
	else if (action.velocity < -TOLERANCE) checkLeft = false;
	else if (action.velocity > TOLERANCE) checkRight = false;

	// выжидаем тик начала движени€, не дела€ ничего
	action.jump = false;
	action.jumpDown = false;
	action.velocity = 0;

	const auto shootMeBullets = strategy_.getShootMeBullets(
		unit.position, unit.size, unit.jumpState, unit.playerId,
		enemyBulletsSimulation, 0, game);
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


	UnitAction runawayUnitAction;
	if (runawayDirection == GoUP)
	{
		runawayUnitAction.jump = true;
		runawayUnitAction.jumpDown = false;
		runawayUnitAction.velocity = 0;
	}
	else if (runawayDirection == GoDOWN)
	{
		runawayUnitAction.jump = false;
		runawayUnitAction.jumpDown = true;
		runawayUnitAction.velocity = 0;
	}
	else if (runawayDirection == GoLEFT)
	{
		runawayUnitAction.jump = false;
		runawayUnitAction.jumpDown = false;
		runawayUnitAction.velocity = -INT_MAX;
	}
	else if (runawayDirection == GoRIGHT)
	{
		runawayUnitAction.jump = false;
		runawayUnitAction.jumpDown = false;
		runawayUnitAction.velocity = INT_MAX;
	}
	else
	{
		throw runtime_error("unknown runawayDirection 2");
	}
	
	if (startRunawayTick == 0)
	{		
		strategy_.setRunaway(runawayDirection, stopRunawayTick - 1);
		action.jump = runawayUnitAction.jump;
		action.jumpDown = runawayUnitAction.jumpDown;
		action.velocity = runawayUnitAction.velocity;
	}	
	
	cout << game.currentTick << ": " << action.jump << " " << action.jumpDown << " " << action.velocity << "\n";

	const auto mePositions = runawayDirection == GoNONE ?
		vector<Vec2Double>{unit.position} :
		getActionPositions(unit, runawayUnitAction, startRunawayTick, stopRunawayTick, game);
	
	setShootingAction(unit, mePositions, nearestEnemy->size, enemyPositions, game, action);
	return action;
}

