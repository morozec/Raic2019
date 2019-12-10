#include "MyStrategy.hpp"
#include <utility>
#include <climits>
#include <map>
#include <tuple>
#include <sstream>
#include "common/Helper.h"
#include "mathcalc/MathHelper.h"
#include "debug/DebugHelper.h"
#include "simulation/Simulator.h"

using namespace std;


MyStrategy::MyStrategy()
{	
}


inline bool operator<(const Bullet& lhs, const Bullet& rhs)
{
	return lhs.position.x < rhs.position.x;
}


void setAttackEnemyAction(
	const Unit& me, const Vec2Double& enemyPosition, bool needGo, const Game& game, UnitAction& action)
{
	double velocity = 0;
	bool jump = false;
	const bool jumpDown = false;

	if (needGo)
	{
		velocity = enemyPosition.x > me.position.x ? INT_MAX : -INT_MAX;
		jump = enemyPosition.x > me.position.x &&
			game.level.tiles[size_t(me.position.x + 1)][size_t(me.position.y)] == WALL ||
			enemyPosition.x < me.position.x &&
			game.level.tiles[size_t(me.position.x - 1)][size_t(me.position.y)] ==
			WALL;
	}

	action.velocity = velocity;
	action.jump = jump;
	action.jumpDown = jumpDown;
	
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
	Vec2Double targetPos = unit.position;
	if (unit.weapon == nullptr && nearestWeapon != nullptr)
	{
		targetPos = nearestWeapon->position;
	}
	else if (nearestEnemy != nullptr)
	{
		targetPos = nearestEnemy->position;
	}

	

	UnitAction action;
	action.aim = nearestEnemy != nullptr ?
		Vec2Double(nearestEnemy->position.x - unit.position.x,
			nearestEnemy->position.y - unit.position.y) :
		Vec2Double(0, 0);
	
	action.reload = false;
	action.swapWeapon = false;
	action.plantMine = false;

	if (unit.weapon == nullptr)
	{
		action.jump = targetPos.y > unit.position.y ||
			targetPos.x > unit.position.x &&
			game.level.tiles[size_t(unit.position.x + 1)][size_t(unit.position.y)] == WALL ||
			targetPos.x < unit.position.x &&
			game.level.tiles[size_t(unit.position.x - 1)][size_t(unit.position.y)] == WALL;

		action.jumpDown = !action.jump;
		if (abs(nearestWeapon->position.x - unit.position.x) < TOLERANCE)
		{
			action.velocity = 0;
		}
		else
		{
			action.velocity = nearestWeapon->position.x > unit.position.x ? INT_MAX : -INT_MAX;
		}
		
		action.shoot = false;
		action.reload = false;
		action.swapWeapon = false;
		action.plantMine = false;

		return action;
	}

	if (nearestEnemy == nullptr) return action;

	const auto tickTime = 1.0 / game.properties.ticksPerSecond;
	auto needGo = false;
	auto needShoot = false;

	if (nearestEnemy != nullptr)
	{		
		needGo = strategy_.getShootEnemyProbability(unit, *nearestEnemy, game, unit.weapon->params.minSpread) <
			WALKING_PROBABILITY;
		needShoot = strategy_.getShootEnemyProbability(unit, *nearestEnemy, game, unit.weapon->spread, &debug) >=
			SHOOTING_PROBABILITY;		
	}
	action.shoot = needShoot;

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
		return action;
	}

	setAttackEnemyAction(unit, nearestEnemy->position, needGo, game, action);

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
	

	return action;
}

