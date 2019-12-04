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
				MathHelper::distanceSqr(unit.position, other.position) <
				MathHelper::distanceSqr(unit.position, nearestEnemy->position))
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
				MathHelper::distanceSqr(unit.position, lootBox.position) <
				MathHelper::distanceSqr(unit.position, nearestWeapon->position))
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
		action.velocity = nearestWeapon->position.x > unit.position.x ? INT_MAX : -INT_MAX;
		
		action.shoot = false;
		action.reload = false;
		action.swapWeapon = false;
		action.plantMine = false;

		return action;
	}

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
	const auto shootMeBullets = strategy_.getShootMeBullets(unit, enemyBulletsSimulation, game);

	drawBullets(debug, game, enemyBulletsSimulation, unit.playerId);
	drawShootingSector(debug, unit, game);
	
	if (strategy_.getStopRunawayTick() == 0)
	{
		const auto runawayDirection = strategy_.getRunawayDirection();
		strategy_.setRunaway(GoNONE, -1);
		
		if (runawayDirection == GoUP)
		{
			action.jump = false;
			action.jumpDown = false;
			action.velocity = 0;
			return action;
		}
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
	else
	{
		if (unit.jumpState.canJump)
		{					

			const auto runawayAction = strategy_.getRunawayAction(unit, shootMeBullets, enemyBulletsSimulation, game);
			debug.draw(CustomData::Log(
				to_string(std::get<0>(runawayAction)) + " " +
				to_string(std::get<1>(runawayAction)) + " " +
				to_string(std::get<2>(runawayAction)) + "\n"));

			/*if (!shootMeBullets.empty())
			{
				stringstream ss;

				const auto smb = shootMeBullets[0];
				ss << "me: " << unit.position.x << " " << unit.position.y << "; bp: " << smb.bullet.position.x << " " <<
					smb.bullet.position.y << "; bv: " << smb.bullet.velocity.x << " " << smb.bullet.velocity.y;

				debug.draw(CustomData::Log(ss.str()));
			}*/

			if (std::get<1>(runawayAction) == 0)
			{
				const auto runawayDirection = std::get<0>(runawayAction);
				const auto stopRunawayTick = std::get<2>(runawayAction);
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
				}else
				{
					throw runtime_error("unknown runawayDirection 2");
				}

				return action;
			}
		}
	}


	debug.draw(CustomData::Log("SHOOT: " + to_string(needShoot)));

	if (nearestEnemy == nullptr) return action;
	
	setAttackEnemyAction(unit, nearestEnemy->position, needGo, game, action);
	//проверяем опасность итогового действия
	//TODO: не только когда shootMeBullets.empty()
	if (shootMeBullets.empty() && !strategy_.isSafeMove(unit, action, enemyBulletsSimulation, game))
	{
		//const auto smb2 = strategy_.getShootMeBullets(unit, game);

		action.jump = false;
		action.jumpDown = false;
		action.velocity = 0;
	}

	return action;
}

