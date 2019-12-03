#include "MyStrategy.hpp"
#include <utility>
#include <climits>
#include <map>
#include <tuple>

#include <sstream>
#include "common/Helper.h"
#include "mathcalc/MathHelper.h"
#include "strategy/ShootMeBullet.h"
#include "debug/DebugHelper.h"

using namespace std;


MyStrategy::MyStrategy()
{	
}


UnitAction MyStrategy::getAction(const Unit& unit, const Game& game,
                                 Debug& debug)
{
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

	drawBullets(debug, game, unit.playerId);
	drawShootingSector(debug, unit, game);

	auto needGo = false;
	auto needShoot = false;

	if (nearestEnemy != nullptr)
	{
		if (unit.weapon != nullptr)
		{
			needGo = strategy_.getShootEnemyProbability(unit, *nearestEnemy, game, unit.weapon->params.minSpread) <
				WALKING_PROBABILITY;
			needShoot = strategy_.getShootEnemyProbability(unit, *nearestEnemy, game, unit.weapon->spread, &debug) >=
				SHOOTING_PROBABILITY;
		}
	}

	const auto shootMeBullets = strategy_.getShootMeBullets(unit, game);
	const auto enemyBulletsShootWallTimes = strategy_.getEnemyBulletsShootWallTimes(game, unit.playerId);


	const auto aim = nearestEnemy != nullptr ?
		Vec2Double(nearestEnemy->position.x - unit.position.x,
			nearestEnemy->position.y - unit.position.y) :
		Vec2Double(0, 0);

	
	bool jump = false;

	double velocity;
	if (unit.weapon != nullptr && !needGo)
	{
		velocity = 0;
	}
	else
	{
		if (targetPos.x > unit.position.x)
		{
			velocity = INT_MAX;
		}
		else
		{
			velocity = -INT_MAX;
		}
	}


	jump = unit.weapon == nullptr && targetPos.y > unit.position.y;
	if ((unit.weapon == nullptr || unit.weapon != nullptr && needGo) &&
		targetPos.x > unit.position.x &&
		game.level.tiles[size_t(unit.position.x + 1)][size_t(unit.position.y)] ==
		WALL)
	{
		jump = true;
	}
	if ((unit.weapon == nullptr || unit.weapon != nullptr && needGo) &&
		targetPos.x < unit.position.x &&
		game.level.tiles[size_t(unit.position.x - 1)][size_t(unit.position.y)] ==
		WALL)
	{
		jump = true;
	}
	auto jumpDown = unit.weapon != nullptr ? false : !jump;

	
	
	if (strategy_.getStopRunawayTick() == 0)
	{
		const auto runawayDirection = strategy_.getRunawayDirection();
		if (runawayDirection == GoUP)
		{
			jump = false;
			velocity = 0;
		}
		else if (runawayDirection == GoDOWN)
		{
			jump = false;
			velocity = 0;
			jumpDown = true;
		}
		strategy_.decreaseStopRunawayTick();
	}
	else if (strategy_.getStopRunawayTick() > 0)
	{
		const auto runawayDirection = strategy_.getRunawayDirection();
		if (runawayDirection == GoUP)
		{
			jump = true;
			velocity = 0;
		}
		else if (runawayDirection == GoDOWN)
		{
			jump = false;
			velocity = 0;
			jumpDown = true;
		}
		else if (runawayDirection == GoLEFT)
		{
			jump = false;
			velocity = -INT_MAX;
		}
		else if (runawayDirection == GoRIGHT)
		{
			jump = false;
			velocity = INT_MAX;
		}
		strategy_.decreaseStopRunawayTick();
	}
	else
	{
		if (unit.jumpState.canJump)
		{						

			const auto jumpAndStopTicks = strategy_.getRunawayAction(unit, shootMeBullets, enemyBulletsShootWallTimes, game);
			debug.draw(CustomData::Log(
				to_string(std::get<0>(jumpAndStopTicks)) + " " +
				to_string(std::get<1>(jumpAndStopTicks)) + " " +
				to_string(std::get<2>(jumpAndStopTicks)) + "\n"));

			if (!shootMeBullets.empty())
			{
				stringstream ss;

				const auto smb = shootMeBullets[0];
				ss << "me: " << unit.position.x << " " << unit.position.y << "; bp: " << smb.bullet.position.x << " " <<
					smb.bullet.position.y << "; bv: " << smb.bullet.velocity.x << " " << smb.bullet.velocity.y;

				debug.draw(CustomData::Log(ss.str()));
			}

			if (std::get<1>(jumpAndStopTicks) == 0)
			{
				const auto runawayDirection = std::get<0>(jumpAndStopTicks);
				const auto stopRunawayTick = std::get<2>(jumpAndStopTicks);
				strategy_.setRunaway(runawayDirection, stopRunawayTick);

				if (runawayDirection == GoUP)
				{
					jump = true;
					velocity = 0;
				}
				else if (runawayDirection == GoDOWN)
				{
					jump = false;
					jumpDown = true;
					velocity = 0;
				}
				else if (runawayDirection == GoLEFT)
				{
					jump = false;

					velocity = -INT_MAX;
				}
				else if (runawayDirection == GoRIGHT)
				{
					jump = false;
					velocity = INT_MAX;
				}
			}
		}
	}


	debug.draw(CustomData::Log("SHOOT: " + to_string(needShoot)));


	UnitAction action;
	action.velocity = velocity;
	action.jump = jump;
	action.jumpDown = jumpDown;
	action.aim = aim;
	action.shoot = needShoot;
	action.reload = false;
	action.swapWeapon = false;
	action.plantMine = false;

	if (shootMeBullets.empty() && !strategy_.isSafeMove(unit, action, enemyBulletsShootWallTimes, game))
	{
		const auto smb2 = strategy_.getShootMeBullets(unit, game);
		
		action.jump = false;
		action.jumpDown = false;
		action.velocity = 0;
	}

	return action;
}

