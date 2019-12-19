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


void prolongatePositions(vector<Vec2Double>& positions, const Vec2Double& unitSize, JumpState& jumpState, const Game& game)
{
	const auto tickTime = 1 / game.properties.ticksPerSecond;
	auto lastPosition = positions.back();

	const auto isFalling = !jumpState.canJump && !jumpState.canCancel;
	const auto isJumpPadJumping = jumpState.canJump && !jumpState.canCancel;
	
	UnitAction action;
	action.jump = false;
	action.jumpDown = false;
	action.velocity = 0;

	bool isFallingWhileJumpPadJumping = false;
	bool isJumpPadJumpingWhileFalling = false;
	
	while (positions.size() < MAX_SIMULATIONS && Simulator::isUnitOnAir(lastPosition, unitSize, game))
	{
		lastPosition = Simulator::getUnitInTimePosition(lastPosition, unitSize, action, tickTime, jumpState, game);
		positions.push_back(lastPosition);

		if (isFallingWhileJumpPadJumping && jumpState.canJump && !jumpState.canCancel) break;//новый цикл прыжка на батуте
		if (isJumpPadJumpingWhileFalling && !jumpState.canJump && !jumpState.canCancel) break;//новый цикл падения при прыжке на батуте

		if (isJumpPadJumping && !jumpState.canJump && !jumpState.canCancel)
			isFallingWhileJumpPadJumping = true;//перешли из прыжка на батуте в падение
		if (isFalling && jumpState.canJump && !jumpState.canCancel)
			isJumpPadJumpingWhileFalling = true;//перешли из падения в прыжок на батуте
	}	
}

inline bool operator<(const Bullet& lhs, const Bullet& rhs)
{
	return lhs.position.x < rhs.position.x;
}

vector<Vec2Double> getActionPositions(
	const Vec2Double& unitPosition, const Vec2Double& unitSize, const UnitAction& action, int startTick, int stopTick, JumpState& jumpState, const Game& game)
{
	if (startTick < 0 || stopTick < 0) throw runtime_error("getActionPositions tick is negative");
	
	const auto tickTime = 1 / game.properties.ticksPerSecond;
	vector<Vec2Double> positions;

	positions.emplace_back(unitPosition);

	UnitAction waitAction;
	waitAction.jump = false;
	waitAction.jumpDown = false;
	waitAction.velocity = 0;
	for (int i = 0; i < startTick; ++i)
	{
		const auto nextPos = Simulator::getUnitInTimePosition(
			positions.back(), unitSize, waitAction, tickTime, jumpState, game);
		positions.emplace_back(nextPos);
	}

	for (int i = 0; i < stopTick - startTick; ++i)
	{
		const auto nextPos = Simulator::getUnitInTimePosition(
			positions.back(), unitSize, action, tickTime, jumpState, game);
		positions.emplace_back(nextPos);
	}
	
	return positions;
}


void setJumpAndJumpDown(const Vec2Double& unitPosition, const JumpState& unitJumpState, 
	const Vec2Double& targetPosition, const Vec2Double& targetSize, const Game& game,
	bool considerYs,
	UnitAction& action, size_t& startedJumpY)
{
	const auto isLeftWall = targetPosition.x < unitPosition.x && targetPosition.x - targetSize.x/2 < size_t(unitPosition.x) &&
		game.level.tiles[size_t(unitPosition.x - 1)][size_t(unitPosition.y)] == WALL;
	const auto isRightWall = targetPosition.x > unitPosition.x && targetPosition.x + targetSize.x/2 > size_t(unitPosition.x + 1) &&
		game.level.tiles[size_t(unitPosition.x + 1)][size_t(unitPosition.y)] == WALL;
	const auto isSameColumnHigher = 
		targetPosition.y > unitPosition.y && targetPosition.x >= unitPosition.x - 1 && targetPosition.x <= unitPosition.x + 1;
	const auto needJump = considerYs && targetPosition.y > unitPosition.y || isLeftWall || isRightWall || 
		considerYs && isSameColumnHigher;

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
			while (game.level.tiles[size_t(unitPosition.x - 1)][size_t(unitPosition.y + TOLERANCE + wallHeight)] == WALL)
			{
				wallHeight++;
			}
			const auto maxJumpHeight = unitJumpState.speed * unitJumpState.maxTime;
			if (startedJumpY == 0 && maxJumpHeight < wallHeight)
			{
				startedJumpY = size_t(unitPosition.y + TOLERANCE);
			}
		}

		else if (isRightWall && unitJumpState.canJump && unitJumpState.canCancel)
		{
			auto wallHeight = 0;
			while (game.level.tiles[size_t(unitPosition.x + 1)][size_t(unitPosition.y + TOLERANCE + wallHeight)] == WALL)
			{
				wallHeight++;
			}
			const auto maxJumpHeight = unitJumpState.speed * unitJumpState.maxTime;
			if (startedJumpY == 0 && maxJumpHeight < wallHeight)
			{
				startedJumpY = size_t(unitPosition.y + TOLERANCE);
			}
		}
		else if (isSameColumnHigher)
		{
			const auto jumpHeight = targetPosition.y - unitPosition.y;			
			const auto maxJumpHeight = unitJumpState.speed * unitJumpState.maxTime;
			if (startedJumpY == 0 && maxJumpHeight < jumpHeight)
			{
				startedJumpY = size_t(unitPosition.y + TOLERANCE);
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

void setMoveToWeaponAction(const Unit& unit, const LootBox& weapon, const Game& game,
	UnitAction& action, Strategy& strategy)
{
	auto startedJumpY = strategy.getStartedJumpY();
	setJumpAndJumpDown(
		unit.position, unit.jumpState, weapon.position, weapon.size, game, true, action, startedJumpY);
	strategy.setStartedJumpY(startedJumpY);
	
	if (abs(weapon.position.x - unit.position.x) < TOLERANCE)
		action.velocity = 0;
	else
		action.velocity = weapon.position.x > unit.position.x ? INT_MAX : -INT_MAX;

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
		action.jump = false;
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
		if (positions.size() == 1) unitJumpState = jumpState;//обновляем jumpState после тика 1 TODO
		
		positions.emplace_back(nextPos);

		if (isFallingWhileJumpPadJumping && jumpState.canJump && !jumpState.canCancel) break;//новый цикл прыжка на батуте
		if (isJumpPadJumpingWhileFalling && !jumpState.canJump && !jumpState.canCancel) break;//новый цикл падения при прыжке на батуте

		if (isJumpPadJumping && !jumpState.canJump && !jumpState.canCancel)
			isFallingWhileJumpPadJumping = true;//перешли из прыжка на батуте в падение
		if (isFalling && jumpState.canJump && !jumpState.canCancel)
			isJumpPadJumpingWhileFalling = true;//перешли из падения в прыжок на батуте
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

	int counter = 1;
	while (counter < MAX_SIMULATIONS)
	{
		lastEnemyPosition = curEnemyPositions.size() == 1 ? curEnemyPositions[0] : curEnemyPositions[1];
		curEnemyPositions = getSimplePositions(lastEnemyPosition, enemy.size, lastEnemyJumpState, game);
		enemyPositions.emplace_back(curEnemyPositions);

		counter++;
	}

	return enemyPositions;
}


double getSimpleProbability(
	const Vec2Double& mePosition, const Vec2Double& meSize,
	const vector<Vec2Double>& enemyPositions, const Vec2Double& enemySize, const Game& game)
{
	const auto bulletPosition = Vec2Double(mePosition.x, mePosition.y + meSize.y / 2);
	int count = 0;
	for (const auto& ep: enemyPositions)
	{
		const auto epCenter = Vec2Double(ep.x, ep.y + enemySize.y / 2);
		auto squares = MathHelper::getLineSquares2(bulletPosition, epCenter);
		const auto wall = find_if(squares.begin(), squares.end(), [game](const auto& p) {return game.level.tiles[p.first][p.second] == Tile::WALL; });
		if (wall == squares.end()) count++;
	}	
	return count * 1.0/enemyPositions.size();
}

bool areRectCross(double left1, double right1, double bottom1, double top1,
	double left2, double right2, double bottom2, double top2)
{
	const auto isHorCross = left2 >= left1 && left2 <= right1 || right2 >= left1 && right2 <= right1;
	const auto isVertCross = bottom2 >= bottom1 && bottom2 <= top1 ||  top2 >= bottom1 && top2 <= top1;
	
	const auto isCross = isHorCross && isVertCross;
	return isCross;
}

void getHealingData(
	const Unit& me,
	vector<Vec2Double>& mePositions,
	vector<JumpState>& meJumpStates,
	const LootBox& lootBox,
	UnitAction& meAction,
	size_t& startJumpY,
	const Game& game
)
{
	const auto tickTime = 1 / game.properties.ticksPerSecond;

	auto lastMePosition = me.position;
	auto lastMeJumpState = me.jumpState;
	auto lastStartJumpY = startJumpY;

	mePositions.emplace_back(lastMePosition);
	meJumpStates.emplace_back(lastMeJumpState);

	const auto lootBoxLeft = lootBox.position.x - lootBox.size.x / 2;
	const auto lootBoxRight = lootBox.position.x + lootBox.size.x / 2;
	const auto lootBoxBottom = lootBox.position.y;
	const auto lootBoxTop = lootBox.position.y + lootBox.size.y;

	int counter = 1;
	while (counter < MAX_SIMULATIONS)
	{
		const auto meLeft = lastMePosition.x - me.size.x / 2;
		const auto meRight = lastMePosition.x + me.size.x / 2;
		const auto meBottom = lastMePosition.y;
		const auto meTop = lastMePosition.y + me.size.y;

		const auto isCross = areRectCross(
			meLeft, meRight, meBottom, meTop, 
			lootBoxLeft, lootBoxRight, lootBoxBottom, lootBoxTop);
			
		if (isCross)
		{
			if (counter == 1)
			{
				meAction.jump = false;
				meAction.jumpDown = false;
				meAction.velocity = 0;
			}
			return;
		}

		UnitAction action;
		action.velocity = lootBox.position.x > lastMePosition.x ? INT_MAX : -INT_MAX;
		setJumpAndJumpDown(
			lastMePosition, lastMeJumpState, lootBox.position, lootBox.size, game, false, action, lastStartJumpY);

		if (counter == 1) {
			startJumpY = lastStartJumpY;
			meAction = action;
		}

		lastMePosition = Simulator::getUnitInTimePosition(lastMePosition, me.size, action, tickTime, lastMeJumpState, game);
		mePositions.emplace_back(lastMePosition);
		meJumpStates.emplace_back(lastMeJumpState);
		
		counter++;
	}
}


void getAttackingData(
	const Unit& me,	
	vector<Vec2Double>& mePositions,
	vector<JumpState>& meJumpStates,	
	const Vec2Double& enemySize,
	const vector<vector<Vec2Double>>& enemyPositions,
	UnitAction& meAction,
	size_t& startJumpY,
	const Game& game)
{
	const auto tickTime = 1 / game.properties.ticksPerSecond;
	
	auto lastMePosition = me.position;
	auto lastMeJumpState = me.jumpState;
	auto lastStartJumpY = startJumpY;
	mePositions.emplace_back(lastMePosition);	
	meJumpStates.emplace_back(lastMeJumpState);	

	int counter = 1;
	while (counter < MAX_SIMULATIONS)
	{
		const auto curEnemyPositions = enemyPositions[counter];
		const auto curEnemyPosition = curEnemyPositions[0];

		if (abs(lastMePosition.x - curEnemyPosition.x) < me.size.x/2 + enemySize.x/2 + TOLERANCE &&
			abs(lastMePosition.y - curEnemyPosition.y) < me.size.y + TOLERANCE)		
		{
			if (counter == 1)
			{
				meAction.jump = false;
				meAction.jumpDown = false;
				meAction.velocity = 0;
			}
			return;
		}		
		
		UnitAction action;
		action.velocity = curEnemyPosition.x > lastMePosition.x ? INT_MAX : -INT_MAX;
		setJumpAndJumpDown(
			lastMePosition, lastMeJumpState, curEnemyPosition, enemySize, game, false, action, lastStartJumpY);

		if (counter == 1) {
			startJumpY = lastStartJumpY;
			meAction = action;
		}
		
		lastMePosition = Simulator::getUnitInTimePosition(lastMePosition, me.size, action, tickTime, lastMeJumpState, game);	
		mePositions.emplace_back(lastMePosition);		
		meJumpStates.emplace_back(lastMeJumpState);
		
		counter++;
	}
}




void setShootingAction(
	const Unit& me, const vector<Vec2Double>& mePositions, const vector<double>& meSimpleProbabilities,
	const Vec2Double& enemySize, const vector<vector<Vec2Double>>& enemyPositions,
	const Game& game, UnitAction& action)
{
	const auto tickTime = 1.0 / game.properties.ticksPerSecond;
	const auto movingTime = me.weapon->fireTimer != nullptr ? *(me.weapon->fireTimer) - TOLERANCE : 0;

	const auto canShootingTick = static_cast<int>(ceil(movingTime / tickTime));

	if (canShootingTick == 1 && movingTime < tickTime)//выстрел в рамках тика 0-1
	{
		double maxProbability = 0.0;
		double okShootingAngle = 0.0;
		
		Vec2Double meShootingPosition;//позиция, откуда произойдет выстрел
		if (mePositions.size() == 1)
		{
			meShootingPosition = mePositions[0];
		}
		else
		{
			const auto part = movingTime / tickTime;
			const auto x0 = mePositions[0].x;
			const auto y0 = mePositions[0].y;
			const auto x1 = mePositions[1].x;
			const auto y1 = mePositions[1].y;
			meShootingPosition = { x0 + (x1 - x0) * part, y0 + (y1 - y0) * part };
		}
		
		const auto enemyShootingPositions = enemyPositions[movingTime < tickTime/2 ? 0 : 1]; //позиции врага, начиная с тика выстрела
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
				shootingAngle = shootingVector.y > 0 ? M_PI / 2 : -M_PI / 2;
			}
			if (shootingAngle < minAngle) minAngle = shootingAngle;
			if (shootingAngle > maxAngle) maxAngle = shootingAngle;
		}

		int directionsCount;
		double deltaAngle;
		if (abs(maxAngle - minAngle) < TOLERANCE)
		{
			directionsCount = 1;
			deltaAngle = 0;
		}
		else
		{
			directionsCount = ANGLE_SPLIT_COUNT;
			deltaAngle = (maxAngle - minAngle) / (ANGLE_SPLIT_COUNT - 1);
		}

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
					spread - me.weapon->params.aimSpeed * movingTime);
			}

			const auto probability = Strategy::getShootEnemyProbability(
				meShootingPosition,
				me.size,
				shootingAngle,
				spread,
				me.weapon->params,
				enemyShootingPositions,
				enemySize,
				game);

			if (probability > maxProbability)
			{
				maxProbability = probability;
				okShootingAngle = shootingAngle;	
			}
		}

		if (maxProbability > OK_SHOOTING_PROBABILITY)
		{
			action.shoot = true;
			action.aim = Vec2Double(cos(okShootingAngle), sin(okShootingAngle));
			return;
		}
	}

	//const auto canSimulateMore = abs(action.velocity) > TOLERANCE || Simulator::isUnitOnAir(meShootingPosition, me.size, game);
		
	
	auto maxShootingProbability = 0.0;
	double okShootingAngle = 0;
	int okAddShootingSimulations = -1;
	
	/*const auto maxSpread = me.weapon->params.maxSpread;
	const auto minSpread = me.weapon->params.minSpread;
	const auto aimSpeed = me.weapon->params.aimSpeed;
	const auto spreadDecrease = aimSpeed * tickTime;*/

	map<int, double> allProbabilities;
	map<int, double> allShootingAngles;
	map<int, double> allSpreads;

	int addShootingSimulations = 0;
	while (canShootingTick + addShootingSimulations < MAX_SIMULATIONS)
	{
		double curMaxShootingProbability = 0.0;
		double curOkShootingAngle = 0.0;
		double curSpread = 0.0;
		
		const auto shootingTick = canShootingTick + addShootingSimulations;
		Vec2Double meShootingPosition;//позиция, откуда произойдет выстрел
		double meSimpleProbability;
		if (shootingTick >= mePositions.size())
		{
			if (addShootingSimulations > 0) break;//стоим на месте. нет смысла симулировать дальше
			meShootingPosition = mePositions.back();
			meSimpleProbability = meSimpleProbabilities.back();
		}
		else
		{
			meShootingPosition = mePositions[shootingTick];
			meSimpleProbability = meSimpleProbabilities[shootingTick];
		}
		if (abs(meSimpleProbability) < TOLERANCE)
		{
			addShootingSimulations++;
			continue;
		}		
		
		const auto enemyShootingPositions = enemyPositions[shootingTick]; //позиции врага, начиная с тика выстрела
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
				shootingAngle = shootingVector.y > 0 ? M_PI/2 : -M_PI/2;
			}
			if (shootingAngle < minAngle) minAngle = shootingAngle;
			if (shootingAngle > maxAngle) maxAngle = shootingAngle;
		}

		int directionsCount;
		double deltaAngle;
		if (abs(maxAngle - minAngle) < TOLERANCE)
		{
			directionsCount = 1;
			deltaAngle = 0;
		}
		else
		{
			directionsCount = ANGLE_SPLIT_COUNT;
			deltaAngle = (maxAngle - minAngle) / (ANGLE_SPLIT_COUNT - 1);
		}
		
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
				me.weapon->params,
				enemyShootingPositions,
				enemySize,
				game);			

			if (probability > curMaxShootingProbability)
			{
				curMaxShootingProbability = probability;
				curOkShootingAngle = shootingAngle;
				curSpread = spread;
			}	
		}

		allProbabilities[shootingTick] = curMaxShootingProbability;
		allShootingAngles[shootingTick] = curOkShootingAngle;
		allSpreads[shootingTick] = curSpread;

		if (curMaxShootingProbability > maxShootingProbability)
		{
			maxShootingProbability = curMaxShootingProbability;
			okShootingAngle = curOkShootingAngle;
			okAddShootingSimulations = addShootingSimulations;
			if (maxShootingProbability >= OK_SHOOTING_PROBABILITY) break;
		}

		addShootingSimulations++;
	}					

	if (maxShootingProbability >= NOT_BAD_SHOOTING_PROBABILITY) 
	{
		while (true)
		{
			auto okAddShootingSimulations2 = -1;
			auto maxProb2 = NOT_BAD_SHOOTING_PROBABILITY;
			auto addShootingSimulations2 = 0;
			auto okShootingAngle2 = 0.0;

			const auto shootDelay = static_cast<int>(ceil((me.weapon->params.fireRate - TOLERANCE) / tickTime));
			while (canShootingTick + addShootingSimulations2 < canShootingTick + okAddShootingSimulations - shootDelay)
			{
				const auto shootingTick2 = canShootingTick + addShootingSimulations2;
				const auto prob2 = allProbabilities[shootingTick2];
				if (prob2 < maxProb2 + TOLERANCE)
				{
					addShootingSimulations2++;
					continue;
				}
				const auto angle2 = allShootingAngles[shootingTick2];


				const auto bestShootingTick = canShootingTick + okAddShootingSimulations;
				//позиция, откуда произойдет выстрел
				Vec2Double meShootingPosition =
					bestShootingTick >= mePositions.size() ?
					mePositions.back() :
					mePositions[bestShootingTick];

				auto spread = allSpreads[shootingTick2];//разброс в момент первого выстрела
				spread = min(me.weapon->params.maxSpread, spread + me.weapon->params.recoil);//отдача
				spread = min(me.weapon->params.maxSpread, spread + abs(angle2 - okShootingAngle));//новое прицеливание
				//затухание разброса
				spread = max(
					me.weapon->params.minSpread,
					spread - me.weapon->params.aimSpeed * ((bestShootingTick - shootingTick2) / game.properties.ticksPerSecond));

				const auto enemyShootingPositions = enemyPositions[bestShootingTick]; //позиции врага, начиная с тика выстрела
				const auto newBestShootProbability = Strategy::getShootEnemyProbability(
					meShootingPosition,
					me.size,
					okShootingAngle,
					spread,
					me.weapon->params,
					enemyShootingPositions,
					enemySize,
					game);

				if (newBestShootProbability > maxShootingProbability - TOLERANCE)
				{
					okAddShootingSimulations2 = addShootingSimulations2;
					maxProb2 = prob2;
					okShootingAngle2 = angle2;
				}

				addShootingSimulations2++;
			}

			if (okAddShootingSimulations2 != -1)
			{
				okAddShootingSimulations = okAddShootingSimulations2;
				maxShootingProbability = maxProb2;
				okShootingAngle = okShootingAngle2;
				
				/*action.shoot = canShootingTick == 0 && okAddShootingSimulations2 == 0;
				action.aim = Vec2Double(cos(okShootingAngle2), sin(okShootingAngle2));*/
			}
			else
			{
				action.shoot = canShootingTick == 0 && okAddShootingSimulations == 0;
				action.aim = Vec2Double(cos(okShootingAngle), sin(okShootingAngle));
				break;
			}
		}
	}	
	//else if (mePositions.size() == 1 && maxShootingProbability > TOLERANCE)// дальше я уже не пойду
	//{
	//	action.shoot = canShootingTick == 0;
	//	action.aim = Vec2Double(cos(okShootingAngle), sin(okShootingAngle));
	//}
	else
	{				
		action.shoot = false;
		const auto enemyPos = enemyPositions[canShootingTick][0];
		const auto mePos = canShootingTick < mePositions.size() ? mePositions[canShootingTick] : mePositions.back();
		action.aim = enemyPos - mePos;
	}				

	//TODO: ракетницей стрелять под ноги врагу
	//TODO: не стрелять ракетницей с риском задеть себя	
}

void initAttackAction(
	const Unit& unit,
	vector<Vec2Double>& meAttackingPositions, const vector<JumpState>& meAttackingJumpStates,
	const JumpState& nextTickMeAttackingJumpState, const Vec2Double& nextTickMeAttackPosition,
	vector<double>& meSimpleProbabilities,
	const vector<vector<Vec2Double>>& enemyPositions, const Vec2Double& enemySize,
	int startJumpY,
	tuple<RunawayDirection, int, int, int> runawayAction,
	const UnitAction& meAttackingAction, UnitAction& action, Strategy strategy, const Game& game)
{
	const auto runawayDirection = std::get<0>(runawayAction);
	const auto runawayStartTick = std::get<1>(runawayAction);
	const auto runawayStopTick = std::get<2>(runawayAction);
	const auto minDamage = std::get<3>(runawayAction);
	
	action.jump = meAttackingAction.jump;
	action.jumpDown = meAttackingAction.jumpDown;
	action.velocity = meAttackingAction.velocity;

	auto lastMeAttackingJumpState = meAttackingJumpStates.back();

	if (runawayDirection != GoNONE)//потом придется убегать
	{
		vector<Vec2Double> runawayMeAttackingPositions;
		runawayMeAttackingPositions.emplace_back(meAttackingPositions[0]);

		UnitAction runawayAttackAction;
		if (runawayDirection == GoUP)
		{
			runawayAttackAction.jump = true;
			runawayAttackAction.jumpDown = false;
			runawayAttackAction.velocity = 0;
		}
		else if (runawayDirection == GoDOWN)
		{
			runawayAttackAction.jump = false;
			runawayAttackAction.jumpDown = true;
			runawayAttackAction.velocity = 0;
		}
		else if (runawayDirection == GoLEFT)
		{
			runawayAttackAction.jump = false;
			runawayAttackAction.jumpDown = false;
			runawayAttackAction.velocity = -INT_MAX;
		}
		else if (runawayDirection == GoRIGHT)
		{
			runawayAttackAction.jump = false;
			runawayAttackAction.jumpDown = false;
			runawayAttackAction.velocity = INT_MAX;
		}
		else
		{
			throw runtime_error("unknown runawayDirection 2");
		}

		lastMeAttackingJumpState = nextTickMeAttackingJumpState;
		auto nextPositions = getActionPositions(
			nextTickMeAttackPosition, unit.size, runawayAttackAction,
			runawayStartTick, runawayStopTick, lastMeAttackingJumpState,
			game);
		for (const auto& nextPos : nextPositions)
		{
			runawayMeAttackingPositions.emplace_back(nextPos);
		}
		meAttackingPositions = runawayMeAttackingPositions;
	}

	prolongatePositions(meAttackingPositions, unit.size, lastMeAttackingJumpState, game);

	for (size_t i = 0; i < meAttackingPositions.size(); ++i)
	{
		const auto& mePosition = meAttackingPositions[i];
		const auto& curEnemyPositions = enemyPositions[i];
		const auto sp = getSimpleProbability(mePosition, unit.size, curEnemyPositions, enemySize, game);
		meSimpleProbabilities.emplace_back(sp);
	}

	setShootingAction(unit, meAttackingPositions, meSimpleProbabilities, enemySize, enemyPositions, game, action);
	strategy.setStartedJumpY(startJumpY);
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
			setMoveToWeaponAction(unit, *nearestWeapon, game, action, strategy_);
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

	vector<double> meSimpleProbabilities;
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
		auto jumpState = unit.jumpState;
		auto mePositions = getActionPositions(unit.position, unit.size, action, 0, curStopRunawayTick, jumpState, game);		
		prolongatePositions(mePositions, unit.size, jumpState, game);

		
		for (size_t i = 0; i < mePositions.size(); ++i)
		{
			const auto& mePosition = mePositions[i];
			const auto& curEnemyPositions = enemyPositions[i];
			const auto sp = getSimpleProbability(mePosition, unit.size, curEnemyPositions, nearestEnemy->size, game);
			meSimpleProbabilities.emplace_back(sp);
		}
		setShootingAction(unit, mePositions,meSimpleProbabilities, nearestEnemy->size, enemyPositions, game, action);

		strategy_.decreaseStopRunawayTick();
		return action;
	}

	//setMoveToEnemyAction(unit, nearestEnemy->position, needGo, game, action, strategy_);
	vector<Vec2Double> meAttackingPositions;
	vector<JumpState> meAttackingJumpStates;
	UnitAction meAttackingAction;
	auto startJumpY = strategy_.getStartedJumpY();

	bool needHeal = false;
	for (const auto& enemy: game.units)
	{
		if (enemy.playerId == unit.playerId) continue;
		if (enemy.health > unit.health)
		{
			needHeal = true;
			break;
		}
	}

	const LootBox* nearestHPLootBox = nullptr;
	double minMHDist = INT_MAX;
	
	if (needHeal)
	{
		for (const auto& lb:game.lootBoxes)
		{
			if (std::dynamic_pointer_cast<Item::HealthPack>(lb.item))
			{
				const auto mhDist = MathHelper::getMHDist(unit.position, lb.position);
				bool hasCloserEnemies = false;
				for (const auto& enemy: game.units )
				{
					if (enemy.playerId == unit.playerId) continue;
					if (MathHelper::getMHDist(enemy.position, lb.position) < mhDist)
					{
						hasCloserEnemies = true;
						break;
					}
				}
				if (hasCloserEnemies) continue;

				if (mhDist < minMHDist)
				{
					nearestHPLootBox = &lb;
					minMHDist = mhDist;
				}
			}			
		}
	}

	//TODO!!!
	/*if (nearestHPLootBox != nullptr)
		getHealingData(
			unit, meAttackingPositions, meAttackingJumpStates,
			*nearestHPLootBox, meAttackingAction, startJumpY, game	);
	else*/
		getAttackingData(
			unit, 
			meAttackingPositions, meAttackingJumpStates,
			nearestEnemy->size, enemyPositions, meAttackingAction, startJumpY, game);

	tuple<RunawayDirection, int, int, int> attackRunawayAction;
	const auto thisTickShootMeBullets = strategy_.isSafeMove(unit, meAttackingAction, enemyBulletsSimulation, game);
	int minAttackDamage = 0;
	
	map<Bullet, BulletSimulation> nextTickEnemyBulletsSimulation;	
	for (const auto& ebs: enemyBulletsSimulation)
	{
		if (thisTickShootMeBullets.count(ebs.first) == 0)
		{
			nextTickEnemyBulletsSimulation[ebs.first] = ebs.second;
		}
		else
		{
			minAttackDamage += ebs.first.damage;
			if (ebs.first.explosionParams != nullptr) minAttackDamage += ebs.first.explosionParams->damage;
		}
	}
	
	const auto nextTickMeAttackPosition = meAttackingPositions.size() == 1 ? meAttackingPositions[0] : meAttackingPositions[1];
	const auto nextTickMeAttackingJumpState = 
		meAttackingJumpStates.size() == 1 ? meAttackingJumpStates[0] : meAttackingJumpStates[1];
	const auto nextTickShootMeBullets = strategy_.getShootMeBullets(
		nextTickMeAttackPosition, unit.size, nextTickMeAttackingJumpState,
		unit.playerId,
		nextTickEnemyBulletsSimulation, 1, game);
	
	attackRunawayAction = strategy_.getRunawayAction(
		nextTickMeAttackPosition, unit.size, unit.playerId, nextTickMeAttackingJumpState,
		nextTickShootMeBullets, nextTickEnemyBulletsSimulation, 1,
		true, true, true, true,
		game);

	minAttackDamage += std::get<3>(attackRunawayAction);
	
	if (minAttackDamage == 0)
	{
		const auto runawayDirection = std::get<0>(attackRunawayAction);
		const auto runawayStartTick = std::get<1>(attackRunawayAction);
		const auto runawayStopTick = std::get<2>(attackRunawayAction);
		
		debug.draw(CustomData::Log(
			to_string(runawayDirection) + " " +
			to_string(runawayStartTick + 1) + " " +
			to_string(runawayStopTick + 1) + " " +
			to_string(minAttackDamage) + "\n"));

		initAttackAction(unit, meAttackingPositions, meAttackingJumpStates,
			nextTickMeAttackingJumpState, nextTickMeAttackPosition, meSimpleProbabilities,
			enemyPositions, nearestEnemy->size, startJumpY,
			attackRunawayAction, meAttackingAction, action, strategy_, game);
		return action;			
	}				
	

	//идти на врага нельзя. пробуем стоять
	
	bool checkUp = true;
	bool checkDown = true;
	bool checkLeft = true;
	bool checkRight = true;

	if (meAttackingAction.jump) checkUp = false;
	else if (meAttackingAction.jumpDown) checkDown = false;
	else if (meAttackingAction.velocity < -TOLERANCE) checkLeft = false;
	else if (meAttackingAction.velocity > TOLERANCE) checkRight = false;

	// выжидаем тик начала движения, не делая ничего
	action.jump = false;
	action.jumpDown = false;
	action.velocity = 0;

	const auto shootMeBullets = strategy_.getShootMeBullets(
		unit.position, unit.size, unit.jumpState, unit.playerId,
		enemyBulletsSimulation, 0, game);
	const auto noAttackRunawayAction = strategy_.getRunawayAction(
		unit.position, unit.size, unit.playerId, unit.jumpState,
		shootMeBullets, enemyBulletsSimulation, 0,
		checkUp, checkDown, checkLeft, checkRight,
		game);

	const auto minNoAttackDamage = std::get<3>(noAttackRunawayAction);
	
	
	if (minNoAttackDamage >= minAttackDamage)
	{
		const auto runawayDirection = std::get<0>(attackRunawayAction);
		const auto runawayStartTick = std::get<1>(attackRunawayAction);
		const auto runawayStopTick = std::get<2>(attackRunawayAction);
		
		debug.draw(CustomData::Log(
			to_string(runawayDirection) + " " +
			to_string(runawayStartTick + 1) + " " +
			to_string(runawayStopTick + 1) + " " +
			to_string(minAttackDamage) + "\n"));
		
		initAttackAction(unit, meAttackingPositions, meAttackingJumpStates,
			nextTickMeAttackingJumpState, nextTickMeAttackPosition, meSimpleProbabilities,
			enemyPositions, nearestEnemy->size, startJumpY,
			attackRunawayAction, meAttackingAction, action, strategy_, game);
		return action;
	}

	const auto runawayDirection = std::get<0>(noAttackRunawayAction);
	const auto startRunawayTick = std::get<1>(noAttackRunawayAction);
	const auto stopRunawayTick = std::get<2>(noAttackRunawayAction);
	
	debug.draw(CustomData::Log(
		to_string(runawayDirection) + " " +
		to_string(startRunawayTick) + " " +
		to_string(stopRunawayTick) + " " +
		to_string(minNoAttackDamage) + "\n"));		


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
	else if (runawayDirection == GoNONE)
	{
		runawayUnitAction.jump = false;
		runawayUnitAction.jumpDown = false;
		runawayUnitAction.velocity = 0;
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
	
	//cout << game.currentTick << ": " << action.jump << " " << action.jumpDown << " " << action.velocity << "\n";

	JumpState jumpState = unit.jumpState;
	auto mePositions = runawayDirection == GoNONE ?
		vector<Vec2Double>{ unit.position } :
		getActionPositions(unit.position, unit.size, runawayUnitAction, startRunawayTick, stopRunawayTick, jumpState, game);
	
	prolongatePositions(mePositions, unit.size, jumpState, game);
	
	for (size_t i = 0; i < mePositions.size(); ++i)
	{
		const auto& mePosition = mePositions[i];
		const auto& curEnemyPositions = enemyPositions[i];
		const auto sp = getSimpleProbability(mePosition, unit.size, curEnemyPositions, nearestEnemy->size, game);
		meSimpleProbabilities.emplace_back(sp);
	}
	
	setShootingAction(unit, mePositions, meSimpleProbabilities, nearestEnemy->size, enemyPositions, game, action);
	return action;
}

