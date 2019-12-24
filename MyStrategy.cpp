#define _USE_MATH_DEFINES

#include "MyStrategy.hpp"
#include "common/Helper.h"
#include <utility>
#include <climits>
#include <map>
#include <tuple>
#include <sstream>
#include <cmath>
#include "common/Helper.h"
#include "mathcalc/MathHelper.h"
#include "myDebug/DebugHelper.h"
#include "simulation/Simulator.h"
#include <iostream>
#include <algorithm>
#include "common/AStar.h"

using namespace std;


MyStrategy::MyStrategy()
{	
}


void prolongatePositions(
	vector<Vec2Double>& positions, const Vec2Double& unitSize, int unitId, JumpState& jumpState, const Game& game)
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
	
	while (positions.size() < MAX_SIMULATIONS && Simulator::isUnitOnAir(lastPosition, unitSize, unitId, game))
	{
		lastPosition = Simulator::getUnitInTimePosition(lastPosition, unitSize, unitId, action, tickTime, jumpState, game);
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
	const Vec2Double& unitPosition, const Vec2Double& unitSize, int unitId,
	const UnitAction& action, int startTick, int stopTick, JumpState& jumpState, const Game& game)
{
	//if (startTick < 0 || stopTick < 0) throw runtime_error("getActionPositions tick is negative");
	
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
			positions.back(), unitSize, unitId, waitAction, tickTime, jumpState, game);
		positions.emplace_back(nextPos);
	}

	for (int i = 0; i < stopTick - startTick; ++i)
	{
		const auto nextPos = Simulator::getUnitInTimePosition(
			positions.back(), unitSize, unitId, action, tickTime, jumpState, game);
		positions.emplace_back(nextPos);
	}
	
	return positions;
}


void setJumpAndJumpDown(const Vec2Double& unitPosition, const Vec2Double& unitSize, const JumpState& unitJumpState, 
	int playerId, int unitId,
	const Vec2Double& targetPosition, const Vec2Double& targetSize, const Game& game,
	bool considerYs,
	UnitAction& action, size_t& startedJumpY, int& jumpingUnitId)
{
	const auto isLeftWall = targetPosition.x < unitPosition.x && targetPosition.x - targetSize.x/2 < size_t(unitPosition.x) &&
		game.level.tiles[size_t(unitPosition.x - 1)][size_t(unitPosition.y)] == WALL;
	const auto isRightWall = targetPosition.x > unitPosition.x && targetPosition.x + targetSize.x/2 > size_t(unitPosition.x + 1) &&
		game.level.tiles[size_t(unitPosition.x + 1)][size_t(unitPosition.y)] == WALL;
	const auto isSameColumnHigher = 
		targetPosition.y > unitPosition.y && targetPosition.x >= unitPosition.x - 1 && targetPosition.x <= unitPosition.x + 1;
	auto needJumpThroughUnit = false;
	
	if (jumpingUnitId == -1 &&
		(!Simulator::isUnitOnAir(unitPosition, unitSize, unitId, game) ||
			unitJumpState.canJump))
	{
		for (const auto& unit : game.units)
		{
			if (unit.playerId != playerId) continue;
			if (unit.id == unitId) continue;

			if ((targetPosition.x - unitPosition.x)* (unit.position.x - unitPosition.x) < 0) continue;//он мне не мешает

			//он в воздухе и прыгает
			if (Simulator::isUnitOnAir(unit.position, unit.size, unit.id, game) && unit.jumpState.canJump) continue;

			const auto xDist = std::abs(unitPosition.x - unit.position.x);
			if (xDist < unitSize.x / 2 + unit.size.x / 2 + TOLERANCE &&
				xDist > unitSize.x / 2 + unit.size.x / 2 - TOLERANCE &&
				unitPosition.y < unit.position.y + unit.size.y + TOLERANCE &&
				unitPosition.y + unitSize.y > unitPosition.y - TOLERANCE)
			{
				needJumpThroughUnit = true;
				jumpingUnitId = unitId;
				break;
			}
		}
	}
	
	const auto needJump = considerYs && targetPosition.y > unitPosition.y || isLeftWall || isRightWall || 
		considerYs && isSameColumnHigher || needJumpThroughUnit;



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


vector<Vec2Double> getSimplePositions(
	const Vec2Double& unitPosition, const Vec2Double& unitSize, int unitId, JumpState& unitJumpState, const Game& game)
{
	const auto tickTime = 1 / game.properties.ticksPerSecond;
	
	vector<Vec2Double> positions;
	positions.emplace_back(unitPosition);

	const auto isOnAir = Simulator::isUnitOnAir(unitPosition, unitSize, unitId, game);
	const auto isFalling = !unitJumpState.canJump && !unitJumpState.canCancel;
	const auto isJumping = isOnAir && unitJumpState.canJump && unitJumpState.canCancel;
	const auto isJumpPadJumping = unitJumpState.canJump && !unitJumpState.canCancel;

	if (isJumping) return positions;// не симулируем прыгуна
	
	UnitAction action;
	if (isFalling || //падает
		isJumpPadJumping || //прыгает на батуте
		!isOnAir) // стоит на земле 
	{
		action.jump = false;
		action.jumpDown = false;
		action.velocity = 0;
	}	
	//else throw runtime_error("unknown enemy position");

	auto jumpState = unitJumpState;

	bool isFallingWhileJumpPadJumping = false;
	bool isJumpPadJumpingWhileFalling = false;
	while (Simulator::isUnitOnAir(positions.back(), unitSize, unitId, game))
	{
		const auto nextPos = Simulator::getUnitInTimePosition(
			positions.back(), unitSize, unitId, action, tickTime, jumpState, game);
		if (positions.size() == 1) unitJumpState = jumpState;//обновляем jumpState после тика 1 TODO
		
		positions.emplace_back(nextPos);

		if (isFallingWhileJumpPadJumping && jumpState.canJump && !jumpState.canCancel) break;//новый цикл прыжка на батуте
		if (isJumpPadJumpingWhileFalling && !jumpState.canJump && !jumpState.canCancel) break;//новый цикл падения при прыжке на батуте

		if (isJumpPadJumping && !jumpState.canJump && !jumpState.canCancel)
			isFallingWhileJumpPadJumping = true;//перешли из прыжка на батуте в падение
		if ((isFalling || isJumping) && jumpState.canJump && !jumpState.canCancel)
			isJumpPadJumpingWhileFalling = true;//перешли из падения в прыжок на батуте
	}

	return positions;
}

vector<vector<Vec2Double>> getSimplePositionsSimulations(const Unit& enemy, const Game& game)
{
	vector<vector<Vec2Double>> enemyPositions;

	auto lastEnemyPosition = enemy.position;
	auto lastEnemyJumpState = enemy.jumpState;
	auto curEnemyPositions = getSimplePositions(lastEnemyPosition, enemy.size, enemy.id, lastEnemyJumpState, game);
	enemyPositions.emplace_back(curEnemyPositions);

	int counter = 1;
	while (counter < MAX_SIMULATIONS)
	{
		lastEnemyPosition = curEnemyPositions.size() == 1 ? curEnemyPositions[0] : curEnemyPositions[1];
		curEnemyPositions = getSimplePositions(lastEnemyPosition, enemy.size, enemy.id, lastEnemyJumpState, game);
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

vector<vector<int>> getGrid(const Game& game)
{
	vector<vector<int>> grid;

	for (size_t i = 0; i < game.level.tiles.size(); ++i)
	{
		grid.emplace_back(vector<int>());
		for (size_t j = 0; j < game.level.tiles[i].size(); ++j)
		{
			int value = 1;
			const auto tile = game.level.tiles[i][j];
			if (
				tile == WALL ||
				(j < game.level.tiles[i].size() - 1 && 
				game.level.tiles[i][j + 1] == WALL))
			{
				value = 0;
			}
			grid[i].emplace_back(value);
		}
	}
	
	return grid;
}

void initAStarAction(const Unit& me, const Vec2Double& targetPos, UnitAction& action, const Game& game, 
	Debug& debug)
{
	const auto tickTime = 1.0 / game.properties.ticksPerSecond;
	const auto grid = getGrid(game);
	const auto endPos =
		make_pair(size_t(targetPos.x), size_t(targetPos.y));

	const auto isOnAir = Simulator::isUnitOnAir(me.position, me.size, me.id, game);
	const auto isJumping = isOnAir && me.jumpState.canJump && me.jumpState.canCancel;
	const auto isFalling = isOnAir && !me.jumpState.canJump && !me.jumpState.canCancel;

	const auto bottomTile = game.level.tiles[size_t(me.position.x)][size_t(me.position.y - 1)];
	const auto maxJumpTiles = static_cast<int>(game.properties.unitJumpTime * game.properties.unitJumpSpeed);
	
	int start_z = 0;
	if (bottomTile == EMPTY || bottomTile == JUMP_PAD)
	{			   		
		if (isFalling) start_z = maxJumpTiles + 1;
		else if (isJumping)
		{
			const auto jumpingTime = game.properties.unitJumpTime - me.jumpState.maxTime;
			start_z = static_cast<int>(jumpingTime * game.properties.unitJumpSpeed);
			//if (size_t(me.position.y + jumpTimeLeft * game.properties.unitJumpSpeed) > size_t(me.position.y)) 
			//	start_z++; //смогу подняться на 1 тайл
		}
		//TODO: прыжок на батуте
	}

	
	
	const auto startPos =
		make_tuple(size_t(me.position.x), size_t(me.position.y), start_z);

	auto path = aStarSearch(grid, startPos, endPos, maxJumpTiles, game);
	const auto myTile = path[0];	
	const auto nextTile = path[1];
	

	for (const auto& step:path)
	{
		debug.draw(CustomData::Rect(
			vec2DoubleToVec2Float({ static_cast<double>(step.first), static_cast<double>(step.second) }),
			{ 1, 1 },
			ColorFloat(255, 255, 255, 0.2)
		));
	}

	
	if (nextTile.first > myTile.first) action.velocity = INT_MAX;
	else if (nextTile.first < myTile.first) action.velocity = -INT_MAX;
	else 
	{
		const auto distToCenter = (myTile.first + 0.5) - me.position.x;
		if (std::abs(distToCenter) < TOLERANCE)
		{
			action.velocity = 0;
		}
		else
		{
			action.velocity = distToCenter / tickTime;
		}
	}

	auto xBorderDist = 0.0;
	if (nextTile.first > myTile.first)
	{
		xBorderDist = nextTile.first - (me.position.x + me.size.x / 2);
	}
	else if (nextTile.first < myTile.first)
	{
		xBorderDist = me.position.x - me.size.x / 2 - (nextTile.first + 1);
	}
	const auto yBorderDist = me.position.y - myTile.second;

	action.jump = false;

	if (nextTile.second > myTile.second)
	{
		if (nextTile.first == myTile.first || isOnAir)
		{
			action.jump = true;
		}
		else
		{
			bool isDangerousCorner = false;			
			for (int i = 1; i < path.size() - 1; ++i)
			{
				const auto& curStep = path[i];
				if (nextTile.first > myTile.first)
				{
					if (game.level.tiles[path[i].first - 1][path[i].second + 2] == WALL &&
						me.position.x < myTile.first + 0.5 - TOLERANCE)
					{
						isDangerousCorner = true;
						break;
					}
				}
				else if (nextTile.first < myTile.first)
				{
					if (game.level.tiles[path[i].first + 1][path[i].second + 2] == WALL &&
						me.position.x > myTile.first + 0.5 + TOLERANCE)
					{
						isDangerousCorner = true;
						break;
					}
				}
				
				const auto& nextStep = path[i + 1];
				if (nextStep.second <= curStep.second) break;
				if (nextTile.first - myTile.first != nextStep.first - curStep.first) break;
			}			
						
			action.jump = !isDangerousCorner;
		}
	}
	else if (nextTile.second <= myTile.second && isJumping &&
		(game.level.tiles[myTile.first][myTile.second - 1] == EMPTY || game.level.tiles[myTile.first][myTile.second - 1] == JUMP_PAD) &&
		xBorderDist > yBorderDist)
	{
		action.jump = true;
	}

	if (nextTile.second < myTile.second) action.jumpDown = true;
	else action.jumpDown = false;

}

void getHealingData(
	const Unit& me,
	vector<Vec2Double>& mePositions,
	vector<JumpState>& meJumpStates,
	const LootBox& lootBox,
	UnitAction& meAction,
	size_t& startJumpY,
	int& jumpingUnitId,
	const Game& game
)
{
	
	const auto tickTime = 1 / game.properties.ticksPerSecond;

	auto lastMePosition = me.position;
	auto lastMeJumpState = me.jumpState;
	auto lastStartJumpY = startJumpY;
	auto lastJumpingUnitId = jumpingUnitId;

	mePositions.emplace_back(lastMePosition);
	meJumpStates.emplace_back(lastMeJumpState);

	int counter = 1;
	while (counter < MAX_SIMULATIONS)
	{
		const auto isCross = Simulator::areRectsCross(
			lastMePosition, me.size, lootBox.position, lootBox.size);
			
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
		if (lootBox.position.x > lastMePosition.x &&
			game.level.tiles[size_t(lootBox.position.x + 1)][size_t(lootBox.position.y + lootBox.size.y / 2)] == JUMP_PAD)
		{
			action.velocity = (lootBox.position.x - lastMePosition.x) / tickTime - TOLERANCE;
		}
		else if (lootBox.position.x < lastMePosition.x && 
			game.level.tiles[size_t(lootBox.position.x - 1)][size_t(lootBox.position.y + lootBox.size.y / 2)] == JUMP_PAD)
		{
			action.velocity = (lootBox.position.x - lastMePosition.x) / tickTime + TOLERANCE;
		}
		else 
			action.velocity = lootBox.position.x > lastMePosition.x ? INT_MAX : -INT_MAX;
		setJumpAndJumpDown(
			lastMePosition, me.size, lastMeJumpState,
			me.playerId, me.id,
			lootBox.position, lootBox.size, game, true, action, lastStartJumpY, lastJumpingUnitId);

		if (counter == 1) {
			startJumpY = lastStartJumpY;
			jumpingUnitId = lastJumpingUnitId;
			meAction = action;
		}

		lastMePosition = Simulator::getUnitInTimePosition(lastMePosition, me.size, me.id, action, tickTime, lastMeJumpState, game);
		mePositions.emplace_back(lastMePosition);
		meJumpStates.emplace_back(lastMeJumpState);
		
		counter++;
	}
}

bool needMonkeyMode(const Vec2Double& mePosition, const Vec2Double& enemyPosition, bool hasWeaponEnemy, int enemyFireTick)
{
	return
		hasWeaponEnemy && mePosition.y >= enemyPosition.y - TOLERANCE &&
		enemyFireTick < MONKEY_FIRE_TICK &&
		MathHelper::getMHDist(mePosition, enemyPosition) < MONKEY_DIST;
}


void getAttackingData(
	const Unit& me,	
	vector<Vec2Double>& mePositions,
	vector<JumpState>& meJumpStates,	
	const Vec2Double& enemySize,
	const vector<vector<Vec2Double>>& enemyPositions,
	double enemyFireTimer,
	UnitAction& meAction,
	size_t& startJumpY,
	int& jumpingUnitId,
	bool& isMonkeyMode,
	const Game& game)
{
	const auto tickTime = 1 / game.properties.ticksPerSecond;
	
	auto lastMePosition = me.position;
	auto lastMeJumpState = me.jumpState;
	auto lastStartJumpY = startJumpY;
	auto lastJumpingUnitId = jumpingUnitId;
	mePositions.emplace_back(lastMePosition);	
	meJumpStates.emplace_back(lastMeJumpState);

	auto isMyClosestUnit = true;
	const auto defaultEnemyPos = enemyPositions[0][0];
	for (const auto& unit:game.units)
	{
		if (unit.playerId != me.playerId) continue;
		if (unit.id == me.id) continue;
		if (MathHelper::getMHDist(unit.position, defaultEnemyPos) < MathHelper::getMHDist(me.position, defaultEnemyPos))
		{
			isMyClosestUnit = false;
			break;
		}
	}
	

	int counter = 1;
	bool needStop = false;
	const auto hasWeaponEnemy = std::abs(enemyFireTimer - INT_MAX) > TOLERANCE;

	double meFireTimer = INT_MAX;
	if (me.weapon != nullptr)
	{
		meFireTimer = me.weapon->fireTimer != nullptr ? *(me.weapon->fireTimer) : 0;
	}
	
	while (counter < MAX_SIMULATIONS)
	{
		UnitAction action;
		
		if (hasWeaponEnemy && isMonkeyMode && lastMeJumpState.canJump && lastMeJumpState.canCancel)
		{
			action.velocity = 0;
			action.jump = true;
			action.jumpDown = false;
		}
		else
		{
			const auto curEnemyPositions = enemyPositions[counter];
			const auto curEnemyPosition = curEnemyPositions[0];

			const auto enemyFireTick = static_cast<int>(enemyFireTimer / tickTime);

			const auto shootingVector = curEnemyPosition - lastMePosition;
			double shootingAngle;
			if (abs(shootingVector.x) > TOLERANCE) {
				shootingAngle = atan2(shootingVector.y, shootingVector.x);
			}
			else
			{
				shootingAngle = shootingVector.y > 0 ? M_PI / 2 : -M_PI / 2;
			}

			const auto isDangerousZone = MathHelper::getMHDist(lastMePosition, curEnemyPosition) < SAFE_SHOOTING_DIST;
			auto isEnemyShootEarlier = false;
			if (std::abs(enemyFireTimer - INT_MAX) < TOLERANCE) isEnemyShootEarlier = false;
			else if (std::abs(meFireTimer - INT_MAX) < TOLERANCE) isEnemyShootEarlier = true;
			else
			{
				const auto curEnemyFireTime = std::max(0.0, enemyFireTimer - tickTime * (counter-1));
				const auto curMeFireTimer = std::max(0.0, meFireTimer - tickTime * (counter-1));
				isEnemyShootEarlier = curEnemyFireTime < curMeFireTimer - TOLERANCE;
			}


			if (!isMonkeyMode && counter == 1 && 
				needMonkeyMode(lastMePosition, curEnemyPosition, hasWeaponEnemy, enemyFireTick) &&
				!Simulator::isUnitOnAir(lastMePosition, me.size, me.id, game))	
			{
				isMonkeyMode = true;
				action.jump = true;
				action.jumpDown = false;
				action.velocity = 0;
			}

			else if (
				hasWeaponEnemy &&
				!isMyClosestUnit && //тормозим дальним,  				
				//и стрелять не опасно
					(me.weapon->params.explosion == nullptr ||
						!Strategy::isDangerousRocketShooting(
							lastMePosition, me.size, shootingAngle, me.weapon->spread, me.weapon->params.bullet.size / 2, game)) 				
				)

			{
				vector<Vec2Double> tmpPositions;
				tmpPositions.emplace_back(lastMePosition);
				prolongatePositions(tmpPositions, me.size, me.id, lastMeJumpState, game);

				//если видим врага
				const auto isEnemyVisible = getSimpleProbability(
					tmpPositions.back(), me.size, curEnemyPositions, enemySize, game) > 1 - TOLERANCE;

				if (isEnemyVisible)
				{
					needStop = true;
					action.jump = false;
					action.jumpDown = false;
					action.velocity = 0;
				}
				else
				{
					action.velocity = curEnemyPosition.x > lastMePosition.x ? INT_MAX : -INT_MAX;
					setJumpAndJumpDown(
						lastMePosition, me.size, lastMeJumpState,
						me.playerId,
						me.id,
						curEnemyPosition, enemySize, game,
						hasWeaponEnemy ? false : true,
						action, lastStartJumpY, lastJumpingUnitId);
				}
			}

			else if ((!hasWeaponEnemy || isMyClosestUnit) && //тормозим ближним, если подошли вплотную
				Simulator::areRectsTouch(lastMePosition, me.size, curEnemyPosition, enemySize) ||
				isMyClosestUnit && isDangerousZone && isEnemyShootEarlier) //или если я близко, а он стреляет раньше
			{
				needStop = true;
				action.jump = false;
				action.jumpDown = false;
				action.velocity = 0;
			}
			else {
				action.velocity = curEnemyPosition.x > lastMePosition.x ? INT_MAX : -INT_MAX;
				setJumpAndJumpDown(
					lastMePosition, me.size, lastMeJumpState,
					me.playerId,
					me.id,
					curEnemyPosition, enemySize, game,
					hasWeaponEnemy ? false : true,
					action, lastStartJumpY, lastJumpingUnitId);
			}
		}
		
			
		if (counter == 1) {
			startJumpY = lastStartJumpY;
			jumpingUnitId = lastJumpingUnitId;
			meAction = action;
		}
		
		lastMePosition = Simulator::getUnitInTimePosition(lastMePosition, me.size, me.id, action, tickTime, lastMeJumpState, game);	
		mePositions.emplace_back(lastMePosition);		
		meJumpStates.emplace_back(lastMeJumpState);

		if (needStop) return;
		
		counter++;
	}
}


bool isSafeShoot(const Unit& me, const Game& game)
{
	for (const auto& unit: game.units)
	{
		if (unit.playerId != me.playerId) continue;
		if (unit.id == me.id) continue;

		const auto dist = MathHelper::getMHDist(me.position, unit.position);
		if (dist > SAFE_SHOOTING_DIST) continue;

		const auto probability = Strategy::getShootEnemyProbability(me, unit, game, me.weapon->spread);
		if (probability < SAFE_SHOOTING_PROBABILITY) continue;

		return false;
	}
	return true;
}



void setShootingAction(
	const Unit& me, const vector<Vec2Double>& mePositions, const vector<double>& meSimpleProbabilities,
	const Vec2Double& enemySize, const vector<vector<Vec2Double>>& enemyPositions,
	const Game& game, UnitAction& action)
{
	if (me.weapon == nullptr)
	{
		action.shoot = false;
		action.aim = { 0,0 };
		return;
	}
	
	const auto tickTime = 1.0 / game.properties.ticksPerSecond;
	const auto movingTime = me.weapon->fireTimer != nullptr ? *(me.weapon->fireTimer) - TOLERANCE : 0;

	const auto canShootingTick = static_cast<int>(ceil(movingTime / tickTime));

	if (canShootingTick == 1 && movingTime < tickTime)//выстрел в рамках тика 0-1
	{
		double maxProbability = 0.0;
		double okShootingAngle = 0.0;
		double okShootingSpread = 0.0;
		
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
				okShootingSpread = spread;
			}
		}

		if (maxProbability > OK_SHOOTING_PROBABILITY)
		{

			bool isDangerousShoot = false;
			if (me.weapon->params.explosion != nullptr && 
				!Simulator::areRectsTouch(me.position, me.size, enemyShootingPositions[0], enemySize))
			{
				isDangerousShoot = Strategy::isDangerousRocketShooting(
					me.position, me.size, 
					okShootingAngle, okShootingSpread,
					me.weapon->params.bullet.size / 2, game);
			}
			
			const auto isSafe = isSafeShoot(me, game);
			if (!isDangerousShoot && isSafe)
			{
				action.shoot = true;
			}
			else
			{
				action.shoot = false;
			}
			
			action.aim = { cos(okShootingAngle), sin(okShootingAngle) };
			return;
		}
	}
		
	
	auto maxShootingProbability = 0.0;
	double okShootingAngle = 0;
	double okShootingSpread = 0;
	
	int okAddShootingSimulations = -1;
	

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
			okShootingSpread = curSpread;
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
			auto okShootingSpread2 = 0.0;

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
					okShootingSpread2 = spread;
				}

				addShootingSimulations2++;
			}

			if (okAddShootingSimulations2 != -1)
			{
				okAddShootingSimulations = okAddShootingSimulations2;
				maxShootingProbability = maxProb2;
				okShootingAngle = okShootingAngle2;
				okShootingSpread = okShootingSpread2;
				
				/*action.shoot = canShootingTick == 0 && okAddShootingSimulations2 == 0;
				action.aim = Vec2Double(cos(okShootingAngle2), sin(okShootingAngle2));*/
			}
			else
			{
				if (canShootingTick == 0 && okAddShootingSimulations == 0)
				{
					bool isDangerousShoot = false;
					if (me.weapon->params.explosion != nullptr &&
						!Simulator::areRectsTouch(me.position, me.size, enemyPositions[0][0], enemySize))
					{
						isDangerousShoot = Strategy::isDangerousRocketShooting(
							me.position, me.size,
							okShootingAngle, okShootingSpread,
							me.weapon->params.bullet.size / 2, game);
					}
					
					const auto isSafe = isSafeShoot(me, game);
					if (!isDangerousShoot && isSafe)
					{
						action.shoot = true;
					}
					else
					{
						action.shoot = false;
					}
				}
				else
				{
					action.shoot = false;
				}
				
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
}

void initAttackAction(
	const Unit& unit,
	vector<Vec2Double>& meAttackingPositions, const vector<JumpState>& meAttackingJumpStates,
	const JumpState& nextTickMeAttackingJumpState, const Vec2Double& nextTickMeAttackPosition,
	vector<double>& meSimpleProbabilities,
	const vector<vector<Vec2Double>>& enemyPositions, const Vec2Double& enemySize,
	int startJumpY, int jumpingUnitId, bool isMonkeyMode,
	tuple<RunawayDirection, int, int, int> runawayAction,
	const UnitAction& meAttackingAction, UnitAction& action, Strategy& strategy, const Game& game)
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
		/*else
		{
			throw runtime_error("unknown runawayDirection 2");
		}*/

		lastMeAttackingJumpState = nextTickMeAttackingJumpState;
		auto nextPositions = getActionPositions(
			nextTickMeAttackPosition, unit.size, unit.id, runawayAttackAction,
			runawayStartTick, runawayStopTick, lastMeAttackingJumpState,
			game);
		for (const auto& nextPos : nextPositions)
		{
			runawayMeAttackingPositions.emplace_back(nextPos);
		}
		meAttackingPositions = runawayMeAttackingPositions;
	}

	prolongatePositions(meAttackingPositions, unit.size, unit.id, lastMeAttackingJumpState, game);

	for (size_t i = 0; i < meAttackingPositions.size(); ++i)
	{
		const auto& mePosition = meAttackingPositions[i];
		const auto& curEnemyPositions = enemyPositions[i];
		const auto sp = getSimpleProbability(mePosition, unit.size, curEnemyPositions, enemySize, game);
		meSimpleProbabilities.emplace_back(sp);
	}

	setShootingAction(unit, meAttackingPositions, meSimpleProbabilities, enemySize, enemyPositions, game, action);
	strategy.setStartedJumpY(unit.id, startJumpY);
	strategy.setJumpingUnitId(jumpingUnitId);
	strategy.setIsMonkeyMode(unit.id, isMonkeyMode);
}

UnitAction MyStrategy::getAction(const Unit& unit, const Game& game,
                                 Debug& debug)
{
	if (!strategy_.isInit)
	{
		for (const auto& u : game.units)
		{
			if (u.playerId != unit.playerId) continue;
			strategy_.setRunaway(u.id, GoNONE, -1);
			strategy_.setStartedJumpY(u.id, 0);
			strategy_.setIsMonkeyMode(u.id, false);
		}
		strategy_.isInit = true;
		strategy_.setJumpingUnitId(-1);
	}
	

	if (strategy_.getJumpingUnitId() == unit.id) strategy_.setJumpingUnitId(-1);
	for (auto it = strategy_.heal_boxes_.begin(); it != strategy_.heal_boxes_.end();)
	{
		if (it->first == unit.id) {
			strategy_.heal_boxes_.erase(it++);
		}
		else ++it;
	}
	
	
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
		if (std::dynamic_pointer_cast<Item::Weapon>(lootBox.item) &&
			std::dynamic_pointer_cast<Item::Weapon>(lootBox.item)->weaponType == ROCKET_LAUNCHER)
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

	//if (unit.weapon == nullptr) {
		initAStarAction(unit, nearestWeapon->position, action, game, debug);
		return action;
	//}

	if (nearestEnemy == nullptr) return action;	

	double enemyFireTimer = INT_MAX;
	if (nearestEnemy->weapon != nullptr)
	{
		const auto ft = (*nearestEnemy->weapon).fireTimer;
		enemyFireTimer = ft == nullptr ? 0 : *ft;
	}
	const auto tickTime = 1 / game.properties.ticksPerSecond;
	const auto enemyFireTick = static_cast<int>(enemyFireTimer / tickTime);
	
	auto isMonkeyMode = strategy_.getIsMonkeyMode(unit.id);
	if (isMonkeyMode && (
		!needMonkeyMode(
			unit.position,
			nearestEnemy->position,
			nearestEnemy->weapon != nullptr, 
			enemyFireTick) || //monkeyMode не нужен
		(!unit.jumpState.canJump && !unit.jumpState.canCancel) || //прыжок окончен
		!Simulator::isUnitOnAir(unit.position, unit.size, unit.id, game))) //я оказался на земле
	{
		strategy_.setIsMonkeyMode(unit.id, false);
		isMonkeyMode = false;
	}

	

	const auto enemyBulletsSimulation = strategy_.getEnemyBulletsSimulation(game, unit.playerId, unit.id);
	const auto enemyPositions = getSimplePositionsSimulations(*nearestEnemy, game);

	drawBullets(debug, game, enemyBulletsSimulation, unit.playerId);
	drawShootingSector(debug, unit, game);
	const auto curStopRunawayTick = strategy_.getStopRunawayTick(unit.id);

	vector<double> meSimpleProbabilities;
	if (curStopRunawayTick == 0)
	{
		strategy_.setRunaway(unit.id, GoNONE, -1);
	}
	else if (curStopRunawayTick > 0)
	{
		const auto runawayDirection = strategy_.getRunawayDirection(unit.id);
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
		/*else
		{
			throw runtime_error("unknown runawayDirection");
		}*/
		auto jumpState = unit.jumpState;
		auto mePositions = getActionPositions(unit.position, unit.size, unit.id, action, 0, curStopRunawayTick, jumpState, game);		
		prolongatePositions(mePositions, unit.size, unit.id, jumpState, game);

		
		for (size_t i = 0; i < mePositions.size(); ++i)
		{
			const auto& mePosition = mePositions[i];
			const auto& curEnemyPositions = enemyPositions[i];
			const auto sp = getSimpleProbability(mePosition, unit.size, curEnemyPositions, nearestEnemy->size, game);
			meSimpleProbabilities.emplace_back(sp);
		}
		setShootingAction(unit, mePositions,meSimpleProbabilities, nearestEnemy->size, enemyPositions, game, action);

		strategy_.decreaseStopRunawayTick(unit.id);
		return action;
	}

	//setMoveToEnemyAction(unit, nearestEnemy->position, needGo, game, action, strategy_);
	vector<Vec2Double> meAttackingPositions;
	vector<JumpState> meAttackingJumpStates;
	UnitAction meAttackingAction;
	auto startJumpY = strategy_.getStartedJumpY(unit.id);
	auto jumpingUnitId = strategy_.getJumpingUnitId();

	if (unit.weapon == nullptr)
	{
		getHealingData(
			unit, meAttackingPositions, meAttackingJumpStates,
			*nearestWeapon, meAttackingAction, startJumpY, jumpingUnitId, game);
	}
	else
	{
		bool needHeal = unit.health <= game.properties.unitMaxHealth / 2;
		if (!needHeal)
		{
			for (const auto& enemy : game.units)
			{
				if (enemy.playerId == unit.playerId) continue;
				if (enemy.health > unit.health)
				{
					needHeal = true;
					break;
				}
			}
		}

		const LootBox* nearestHPLootBox = nullptr;
		double minMHDist = INT_MAX;

		if (needHeal)
		{
			for (const auto& lb : game.lootBoxes)
			{
				if (std::dynamic_pointer_cast<Item::HealthPack>(lb.item))
				{
					bool isGot = false;
					for (const auto& item: strategy_.heal_boxes_)
					{
						if (std::abs(item.second.x - lb.position.x) < TOLERANCE &&
							std::abs(item.second.y - lb.position.y) < TOLERANCE)
						{
							isGot = true;
							break;
						}
					}
					if (isGot) continue;
					
					const auto dist = MathHelper::getMHDist(unit.position, lb.position);
					if (dist > minMHDist) continue;

					vector<Vec2Double> curMeAttackingPositions;
					vector<JumpState> curMeAttackingJumpStates;
					UnitAction curMeAttackingAction;
					size_t curStartJumpY = startJumpY;
					int curJumpingUnitId = jumpingUnitId;
					getHealingData(
						unit, curMeAttackingPositions, curMeAttackingJumpStates,
						lb, curMeAttackingAction, curStartJumpY, curJumpingUnitId, game);

					auto goodWay = true;
					for (const auto& pos : curMeAttackingPositions)
					{
						for (const auto& enemyUnit : game.units)
						{
							if (enemyUnit.playerId == unit.playerId) continue;
							if (Simulator::areRectsTouch(pos, unit.size, enemyUnit.position, enemyUnit.size))
							{
								goodWay = false;
								break;
							}
						}
						if (!goodWay) break;
					}

					if (goodWay)
					{
						minMHDist = dist;
						nearestHPLootBox = &lb;
						meAttackingPositions = curMeAttackingPositions;
						meAttackingJumpStates = curMeAttackingJumpStates;
						meAttackingAction = curMeAttackingAction;
						startJumpY = curStartJumpY;
						jumpingUnitId = curJumpingUnitId;

						strategy_.heal_boxes_[unit.id] = lb.position;
					}
				}
			}
		}

		if (nearestHPLootBox == nullptr)
		{
			getAttackingData(
				unit,
				meAttackingPositions, meAttackingJumpStates,
				nearestEnemy->size, enemyPositions,
				enemyFireTimer,
				meAttackingAction, startJumpY, jumpingUnitId, isMonkeyMode, game);
		}
	}
	
	

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
			minAttackDamage += thisTickShootMeBullets.at(ebs.first);
		}
	}

	
	const auto nextTickMeAttackPosition = meAttackingPositions.size() == 1 ? meAttackingPositions[0] : meAttackingPositions[1];
	const auto nextTickMeAttackingJumpState = 
		meAttackingJumpStates.size() == 1 ? meAttackingJumpStates[0] : meAttackingJumpStates[1];
	const auto nextTickShootMeBullets = strategy_.getShootMeBullets(
		nextTickMeAttackPosition, unit.size, nextTickMeAttackingJumpState,
		unit.playerId,
		unit.id,
		nextTickEnemyBulletsSimulation, 1, game);
	
	attackRunawayAction = strategy_.getRunawayAction(
		nextTickMeAttackPosition, unit.size, unit.playerId, unit.id, nextTickMeAttackingJumpState,
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
			enemyPositions, nearestEnemy->size, startJumpY, jumpingUnitId, isMonkeyMode,
			attackRunawayAction, meAttackingAction, action, strategy_, game);
		//cerr << game.currentTick << ": (" << unit.id << "-0) " << action.jump << " " << action.jumpDown << " " << action.velocity << "\n";
		return action;			
	}				
	

	//идти на врага нельзя. пробуем стоять
	
	bool checkUp = true;
	bool checkDown = true;
	bool checkLeft = true;
	bool checkRight = true;

	if (meAttackingAction.jump && std::abs(meAttackingAction.velocity) < TOLERANCE) checkUp = false;
	else if (meAttackingAction.jumpDown && std::abs(meAttackingAction.velocity) < TOLERANCE) checkDown = false;
	else if (!meAttackingAction.jump && !meAttackingAction.jumpDown && meAttackingAction.velocity < -TOLERANCE) checkLeft = false;
	else if (!meAttackingAction.jump && !meAttackingAction.jumpDown && meAttackingAction.velocity > TOLERANCE) checkRight = false;

	// выжидаем тик начала движения, не делая ничего
	action.jump = false;
	action.jumpDown = false;
	action.velocity = 0;

	const auto shootMeBullets = strategy_.getShootMeBullets(
		unit.position, unit.size, unit.jumpState, unit.playerId, unit.id,
		enemyBulletsSimulation, 0, game);
	const auto noAttackRunawayAction = strategy_.getRunawayAction(
		unit.position, unit.size, unit.playerId, unit.id, unit.jumpState,
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
			enemyPositions, nearestEnemy->size, startJumpY, jumpingUnitId, isMonkeyMode,
			attackRunawayAction, meAttackingAction, action, strategy_, game);
		//cerr << game.currentTick << ": (" << unit.id << "-1) " << action.jump << " " << action.jumpDown << " " << action.velocity << "\n";
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
	/*else
	{
		throw runtime_error("unknown runawayDirection 2");
	}*/
	
	if (startRunawayTick == 0)
	{		
		strategy_.setRunaway(unit.id, runawayDirection, stopRunawayTick - 1);
		action.jump = runawayUnitAction.jump;
		action.jumpDown = runawayUnitAction.jumpDown;
		action.velocity = runawayUnitAction.velocity;
	}	
	
	

	JumpState jumpState = unit.jumpState;
	auto mePositions = runawayDirection == GoNONE ?
		vector<Vec2Double>{ unit.position } :
		getActionPositions(unit.position, unit.size, unit.id, runawayUnitAction, startRunawayTick, stopRunawayTick, jumpState, game);
	
	prolongatePositions(mePositions, unit.size, unit.id, jumpState, game);
	
	for (size_t i = 0; i < mePositions.size(); ++i)
	{
		const auto& mePosition = mePositions[i];
		const auto& curEnemyPositions = enemyPositions[i];
		const auto sp = getSimpleProbability(mePosition, unit.size, curEnemyPositions, nearestEnemy->size, game);
		meSimpleProbabilities.emplace_back(sp);
	}
	
	setShootingAction(unit, mePositions, meSimpleProbabilities, nearestEnemy->size, enemyPositions, game, action);
	//cerr << game.currentTick << ": (" <<unit.id << "-2) " << action.jump << " " << action.jumpDown << " " << action.velocity <<  "\n";
	return action;
}

