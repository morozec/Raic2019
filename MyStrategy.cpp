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

		if (isFallingWhileJumpPadJumping && jumpState.canJump && !jumpState.canCancel) break;//����� ���� ������ �� ������
		if (isJumpPadJumpingWhileFalling && !jumpState.canJump && !jumpState.canCancel) break;//����� ���� ������� ��� ������ �� ������

		if (isJumpPadJumping && !jumpState.canJump && !jumpState.canCancel)
			isFallingWhileJumpPadJumping = true;//������� �� ������ �� ������ � �������
		if (isFalling && jumpState.canJump && !jumpState.canCancel)
			isJumpPadJumpingWhileFalling = true;//������� �� ������� � ������ �� ������
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

			if ((targetPosition.x - unitPosition.x)* (unit.position.x - unitPosition.x) < 0) continue;//�� ��� �� ������

			//�� � ������� � �������
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
	//return positions;

	const auto isOnAir = Simulator::isUnitOnAir(unitPosition, unitSize, unitId, game);
	const auto isFalling = !unitJumpState.canJump && !unitJumpState.canCancel;
	const auto isJumping = isOnAir && unitJumpState.canJump && unitJumpState.canCancel;
	const auto isJumpPadJumping = unitJumpState.canJump && !unitJumpState.canCancel;

	if (isJumping) return positions;// �� ���������� �������
	
	UnitAction action;
	if (isFalling || //������
		isJumpPadJumping || //������� �� ������
		!isOnAir) // ����� �� ����� 
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
		if (positions.size() == 1) unitJumpState = jumpState;//��������� jumpState ����� ���� 1 TODO
		
		positions.emplace_back(nextPos);

		if (isFallingWhileJumpPadJumping && jumpState.canJump && !jumpState.canCancel) break;//����� ���� ������ �� ������
		if (isJumpPadJumpingWhileFalling && !jumpState.canJump && !jumpState.canCancel) break;//����� ���� ������� ��� ������ �� ������

		if (isJumpPadJumping && !jumpState.canJump && !jumpState.canCancel)
			isFallingWhileJumpPadJumping = true;//������� �� ������ �� ������ � �������
		if ((isFalling || isJumping) && jumpState.canJump && !jumpState.canCancel)
			isJumpPadJumpingWhileFalling = true;//������� �� ������� � ������ �� ������
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

int** getGrid(const Game& game, int cols, int rows)
{
	int** grid = new int*[cols];	

	for (size_t i = 0; i < cols; ++i)
	{
		int* col = new int[rows];
		for (size_t j = 0; j < rows; ++j)
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
			col[j] = value;
		}
		grid[i] = col;
	}
	
	return grid;
}

void initOneStepAction(const Four& myTile, const Four& nextTile, const vector<Four>& path,
	const Vec2Double& curPosition, const Vec2Double& unitSize, const JumpState& curJumpState, int unitId,
	UnitAction& action, double tickTime, const Game& game)
{
	const auto isOnAir = Simulator::isUnitOnAir(curPosition, unitSize, unitId, game);
	const auto isJumping = isOnAir && curJumpState.canJump && curJumpState.canCancel;
	const auto isFalling = isOnAir && !curJumpState.canJump && !curJumpState.canCancel;

	const auto myTileX = get<0>(myTile);
	const auto myTileY = get<1>(myTile);

	const auto nextTileX = get<0>(nextTile);
	const auto nextTileY = get<1>(nextTile);

	const auto needStopToRelax = nextTileX == myTileX && nextTileY == myTileY && get<2>(nextTile) == 0;
	if (needStopToRelax)
	{
		action.velocity = 0;
		action.jump = false;
		action.jumpDown = false;
		return;
	}	
	
	if (nextTileX > myTileX) action.velocity = INT_MAX;
	else if (nextTileX < myTileX) action.velocity = -INT_MAX;
	else
	{
		const auto distToCenter = (myTileX + 0.5) - curPosition.x;
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
	if (nextTileX > myTileX)
	{
		xBorderDist = nextTileX - curPosition.x;
	}
	else if (nextTileX < myTileX)
	{
		xBorderDist = curPosition.x - (nextTileX + 1);
	}
	const auto yBorderDist = curPosition.y - myTileY;

	action.jump = false;
	
	
	if (nextTileY > myTileY)
	{
		if (nextTileX == myTileX || isOnAir)
		{
			action.jump = true;
		}
		else
		{
			bool isDangerousCorner = false;
			for (int i = 1; i < path.size() - 1; ++i)
			{
				const auto& curStep = path[i];
				if (nextTileX > myTileX)
				{
					if (game.level.tiles[get<0>(path[i]) - 1][get<1>(path[i]) + 2] == WALL &&
						curPosition.x < myTileX + 0.5 - TOLERANCE)
					{
						isDangerousCorner = true;
						break;
					}
				}
				else if (nextTileX < myTileX)
				{
					if (game.level.tiles[get<0>(path[i]) + 1][get<1>(path[i]) + 2] == WALL &&
						curPosition.x > myTileX + 0.5 + TOLERANCE)
					{
						isDangerousCorner = true;
						break;
					}
				}

				const auto& nextStep = path[i + 1];
				if (get<1>(nextStep) <= get<1>(curStep)) break;
				if (nextTileX - myTileX != get<0>(nextStep) - get<0>(curStep)) break;
			}

			action.jump = !isDangerousCorner;
		}
	}
	else if (nextTileY <= myTileY && isJumping &&
		(game.level.tiles[myTileX][myTileY - 1] == EMPTY || game.level.tiles[myTileX][myTileY - 1] == JUMP_PAD) &&
		xBorderDist > yBorderDist)
	{
		action.jump = true;
	}
	

	if (nextTileY < myTileY) action.jumpDown = true;
	else action.jumpDown = false;
}

void initAStarAction(
	const Unit& me, const Vec2Double& targetPos, const Vec2Double& targetSize,
	vector<Vec2Double>& mePositions,
	vector<JumpState>& meJumpStates,
	UnitAction& action,
	Strategy& strategy,
	const Game& game, Debug& debug, bool& isWayFound)
{
	const auto tickTime = 1.0 / game.properties.ticksPerSecond;
	
	const auto endPos =
		make_pair(size_t(targetPos.x), size_t(targetPos.y));

	const auto isOnAir = Simulator::isUnitOnAir(me.position, me.size, me.id, game);
	const auto isJumping = me.jumpState.canJump && me.jumpState.canCancel && me.jumpState.maxTime < game.properties.unitJumpTime - TOLERANCE;
	const auto isFalling = isOnAir && !me.jumpState.canJump && !me.jumpState.canCancel;
	const auto isJumpPadJumping = me.jumpState.canJump && !me.jumpState.canCancel;

	const auto bottomTile = game.level.tiles[size_t(me.position.x)][size_t(me.position.y - 1)];
	const auto maxJumpTiles = static_cast<int>(game.properties.unitJumpTime * game.properties.unitJumpSpeed);
	const auto maxJumpPadJumpTiles = static_cast<int>(game.properties.jumpPadJumpTime * game.properties.jumpPadJumpSpeed);
	
	int start_z = 0;
	
	if (isJumpPadJumping)
	{
		const auto jumpingTime = game.properties.jumpPadJumpTime - me.jumpState.maxTime;
		start_z = static_cast<int>(jumpingTime * game.properties.jumpPadJumpSpeed);
	}
	
	if (isFalling && (bottomTile == EMPTY || bottomTile == JUMP_PAD)) start_z = maxJumpPadJumpTiles + 1;
	else if (isJumping)
	{
		const auto jumpingTime = game.properties.unitJumpTime - me.jumpState.maxTime;
		start_z = static_cast<int>(jumpingTime * game.properties.unitJumpSpeed);
	}
	
	const auto startPos =
		make_tuple(size_t(me.position.x), size_t(me.position.y), start_z, isJumpPadJumping ? 1 : 0);

	const auto path = aStarSearch(
		strategy.grid, game.level.tiles.size(), game.level.tiles[0].size(),
		strategy.closedList, strategy.cellDetails, startPos, endPos, 
		maxJumpTiles, maxJumpPadJumpTiles,
		game, isWayFound);
	if (!isWayFound) return;
	
	auto curPosition = me.position;
	auto curJumpState = me.jumpState;
	mePositions.emplace_back(curPosition);
	meJumpStates.emplace_back(curJumpState);

	if (path.empty()) //��� � ������ �����
	{
		UnitAction curAction;
		const auto dx = targetPos.x - me.position.x;
		if (std::abs(dx) < TOLERANCE) curAction.velocity = 0;
		else if (dx > 0) curAction.velocity = INT_MAX;
		else curAction.velocity = -INT_MAX;

		const auto dy = targetPos.y - me.position.y;
		if (std::abs(dy) < TOLERANCE)
		{
			curAction.jump = false;
			curAction.jumpDown = false;
		}
		else if (dy > 0)
		{
			curAction.jump = true;
			curAction.jumpDown = false;
		}
		else
		{
			curAction.jump = false;
			curAction.jumpDown = true;
		}
		
		curPosition = Simulator::getUnitInTimePosition(curPosition, me.size, me.id, curAction, tickTime, curJumpState, game);

		action = curAction;
		mePositions.emplace_back(curPosition);
		meJumpStates.emplace_back(curJumpState);
		
		return;
	}

	for (int i = 0; i < path.size() - 1; ++i)
	{
		const auto& myTile = path[i];
		const auto& nextTile = path[i + 1];

		UnitAction curAction;
		initOneStepAction(myTile, nextTile, path, curPosition, me.size, curJumpState, me.id, curAction, tickTime, game);
		curPosition = Simulator::getUnitInTimePosition(curPosition, me.size, me.id, curAction, tickTime, curJumpState, game);

		mePositions.emplace_back(curPosition);
		meJumpStates.emplace_back(curJumpState);

		if (i == 0) action = curAction;

		const auto isCross = Simulator::areRectsCross(
			curPosition, me.size, targetPos, targetSize);
		if (isCross) return;
		
		debug.draw(CustomData::Rect(
			vec2DoubleToVec2Float({ static_cast<double>(get<0>(myTile)), static_cast<double>(get<1>(myTile)) }),
			{ 1, 1 },
			ColorFloat(255, 255, 255, 0.2)
		));
	}

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

bool isEnoughTimeToRunaway(const Unit& me, const Unit& enemy, double tickTime, double speed)
{
	if (enemy.weapon == nullptr) return true;

	const auto meToEnemyDist = MathHelper::getVectorLength(me.position, enemy.position);
	const auto runawayTick = enemy.weapon->fireTimer == nullptr ?
		0 :
		static_cast<int>(floor(*(enemy.weapon->fireTimer) / tickTime));

	const auto runawayTime = runawayTick * tickTime;
	return meToEnemyDist + runawayTime * speed > SAFE_ATTACK_DIST;

}

void getAttackingData2(
	const Unit& me,
	vector<Vec2Double>& mePositions,
	vector<JumpState>& meJumpStates,
	UnitAction& meAction,
	const Unit& enemy,
	Strategy& strategy,
	const Game& game,
	Debug& debug)
{
	const auto distToEnemy = MathHelper::getVectorLength(me.position, enemy.position);

	if (me.weapon == nullptr) throw runtime_error("no weapon my unit in getAttackingData2");
	if (enemy.weapon == nullptr) throw runtime_error("no weapon enemy unit in getAttackingData2");

	const auto myFireTimer = me.weapon->fireTimer != nullptr ? *(me.weapon->fireTimer) : 0.0;
	const auto enemyFireTime = enemy.weapon->fireTimer != nullptr ? *(enemy.weapon->fireTimer) : 0.0;

	const auto tickTime = 1.0 / game.properties.ticksPerSecond;

	bool isWayFound;
	const auto canRunaway = isEnoughTimeToRunaway(me, enemy, tickTime, game.properties.unitMaxHorizontalSpeed);
	if (canRunaway)
	{
		initAStarAction(me, enemy.position, enemy.size, mePositions, meJumpStates, meAction, strategy, game, debug, isWayFound);
		if (!isWayFound) throw runtime_error("Failed to find the Destination Cell from" +
			to_string(me.position.x) + ", " + to_string(me.position.y) + " to " +
			to_string(enemy.position.x) + ", " + to_string(enemy.position.y));
		return;
	}

	vector<pair<int, int>> unitWalls;
	for (const auto& unit: game.units)
	{
		if (unit.playerId == me.playerId) continue;
		const int x = static_cast<int>(unit.position.x);
		const int y = static_cast<int>(unit.position.y);
		if (strategy.grid[x][y] == 1) {
			strategy.grid[x][y] = 0;
			unitWalls.emplace_back(make_pair(x, y));
		}
		if (strategy.grid[x][y + 1] == 1)
		{
			strategy.grid[x][y + 1] = 0;
			unitWalls.emplace_back(make_pair(x, y + 1));
		}

		const bool isHigh = unit.position.y + unit.size.y > y + 2;
		
		if (isHigh && strategy.grid[x][y + 2] == 1)
		{
			strategy.grid[x][y + 2] = 0;
			unitWalls.emplace_back(make_pair(x, y + 2));
		}
		
		if (unit.position.x - unit.size.x/2 < x)
		{
			if (strategy.grid[x - 1][y] == 1) {
				strategy.grid[x - 1][y] = 0;
				unitWalls.emplace_back(make_pair(x - 1, y));
			}
			if (strategy.grid[x - 1][y + 1] == 1)
			{
				strategy.grid[x - 1][y + 1] = 0;
				unitWalls.emplace_back(make_pair(x - 1, y + 1));
			}
			if (isHigh && strategy.grid[x - 1][y + 2] == 1)
			{
				strategy.grid[x - 1][y + 2] = 0;
				unitWalls.emplace_back(make_pair(x - 1, y + 2));
			}
		}

		if (unit.position.x + unit.size.x / 2 > x + 1)
		{
			if (strategy.grid[x + 1][y] == 1) {
				strategy.grid[x + 1][y] = 0;
				unitWalls.emplace_back(make_pair(x + 1, y));
			}
			if (strategy.grid[x + 1][y + 1] == 1)
			{
				strategy.grid[x + 1][y + 1] = 0;
				unitWalls.emplace_back(make_pair(x + 1, y + 1));
			}
			if (isHigh && strategy.grid[x + 1][y + 2] == 1)
			{
				strategy.grid[x + 1][y + 2] = 0;
				unitWalls.emplace_back(make_pair(x + 1, y + 2));
			}
		}
	}


	double runawayX;
	int dx = 0;
	int coeff = enemy.position.x <= me.position.x ? 1 : -1;
	int intRunawayY = static_cast<int>(me.position.y);
	auto isOkRunaway = false;

	while (!isOkRunaway)
	{
		runawayX = enemy.position.x + (SAFE_ATTACK_DIST + dx)* coeff;
		if (runawayX > game.level.tiles.size() - SAFE_DIST_TO_BORDER || 
			runawayX < SAFE_DIST_TO_BORDER)
		{
			coeff *= -1;
			dx = 0;
			runawayX = enemy.position.x + (SAFE_ATTACK_DIST + dx) * coeff;
		}		

		const int intRunawayX = static_cast<int>(runawayX);		

		while (intRunawayY < game.level.tiles[0].size())
		{
			const auto thisTile = game.level.tiles[intRunawayX][intRunawayY];
			const auto bottomTile = game.level.tiles[intRunawayX][intRunawayY - 1];
			if (strategy.grid[intRunawayX][intRunawayY] == 1 &&
				(thisTile == EMPTY || thisTile == LADDER) &&
				(bottomTile != EMPTY && bottomTile != JUMP_PAD))
			{
				isOkRunaway = true;
				break;
			}
			intRunawayY++;
		}

		if (!isOkRunaway)
		{
			intRunawayY = static_cast<int>(me.position.y) - 1;
			while (intRunawayY > 1)
			{
				const auto thisTile = game.level.tiles[intRunawayX][intRunawayY];
				const auto bottomTile = game.level.tiles[intRunawayX][intRunawayY - 1];
				if (strategy.grid[intRunawayX][intRunawayY] == 1 &&
					(thisTile == EMPTY || thisTile == LADDER) &&
					(bottomTile != EMPTY && bottomTile != JUMP_PAD))
				{
					isOkRunaway = true;
					break;
				}
				intRunawayY--;
			}
		}
		dx++;
	}

	
	if (isOkRunaway)
	{
		const Vec2Double runawayPos = { runawayX, intRunawayY + 0.5 };

		initAStarAction(me, runawayPos, { 1, 1 }, mePositions, meJumpStates, meAction, strategy, game, debug, isWayFound);
		if (!isWayFound)
		{
			//TODO: ��������� ������ ����� ������
			mePositions.emplace_back(me.position);		
			meJumpStates.emplace_back(me.jumpState);
			meAction.velocity = 0;
			meAction.jump = false;
			meAction.jumpDown = false;

			auto nextTickMeJumpState = me.jumpState;
			const auto nextTickMePosition = Simulator::getUnitInTimePosition(me.position, me.size, me.id,
				meAction, tickTime, nextTickMeJumpState, game);

			mePositions.emplace_back(nextTickMePosition);
			meJumpStates.emplace_back(nextTickMeJumpState);
		}
	}
	else
	{
		mePositions.emplace_back(me.position);
		meJumpStates.emplace_back(me.jumpState);
		meAction.velocity = 0;
		meAction.jump = false;
		meAction.jumpDown = false;

		auto nextTickMeJumpState = me.jumpState;
		const auto nextTickMePosition = Simulator::getUnitInTimePosition(me.position, me.size, me.id,
			meAction, tickTime, nextTickMeJumpState, game);

		mePositions.emplace_back(nextTickMePosition);
		meJumpStates.emplace_back(nextTickMeJumpState);
	}

	for (const auto& uw : unitWalls)
	{
		strategy.grid[uw.first][uw.second] = 1;
	}	
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
				!isMyClosestUnit && //�������� �������,  				
				//� �������� �� ������
					(me.weapon->params.explosion == nullptr ||
						!Strategy::isDangerousRocketShooting(
							lastMePosition, me.size, shootingAngle, me.weapon->spread, me.weapon->params.bullet.size / 2, game)) 				
				)

			{
				vector<Vec2Double> tmpPositions;
				tmpPositions.emplace_back(lastMePosition);
				prolongatePositions(tmpPositions, me.size, me.id, lastMeJumpState, game);

				//���� ����� �����
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

			else if ((!hasWeaponEnemy || isMyClosestUnit) && //�������� �������, ���� ������� ��������
				Simulator::areRectsTouch(lastMePosition, me.size, curEnemyPosition, enemySize) ||
				isMyClosestUnit && isDangerousZone && isEnemyShootEarlier) //��� ���� � ������, � �� �������� ������
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
	const Unit& enemy, const vector<vector<Vec2Double>>& enemyPositions,
	const map<Bullet, BulletSimulation>& enemyBulletsSimulations,
	const Game& game, UnitAction& action, bool isHealing)
{
	if (me.weapon == nullptr)
	{
		action.shoot = false;
		action.aim = { 0,0 };
		return;
	}

	const auto tickTime = 1.0 / game.properties.ticksPerSecond;

	const auto centerBottomTile = game.level.tiles[size_t(me.position.x)][size_t(me.position.y - 0.5)];
	const auto isGoodMinePos = 
		!Simulator::isUnitOnAir(me.position, me.size, me.id, game) &&
		(centerBottomTile == WALL || centerBottomTile == PLATFORM);

	auto areSamePosMines = false;
	for (const auto& mine : game.mines)
	{
		if (std::abs(mine.position.x - me.position.x) < TOLERANCE && std::abs(mine.position.y - me.position.y) < TOLERANCE)
		{
			areSamePosMines = true;
			break;
		}
	}

	if (isGoodMinePos && !areSamePosMines && me.mines >= 2 &&
		(me.weapon->fireTimer == nullptr || *(me.weapon->fireTimer) - 2 * tickTime < 0))
	{
		int meLeftCount = 0;
		int enemyLeftCount = 0;
		int meKilledCount = 0;
		int enemyKilledCount = 0;
		
		
		for (const auto& unit : game.units)
		{
			if (unit.playerId == me.playerId) meLeftCount++;
			else enemyLeftCount++;

			if (unit.id == me.id)
			{
				meKilledCount++;
				continue;
			}

			if (unit.playerId != me.playerId && unit.weapon == nullptr) continue;//�� ��������� ����������

			const auto isShootUnit = Strategy::isMineExplosionShootUnit(me.position, game.properties.mineSize, game.properties.mineExplosionParams.radius,
				unit.position, unit.size,
				game.properties.unitMaxHorizontalSpeed * 2 * tickTime, game.properties.unitJumpSpeed *  2 * tickTime);

			if (isShootUnit)
			{
				if (unit.playerId == me.playerId) meKilledCount++;
				else enemyKilledCount++;
			}
		}

		meLeftCount -= meKilledCount;
		enemyLeftCount -= enemyKilledCount;

		if (enemyKilledCount > 0 && meLeftCount >= enemyLeftCount)
		{
			action.plantMine = true;
			action.aim = { 0, -1 };
			action.shoot = false;
			return;
		}
	}
	
	
	if (isGoodMinePos && me.mines > 0 &&
		(me.weapon->fireTimer == nullptr || *(me.weapon->fireTimer) - tickTime < 0))
	{
		int meLeftCount = 0;
		int enemyLeftCount = 0;
		int meKilledCount = 0;
		int enemyKilledCount = 0;
		int meDamagedCount = 0;
		int enemyDamagedCount = 0;
		
		for (const auto& unit: game.units)
		{
			if (unit.playerId == me.playerId) meLeftCount++;
			else enemyLeftCount++;
			//if (!areSamePosMines && unit.health > game.properties.mineExplosionParams.damage) continue;			
			
			if (unit.id == me.id)
			{
				if (areSamePosMines || unit.health <= game.properties.mineExplosionParams.damage)
					meKilledCount++;
				else
					meDamagedCount++;
				continue;
			}

			if (unit.playerId != me.playerId && unit.weapon == nullptr) continue;//�� ��������� ����������

			const auto isShootUnit = Strategy::isMineExplosionShootUnit(me.position, game.properties.mineSize, game.properties.mineExplosionParams.radius,
				unit.position, unit.size,
				game.properties.unitMaxHorizontalSpeed * tickTime, game.properties.unitJumpSpeed * tickTime);		

			if (isShootUnit)
			{
				if (unit.playerId == me.playerId)
				{
					if (areSamePosMines || unit.health <= game.properties.mineExplosionParams.damage)
						meKilledCount++;
					else
						meDamagedCount++;
				}
				else
				{
					if (areSamePosMines || unit.health <= game.properties.mineExplosionParams.damage)
						enemyKilledCount++;
					else
						enemyDamagedCount++;
				}
			}			
		}

		meLeftCount -= meKilledCount;
		enemyLeftCount -= enemyKilledCount;

		if (meLeftCount >= enemyLeftCount &&
			(enemyKilledCount > 0 || enemyDamagedCount > meDamagedCount))
		{
			action.plantMine = true;
			action.aim = { 0, -1 };
			action.shoot = true;
			return;
		}
	}	

	if (isGoodMinePos && me.mines > 0 && isHealing)
	{
		double myOtherUnitDist = INT_MAX;
		for (const auto& unit : game.units)
		{
			if (unit.playerId != me.playerId) continue;
			if (unit.id == me.id) continue;
			const auto dist = MathHelper::getMHDist(me.position, unit.position);
			if (dist < myOtherUnitDist) myOtherUnitDist = dist;
		}
		const auto isSafeOtherUnitPlantMine = myOtherUnitDist > SAFE_OTHER_UNIT_PLANT_MINE_DIST;

		bool isSafeBulletsPlantMine = true;
		const auto& mineSize = game.properties.mineSize;
		for (const auto& ebs : enemyBulletsSimulations)
		{
			const auto& bullet = ebs.first;
			const auto& simulation = ebs.second;

			Vec2Double crossPoint;
			Vec2Double bulletCorner;
			const auto isShooting = Simulator::getBulletRectangleFirstCrossPoint(
				bullet.position, bullet.velocity, bullet.size / 2,
				me.position.x - mineSize.x / 2, me.position.y, me.position.x + mineSize.x / 2, me.position.y + mineSize.y,
				crossPoint, bulletCorner);
			if (!isShooting) continue;

			if (MathHelper::getVectorLength2(bulletCorner, crossPoint) <=
				MathHelper::getVectorLength2(simulation.bulletCrossCorner, simulation.targetCrossPoint))
			{
				isSafeBulletsPlantMine = false;
				break;
			}
		}

		auto isFarFromOtherMines = true;
		for (const auto& mine : game.mines)
		{
			const auto dist = MathHelper::getMHDist(mine.position, me.position);
			if (dist < MIN_SET_MINE_DIST)
			{
				isFarFromOtherMines = false;
				break;
			}
		}

		if (isSafeOtherUnitPlantMine && isSafeBulletsPlantMine && isFarFromOtherMines)
		{
			action.plantMine = true;
		}
	}
		
	
	const auto movingTime = me.weapon->fireTimer != nullptr ? *(me.weapon->fireTimer) - TOLERANCE : 0;
	
	//const auto canShootingTick = static_cast<int>(ceil(movingTime / tickTime));


	double maxProbability = 0.0;
	double okShootingAngle = 0.0;
	double okShootingSpread = 0.0;

	Vec2Double meShootingPosition;

	if (movingTime < tickTime)
	{
		const auto part = movingTime / tickTime;
		auto x = mePositions[0].x + (mePositions[1].x - mePositions[0].x) * part;
		auto y = mePositions[0].y + (mePositions[1].y - mePositions[0].y) * part;
		meShootingPosition = Vec2Double(x, y);//�������, ������ ���������� �������
	}
	else meShootingPosition = mePositions[1];

	bool canRunaway = isEnoughTimeToRunaway(me, enemy, tickTime, game.properties.unitMaxHorizontalSpeed);
	
	const auto enemyShootingPositionsIndex = static_cast<int>(round(movingTime / tickTime));
	const auto enemyShootingPositions = enemyPositions[enemyShootingPositionsIndex]; //������� �����, ������� � ���� ��������
		
	const auto isCloseToEnemy =
		Simulator::areRectsTouch(meShootingPosition, me.size, enemyShootingPositions[0], enemy.size) ||
		Simulator::areRectsCross(meShootingPosition, me.size, enemyShootingPositions[0], enemy.size);
	
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
			enemy.size,
			game);

		if (probability > maxProbability)
		{
			maxProbability = probability;
			okShootingAngle = shootingAngle;
			okShootingSpread = spread;
		}
	}

	if (movingTime < tickTime && (!canRunaway || isCloseToEnemy) && maxProbability > NOT_BAD_SHOOTING_PROBABILITY)
	{
		bool isDangerousShoot = false;
		if (me.weapon->params.explosion != nullptr &&
			!Simulator::areRectsTouch(me.position, me.size, enemyShootingPositions[0], enemy.size))
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
		action.shoot = false;

	action.aim = maxProbability > TOLERANCE ?
		Vec2Double(cos(okShootingAngle), sin(okShootingAngle)) :
		enemyPositions[1][0] - mePositions[1];		

		
	
	//auto maxShootingProbability = 0.0;
	//double okShootingAngle = 0;
	//double okShootingSpread = 0;
	//
	//int okAddShootingSimulations = -1;
	//

	//map<int, double> allProbabilities;
	//map<int, double> allShootingAngles;
	//map<int, double> allSpreads;


	//const auto& meShootingPosition = mePositions.second;//�������, ������ ���������� �������
	//int addShootingSimulations = 0;
	//while (canShootingTick + addShootingSimulations < MAX_SIMULATIONS)
	//{
	//	double curMaxShootingProbability = 0.0;
	//	double curOkShootingAngle = 0.0;
	//	double curSpread = 0.0;
	//	
	//	const auto shootingTick = canShootingTick + addShootingSimulations;
	//	
	//	double meSimpleProbability;
	//	
	//	if (shootingTick >= meSimpleProbabilities.size())
	//	{
	//		if (addShootingSimulations > 0) break;//����� �� �����. ��� ������ ������������ ������
	//		meSimpleProbability = meSimpleProbabilities.back();
	//	}
	//	else
	//	{
	//		meSimpleProbability = meSimpleProbabilities[shootingTick];
	//	}
	//	
	//	if (abs(meSimpleProbability) < TOLERANCE)
	//	{
	//		addShootingSimulations++;
	//		continue;
	//	}		
	//	
	//	const auto enemyShootingPositions = enemyPositions[shootingTick]; //������� �����, ������� � ���� ��������
	//	double minAngle = INT_MAX;
	//	double maxAngle = -INT_MAX;
	//	for (const auto& ep : enemyShootingPositions)
	//	{
	//		const auto shootingVector = ep - meShootingPosition;
	//		double shootingAngle;
	//		if (abs(shootingVector.x) > TOLERANCE) {
	//			shootingAngle = atan2(shootingVector.y, shootingVector.x);
	//		}
	//		else
	//		{
	//			shootingAngle = shootingVector.y > 0 ? M_PI/2 : -M_PI/2;
	//		}
	//		if (shootingAngle < minAngle) minAngle = shootingAngle;
	//		if (shootingAngle > maxAngle) maxAngle = shootingAngle;
	//	}

	//	int directionsCount;
	//	double deltaAngle;
	//	if (abs(maxAngle - minAngle) < TOLERANCE)
	//	{
	//		directionsCount = 1;
	//		deltaAngle = 0;
	//	}
	//	else
	//	{
	//		directionsCount = ANGLE_SPLIT_COUNT;
	//		deltaAngle = (maxAngle - minAngle) / (ANGLE_SPLIT_COUNT - 1);
	//	}
	//	
	//	for (int i = 0; i < directionsCount; ++i)
	//	{
	//		const auto shootingAngle = minAngle + i * deltaAngle;
	//		double spread;
	//		if (me.weapon->lastAngle == nullptr)
	//		{
	//			spread = me.weapon->spread;
	//		}
	//		else
	//		{	
	//			spread = me.weapon->spread + abs(*(me.weapon->lastAngle) - shootingAngle);
	//			spread = min(spread, me.weapon->params.maxSpread);
	//			spread = max(
	//				me.weapon->params.minSpread, 
	//				spread - me.weapon->params.aimSpeed*(shootingTick / game.properties.ticksPerSecond));
	//		}

	//		const auto probability = Strategy::getShootEnemyProbability(
	//			meShootingPosition,
	//			me.size,
	//			shootingAngle,
	//			spread,
	//			me.weapon->params,
	//			enemyShootingPositions,
	//			enemySize,
	//			game);			

	//		if (probability > curMaxShootingProbability)
	//		{
	//			curMaxShootingProbability = probability;
	//			curOkShootingAngle = shootingAngle;
	//			curSpread = spread;
	//		}	
	//	}

	//	allProbabilities[shootingTick] = curMaxShootingProbability;
	//	allShootingAngles[shootingTick] = curOkShootingAngle;
	//	allSpreads[shootingTick] = curSpread;

	//	if (curMaxShootingProbability > maxShootingProbability)
	//	{
	//		maxShootingProbability = curMaxShootingProbability;
	//		okShootingAngle = curOkShootingAngle;
	//		okShootingSpread = curSpread;
	//		okAddShootingSimulations = addShootingSimulations;
	//		if (maxShootingProbability >= OK_SHOOTING_PROBABILITY) break;
	//	}

	//	addShootingSimulations++;
	//}					

	//if (maxShootingProbability >= NOT_BAD_SHOOTING_PROBABILITY) 
	//{
	//	while (true)
	//	{
	//		auto okAddShootingSimulations2 = -1;
	//		auto maxProb2 = NOT_BAD_SHOOTING_PROBABILITY;
	//		auto addShootingSimulations2 = 0;
	//		auto okShootingAngle2 = 0.0;
	//		auto okShootingSpread2 = 0.0;

	//		const auto shootDelay = static_cast<int>(ceil((me.weapon->params.fireRate - TOLERANCE) / tickTime));
	//		while (canShootingTick + addShootingSimulations2 < canShootingTick + okAddShootingSimulations - shootDelay)
	//		{
	//			const auto shootingTick2 = canShootingTick + addShootingSimulations2;
	//			const auto prob2 = allProbabilities[shootingTick2];
	//			if (prob2 < maxProb2 + TOLERANCE)
	//			{
	//				addShootingSimulations2++;
	//				continue;
	//			}
	//			const auto angle2 = allShootingAngles[shootingTick2];


	//			const auto bestShootingTick = canShootingTick + okAddShootingSimulations;
	//			//�������, ������ ���������� �������
	//			Vec2Double meShootingPosition =
	//				bestShootingTick >= mePositions.size() ?
	//				mePositions.back() :
	//				mePositions[bestShootingTick];

	//			auto spread = allSpreads[shootingTick2];//������� � ������ ������� ��������
	//			spread = min(me.weapon->params.maxSpread, spread + me.weapon->params.recoil);//������
	//			spread = min(me.weapon->params.maxSpread, spread + abs(angle2 - okShootingAngle));//����� ������������
	//			//��������� ��������
	//			spread = max(
	//				me.weapon->params.minSpread,
	//				spread - me.weapon->params.aimSpeed * ((bestShootingTick - shootingTick2) / game.properties.ticksPerSecond));

	//			const auto enemyShootingPositions = enemyPositions[bestShootingTick]; //������� �����, ������� � ���� ��������
	//			const auto newBestShootProbability = Strategy::getShootEnemyProbability(
	//				meShootingPosition,
	//				me.size,
	//				okShootingAngle,
	//				spread,
	//				me.weapon->params,
	//				enemyShootingPositions,
	//				enemySize,
	//				game);

	//			if (newBestShootProbability > maxShootingProbability - TOLERANCE)
	//			{
	//				okAddShootingSimulations2 = addShootingSimulations2;
	//				maxProb2 = prob2;
	//				okShootingAngle2 = angle2;
	//				okShootingSpread2 = spread;
	//			}

	//			addShootingSimulations2++;
	//		}

	//		if (okAddShootingSimulations2 != -1)
	//		{
	//			okAddShootingSimulations = okAddShootingSimulations2;
	//			maxShootingProbability = maxProb2;
	//			okShootingAngle = okShootingAngle2;
	//			okShootingSpread = okShootingSpread2;
	//			
	//			/*action.shoot = canShootingTick == 0 && okAddShootingSimulations2 == 0;
	//			action.aim = Vec2Double(cos(okShootingAngle2), sin(okShootingAngle2));*/
	//		}
	//		else
	//		{
	//			if (canShootingTick == 0 && okAddShootingSimulations == 0)
	//			{
	//				bool isDangerousShoot = false;
	//				if (me.weapon->params.explosion != nullptr &&
	//					!Simulator::areRectsTouch(me.position, me.size, enemyPositions[0][0], enemySize))
	//				{
	//					isDangerousShoot = Strategy::isDangerousRocketShooting(
	//						me.position, me.size,
	//						okShootingAngle, okShootingSpread,
	//						me.weapon->params.bullet.size / 2, game);
	//				}
	//				
	//				const auto isSafe = isSafeShoot(me, game);
	//				if (!isDangerousShoot && isSafe)
	//				{
	//					action.shoot = true;
	//				}
	//				else
	//				{
	//					action.shoot = false;
	//				}
	//			}
	//			else
	//			{
	//				action.shoot = false;
	//			}
	//			
	//			action.aim = Vec2Double(cos(okShootingAngle), sin(okShootingAngle));
	//			break;
	//		}
	//	}
	//}	
	////else if (mePositions.size() == 1 && maxShootingProbability > TOLERANCE)// ������ � ��� �� �����
	////{
	////	action.shoot = canShootingTick == 0;
	////	action.aim = Vec2Double(cos(okShootingAngle), sin(okShootingAngle));
	////}
	//else
	//{				
	//	action.shoot = false;
	//	const auto enemyPos = enemyPositions[canShootingTick][0];
	//	const auto mePos = canShootingTick < mePositions.size() ? mePositions[canShootingTick] : mePositions.back();
	//	action.aim = enemyPos - mePos;
	//}				

	//TODO: ���������� �������� ��� ���� �����
}

void initAttackAction(
	const Unit& unit,
	vector<Vec2Double>& meAttackingPositions, const vector<JumpState>& meAttackingJumpStates,
	const JumpState& nextTickMeAttackingJumpState, const Vec2Double& nextTickMeAttackPosition,
	vector<double>& meSimpleProbabilities,
	const vector<vector<Vec2Double>>& enemyPositions, const Unit& enemy,
	int startJumpY, int jumpingUnitId, bool isMonkeyMode,
	tuple<RunawayDirection, int, int, int> runawayAction,
	const UnitAction& meAttackingAction, UnitAction& action, Strategy& strategy,
	const map<Bullet, BulletSimulation>& enemyBulletsSimulations,
	const Game& game,
	bool isHealing)
{
	const auto runawayDirection = std::get<0>(runawayAction);
	const auto runawayStartTick = std::get<1>(runawayAction);
	const auto runawayStopTick = std::get<2>(runawayAction);
	const auto minDamage = std::get<3>(runawayAction);
	
	action.jump = meAttackingAction.jump;
	action.jumpDown = meAttackingAction.jumpDown;
	action.velocity = meAttackingAction.velocity;

	auto lastMeAttackingJumpState = meAttackingJumpStates.back();

	if (runawayDirection != GoNONE)//����� �������� �������
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
		const auto sp = getSimpleProbability(mePosition, unit.size, curEnemyPositions, enemy.size, game);
		meSimpleProbabilities.emplace_back(sp);
	}

	setShootingAction(
		unit, meAttackingPositions, meSimpleProbabilities, enemy, enemyPositions, enemyBulletsSimulations, game, action, isHealing);
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
		const auto cols = game.level.tiles.size();
		const auto rows = game.level.tiles[0].size();
		strategy_.grid = getGrid(game, cols, rows);

		const auto maxJumpTiles = static_cast<int>(game.properties.jumpPadJumpTime * game.properties.jumpPadJumpSpeed);
		const auto Z_SIZE = maxJumpTiles + 2; //+1 - �� �������, +1 - �� �������
		const auto PAD_JUMP_STATE_SIZE = 2;

		bool**** closedList = new bool***[cols];
		cell**** cellDetails = new cell***[cols];
		
		for (int i = 0; i < cols; ++i)
		{
			closedList[i] = new bool**[rows];
			cellDetails[i] = new cell**[rows];
			for (int j = 0; j < rows; ++j)
			{
				closedList[i][j] = new bool*[Z_SIZE];
				cellDetails[i][j] = new cell*[Z_SIZE];
				for (int k = 0; k < Z_SIZE; ++k)
				{
					closedList[i][j][k] = new bool[PAD_JUMP_STATE_SIZE];
					cellDetails[i][j][k] = new cell[PAD_JUMP_STATE_SIZE];
					for (int l = 0; l < PAD_JUMP_STATE_SIZE; ++l)
					{
						closedList[i][j][k][l] = false;
					}
				}
			}
		}		
		strategy_.closedList = closedList;
		strategy_.cellDetails = cellDetails;
	}
	

	if (strategy_.getJumpingUnitId() == unit.id) strategy_.setJumpingUnitId(-1);
	for (auto it = strategy_.heal_boxes_.begin(); it != strategy_.heal_boxes_.end();)
	{
		if (it->first == unit.id) {
			strategy_.heal_boxes_.erase(it++);
		}
		else ++it;
	}

	UnitAction action;
	action.aim = Vec2Double(0, 0);
	action.reload = false;
	action.swapWeapon = false;
	action.plantMine = false;
	action.shoot = false;

	if (game.properties.teamSize == 2 && game.currentTick < 9)
	{
		bool isFarUnit = false;
		for (const auto& u: game.units)
		{
			if (u.playerId != unit.playerId) continue;
			if (u.id == unit.id) continue;
			if (u.id > unit.id)
			{
				isFarUnit = true;
				break;
			}
		}
		if (isFarUnit)
		{
			action.velocity = 0;
			action.jump = false;
			action.jumpDown = false;
			return action;
		}
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
	

	/*if (game.currentTick < 451)
	{
		action.velocity = 0;
		action.jump = false;
		action.jumpDown = false;
		return action;
	}*/


	vector<Vec2Double> meAttackingPositions;
	vector<JumpState> meAttackingJumpStates;
	UnitAction meAttackingAction;	

	//if (unit.weapon == nullptr) {
		/*initAStarAction(unit, nearestWeapon->position, nearestWeapon->size,
			meAttackingPositions, meAttackingJumpStates, meAttackingAction, strategy_, game, debug);
		action.velocity = meAttackingAction.velocity;
		action.jump = meAttackingAction.jump;
		action.jumpDown = meAttackingAction.jumpDown;
		return action;*/
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
			enemyFireTick) || //monkeyMode �� �����
		(!unit.jumpState.canJump && !unit.jumpState.canCancel) || //������ �������
		!Simulator::isUnitOnAir(unit.position, unit.size, unit.id, game))) //� �������� �� �����
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
		setShootingAction(unit, mePositions,meSimpleProbabilities, *nearestEnemy, enemyPositions, enemyBulletsSimulation, game, action, false);

		strategy_.decreaseStopRunawayTick(unit.id);
		if (action.plantMine) debug.draw(CustomData::Log("MINE"));
		return action;
	}

	//setMoveToEnemyAction(unit, nearestEnemy->position, needGo, game, action, strategy_);
	
	auto startJumpY = strategy_.getStartedJumpY(unit.id);
	auto jumpingUnitId = strategy_.getJumpingUnitId();

	auto isHealing = false;
	bool isWayFound;

	if (unit.weapon == nullptr)
	{
		const LootBox* nearestWeapon = nullptr;
		for (const LootBox& lootBox : game.lootBoxes)
		{
			if (std::dynamic_pointer_cast<Item::Weapon>(lootBox.item))
			{
				auto isOtherUnitWeapon = false;
				for (const auto& item : strategy_.lootboxes_)
				{
					if (item.first != unit.id &&
						std::abs(item.second.x - lootBox.position.x) < TOLERANCE &&
						std::abs(item.second.y - lootBox.position.y) < TOLERANCE)
					{
						isOtherUnitWeapon = true;
						break;
					}
				}
				if (isOtherUnitWeapon) continue;

				if (nearestWeapon == nullptr ||
					MathHelper::getVectorLength2(unit.position, lootBox.position) <
					MathHelper::getVectorLength2(unit.position, nearestWeapon->position))
				{
					nearestWeapon = &lootBox;
				}
			}
		}
		strategy_.lootboxes_[unit.id] = nearestWeapon->position;
		
		initAStarAction(
			unit, nearestWeapon->position, nearestWeapon->size, 
			meAttackingPositions, meAttackingJumpStates, meAttackingAction,
			strategy_,
			game, debug, isWayFound);

		if (!isWayFound) throw runtime_error("Failed to find the Destination Cell from" +
			to_string(unit.position.x) + ", " + to_string(unit.position.y) + " to " +
			to_string(nearestWeapon->position.x) + ", " + to_string(nearestWeapon->position.y));
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
					if (dist < minMHDist)
					{
						minMHDist = dist;
						nearestHPLootBox = &lb;
					}				
				}
			}
		}
		
		if (nearestHPLootBox != nullptr)
		{
			isHealing = true;
			vector<Vec2Double> curMeAttackingPositions;
			vector<JumpState> curMeAttackingJumpStates;
			UnitAction curMeAttackingAction;
			size_t curStartJumpY = startJumpY;
			int curJumpingUnitId = jumpingUnitId;
			/*getHealingData(
				unit, curMeAttackingPositions, curMeAttackingJumpStates,
				lb, curMeAttackingAction, curStartJumpY, curJumpingUnitId, game);*/
			initAStarAction(
				unit,  nearestHPLootBox->position, nearestHPLootBox->size,
				curMeAttackingPositions, curMeAttackingJumpStates, curMeAttackingAction,
				strategy_,
				game, debug, isWayFound);
			if (!isWayFound) throw runtime_error("Failed to find the Destination Cell from" +
				to_string(unit.position.x) + ", " + to_string(unit.position.y) + " to " +
				to_string(nearestHPLootBox->position.x) + ", " + to_string(nearestHPLootBox->position.y));
						
			for (size_t i = 1; i < curMeAttackingPositions.size(); ++i)
			{
				const auto& pos = curMeAttackingPositions[i];
				for (const auto& enemyUnit : game.units)
				{
					if (enemyUnit.playerId == unit.playerId) continue;
					if (Simulator::areRectsTouch(pos, unit.size, enemyUnit.position, enemyUnit.size))
					{
						isHealing = false;
						break;
					}
				}
				if (!isHealing) break;
			}

			if (isHealing)
			{
				meAttackingPositions = curMeAttackingPositions;
				meAttackingJumpStates = curMeAttackingJumpStates;
				meAttackingAction = curMeAttackingAction;
				startJumpY = curStartJumpY;
				jumpingUnitId = curJumpingUnitId;

				strategy_.heal_boxes_[unit.id] = nearestHPLootBox->position;
				
			}
		}

		if (!isHealing)
		{
			const Unit* noWeaponEnemyUnit = nullptr;
			minMHDist = INT_MAX;
			for (const auto& u:game.units)
			{
				if (u.playerId == unit.playerId) continue;
				if (u.weapon != nullptr) continue;
				const auto dist = MathHelper::getMHDist(unit.position, u.position);
				if (dist < minMHDist)
				{
					minMHDist = dist;
					noWeaponEnemyUnit = &u;
				}
			}

			if (noWeaponEnemyUnit != nullptr)
			{
				initAStarAction(
					unit, noWeaponEnemyUnit->position, noWeaponEnemyUnit->size,
					meAttackingPositions, meAttackingJumpStates, meAttackingAction,
					strategy_,
					game, debug, isWayFound);
				if (!isWayFound) throw runtime_error("Failed to find the Destination Cell from" +
					to_string(unit.position.x) + ", " + to_string(unit.position.y) + " to " +
					to_string(noWeaponEnemyUnit->position.x) + ", " + to_string(noWeaponEnemyUnit->position.y));

			}
			else
			{

				/*const LootBox* nearestMine = nullptr;
				minMHDist = INT_MAX;
				for (const LootBox& lootBox : game.lootBoxes)
				{
					if (std::dynamic_pointer_cast<Item::Mine>(lootBox.item))
					{
						auto isOtherUnitMine = false;
						for (const auto& item : strategy_.lootboxes_)
						{
							if (item.first != unit.id &&
								std::abs(item.second.x - lootBox.position.x) < TOLERANCE &&
								std::abs(item.second.y - lootBox.position.y) < TOLERANCE)
							{
								isOtherUnitMine = true;
								break;
							}
						}
						if (isOtherUnitMine) continue;

						if (nearestMine == nullptr ||
							MathHelper::getVectorLength2(unit.position, lootBox.position) <
							MathHelper::getVectorLength2(unit.position, nearestMine->position))
						{
							nearestMine = &lootBox;
						}
					}
				}
				if (nearestMine != nullptr)
				{
					strategy_.lootboxes_[unit.id] = nearestMine->position;
					initAStarAction(
						unit, nearestMine->position, nearestMine->size,
						meAttackingPositions, meAttackingJumpStates, meAttackingAction,
						strategy_,
						game, debug, isWayFound);

					if (!isWayFound) throw runtime_error("Failed to find the Destination Cell from" +
						to_string(unit.position.x) + ", " + to_string(unit.position.y) + " to " +
						to_string(nearestMine->position.x) + ", " + to_string(nearestMine->position.y));
				}
				else				
				{*/
					getAttackingData2(
						unit, meAttackingPositions, meAttackingJumpStates, meAttackingAction, *nearestEnemy, strategy_, game, debug);
				//}
			}
		}
	}
	
	

	tuple<RunawayDirection, int, int, int> attackRunawayAction;
	int minesDamage;
	const auto thisTickShootMeBullets = strategy_.isSafeMove(unit, meAttackingAction, enemyBulletsSimulation, game, minesDamage);
	int minAttackDamage = minesDamage;
	
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
	const auto nextTickShootMeMines = strategy_.getShootMeMines(nextTickMeAttackPosition, unit.size, 1, game);
	
	
	attackRunawayAction = strategy_.getRunawayAction(
		nextTickMeAttackPosition, unit.size, unit.playerId, unit.id, nextTickMeAttackingJumpState,
		nextTickShootMeBullets, nextTickShootMeMines, nextTickEnemyBulletsSimulation, 1,
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
			enemyPositions, *nearestEnemy, startJumpY, jumpingUnitId, isMonkeyMode,
			attackRunawayAction, meAttackingAction, action, strategy_, enemyBulletsSimulation, game, isHealing);
		//cerr << game.currentTick << ": (" << unit.id << "-0) " << action.jump << " " << action.jumpDown << " " << action.velocity << "\n";
		if (action.plantMine) debug.draw(CustomData::Log("MINE"));
		return action;			
	}				
	

	//���� �� ����� ������. ������� ������
	
	bool checkUp = true;
	bool checkDown = true;
	bool checkLeft = true;
	bool checkRight = true;

	if (meAttackingAction.jump && std::abs(meAttackingAction.velocity) < TOLERANCE) checkUp = false;
	else if (meAttackingAction.jumpDown && std::abs(meAttackingAction.velocity) < TOLERANCE) checkDown = false;
	else if (!meAttackingAction.jump && !meAttackingAction.jumpDown && meAttackingAction.velocity < -TOLERANCE) checkLeft = false;
	else if (!meAttackingAction.jump && !meAttackingAction.jumpDown && meAttackingAction.velocity > TOLERANCE) checkRight = false;

	// �������� ��� ������ ��������, �� ����� ������
	action.jump = false;
	action.jumpDown = false;
	action.velocity = 0;

	const auto shootMeBullets = strategy_.getShootMeBullets(
		unit.position, unit.size, unit.jumpState, unit.playerId, unit.id,
		enemyBulletsSimulation, 0, game);
	const auto shootMeMines = strategy_.getShootMeMines(
		unit.position, unit.size,0, game);
	
	const auto noAttackRunawayAction = strategy_.getRunawayAction(
		unit.position, unit.size, unit.playerId, unit.id, unit.jumpState,
		shootMeBullets, shootMeMines, enemyBulletsSimulation, 0,
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
			enemyPositions, *nearestEnemy, startJumpY, jumpingUnitId, isMonkeyMode,
			attackRunawayAction, meAttackingAction, action, strategy_, enemyBulletsSimulation, game, isHealing);
		//cerr << game.currentTick << ": (" << unit.id << "-1) " << action.jump << " " << action.jumpDown << " " << action.velocity << "\n";
		if (action.plantMine) debug.draw(CustomData::Log("MINE"));
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
	vector<Vec2Double> mePositions;
	if (runawayDirection == GoNONE)
	{
		mePositions.emplace_back(unit.position);
		auto nextJumpState = unit.jumpState;
		const auto nextPosition = Simulator::getUnitInTimePosition(unit.position, unit.size, unit.id, action, tickTime, jumpState, game);
		mePositions.emplace_back(nextPosition);
	}
	else
		mePositions = getActionPositions(unit.position, unit.size, unit.id, runawayUnitAction, startRunawayTick, stopRunawayTick, jumpState, game);
		
	
	prolongatePositions(mePositions, unit.size, unit.id, jumpState, game);
	
	for (size_t i = 0; i < mePositions.size(); ++i)
	{
		const auto& mePosition = mePositions[i];
		const auto& curEnemyPositions = enemyPositions[i];
		const auto sp = getSimpleProbability(mePosition, unit.size, curEnemyPositions, nearestEnemy->size, game);
		meSimpleProbabilities.emplace_back(sp);
	}
	
	setShootingAction(unit, mePositions, meSimpleProbabilities, *nearestEnemy, enemyPositions, enemyBulletsSimulation, game, action, false);
	//cerr << game.currentTick << ": (" <<unit.id << "-2) " << action.jump << " " << action.jumpDown << " " << action.velocity <<  "\n";
	if (action.plantMine) debug.draw(CustomData::Log("MINE"));
	return action;
}

