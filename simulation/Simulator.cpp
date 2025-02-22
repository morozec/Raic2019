#include "Simulator.h"
#include "../mathcalc/MathHelper.h"
#include "../common/Helper.h"
#include "../model/Game.hpp"
#include "../strategy/ShootMeBulletCrossPoint.h"
#include "../model/UnitAction.hpp"
#include <algorithm>
#include <climits>
#include <cmath>


bool Simulator::canGoThroughTile(const Tile& tile)
{
	return tile != WALL;
}

Vec2Double Simulator::getBulletBorderCross(const Vec2Double& bulletPos, const Vec2Double& bulletVelocity, const Game& game)
{
	const double minX = 1;
	const double minY = 1;
	const double maxX = game.level.tiles.size() - 1;
	const double maxY = game.level.tiles[0].size() - 1;

	const auto x0 = bulletPos.x;
	const auto y0 = bulletPos.y;
	const auto x1 = bulletPos.x + bulletVelocity.x;
	const auto y1 = bulletPos.y + bulletVelocity.y;
	
	if (bulletVelocity.x > 0)
	{
		const auto vertCross = MathHelper::getLinesCross(
			x0, y0, x1, y1, maxX, minY, maxX, maxY);
		if (MathHelper::IsBetween(vertCross, { maxX, minY }, { maxX, maxY }))
		{
			return vertCross;
		}

		return bulletVelocity.y > 0 ?
			MathHelper::getLinesCross(x0, y0, x1, y1, minX, maxY, maxX, maxY) :
			MathHelper::getLinesCross(x0, y0, x1, y1, minX, minY, maxX, minY);
	}
	else
	{
		const auto vertCross = MathHelper::getLinesCross(
			x0, y0, x1, y1, minX, minY, minX, maxY);
		if (MathHelper::IsBetween(vertCross, { minX, minY }, { minX, maxY }))
		{
			return vertCross;
		}

		return bulletVelocity.y > 0 ?
			MathHelper::getLinesCross(x0, y0, x1, y1, minX, maxY, maxX, maxY) :
			MathHelper::getLinesCross(x0, y0, x1, y1, minX, minY, maxX, minY);
	}
}

bool Simulator::getBulletPointRectangleFirstCrossPoint(const Vec2Double& bulletPos, const Vec2Double& bulletVelocity,
	double xLeft, double yDown, double xRight, double yUp, Vec2Double& crossPoint, double& minDist2)
{
	bool hasCross = false;

	const auto x0 = bulletPos.x;
	const auto y0 = bulletPos.y;
	const auto x1 = bulletPos.x + bulletVelocity.x;
	const auto y1 = bulletPos.y+ bulletVelocity.y;
	
	
	const auto cpLeft = MathHelper::getLinesCross(x0, y0, x1, y1,
		xLeft, yDown, xLeft, yUp);
	if ((cpLeft.x - x0) * (x1- x0) > 0 &&
		(cpLeft.y - y0) * (y1 - y0) > 0 &&
		MathHelper::IsBetween(xLeft, cpLeft.y, xLeft, yDown, xLeft, yUp))
	{
		hasCross = true;
		const auto dist2 = MathHelper::getVectorLength2(bulletPos, cpLeft);
		if (dist2 < minDist2)
		{
			minDist2 = dist2;
			crossPoint = cpLeft;
		}
	}

	const auto cpUp = MathHelper::getLinesCross(x0, y0, x1, y1,
		xLeft, yUp, xRight, yUp);
	if ((cpUp.x - x0) * (x1 - x0) > 0 &&
		(cpUp.y - y0) * (y1 - y0) > 0 &&
		MathHelper::IsBetween(cpUp.x, yUp, xLeft, yUp, xRight, yUp))
	{
		hasCross = true;
		const auto dist2 = MathHelper::getVectorLength2(bulletPos, cpUp);
		if (dist2 < minDist2)
		{
			minDist2 = dist2;
			crossPoint = cpUp;
		}
	}

	const auto cpRight = MathHelper::getLinesCross(x0, y0, x1, y1,
		xRight, yUp, xRight, yDown);
	if ((cpRight.x - x0) * (x1 - x0) > 0 &&
		(cpRight.y - y0) * (y1 - y0) > 0 &&
		MathHelper::IsBetween(xRight, cpRight.y, xRight, yUp, xRight, yDown))
	{
		hasCross = true;
		const auto dist2 = MathHelper::getVectorLength2(bulletPos, cpRight);
		if (dist2 < minDist2)
		{
			minDist2 = dist2;
			crossPoint = cpRight;
		}
	}

	const auto cpDown = MathHelper::getLinesCross(x0, y0, x1, y1,
		xRight, yDown, xLeft, yDown);
	if ((cpDown.x - x0) * (x1 - x0) > 0 &&
		(cpDown.y - y0) * (y1 - y0) > 0 &&
		MathHelper::IsBetween(cpDown.x, yDown, xRight, yDown, xLeft, yDown))
	{
		hasCross = true;
		const auto dist2 = MathHelper::getVectorLength2(bulletPos, cpDown);
		if (dist2 < minDist2)
		{
			minDist2 = dist2;
			crossPoint = cpDown;
		}
	}

	return hasCross;
}



bool Simulator::getBulletInTimePosition(
	const Vec2Double& bulletPosition, const Vec2Double& bulletVelocity, double time, double targetCrossTime, const Game& game,
	Vec2Double& position)
{
	if (time > targetCrossTime) {
		position = Vec2Double(
			bulletPosition.x + bulletVelocity.x * targetCrossTime,
			bulletPosition.y + bulletVelocity.y * targetCrossTime);
		return false;
	}
	position = bulletPosition + bulletVelocity * time;
	return true;
}

BulletSimulation Simulator::getBulletSimulation(
	const Vec2Double& bulletPosition, const Vec2Double& bulletVelocity, double halfBulletSize, const Game& game)
{
	const auto bulletLD0 = Vec2Double(bulletPosition.x - halfBulletSize, bulletPosition.y - halfBulletSize);
	const auto bulletLU0 = Vec2Double(bulletPosition.x - halfBulletSize, bulletPosition.y + halfBulletSize);
	const auto bulletRU0 = Vec2Double(bulletPosition.x + halfBulletSize, bulletPosition.y + halfBulletSize);
	const auto bulletRD0 = Vec2Double(bulletPosition.x + halfBulletSize, bulletPosition.y - halfBulletSize);
	
	auto wallCrossPoint = Vec2Double(0, 0);
	auto bulletWallCrossCorner = Vec2Double(0, 0);
	double minWallCrossPointDist2 = INT_MAX;

	//����������� � ���������
	auto crossWallLD = getBulletBorderCross(bulletLD0, bulletVelocity, game);
	auto dist2 = MathHelper::getVectorLength2(crossWallLD, bulletLD0);
	if (dist2 < minWallCrossPointDist2)
	{
		minWallCrossPointDist2 = dist2;
		wallCrossPoint = crossWallLD;
		bulletWallCrossCorner = bulletLD0;
	}
	
	auto crossWallLU = getBulletBorderCross(bulletLU0, bulletVelocity, game);
	dist2 = MathHelper::getVectorLength2(crossWallLU, bulletLU0);
	if (dist2 < minWallCrossPointDist2)
	{
		minWallCrossPointDist2 = dist2;
		wallCrossPoint = crossWallLU;
		bulletWallCrossCorner = bulletLU0;
	}
	
	auto crossWallRU = getBulletBorderCross(bulletRU0, bulletVelocity, game);
	dist2 = MathHelper::getVectorLength2(crossWallRU, bulletRU0);
	if (dist2 < minWallCrossPointDist2)
	{
		minWallCrossPointDist2 = dist2;
		wallCrossPoint = crossWallRU;
		bulletWallCrossCorner = bulletRU0;
	}
	
	auto crossWallRD = getBulletBorderCross(bulletRD0, bulletVelocity, game);
	dist2 = MathHelper::getVectorLength2(crossWallRD, bulletRD0);
	if (dist2 < minWallCrossPointDist2)
	{
		minWallCrossPointDist2 = dist2;
		wallCrossPoint = crossWallRD;
		bulletWallCrossCorner = bulletRD0;
	}

	const auto squaresLD = MathHelper::getLineSquares2(bulletLD0, crossWallLD);
	const auto squaresLU = MathHelper::getLineSquares2(bulletLU0, crossWallLU);
	const auto squaresRU = MathHelper::getLineSquares2(bulletRU0, crossWallRU);
	const auto squaresRD = MathHelper::getLineSquares2(bulletRD0, crossWallRD);


	const std::pair<int, int>* firstWallTileLD = nullptr;
	for (const auto& square:squaresLD)
	{
		if (square.first == 0 || square.second == 0 || 
			square.first == game.level.tiles.size() - 1 || square.second == game.level.tiles[0].size() - 1)
		{
			break; //����� ������� ����
		}
		
		if (game.level.tiles[square.first][square.second] == WALL)
		{
			firstWallTileLD = &square;
			break;
		}
	}

	if (firstWallTileLD != nullptr)
	{
		const auto xLeft = (*firstWallTileLD).first;
		const auto yDown = (*firstWallTileLD).second;
		const auto xRight = (*firstWallTileLD).first + 1;
		const auto yUp = (*firstWallTileLD).second + 1;
		
		Vec2Double crossPointCur;
		double minDist2Cur = INT_MAX;
		auto hasWallCross = getBulletPointRectangleFirstCrossPoint(bulletLD0, bulletVelocity,
			xLeft, yDown, xRight, yUp, crossPointCur, minDist2Cur);
		/*if (!hasWallCross)
		{
			throw std::runtime_error("no wall cross");
		}*/
		if (minDist2Cur < minWallCrossPointDist2)
		{
			minWallCrossPointDist2 = minDist2Cur;
			wallCrossPoint = crossPointCur;
			bulletWallCrossCorner = bulletLD0;
		}	
	}

	const std::pair<int, int>* firstWallTileLU = nullptr;
	for (const auto& square : squaresLU)
	{
		if (square.first == 0 || square.second == 0 ||
			square.first == game.level.tiles.size() - 1 || square.second == game.level.tiles[0].size() - 1)
		{
			break; //����� ������� ����
		}

		if (game.level.tiles[square.first][square.second] == WALL)
		{
			firstWallTileLU = &square;
			break;
		}
	}


	if (firstWallTileLU != nullptr)
	{
		const auto xLeft = (*firstWallTileLU).first;
		const auto yDown = (*firstWallTileLU).second;
		const auto xRight = (*firstWallTileLU).first + 1;
		const auto yUp = (*firstWallTileLU).second + 1;

		Vec2Double crossPointCur;
		double minDist2Cur = INT_MAX;
		auto hasWallCross = getBulletPointRectangleFirstCrossPoint(bulletLU0, bulletVelocity,
			xLeft, yDown, xRight, yUp, crossPointCur, minDist2Cur);
		/*if (!hasWallCross) {
			throw std::runtime_error("no wall cross");
		}*/
		if (minDist2Cur < minWallCrossPointDist2)
		{
			minWallCrossPointDist2 = minDist2Cur;
			wallCrossPoint = crossPointCur;
			bulletWallCrossCorner = bulletLU0;
		}
	}

	const std::pair<int, int>* firstWallTileRU = nullptr;
	for (const auto& square : squaresRU)
	{
		if (square.first == 0 || square.second == 0 ||
			square.first == game.level.tiles.size() - 1 || square.second == game.level.tiles[0].size() - 1)
		{
			break; //����� ������� ����
		}

		if (game.level.tiles[square.first][square.second] == WALL)
		{
			firstWallTileRU = &square;
			break;
		}
	}

	if (firstWallTileRU != nullptr)
	{
		const auto xLeft = (*firstWallTileRU).first;
		const auto yDown = (*firstWallTileRU).second;
		const auto xRight = (*firstWallTileRU).first + 1;
		const auto yUp = (*firstWallTileRU).second + 1;

		Vec2Double crossPointCur;
		double minDist2Cur = INT_MAX;
		auto hasWallCross = getBulletPointRectangleFirstCrossPoint(bulletRU0, bulletVelocity,
			xLeft, yDown, xRight, yUp, crossPointCur, minDist2Cur);
		/*if (!hasWallCross) {
			throw std::runtime_error("no wall cross");
		}*/
		if (minDist2Cur < minWallCrossPointDist2)
		{
			minWallCrossPointDist2 = minDist2Cur;
			wallCrossPoint = crossPointCur;
			bulletWallCrossCorner = bulletRU0;
		}
	}

	const std::pair<int, int>* firstWallTileRD = nullptr;
	for (const auto& square : squaresRD)
	{
		if (square.first == 0 || square.second == 0 ||
			square.first == game.level.tiles.size() - 1 || square.second == game.level.tiles[0].size() - 1)
		{
			break; //����� ������� ����
		}

		
		if (game.level.tiles[square.first][square.second] == WALL)
		{
			firstWallTileRD = &square;
			break;
		}
	}

	if (firstWallTileRD != nullptr)
	{
		const auto xLeft = (*firstWallTileRD).first;
		const auto yDown = (*firstWallTileRD).second;
		const auto xRight = (*firstWallTileRD).first + 1;
		const auto yUp = (*firstWallTileRD).second + 1;

		Vec2Double crossPointCur;
		double minDist2Cur = INT_MAX;
		auto hasWallCross = getBulletPointRectangleFirstCrossPoint(bulletRD0, bulletVelocity,
			xLeft, yDown, xRight, yUp, crossPointCur, minDist2Cur);
		/*if (!hasWallCross) {
			throw std::runtime_error("no wall cross");
		}*/
		if (minDist2Cur < minWallCrossPointDist2)
		{
			minWallCrossPointDist2 = minDist2Cur;
			wallCrossPoint = crossPointCur;
			bulletWallCrossCorner = bulletRD0;
		}
	}

	const auto shootWallTime = MathHelper::getVectorLength(bulletWallCrossCorner, wallCrossPoint) /
		MathHelper::getVectorLength(bulletVelocity);

	BulletSimulation simulation = { wallCrossPoint, bulletWallCrossCorner, shootWallTime };	
	return simulation;
}


Vec2Double Simulator::getUnitInTimePosition(
	const Vec2Double& unitPosition, const Vec2Double& unitSize, int unitId,
	const UnitAction& action, double time,
	JumpState& jumpState,
	const Game& game)
{
	const auto tickTime = 1.0 / game.properties.ticksPerSecond;
	//if (time > tickTime + TOLERANCE) throw std::runtime_error("impossible to simulate unit for more then 1 tick");
	const auto isOnAir = isUnitOnAir(unitPosition, unitSize, unitId, game);
	
	if (!action.jump && !action.jumpDown && std::abs(action.velocity) < TOLERANCE &&
		!isOnAir)
		return unitPosition;
	
	const auto isOnLadder = isUnitOnLadder(unitPosition, unitSize, game);
	const auto isOnPlatform = isUnitOnPlatform(unitPosition, unitSize, game);

	const auto isPadJump = jumpState.canJump && !jumpState.canCancel;	
	const auto isFall = isOnAir && !jumpState.canJump && !jumpState.canCancel;
	const auto wasJump = isOnAir && jumpState.canJump && jumpState.canCancel;
	const auto canJump = !isOnAir || wasJump;//���� ���� �� ����� ��� ��� ������
	const auto isJump = canJump && action.jump;
	
	auto x = unitPosition.x;
	auto y = unitPosition.y;	

	auto velocityX = action.velocity >= 0 ?
		std::min(action.velocity, game.properties.unitMaxHorizontalSpeed) :
		std::max(action.velocity, -game.properties.unitMaxHorizontalSpeed);

	auto velocityY = 0.0;

	if (isPadJump)
	{
		velocityY = game.properties.jumpPadJumpSpeed;
	}
	else if (isJump)
	{		
		velocityY = game.properties.unitJumpSpeed;
	}
	else if (isFall || 
		(wasJump && !isJump) ||
		(action.jumpDown && (isOnLadder || isOnPlatform)))
	{
		velocityY = -game.properties.unitFallSpeed;
	}
	
	auto nextX = x + velocityX * time;
	auto nextY = y + velocityY * time;
	if (size_t(nextX + TOLERANCE) != size_t(nextX)) nextX += TOLERANCE;
	if (size_t(nextY + TOLERANCE) != size_t(nextY)) nextY += TOLERANCE;

	auto leftBottomTile = game.level.tiles[size_t(nextX - unitSize.x / 2)][size_t(nextY)];
	auto leftTopTile = game.level.tiles[size_t(nextX - unitSize.x / 2)][size_t(nextY + unitSize.y)];
	auto rightTopTile = game.level.tiles[size_t(nextX + unitSize.x / 2)][size_t(nextY + unitSize.y)];
	auto rightBottomTile = game.level.tiles[size_t(nextX + unitSize.x / 2)][size_t(nextY)];


	auto becameOnAir = !isOnAir && !isJump && isUnitOnAir({ nextX, nextY }, unitSize, unitId, game);

	auto isPlatformCross = !action.jumpDown && (isFall || wasJump && !isJump) && (leftBottomTile == PLATFORM || rightBottomTile == PLATFORM);

	const auto squaresLD = MathHelper::getLineSquares2(
		{ unitPosition.x - unitSize.x / 2, unitPosition.y }, { nextX - unitSize.x / 2, nextY });
	const auto squaresLU = MathHelper::getLineSquares2(
		{ unitPosition.x - unitSize.x / 2, unitPosition.y + unitSize.y }, { nextX - unitSize.x / 2, nextY + unitSize.y });
	const auto squaresRU = MathHelper::getLineSquares2(
		{ unitPosition.x + unitSize.x / 2, unitPosition.y + unitSize.y }, { nextX + unitSize.x / 2, nextY + unitSize.y });
	const auto squaresRD = MathHelper::getLineSquares2(
		{ unitPosition.x + unitSize.x / 2, unitPosition.y }, { nextX + unitSize.x / 2, nextY });

	auto isWallCross = false;
	auto isJumpPadCross = false;
	for (const auto& s :squaresLD)
	{
		if (game.level.tiles[s.first][s.second] == WALL)
		{
			isWallCross = true;
			break;
		}
		if (game.level.tiles[s.first][s.second] == JUMP_PAD) {
			isJumpPadCross = true;
		}
	}
	if (!isWallCross)
	{
		for (const auto& s : squaresLU)
		{
			if (game.level.tiles[s.first][s.second] == WALL)
			{
				isWallCross = true;
				break;
			}
			if (!isJumpPadCross && game.level.tiles[s.first][s.second] == JUMP_PAD) {
				isJumpPadCross = true;
				break;
			}
		}
	}
	if (!isWallCross)
	{
		for (const auto& s : squaresRU)
		{
			if (game.level.tiles[s.first][s.second] == WALL)
			{
				isWallCross = true;
				break;
			}
			if (!isJumpPadCross && game.level.tiles[s.first][s.second] == JUMP_PAD) {
				isJumpPadCross = true;
				break;
			}
		}
	}

	if (!isWallCross)
	{
		for (const auto& s : squaresRD)
		{
			if (game.level.tiles[s.first][s.second] == WALL)
			{
				isWallCross = true;
				break;
			}
			if (!isJumpPadCross && game.level.tiles[s.first][s.second] == JUMP_PAD) {
				isJumpPadCross = true;
				break;
			}
		}
	}
	bool isJumpFinished = (isPadJump || wasJump) && jumpState.maxTime < time;

	bool areUnitsCross = false;
	for (const auto& unit: game.units)
	{
		if (unit.id == unitId) continue;
		if (areRectsCross({ nextX, nextY }, unitSize, unit.position, unit.size))
		{
			areUnitsCross = true;
			break;
		}
	}
	

	if (!isWallCross && !areUnitsCross && !isPlatformCross && !isJumpPadCross && !becameOnAir && !isJumpFinished)
	{
		x = nextX;
		y = nextY;

		const Vec2Double newUnitPosition = { x, y };
		updateJumpState(jumpState, time, newUnitPosition, unitSize, unitId, isPadJump, wasJump, isJump, isFall, game);		
		return newUnitPosition;
	}

	//��������� �� ����������

	bool jumpStopped = false;
	
	const auto microTickTime = tickTime / game.properties.updatesPerTick;
	const auto microTicksCount = static_cast<int>(round(time / microTickTime));

		
	if (isWallCross || areUnitsCross)
	{
		const auto startTickVelocityX = velocityX;
		const auto startTickVelocityY = velocityY;
		
		for (int j = 0; j < microTicksCount; ++j)
		{
			const auto prevX = x;
			const auto prevY = y;
			
			nextX = x + velocityX * microTickTime;

			leftBottomTile = game.level.tiles[size_t(nextX - unitSize.x / 2)][size_t(y)];
			leftTopTile = game.level.tiles[size_t(nextX - unitSize.x / 2)][size_t(y + unitSize.y)];
			rightTopTile = game.level.tiles[size_t(nextX + unitSize.x / 2)][size_t(y + unitSize.y)];
			rightBottomTile = game.level.tiles[size_t(nextX + unitSize.x / 2)][size_t(y)];

			isWallCross =
				!(canGoThroughTile(leftBottomTile) && canGoThroughTile(leftTopTile) &&
					canGoThroughTile(rightTopTile) && canGoThroughTile(rightBottomTile));

			const Unit* crossUnit = nullptr;
			for (const auto& unit : game.units)
			{
				if (unit.id == unitId) continue;
				if (areRectsCross({nextX, y}, unitSize, unit.position, unit.size))
				{
					crossUnit = &unit;
					break;
				}
			}

			if (!isWallCross && crossUnit == nullptr) x = nextX;

			nextY = y + velocityY * microTickTime;

			leftBottomTile = game.level.tiles[size_t(x - unitSize.x / 2)][size_t(nextY)];
			leftTopTile = game.level.tiles[size_t(x - unitSize.x / 2)][size_t(nextY + unitSize.y)];
			rightTopTile = game.level.tiles[size_t(x + unitSize.x / 2)][size_t(nextY + unitSize.y)];
			rightBottomTile = game.level.tiles[size_t(x + unitSize.x / 2)][size_t(nextY)];

			isWallCross =
				!(canGoThroughTile(leftBottomTile) && canGoThroughTile(leftTopTile) &&
					canGoThroughTile(rightTopTile) && canGoThroughTile(rightBottomTile));

			crossUnit = nullptr;
			for (const auto& unit : game.units)
			{
				if (unit.id == unitId) continue;
				if (areRectsCross({ x, nextY }, unitSize, unit.position, unit.size))
				{
					crossUnit = &unit;
					break;
				}
			}
			
			
			if (!isWallCross && crossUnit == nullptr)
			{	
				if (isPlatformCross)//�������� ����������� � ����������
				{
					if (game.level.tiles[size_t(nextX - unitSize.x / 2)][size_t(nextY)] == PLATFORM ||
						game.level.tiles[size_t(nextX + unitSize.x / 2)][size_t(nextY)] == PLATFORM)
					{
						if (size_t(y) > size_t(nextY))//� �� ����� ��� ������ ����
						{
							jumpState.canJump = true;
							jumpState.speed = game.properties.unitJumpSpeed;
							jumpState.maxTime = game.properties.unitJumpTime;
							jumpState.canCancel = true;

							y = floor(y);
							velocityY = 0;
							continue;
						}						
					}
				}
				y = nextY;
			}
			else
			{
				if (startTickVelocityY > TOLERANCE)//��� ������
				{
					if (isUnitOnAir({ x, y }, unitSize, unitId, game))
					{
						velocityY = -game.properties.unitFallSpeed;
						y = y + velocityY * microTickTime;
						jumpStopped = true;
					}					
				}
				else if (isWallCross) //���� �������
				{
					y = trunc(y);
				}
				else if (crossUnit != nullptr && startTickVelocityY < -TOLERANCE)
				{
					y = crossUnit->position.y + crossUnit->size.y;
				}
			}

			if (std::abs(x - prevX) < TOLERANCE && std::abs(y - prevY) < TOLERANCE)
			{
				break;
			}
			
		}		
	}

	else if (isPlatformCross)
	{
		for (int j = 0; j < microTicksCount; ++j)
		{
			const auto xj = x + velocityX * microTickTime;
			const auto yj = y + velocityY * microTickTime;

			if (game.level.tiles[size_t(xj - unitSize.x / 2)][size_t(yj)] == PLATFORM ||
				game.level.tiles[size_t(xj + unitSize.x / 2)][size_t(yj)] == PLATFORM)
			{
				if (size_t(y) > size_t(yj))//� �� ����� ��� ������ ����
				{
					jumpState.canJump = true;
					jumpState.speed = game.properties.unitJumpSpeed;
					jumpState.maxTime = game.properties.unitJumpTime;
					jumpState.canCancel = true;

					return { nextX, floor(y) };
				}
				else //��������� �� �������� �� ��� ��������
				{
					return { nextX, nextY };
				}
			}
			x = xj;
			y = yj;
		}
		//throw std::runtime_error("wrong platform simulation");
	}

	else if (isJumpPadCross)
	{
		int microTicksGone = 0;
		for (int j = 0; j < microTicksCount; ++j)
		{
			x += velocityX * microTickTime;
			y += velocityY * microTickTime;

			if (!isUnitOnJumpPad({x, y}, unitSize, game)) microTicksGone++;
			else break;
		}

		velocityY = game.properties.jumpPadJumpSpeed;
		const auto timeLeft = (microTicksCount - microTicksGone) * microTickTime;
		x += velocityX * timeLeft;
		y += velocityY * timeLeft;

		if (size_t(x + TOLERANCE) != size_t(x)) x += TOLERANCE;
		if (size_t(y + TOLERANCE) != size_t(y)) y += TOLERANCE;

		const Vec2Double newUnitPosition = { x, y };
		jumpState.canJump = true;
		jumpState.speed = game.properties.jumpPadJumpSpeed;
		jumpState.maxTime = game.properties.jumpPadJumpTime - timeLeft;
		jumpState.canCancel = false;
		return newUnitPosition;
	}

	else if (becameOnAir)
	{
		int microTicksGone = 0;
		for (int j = 0; j < microTicksCount; ++j)
		{
			microTicksGone++;
			x += velocityX * microTickTime;
			y += velocityY * microTickTime;
			if (isUnitOnAir({ x, y }, unitSize, unitId, game)) {
				y -= game.properties.unitFallSpeed * microTickTime;//��� ������ ������
				break;
			}
		}

		bool isOnJumpPad = false;
		velocityY = -game.properties.unitFallSpeed;
		const auto microTicksLeft = microTicksCount - microTicksGone;
		for (int j = 0; j < microTicksLeft; ++j)
		{
			microTicksGone++;
			x += velocityX * microTickTime;
			y += velocityY * microTickTime;

			if (isUnitOnJumpPad({ x, y }, unitSize, game))
			{			
				isOnJumpPad = true;
				break;
			}
		}

		if (isOnJumpPad)
		{
			velocityY = game.properties.jumpPadJumpSpeed;
			const auto timeLeft = (microTicksCount - microTicksGone) * microTickTime;
			x += velocityX * timeLeft;
			y += velocityY * timeLeft;

			jumpState.canJump = true;
			jumpState.speed = game.properties.jumpPadJumpSpeed;
			jumpState.maxTime = game.properties.jumpPadJumpTime - timeLeft;
			jumpState.canCancel = false;
		}		
		else
		{
			jumpState.canJump = false;
			jumpState.speed = 0;
			jumpState.maxTime = 0;
			jumpState.canCancel = false;
		}

		if (size_t(x + TOLERANCE) != size_t(x)) x += TOLERANCE;
		if (size_t(y + TOLERANCE) != size_t(y)) y += TOLERANCE;
		
		const Vec2Double newUnitPosition = { x, y };
		return newUnitPosition;
	}

	else if (isJumpFinished)
	{	
		const auto jumpMicroTicksCount = static_cast<int> (ceil(jumpState.maxTime / microTickTime));
		const auto fallMicroTicksCount = microTicksCount - jumpMicroTicksCount;
		
		x += velocityX * (microTickTime * jumpMicroTicksCount);
		y += velocityY * (microTickTime * jumpMicroTicksCount);

		velocityY = -game.properties.unitFallSpeed;
		x += velocityX * (microTickTime * fallMicroTicksCount);
		y += velocityY * (microTickTime * fallMicroTicksCount);
		
		if (size_t(x + TOLERANCE) != size_t(x)) x += TOLERANCE;
		if (size_t(y + TOLERANCE) != size_t(y)) y += TOLERANCE;
		
		const Vec2Double newUnitPosition = { x, y };
		updateJumpState(jumpState, time, newUnitPosition, unitSize, unitId, isPadJump, wasJump, isJump, isFall, game);
		return newUnitPosition;
	}

	/*else
	{
		throw std::runtime_error("unknown mt simulation case");
	}*/
	
	const Vec2Double newUnitPosition = { x, y };
	if (jumpStopped)
	{
		jumpState.canJump = false;
		jumpState.speed = 0;
		jumpState.maxTime = 0;
		jumpState.canCancel = false;
	}
	else updateJumpState(jumpState, time, newUnitPosition, unitSize, unitId, isPadJump, wasJump, isJump, isFall, game);
	return newUnitPosition;
}

void Simulator::getPolygon(const Vec2Double& unitPos, const Vec2Double& newUnitPos, const Vec2Double& unitSize, Vec2Double polygon[6])
{
	Vec2Double rect[4] = {
		{unitPos.x + unitSize.x / 2, unitPos.y},
		{unitPos.x - unitSize.x / 2, unitPos.y},
		{unitPos.x - unitSize.x / 2, unitPos.y + unitSize.y},
		{unitPos.x + unitSize.x / 2, unitPos.y + unitSize.y}
	};
	Vec2Double newRect[4] = {
		{newUnitPos.x + unitSize.x / 2, newUnitPos.y},
		{newUnitPos.x - unitSize.x / 2, newUnitPos.y},
		{newUnitPos.x - unitSize.x / 2, newUnitPos.y + unitSize.y},
		{newUnitPos.x + unitSize.x / 2, newUnitPos.y + unitSize.y}
	};
	getPolygon(rect, newRect, polygon);	
}

void Simulator::getPolygon(const Vec2Double& bulletPos, const Vec2Double& newBulletPos, double halfBulletSize,
	Vec2Double polygon[6])
{
	Vec2Double rect[4] = {
		{bulletPos.x + halfBulletSize, bulletPos.y - halfBulletSize},
		{bulletPos.x - halfBulletSize, bulletPos.y - halfBulletSize},
		{bulletPos.x - halfBulletSize, bulletPos.y + halfBulletSize},
		{bulletPos.x + halfBulletSize, bulletPos.y + halfBulletSize}
	};
	Vec2Double newRect[4] = {
		{newBulletPos.x + halfBulletSize, newBulletPos.y - halfBulletSize},
		{newBulletPos.x - halfBulletSize, newBulletPos.y - halfBulletSize},
		{newBulletPos.x - halfBulletSize, newBulletPos.y + halfBulletSize},
		{newBulletPos.x + halfBulletSize, newBulletPos.y + halfBulletSize}
	};
	getPolygon(rect, newRect, polygon);
}

void Simulator::getPolygon(const Vec2Double rect[4], const Vec2Double newRect[4], Vec2Double polygon[6])
{
	int start;
	const auto velocity = newRect[0] - rect[0];
	if (velocity.x >= 0 && velocity.y >= 0) start = 0;
	else if (velocity.x < 0 && velocity.y >= 0) start = 3;
	else if (velocity.x >= 0 && velocity.y < 0) start = 1;
	else start = 2;

	for (int i = 0; i < 3; ++i)
	{
		const int index = (start + i) & 0b11;
		polygon[i] = rect[index];
	}

	for (int i = 0; i < 3; ++i)
	{
		const int index = (start + i + 2) & 0b11;
		polygon[i + 3] = rect[index] + velocity;
	}
}

void Simulator::updateJumpState(JumpState& jumpState, double time,
	const Vec2Double& unitPosition, const Vec2Double& unitSize, int unitId,
	bool isPadJump, bool wasJump, bool isJump, bool isFall,
	const Game& game)
{
	if (isPadJump || wasJump && isJump)//�������
	{
		if (jumpState.maxTime > time) jumpState.maxTime -= time;//���������� ������
		else //������
		{
			jumpState.canJump = false;
			jumpState.speed = 0;
			jumpState.maxTime = 0;
			jumpState.canCancel = false;
		}
		return;
	}

	if (!wasJump && isJump) //�������� ������
	{
		jumpState.canJump = true;
		jumpState.speed = game.properties.unitJumpSpeed;
		jumpState.maxTime = game.properties.unitJumpTime - time;
		jumpState.canCancel = true;
	}

	if (wasJump && !isJump) //���������� ������
	{
		if (isUnitOnAir(unitPosition, unitSize, unitId, game)) //�� � �������. ������
		{
			jumpState.canJump = false;
			jumpState.speed = 0;
			jumpState.maxTime = 0;
			jumpState.canCancel = false;
		}
		else//������ ������ �� �����. ������� �� �����
		{
			jumpState.canJump = true;
			jumpState.speed = game.properties.unitJumpSpeed;
			jumpState.maxTime = game.properties.unitJumpTime;
			jumpState.canCancel = true;
		}
	}

	if (isFall) //������
	{
		if (!isUnitOnAir(unitPosition, unitSize, unitId, game)) //������ �� �����. ������� �������
		{
			jumpState.canJump = true;
			jumpState.speed = game.properties.unitJumpSpeed;
			jumpState.maxTime = game.properties.unitJumpTime;
			jumpState.canCancel = true;
		}
	}
}

bool Simulator::isUnitOnWall(const Vec2Double& unitPosition, const Vec2Double& unitSize, const Game& game)
{
	const auto leftSideDownTile = game.level.tiles[size_t(unitPosition.x - unitSize.x / 2)][size_t(
		unitPosition.y - TOLERANCE)];
	const auto rightSideDownTile = game.level.tiles[size_t(unitPosition.x + unitSize.x / 2)][size_t(
		unitPosition.y - TOLERANCE)];
	return leftSideDownTile == WALL || rightSideDownTile == WALL;
}


bool Simulator::isUnitOnLadder(const Vec2Double& unitPosition, const Vec2Double& unitSize, const Game& game)
{
	/*const auto centerUnitTile = game.level.tiles[size_t(unit.position.x)][size_t(unit.position.y + unit.size.y / 2)];
	const auto downUnitTile = game.level.tiles[size_t(unit.position.x)][size_t(unit.position.y)];
	return centerUnitTile == LADDER || downUnitTile == LADDER;*/

	if (isUnitOnWall(unitPosition, unitSize, game)) return false;

	const auto centerDownTile = game.level.tiles[size_t(unitPosition.x)][size_t(unitPosition.y - TOLERANCE)];

	const auto centerY = unitPosition.y + unitSize.y / 2;
	const auto centerTile = game.level.tiles[size_t(unitPosition.x)][size_t(centerY)];
	return centerDownTile == LADDER || centerTile == LADDER;
	
}


bool Simulator::isUnitOnPlatform(const Vec2Double& unitPosition, const Vec2Double& unitSize, const Game& game)
{
	if (isUnitOnWall(unitPosition, unitSize, game)) return false;
	
	const auto leftSideDownTile = game.level.tiles[size_t(unitPosition.x - unitSize.x / 2)][size_t(
		unitPosition.y - TOLERANCE)];
	const auto rightSideDownTile = game.level.tiles[size_t(unitPosition.x + unitSize.x / 2)][size_t(
		unitPosition.y - TOLERANCE)];

	return (leftSideDownTile == PLATFORM || rightSideDownTile == PLATFORM) &&
		size_t(unitPosition.y) != size_t(unitPosition.y - TOLERANCE);
}

bool Simulator::isUnitOnAir(const Vec2Double& unitPosition, const Vec2Double& unitSize, int unitId, const Game& game)
{
	return !isUnitOnWall(unitPosition, unitSize, game) &&
		!isUnitOnLadder(unitPosition, unitSize, game) &&
		!isUnitOnPlatform(unitPosition, unitSize, game) &&
		!isUnitOnUnit(unitPosition, unitSize, unitId, game);
}

bool Simulator::isUnitOnJumpPad(const Vec2Double& unitPosition, const Vec2Double& unitSize, const Game& game)
{
	return
		game.level.tiles[size_t(unitPosition.x - unitSize.x / 2)][size_t(unitPosition.y)] == JUMP_PAD ||
		game.level.tiles[size_t(unitPosition.x - unitSize.x / 2)][size_t(unitPosition.y + unitSize.y)] == JUMP_PAD ||
		game.level.tiles[size_t(unitPosition.x + unitSize.x / 2)][size_t(unitPosition.y + unitSize.y)] == JUMP_PAD ||
		game.level.tiles[size_t(unitPosition.x + unitSize.x / 2)][size_t(unitPosition.y)] == JUMP_PAD;
}

bool Simulator::isUnitOnUnit(const Vec2Double& unitPosition, const Vec2Double& unitSize, int unitId, const Game& game)
{
	
	for (const auto& unit: game.units)
	{
		if (unitId == unit.id) continue;
		if (unitPosition.y - unit.position.y < unit.size.y + TOLERANCE &&
			unitPosition.y - unit.position.y > unit.size.y - TOLERANCE &&
			std::abs(unitPosition.x - unit.position.x) < unitSize.x / 2 + unit.size.x/2)
		{
			return true;
		}
	}
	return false;
}

bool Simulator::areRectsTouch(const Vec2Double& pos1, const Vec2Double& size1, const Vec2Double& pos2,
	const Vec2Double& size2)
{
	const auto isXCross = std::abs(pos1.x - pos2.x) < size1.x / 2 + size2.x / 2 + TOLERANCE;
	bool isYCross = false;
	if (pos1.y > pos2.y)
	{
		isYCross = std::abs(pos1.y - pos2.y) < size2.y + TOLERANCE;
	}
	else
	{
		isYCross = std::abs(pos1.y - pos2.y) < size1.y + TOLERANCE;
	}
	return isXCross && isYCross;
}

bool Simulator::areRectsCross(const Vec2Double& pos1, const Vec2Double& size1, const Vec2Double& pos2,
	const Vec2Double& size2)
{
	const auto isXCross = std::abs(pos1.x - pos2.x) < size1.x / 2 + size2.x / 2 - TOLERANCE;

	bool isYCross = false;
	if (pos1.y > pos2.y)
	{
		isYCross = std::abs(pos1.y - pos2.y) < size2.y - TOLERANCE;
	}
	else
	{
		isYCross = std::abs(pos1.y - pos2.y) < size1.y - TOLERANCE;
	}
	return isXCross && isYCross;
}


bool Simulator::getBulletRectangleFirstCrossPoint(const Vec2Double& bulletPos, const Vec2Double& bulletVelocity, double halfBulletSize,
	double xLeft, double yDown, double xRight, double yUp,
	Vec2Double& crossPoint, Vec2Double& bulletCorner)
{
	bool hasCross = false;
	double minDist2 = INT_MAX;
	crossPoint = Vec2Double(0, 0);
	bulletCorner = Vec2Double(0, 0);
	
	const auto bulletLD = Vec2Double(bulletPos.x - halfBulletSize, bulletPos.y - halfBulletSize);
	Vec2Double crossPointCur = Vec2Double(0, 0);
	double minDist2Cur = INT_MAX;
	auto hasWallCross = getBulletPointRectangleFirstCrossPoint(bulletLD, bulletVelocity,
		xLeft, yDown, xRight, yUp, crossPointCur, minDist2Cur);
	if (hasWallCross && minDist2Cur < minDist2)
	{
		hasCross = true;
		minDist2 = minDist2Cur;
		crossPoint = crossPointCur;
		bulletCorner = bulletLD;
	}

	const auto bulletLU = Vec2Double(bulletPos.x - halfBulletSize, bulletPos.y + halfBulletSize);
	crossPointCur = Vec2Double(0, 0);
	minDist2Cur = INT_MAX;
	hasWallCross = getBulletPointRectangleFirstCrossPoint(bulletLU, bulletVelocity,
		xLeft, yDown, xRight, yUp, crossPointCur, minDist2Cur);
	if (hasWallCross && minDist2Cur < minDist2)
	{
		hasCross = true;
		minDist2 = minDist2Cur;
		crossPoint = crossPointCur;
		bulletCorner = bulletLU;
	}

	const auto bulletRU = Vec2Double(bulletPos.x + halfBulletSize, bulletPos.y + halfBulletSize);
	crossPointCur = Vec2Double(0, 0);
	minDist2Cur = INT_MAX;
	hasWallCross = getBulletPointRectangleFirstCrossPoint(bulletRU, bulletVelocity,
		xLeft, yDown, xRight, yUp, crossPointCur, minDist2Cur);
	if (hasWallCross && minDist2Cur < minDist2)
	{
		hasCross = true;
		minDist2 = minDist2Cur;
		crossPoint = crossPointCur;
		bulletCorner = bulletRU;
	}

	const auto bulletRD = Vec2Double(bulletPos.x + halfBulletSize, bulletPos.y - halfBulletSize);
	crossPointCur = Vec2Double(0, 0);
	minDist2Cur = INT_MAX;
	hasWallCross = getBulletPointRectangleFirstCrossPoint(bulletRD, bulletVelocity,
		xLeft, yDown, xRight, yUp, crossPointCur, minDist2Cur);
	if (hasWallCross && minDist2Cur < minDist2)
	{
		hasCross = true;
		minDist2 = minDist2Cur;
		crossPoint = crossPointCur;
		bulletCorner = bulletRD;
	}

	return hasCross;
}

std::map<int, Vec2Double> Simulator::getBulletPositions(
	const Vec2Double& bulletPosition, const Vec2Double& bulletVelocity, double targetCrossTime, const Game& game)
{
	const auto shootWallTick = static_cast<int>(ceil(targetCrossTime * game.properties.ticksPerSecond));
	std::map<int, Vec2Double> bulletPositions;
	bulletPositions[0] = bulletPosition;

	const auto tickTime = 1.0 / game.properties.ticksPerSecond;
	for (int i = 1; i <= shootWallTick; ++i)
	{
		Vec2Double position;
		auto exists = Simulator::getBulletInTimePosition(
			bulletPosition, bulletVelocity, tickTime * i, targetCrossTime, game, position);
		if (exists && i == shootWallTick) {
			exists = false;
		}
		bulletPositions[exists ? i : -1] = position;
	}

	return bulletPositions;
}
