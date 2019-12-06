#include "Simulator.h"
#include "../mathcalc/MathHelper.h"
#include "../common/Helper.h"
#include "../model/Game.hpp"
#include "../strategy/ShootMeBulletCrossPoint.h"
#include "../model/UnitAction.hpp"
#include <algorithm>
#include <climits>
#include <cmath>


bool Simulator::canGoThroughTile(const Tile& tile, bool jumpDown)
{
	if (tile == WALL) return false;
	return !jumpDown || jumpDown && (tile == LADDER || tile == PLATFORM || tile == EMPTY);
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
	if (MathHelper::IsBetween(xLeft, cpLeft.y, xLeft, yDown, xLeft, yUp))
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
	if (MathHelper::IsBetween(cpUp.x, yUp, xLeft, yUp, xRight, yUp))
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
	if (MathHelper::IsBetween(xRight, cpRight.y, xRight, yUp, xRight, yDown))
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
	if (MathHelper::IsBetween(cpDown.x, yDown, xRight, yDown, xLeft, yDown))
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
	const Bullet& bullet, double time, const BulletSimulation& bulletSimulation, const Game& game,
	Vec2Double& position)
{
	if (time > bulletSimulation.targetCrossTime) {
		position = Vec2Double(
			bullet.position.x + bullet.velocity.x * bulletSimulation.targetCrossTime,
			bullet.position.y + bullet.velocity.y * bulletSimulation.targetCrossTime);
		return false;
	}
	position = Vec2Double(bullet.position.x + bullet.velocity.x * time, bullet.position.y + bullet.velocity.y * time);
	return true;
}

BulletSimulation Simulator::getBulletSimulation(const Vec2Double& bulletPosition, const Vec2Double& bulletVelocity, double halfBulletSize, const Game& game)
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

	const auto squaresLD = MathHelper::getLineSquares(bulletLD0, crossWallLD, 1);
	const auto squaresLU = MathHelper::getLineSquares(bulletLU0, crossWallLU, 1);
	const auto squaresRU = MathHelper::getLineSquares(bulletRU0, crossWallRU, 1);
	const auto squaresRD = MathHelper::getLineSquares(bulletRD0, crossWallRD, 1);


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
		if (!hasWallCross)
		{
			hasWallCross = getBulletPointRectangleFirstCrossPoint(bulletLD0, bulletVelocity,
				xLeft, yDown, xRight, yUp, crossPointCur, minDist2Cur);
			throw std::runtime_error("no wall cross");
		}
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
		if (!hasWallCross) {
			hasWallCross = getBulletPointRectangleFirstCrossPoint(bulletLU0, bulletVelocity,
				xLeft, yDown, xRight, yUp, crossPointCur, minDist2Cur);
			throw std::runtime_error("no wall cross");
		}
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
		if (!hasWallCross) {
			hasWallCross = getBulletPointRectangleFirstCrossPoint(bulletRU0, bulletVelocity,
				xLeft, yDown, xRight, yUp, crossPointCur, minDist2Cur);
			throw std::runtime_error("no wall cross");
		}
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
		if (!hasWallCross) {
			hasWallCross = getBulletPointRectangleFirstCrossPoint(bulletRD0, bulletVelocity,
				xLeft, yDown, xRight, yUp, crossPointCur, minDist2Cur);
			throw std::runtime_error("no wall cross");
		}
		if (minDist2Cur < minWallCrossPointDist2)
		{
			minWallCrossPointDist2 = minDist2Cur;
			wallCrossPoint = crossPointCur;
			bulletWallCrossCorner = bulletRD0;
		}
	}

	const auto shootWallTime = MathHelper::getVectorLength(bulletWallCrossCorner, wallCrossPoint) /
		MathHelper::getVectorLength(bulletVelocity);
	return { wallCrossPoint, bulletWallCrossCorner, shootWallTime };
}


//TODO: ����� ����� ������ ��� ����� � ������� ���������� �������
Vec2Double Simulator::getUnitInTimePosition(
	const Vec2Double& unitPosition, const Vec2Double& unitSize,  const UnitAction& action, double time, const Game& game)
{	
	auto x = unitPosition.x;
	auto y = unitPosition.y;	

	const auto actionVelocityX = action.velocity >= 0 ?
		std::min(action.velocity, game.properties.unitMaxHorizontalSpeed) :
		std::max(action.velocity, -game.properties.unitMaxHorizontalSpeed);

	const auto actionVelocityY = action.jump ? game.properties.unitJumpSpeed : -game.properties.unitFallSpeed;

	auto velocityX = actionVelocityX;
	auto velocityY = 0;
	if (action.jump)
	{
		velocityY = game.properties.unitJumpSpeed;
	}
	else if (isUnitOnAir(unitPosition, unitSize, game) ||
		action.jumpDown && (isUnitOnLadder(unitPosition, unitSize, game) ||
			isUnitOnPlatform(unitPosition, unitSize, game)))
	{
		velocityY = -game.properties.unitFallSpeed;
	}
	
	auto nextX = x + velocityX * time;
	auto nextY = y + velocityY * time;

	auto leftBottomTile = game.level.tiles[size_t(nextX - unitSize.x / 2)][size_t(nextY)];
	auto leftTopTile = game.level.tiles[size_t(nextX - unitSize.x / 2)][size_t(nextY + unitSize.y)];
	auto rightTopTile = game.level.tiles[size_t(nextX + unitSize.x / 2)][size_t(nextY + unitSize.y)];
	auto rightBottomTile = game.level.tiles[size_t(nextX + unitSize.x / 2)][size_t(nextY)];

	auto canGoThrough = 
		canGoThroughTile(leftBottomTile, action.jumpDown) && canGoThroughTile(leftTopTile, action.jumpDown) && 
		canGoThroughTile(rightTopTile, action.jumpDown) && canGoThroughTile(rightBottomTile, action.jumpDown);

	auto canGoAction = false;
	if (abs(velocityX - actionVelocityX) > TOLERANCE )
	{
		if (actionVelocityX > 0 && game.level.tiles[size_t(x + unitSize.x / 2 + TOLERANCE)][size_t(y)] != WALL &&
								   game.level.tiles[size_t(x + unitSize.x / 2 + TOLERANCE)][size_t(y + unitSize.y)] != WALL ||
			actionVelocityX < 0 && game.level.tiles[size_t(x - unitSize.x / 2 - TOLERANCE)][size_t(y)] != WALL &&
								   game.level.tiles[size_t(x - unitSize.x / 2 - TOLERANCE)][size_t(y + unitSize.y)] != WALL)			
		{
			canGoAction = true;
		}
	}
	else if (abs(velocityY - actionVelocityY) > TOLERANCE)
	{
		if (actionVelocityY > 0 && game.level.tiles[size_t(x - unitSize.x / 2)][size_t(y + unitSize.y + TOLERANCE)] != WALL &&
			game.level.tiles[size_t(x + unitSize.x / 2)][size_t(y + unitSize.y + TOLERANCE)] != WALL ||
			actionVelocityY < 0 && game.level.tiles[size_t(x - unitSize.x / 2)][size_t(y - TOLERANCE)] == EMPTY &&
			game.level.tiles[size_t(x + unitSize.x / 2)][size_t(y - TOLERANCE)] == EMPTY)
		{
			canGoAction = true;
		}
	}

	if (canGoThrough && !canGoAction)
	{
		x = nextX;
		y = nextY;
		return { x, y };
	}

	//��������� �� ����������

	const auto tickTime = 1.0 / game.properties.ticksPerSecond;
	const auto microTickTime = tickTime / game.properties.updatesPerTick;
	const auto microTicksCount = static_cast<int>(round(time / microTickTime));

	if (!canGoThrough)
	{
		const auto startTickVelocityX = velocityX;
		const auto startTickVelocityY = velocityY;
		for (int j = 0; j < microTicksCount; ++j)
		{
			nextX = x + velocityX * microTickTime;
			nextY = y + velocityY * microTickTime;

			leftBottomTile = game.level.tiles[size_t(nextX - unitSize.x / 2)][size_t(nextY)];
			leftTopTile = game.level.tiles[size_t(nextX - unitSize.x / 2)][size_t(nextY + unitSize.y)];
			rightTopTile = game.level.tiles[size_t(nextX + unitSize.x / 2)][size_t(nextY + unitSize.y)];
			rightBottomTile = game.level.tiles[size_t(nextX + unitSize.x / 2)][size_t(nextY)];

			canGoThrough =
				canGoThroughTile(leftBottomTile, action.jumpDown) && canGoThroughTile(leftTopTile, action.jumpDown) &&
				canGoThroughTile(rightTopTile, action.jumpDown) && canGoThroughTile(rightBottomTile, action.jumpDown);

			if (canGoThrough)
			{
				x = nextX;
				y = nextY;
				continue;
			}
			
			if (abs(velocityX) > TOLERANCE)//��������� �������������� ��������
			{
				velocityX = 0;
				nextX = x + velocityX * microTickTime;
				nextY = y + velocityY * microTickTime;

				leftBottomTile = game.level.tiles[size_t(nextX - unitSize.x / 2)][size_t(nextY)];
				leftTopTile = game.level.tiles[size_t(nextX - unitSize.x / 2)][size_t(nextY + unitSize.y)];
				rightTopTile = game.level.tiles[size_t(nextX + unitSize.x / 2)][size_t(nextY + unitSize.y)];
				rightBottomTile = game.level.tiles[size_t(nextX + unitSize.x / 2)][size_t(nextY)];

				canGoThrough =
					canGoThroughTile(leftBottomTile, action.jumpDown) && canGoThroughTile(leftTopTile, action.jumpDown) &&
					canGoThroughTile(rightTopTile, action.jumpDown) && canGoThroughTile(rightBottomTile, action.jumpDown);

				if (canGoThrough)
				{
					x = nextX;
					y = nextY;
					continue;
				}
			}

			if (abs(velocityY) > TOLERANCE)//��������� ������������ ��������
			{
				velocityX = startTickVelocityX;
				velocityY = 0;
				nextX = x + velocityX * microTickTime;
				nextY = y + velocityY * microTickTime;

				leftBottomTile = game.level.tiles[size_t(nextX - unitSize.x / 2)][size_t(nextY)];
				leftTopTile = game.level.tiles[size_t(nextX - unitSize.x / 2)][size_t(nextY + unitSize.y)];
				rightTopTile = game.level.tiles[size_t(nextX + unitSize.x / 2)][size_t(nextY + unitSize.y)];
				rightBottomTile = game.level.tiles[size_t(nextX + unitSize.x / 2)][size_t(nextY)];

				canGoThrough =
					canGoThroughTile(leftBottomTile, action.jumpDown) && canGoThroughTile(leftTopTile, action.jumpDown) &&
					canGoThroughTile(rightTopTile, action.jumpDown) && canGoThroughTile(rightBottomTile, action.jumpDown);

				if (canGoThrough)
				{
					x = nextX;
					y = nextY;
					continue;
				}
			}

			//�� ����� ���� �� �� �����������, �� �� ���������
			return { x, y };
		}
	}

	else//canGoAction
	{
		/*auto tickVelocityX = actionVelocityX;
		auto tickVelocityY = actionVelocityY;*/

		for (int j = 0; j < microTicksCount; ++j)
		{
			nextX = x + actionVelocityX * microTickTime;
			nextY = y + actionVelocityY * microTickTime;

			leftBottomTile = game.level.tiles[size_t(nextX - unitSize.x / 2)][size_t(nextY)];
			leftTopTile = game.level.tiles[size_t(nextX - unitSize.x / 2)][size_t(nextY + unitSize.y)];
			rightTopTile = game.level.tiles[size_t(nextX + unitSize.x / 2)][size_t(nextY + unitSize.y)];
			rightBottomTile = game.level.tiles[size_t(nextX + unitSize.x / 2)][size_t(nextY)];

			canGoThrough =
				canGoThroughTile(leftBottomTile, action.jumpDown) && canGoThroughTile(leftTopTile, action.jumpDown) &&
				canGoThroughTile(rightTopTile, action.jumpDown) && canGoThroughTile(rightBottomTile, action.jumpDown);

			if (canGoThrough)
			{
				x = nextX;
				y = nextY;
				continue;
			}

			if (abs(actionVelocityX) > 0)//��������� �������������� ��������
			{
				nextX = x + velocityX * microTickTime;
				nextY = y + actionVelocityY * microTickTime;

				leftBottomTile = game.level.tiles[size_t(nextX - unitSize.x / 2)][size_t(nextY)];
				leftTopTile = game.level.tiles[size_t(nextX - unitSize.x / 2)][size_t(nextY + unitSize.y)];
				rightTopTile = game.level.tiles[size_t(nextX + unitSize.x / 2)][size_t(nextY + unitSize.y)];
				rightBottomTile = game.level.tiles[size_t(nextX + unitSize.x / 2)][size_t(nextY)];

				canGoThrough =
					canGoThroughTile(leftBottomTile, action.jumpDown) && canGoThroughTile(leftTopTile, action.jumpDown) &&
					canGoThroughTile(rightTopTile, action.jumpDown) && canGoThroughTile(rightBottomTile, action.jumpDown);

				if (canGoThrough)
				{
					x = nextX;
					y = nextY;
					continue;
				}
			}

			if (abs(actionVelocityY) > 0)//��������� ������������ ��������
			{
				nextX = x + actionVelocityX * microTickTime;
				nextY = y + velocityY * microTickTime;

				leftBottomTile = game.level.tiles[size_t(nextX - unitSize.x / 2)][size_t(nextY)];
				leftTopTile = game.level.tiles[size_t(nextX - unitSize.x / 2)][size_t(nextY + unitSize.y)];
				rightTopTile = game.level.tiles[size_t(nextX + unitSize.x / 2)][size_t(nextY + unitSize.y)];
				rightBottomTile = game.level.tiles[size_t(nextX + unitSize.x / 2)][size_t(nextY)];

				canGoThrough =
					canGoThroughTile(leftBottomTile, action.jumpDown) && canGoThroughTile(leftTopTile, action.jumpDown) &&
					canGoThroughTile(rightTopTile, action.jumpDown) && canGoThroughTile(rightBottomTile, action.jumpDown);

				if (canGoThrough)
				{
					x = nextX;
					y = nextY;
					continue;
				}
			}

			//���� ��������� �� ������� ����������
			x += velocityX * microTickTime;
			y += velocityY * microTickTime;
		}
	}
	
	return { x, y };
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

	const auto leftSideDownTile = game.level.tiles[size_t(unitPosition.x - unitSize.x / 2)][size_t(
		unitPosition.y - TOLERANCE)];
	const auto rightSideDownTile = game.level.tiles[size_t(unitPosition.x + unitSize.x / 2)][size_t(
		unitPosition.y - TOLERANCE)];

	return leftSideDownTile == LADDER || rightSideDownTile == LADDER;
	
}


bool Simulator::isUnitOnPlatform(const Vec2Double& unitPosition, const Vec2Double& unitSize, const Game& game)
{
	if (isUnitOnWall(unitPosition, unitSize, game)) return false;
	
	const auto leftSideDownTile = game.level.tiles[size_t(unitPosition.x - unitSize.x / 2)][size_t(
		unitPosition.y - TOLERANCE)];
	const auto rightSideDownTile = game.level.tiles[size_t(unitPosition.x + unitSize.x / 2)][size_t(
		unitPosition.y - TOLERANCE)];

	return leftSideDownTile == PLATFORM || rightSideDownTile == PLATFORM;
}

bool Simulator::isUnitOnAir(const Vec2Double& unitPosition, const Vec2Double& unitSize, const Game& game)
{
	return !isUnitOnWall(unitPosition, unitSize, game) && !isUnitOnLadder(unitPosition, unitSize, game) && !isUnitOnPlatform(unitPosition, unitSize, game);
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


