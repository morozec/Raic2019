#include "Simulator.h"
#include "../mathcalc/MathHelper.h"
#include "../common/Helper.h"
#include "../model/Game.hpp"
#include "../strategy/ShootMeBulletCrossPoint.h"
#include "../model/UnitAction.hpp"
#include <algorithm>
#include <climits>

//TODO: учесть граничный TILE
Vec2Double Simulator::getBulletCrossBorderPoint(const Vec2Double& bulletPosition, const Vec2Double& bulletVelocity, double maxX,
	double maxY)
{
	if (abs(bulletVelocity.y) < TOLERACNE)
	{
		return Vec2Double(bulletVelocity.x > 0 ? maxX : 0, bulletPosition.y);
	}

	if (abs(bulletVelocity.x) < TOLERACNE)
	{
		return Vec2Double(bulletPosition.x, bulletVelocity.y > 0 ? maxY : 0);
	}

	const auto x1 = bulletPosition.x;
	const auto y1 = bulletPosition.y;
	const auto x2 = bulletPosition.x + bulletVelocity.x;
	const auto y2 = bulletPosition.y + bulletVelocity.y;

	if (bulletVelocity.x > 0)
	{
		const auto vertCross = MathHelper::getLinesCross(x1, y1, x2, y2, maxX, 0, maxX, maxY);
		if (vertCross.y >= 0 && vertCross.y < maxY)
		{
			return vertCross;
		}

		return bulletVelocity.y < 0
			? MathHelper::getLinesCross(x1, y1, x2, y2, 0, 0, maxX, 0)
			: MathHelper::getLinesCross(x1, y1, x2, y2, 0, maxY, maxX, maxY);
	}
	const auto vertCross = MathHelper::getLinesCross(x1, y1, x2, y2, 0, 0, 0, maxY);
	if (vertCross.y >= 0 && vertCross.y < maxY)
	{
		return vertCross;
	}

	return bulletVelocity.y < 0
		? MathHelper::getLinesCross(x1, y1, x2, y2, 0, 0, maxX, 0)
		: MathHelper::getLinesCross(x1, y1, x2, y2, 0, maxY, maxX, maxY);
}


Vec2Double Simulator::getBulletCornerCrossWallPoint(
	const Vec2Double& bulletPosition, const Vec2Double& bulletVelocity, double maxX, double maxY, const Game& game)
{
	auto crossPoint = Simulator::getBulletCrossBorderPoint(bulletPosition, bulletVelocity, maxX, maxY);
	const auto bulletTiles = MathHelper::getLineSquares(bulletPosition, crossPoint, 1);
	const std::pair<int, int>* firstWallTile = nullptr;

	for (const auto& bt : bulletTiles)
	{
		if (bt.first == 0 || bt.second == 0 || bt.first == game.level.tiles.size() - 1 || bt.second == game.level.tiles[
			0].size() - 1)
		{
			break; //игнор крайних стен
		}

			if (game.level.tiles[bt.first][bt.second] == WALL)
			{
				firstWallTile = &bt;
				break;
			}
	}

	if (firstWallTile != nullptr)
	{
		const auto x1 = bulletPosition.x;
		const auto y1 = bulletPosition.y;
		const auto x2 = bulletPosition.x + bulletVelocity.x;
		const auto y2 = bulletPosition.y + bulletVelocity.y;

		int minDist = INT_MAX;
		const Vec2Double* minDistCp = nullptr;

		auto cp1 = MathHelper::getLinesCross(
			x1, y1, x2, y2, firstWallTile->first, firstWallTile->second, firstWallTile->first + TILE_SIZE,
			firstWallTile->second);
		if (cp1.x >= firstWallTile->first && cp1.x <= firstWallTile->first + TILE_SIZE)
		{
			const auto dist2 = MathHelper::getVectorLength2(Vec2Double(x1 - cp1.x, y1 - cp1.y));
			if (minDistCp == nullptr || dist2 < minDist)
			{
				minDist = dist2;
				minDistCp = &cp1;
			}
		}

		auto cp2 = MathHelper::getLinesCross(
			x1, y1, x2, y2, firstWallTile->first, firstWallTile->second + TILE_SIZE, firstWallTile->first + TILE_SIZE,
			firstWallTile->second + TILE_SIZE);
		if (cp2.x >= firstWallTile->first && cp2.x <= firstWallTile->first + TILE_SIZE)
		{
			const auto dist2 = MathHelper::getVectorLength2(Vec2Double(x1 - cp2.x, y1 - cp2.y));
			if (minDistCp == nullptr || dist2 < minDist)
			{
				minDist = dist2;
				minDistCp = &cp2;
			}
		}

		auto cp3 = MathHelper::getLinesCross(
			x1, y1, x2, y2, firstWallTile->first, firstWallTile->second, firstWallTile->first,
			firstWallTile->second + TILE_SIZE);
		if (cp3.y >= firstWallTile->second && cp3.y <= firstWallTile->second + TILE_SIZE)
		{
			const auto dist2 = MathHelper::getVectorLength2(Vec2Double(x1 - cp3.x, y1 - cp3.y));
			if (minDistCp == nullptr || dist2 < minDist)
			{
				minDist = dist2;
				minDistCp = &cp3;
			}
		}

		auto cp4 = MathHelper::getLinesCross(
			x1, y1, x2, y2, firstWallTile->first + TILE_SIZE, firstWallTile->second, firstWallTile->first + TILE_SIZE,
			firstWallTile->second + TILE_SIZE);
		if (cp4.y >= firstWallTile->second && cp4.y <= firstWallTile->second + TILE_SIZE)
		{
			const auto dist2 = MathHelper::getVectorLength2(Vec2Double(x1 - cp4.x, y1 - cp4.y));
			if (minDistCp == nullptr || dist2 < minDist)
			{
				minDist = dist2;
				minDistCp = &cp4;
			}
		}

		/*if (minDistCp == nullptr)
		{
			throw std::runtime_error("no wall tile cross");
		}*/
		//TODO: ошибка при отправке стратегии
		return minDistCp != nullptr ? *minDistCp : Vec2Double(0,0); 
	}

	return crossPoint;
}

bool Simulator::canGoThroughTile(const Tile& tile, bool jumpDown)
{
	if (tile == WALL) return false;
	return !jumpDown || jumpDown && (tile == LADDER || tile == PLATFORM || tile == EMPTY);
}


Vec2Double Simulator::getBulletCrossWallPoint(const Bullet& bullet, double maxX, double maxY, const Game& game)
{
	double minCrossPointDist2 = INT_MAX;
	const Vec2Double* minCrossPoint = nullptr;

	Vec2Double bulletPosition1 = Vec2Double(bullet.position.x - bullet.size / 2, bullet.position.y - bullet.size / 2);
	const auto cp1 = getBulletCornerCrossWallPoint(bulletPosition1, bullet.velocity, maxX, maxY, game);
	auto dist2 = MathHelper::getVectorLength2(Vec2Double(cp1.x - bulletPosition1.x, cp1.y - bulletPosition1.y));
	if (dist2 < minCrossPointDist2)
	{
		minCrossPointDist2 = dist2;
		minCrossPoint = &cp1;
	}

	Vec2Double bulletPosition2 = Vec2Double(bullet.position.x + bullet.size / 2, bullet.position.y - bullet.size / 2);
	const auto cp2 = getBulletCornerCrossWallPoint(bulletPosition2, bullet.velocity, maxX, maxY, game);
	dist2 = MathHelper::getVectorLength2(Vec2Double(cp2.x - bulletPosition2.x, cp2.y - bulletPosition2.y));
	if (dist2 < minCrossPointDist2)
	{
		minCrossPointDist2 = dist2;
		minCrossPoint = &cp2;
	}

	Vec2Double bulletPosition3 = Vec2Double(bullet.position.x - bullet.size / 2, bullet.position.y + bullet.size / 2);
	const auto cp3 = getBulletCornerCrossWallPoint(bulletPosition3, bullet.velocity, maxX, maxY, game);
	dist2 = MathHelper::getVectorLength2(Vec2Double(cp3.x - bulletPosition3.x, cp3.y - bulletPosition3.y));
	if (dist2 < minCrossPointDist2)
	{
		minCrossPointDist2 = dist2;
		minCrossPoint = &cp3;
	}

	Vec2Double bulletPosition4 = Vec2Double(bullet.position.x + bullet.size / 2, bullet.position.y + bullet.size / 2);
	const auto cp4 = getBulletCornerCrossWallPoint(bulletPosition4, bullet.velocity, maxX, maxY, game);
	dist2 = MathHelper::getVectorLength2(Vec2Double(cp4.x - bulletPosition4.x, cp4.y - bulletPosition4.y));
	if (dist2 < minCrossPointDist2)
	{
		minCrossPointDist2 = dist2;
		minCrossPoint = &cp4;
	}

	if (minCrossPoint == nullptr) throw std::runtime_error("no bullet cross wall point");

	return *minCrossPoint;
}


ShootMeBulletCrossPoint Simulator::get_shoot_me_bullet_cross_point(
	double x1, double y1, double x2, double y2,
	const Vec2Double& bulletPosition, const Vec2Double& bulletVelocity,
	const Game& game)
{
	const auto maxX = game.level.tiles.size() * TILE_SIZE;
	const auto maxY = game.level.tiles[0].size() * TILE_SIZE;

	const auto bulletX1 = bulletPosition.x;
	const auto bulletY1 = bulletPosition.y;
	const auto bulletX2 = bulletPosition.x + bulletVelocity.x;
	const auto bulletY2 = bulletPosition.y + bulletVelocity.y;

	auto cross1 = MathHelper::getLinesCross(bulletX1, bulletY1, bulletX2, bulletY2,
		x1, y1, x1, y2);
	auto cross2 = MathHelper::getLinesCross(bulletX1, bulletY1, bulletX2, bulletY2,
		x1, y2, x2, y2);
	auto cross3 = MathHelper::getLinesCross(bulletX1, bulletY1, bulletX2, bulletY2,
		x2, y2, x2, y1);
	auto cross4 = MathHelper::getLinesCross(bulletX1, bulletY1, bulletX2, bulletY2,
		x2, y1, x1, y1);

	double minCrossDist2Shooting = INT_MAX;
	Vec2Double* minCrossDist2ShootingPoint = nullptr;

	auto dist2 = MathHelper::getVectorLength2(Vec2Double(cross1.x - bulletX1, cross1.y - bulletY1));
	if (cross1.y >= y1 && cross1.y <= y2)
	{
		if (dist2 < minCrossDist2Shooting)
		{
			minCrossDist2Shooting = dist2;
			minCrossDist2ShootingPoint = &cross1;
		}
	}

	dist2 = MathHelper::getVectorLength2(Vec2Double(cross2.x - bulletX1, cross2.y - bulletY1));
	if (cross2.x >= x1 && cross2.x <= x2)
	{
		if (dist2 < minCrossDist2Shooting)
		{
			minCrossDist2Shooting = dist2;
			minCrossDist2ShootingPoint = &cross2;
		}
	}

	dist2 = MathHelper::getVectorLength2(Vec2Double(cross3.x - bulletX1, cross3.y - bulletY1));
	if (cross3.y >= y1 && cross3.y <= y2)
	{
		if (dist2 < minCrossDist2Shooting)
		{
			minCrossDist2Shooting = dist2;
			minCrossDist2ShootingPoint = &cross3;
		}
	}

	dist2 = MathHelper::getVectorLength2(Vec2Double(cross4.x - bulletX1, cross4.y - bulletY1));
	if (cross4.x >= x1 && cross4.x <= x2)
	{
		if (dist2 < minCrossDist2Shooting)
		{
			minCrossDist2Shooting = dist2;
			minCrossDist2ShootingPoint = &cross4;
		}
	}


	return {
		minCrossDist2ShootingPoint != nullptr ? *minCrossDist2ShootingPoint : Vec2Double(0, 0),
		minCrossDist2ShootingPoint != nullptr,
		minCrossDist2Shooting
	};

}





bool Simulator::isBulletCrossWall(const Vec2Double& bulletPosition, const Vec2Double& bulletVelocity, double time,
	const Game& game)
{
	const auto targetPosition = Vec2Double(bulletPosition.x + bulletVelocity.x * time,
		bulletPosition.y + bulletVelocity.y * time);

	const auto bulletTiles = MathHelper::getLineSquares(bulletPosition, targetPosition, 1);
	const std::pair<int, int>* firstWallTile = nullptr;
	for (const auto& bt : bulletTiles)
	{
		if (bt.first == 0 || bt.second == 0 || bt.first == game.level.tiles.size() - 1 || bt.second == game.level.tiles[
			0].size() - 1)
		{
			break; //игнор крайних стен
		}

			if (game.level.tiles[bt.first][bt.second] == WALL)
			{
				firstWallTile = &bt;
				break;
			}
	}

	return firstWallTile != nullptr;
}



Vec2Double Simulator::getBulletInTimePosition(const Bullet& bullet, double time, const Game& game)
{
	return Vec2Double(bullet.position.x + bullet.velocity.x * time, bullet.position.y + bullet.velocity.y * time);
}


//TODO: после конца прыжка или удара о потолок начинается падение
Vec2Double Simulator::getUnitNextTickPosition(
	const Vec2Double& unitPosition, const Vec2Double& unitSize,  const UnitAction& action, const Game& game)
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

	const auto tickTime = 1.0 / game.properties.ticksPerSecond;
	const auto microTickTime = tickTime / game.properties.updatesPerTick;

	auto nextX = x + velocityX * tickTime;
	auto nextY = y + velocityY * tickTime;

	auto leftBottomTile = game.level.tiles[size_t(nextX - unitSize.x / 2)][size_t(nextY)];
	auto leftTopTile = game.level.tiles[size_t(nextX - unitSize.x / 2)][size_t(nextY + unitSize.y)];
	auto rightTopTile = game.level.tiles[size_t(nextX + unitSize.x / 2)][size_t(nextY + unitSize.y)];
	auto rightBottomTile = game.level.tiles[size_t(nextX + unitSize.x / 2)][size_t(nextY)];

	auto canGoThrough = 
		canGoThroughTile(leftBottomTile, action.jumpDown) && canGoThroughTile(leftTopTile, action.jumpDown) && 
		canGoThroughTile(rightTopTile, action.jumpDown) && canGoThroughTile(rightBottomTile, action.jumpDown);

	auto canGoAction = false;
	if (abs(velocityX - actionVelocityX) > TOLERACNE )
	{
		if (actionVelocityX > 0 && game.level.tiles[size_t(x + unitSize.x / 2 + TOLERACNE)][size_t(y)] != WALL &&
								   game.level.tiles[size_t(x + unitSize.x / 2 + TOLERACNE)][size_t(y + unitSize.y)] != WALL ||
			actionVelocityX < 0 && game.level.tiles[size_t(x - unitSize.x / 2 - TOLERACNE)][size_t(y)] != WALL &&
								   game.level.tiles[size_t(x - unitSize.x / 2 - TOLERACNE)][size_t(y + unitSize.y)] != WALL)			
		{
			canGoAction = true;
		}
	}
	else if (abs(velocityY - actionVelocityY) > TOLERACNE)
	{
		if (actionVelocityY > 0 && game.level.tiles[size_t(x - unitSize.x / 2)][size_t(y + unitSize.y + TOLERACNE)] != WALL &&
			game.level.tiles[size_t(x + unitSize.x / 2)][size_t(y + unitSize.y + TOLERACNE)] != WALL ||
			actionVelocityY < 0 && game.level.tiles[size_t(x - unitSize.x / 2)][size_t(y - TOLERACNE)] == EMPTY &&
			game.level.tiles[size_t(x + unitSize.x / 2)][size_t(y - TOLERACNE)] == EMPTY)
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

	//симуляция по микротикам

	if (!canGoThrough)
	{
		const auto startTickVelocityX = velocityX;
		const auto startTickVelocityY = velocityY;
		for (int j = 0; j < game.properties.updatesPerTick; ++j)
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
			
			if (abs(velocityX) > TOLERACNE)//отключаем горизонтальное движение
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

			if (abs(velocityY) > TOLERACNE)//отключаем вертикальное движение
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

			//не можем идти ни по горизонтали, ни по вертикали
			return { x, y };
		}
	}

	else//canGoAction
	{
		/*auto tickVelocityX = actionVelocityX;
		auto tickVelocityY = actionVelocityY;*/

		for (int j = 0; j < game.properties.updatesPerTick; ++j)
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

			if (abs(actionVelocityX) > 0)//отключаем горизонтальное движение
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

			if (abs(actionVelocityY) > 0)//отключаем вертикальное движение
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

			//пока двигаемся со старыми скоростями
			x += velocityX * microTickTime;
			y += velocityY * microTickTime;
		}
	}
	
	return { x, y };
}

bool Simulator::isUnitOnWall(const Vec2Double& unitPosition, const Vec2Double& unitSize, const Game& game)
{
	const auto leftSideDownTile = game.level.tiles[size_t(unitPosition.x - unitSize.x / 2)][size_t(
		unitPosition.y - TOLERACNE)];
	const auto rightSideDownTile = game.level.tiles[size_t(unitPosition.x + unitSize.x / 2)][size_t(
		unitPosition.y - TOLERACNE)];
	return leftSideDownTile == WALL || rightSideDownTile == WALL;
}


bool Simulator::isUnitOnLadder(const Vec2Double& unitPosition, const Vec2Double& unitSize, const Game& game)
{
	/*const auto centerUnitTile = game.level.tiles[size_t(unit.position.x)][size_t(unit.position.y + unit.size.y / 2)];
	const auto downUnitTile = game.level.tiles[size_t(unit.position.x)][size_t(unit.position.y)];
	return centerUnitTile == LADDER || downUnitTile == LADDER;*/

	if (isUnitOnWall(unitPosition, unitSize, game)) return false;

	const auto leftSideDownTile = game.level.tiles[size_t(unitPosition.x - unitSize.x / 2)][size_t(
		unitPosition.y - TOLERACNE)];
	const auto rightSideDownTile = game.level.tiles[size_t(unitPosition.x + unitSize.x / 2)][size_t(
		unitPosition.y - TOLERACNE)];

	return leftSideDownTile == LADDER || rightSideDownTile == LADDER;
	
}


bool Simulator::isUnitOnPlatform(const Vec2Double& unitPosition, const Vec2Double& unitSize, const Game& game)
{
	if (isUnitOnWall(unitPosition, unitSize, game)) return false;
	
	const auto leftSideDownTile = game.level.tiles[size_t(unitPosition.x - unitSize.x / 2)][size_t(
		unitPosition.y - TOLERACNE)];
	const auto rightSideDownTile = game.level.tiles[size_t(unitPosition.x + unitSize.x / 2)][size_t(
		unitPosition.y - TOLERACNE)];

	return leftSideDownTile == PLATFORM || rightSideDownTile == PLATFORM;
}

bool Simulator::isUnitOnAir(const Vec2Double& unitPosition, const Vec2Double& unitSize, const Game& game)
{
	return !isUnitOnWall(unitPosition, unitSize, game) && !isUnitOnLadder(unitPosition, unitSize, game) && !isUnitOnPlatform(unitPosition, unitSize, game);
}


