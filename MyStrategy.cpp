#define _USE_MATH_DEFINES

#include "MyStrategy.hpp"
#include "MathHelper.h"
#include "Helper.h"
#include <utility>
#include <algorithm>
#include <math.h>
#include <climits>
#include "ShootMeBullet.h"
#include "ShootMeBulletCrossPoint.h"
#include <map>
#include <tuple>
#include "Segment.h"
#include "RunawayDirection.h"

#include <sstream>

using namespace std;


MyStrategy::MyStrategy()
{
}

inline bool operator<(const Bullet& lhs, const Bullet& rhs)
{
	return lhs.position.x < rhs.position.x;
}

double distanceSqr(Vec2Double a, Vec2Double b)
{
	return (a.x - b.x) * (a.x - b.x) + (a.y - b.x) * (a.y - b.y);
}

//TODO: учесть граничный TILE
Vec2Double getBulletCrossBorderPoint(const Vec2Double& bulletPosition, const Vec2Double& bulletVelocity, double maxX,
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

Vec2Double getBulletCornerCrossWallPoint(
	const Vec2Double& bulletPosition, const Vec2Double& bulletVelocity, double maxX, double maxY, const Game& game)
{
	auto crossPoint = getBulletCrossBorderPoint(bulletPosition, bulletVelocity, maxX, maxY);
	const auto bulletTiles = MathHelper::getLineSquares(bulletPosition, crossPoint, 1);
	const pair<int, int>* firstWallTile = nullptr;

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

		if (minDistCp == nullptr)
		{
			throw runtime_error("no wall tile cross");
		}
		return *minDistCp;
	}

	return crossPoint;
}

Vec2Double getBulletCrossWallPoint(const Bullet& bullet, double maxX, double maxY, const Game& game)
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

	if (minCrossPoint == nullptr) throw runtime_error("no bullet cross wall point");

	return *minCrossPoint;
}


Vec2Double getShootingCrossBorderPoint(const Vec2Double& position, double lastAngle, double maxX, double maxY)
{
	if (abs(lastAngle) < TOLERACNE)
	{
		return Vec2Double(maxX, position.y);
	}

	if (abs(lastAngle - M_PI) < TOLERACNE)
	{
		return Vec2Double(0, position.y);
	}

	if (abs(lastAngle - M_PI / 2) < TOLERACNE)
	{
		return Vec2Double(position.x, 0);
	}

	if (abs(lastAngle + M_PI / 2) < TOLERACNE)
	{
		return Vec2Double(position.x, maxY);
	}

	const auto x1 = position.x;
	const auto y1 = position.y;
	const auto x2 = position.x + cos(lastAngle);
	const auto y2 = position.y + sin(lastAngle);

	if (abs(lastAngle) < M_PI / 2)
	{
		const auto vertCross = MathHelper::getLinesCross(x1, y1, x2, y2, maxX, 0, maxX, maxY);
		if (vertCross.y >= 0 && vertCross.y < maxY)
		{
			return vertCross;
		}

		return lastAngle > 0
			       ? MathHelper::getLinesCross(x1, y1, x2, y2, 0, maxY, maxX, maxY)
			       : MathHelper::getLinesCross(x1, y1, x2, y2, 0, 0, maxX, 0);
	}
	const auto vertCross = MathHelper::getLinesCross(x1, y1, x2, y2, 0, 0, 0, maxY);
	if (vertCross.y >= 0 && vertCross.y < maxY)
	{
		return vertCross;
	}

	return lastAngle > 0
		       ? MathHelper::getLinesCross(x1, y1, x2, y2, 0, maxY, maxX, maxY)
		       : MathHelper::getLinesCross(x1, y1, x2, y2, 0, 0, maxX, 0);
}

void drawBullets(Debug& debug, const Game& game, int meId)
{
	const auto maxX = game.level.tiles.size() * TILE_SIZE;
	const auto maxY = game.level.tiles[0].size() * TILE_SIZE;
	for (const auto& bullet : game.bullets)
	{
		auto crossPoint = getBulletCrossWallPoint(bullet, maxX, maxY, game);

		const auto debugBullet = vec2DoubleToVec2Float(bullet.position);
		const auto debugCrossPoint = vec2DoubleToVec2Float(crossPoint);
		debug.draw(CustomData::Line(debugBullet, debugCrossPoint, 0.1,
		                            ColorFloat(bullet.playerId == meId ? 0 : 255, bullet.playerId == meId ? 255 : 0, 0,
		                                       0.25)));
	}
}

void drawShootingLine(
	Debug& debug, const Game& game, const Vec2Double& weaponPoistion, double angle, double maxX, double maxY,
	const ColorFloat& color)
{
	auto crossPoint = getShootingCrossBorderPoint(
		weaponPoistion,
		angle, maxX, maxY);

	const auto bulletTiles = MathHelper::getLineSquares(weaponPoistion, crossPoint, 1);
	const pair<int, int>* firstWallTile = nullptr;
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
		//TODO: брать нормальное пересечение со стеной
		crossPoint = Vec2Double(firstWallTile->first + 0.5, firstWallTile->second + 0.5);
	}

	const auto debugUnit = vec2DoubleToVec2Float(weaponPoistion);
	const auto debugCrossPoint = vec2DoubleToVec2Float(crossPoint);
	debug.draw(CustomData::Line(debugUnit, debugCrossPoint, 0.1, color));
}


void drawShootingSector(Debug& debug, const Unit& unit, const Game& game)
{
	if (unit.weapon == nullptr || (*unit.weapon).lastAngle == nullptr) return;

	const auto maxX = game.level.tiles.size() * TILE_SIZE;
	const auto maxY = game.level.tiles[0].size() * TILE_SIZE;

	auto angle1 = *(*unit.weapon).lastAngle - (*unit.weapon).spread;
	if (angle1 > M_PI) angle1 -= 2 * M_PI;
	else if (angle1 < -M_PI) angle1 += 2 * M_PI;

	auto angle2 = *(*unit.weapon).lastAngle + (*unit.weapon).spread;
	if (angle2 > M_PI) angle2 -= 2 * M_PI;
	else if (angle2 < -M_PI) angle2 += 2 * M_PI;


	const auto weaponPoistion = Vec2Double(unit.position.x, unit.position.y + unit.size.y / 2);
	//drawShootingLine(debug, game, weaponPoistion, *(*unit.weapon).lastAngle, maxX, maxY, ColorFloat(0, 0, 255, 0.5));
	drawShootingLine(debug, game, weaponPoistion, angle1, maxX, maxY, ColorFloat(0, 0, 255, 0.5));
	drawShootingLine(debug, game, weaponPoistion, angle2, maxX, maxY, ColorFloat(0, 0, 255, 0.5));
}


ShootMeBulletCrossPoint get_shoot_me_bullet_cross_point(
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

	//const auto crossWallPoint = getBulletCrossWallPoint(bullet, maxX, maxY, game);
	//auto wallDist = MathHelper::getVectorLength(Vec2Double(bullet.position.x - crossWallPoint.x, bullet.position.y - crossWallPoint.y));
	//auto bulletVelocity = MathHelper::getVectorLength(bullet.velocity);
	//int shootWallTick = (int)(ceil(wallDist / bulletVelocity * game.properties.ticksPerSecond));

	//if (cross1.y >= y1 && cross1.y <= y2) {
	//	const auto bulletTiles = MathHelper::getLineSquares(bullet.position, cross1, 1);
	//	const pair<int, int>* firstWallTile = nullptr;
	//	for (const auto& bt : bulletTiles) {
	//		if (bt.first == 0 || bt.second == 0 || bt.first == game.level.tiles.size() - 1 || bt.second == game.level.tiles[0].size() - 1) {
	//			break; //игнор крайних стен
	//		}

	//		if (game.level.tiles[bt.first][bt.second] == Tile::WALL) {
	//			firstWallTile = &bt;
	//			break;
	//		}
	//	}

	//	if (firstWallTile == nullptr) {
	//		auto dist = MathHelper::getVectorLength(Vec2Double(bullet.position.x - cross1.x, bullet.position.y - cross1.y));
	//		int shootMeTick = (int)(ceil(dist / bulletVelocity * game.properties.ticksPerSecond));
	//		return make_pair(shootMeTick, shootWallTick);
	//	}
	//}

	//if (cross2.x >= x1 && cross2.x <= x2) {
	//	const auto bulletTiles = MathHelper::getLineSquares(bullet.position, cross2, 1);
	//	const pair<int, int>* firstWallTile = nullptr;
	//	for (const auto& bt : bulletTiles) {
	//		if (bt.first == 0 || bt.second == 0 || bt.first == game.level.tiles.size() - 1 || bt.second == game.level.tiles[0].size() - 1) {
	//			break; //игнор крайних стен
	//		}

	//		if (game.level.tiles[bt.first][bt.second] == Tile::WALL) {
	//			firstWallTile = &bt;
	//			break;
	//		}
	//	}

	//	if (firstWallTile == nullptr) {
	//		auto dist = MathHelper::getVectorLength(Vec2Double(bullet.position.x - cross2.x, bullet.position.y - cross2.y));
	//		int shootMeTick = (int)(ceil(dist / bulletVelocity * game.properties.ticksPerSecond));
	//		return make_pair(shootMeTick, shootWallTick);
	//	}
	//}

	//if (cross3.y >= y1 && cross3.y <= y2) {
	//	const auto bulletTiles = MathHelper::getLineSquares(bullet.position, cross3, 1);
	//	const pair<int, int>* firstWallTile = nullptr;
	//	for (const auto& bt : bulletTiles) {
	//		if (bt.first == 0 || bt.second == 0 || bt.first == game.level.tiles.size() - 1 || bt.second == game.level.tiles[0].size() - 1) {
	//			break; //игнор крайних стен
	//		}

	//		if (game.level.tiles[bt.first][bt.second] == Tile::WALL) {
	//			firstWallTile = &bt;
	//			break;
	//		}
	//	}

	//	if (firstWallTile == nullptr) {
	//		auto dist = MathHelper::getVectorLength(Vec2Double(bullet.position.x - cross3.x, bullet.position.y - cross3.y));
	//		int shootMeTick = (int)(ceil(dist / bulletVelocity * game.properties.ticksPerSecond));
	//		return make_pair(shootMeTick, shootWallTick);
	//	}
	//}

	//if (cross4.x >= x1 && cross4.x <= x2) {
	//	const auto bulletTiles = MathHelper::getLineSquares(bullet.position, cross4, 1);
	//	const pair<int, int>* firstWallTile = nullptr;
	//	for (const auto& bt : bulletTiles) {
	//		if (bt.first == 0 || bt.second == 0 || bt.first == game.level.tiles.size() - 1 || bt.second == game.level.tiles[0].size() - 1) {
	//			break; //игнор крайних стен
	//		}

	//		if (game.level.tiles[bt.first][bt.second] == Tile::WALL) {
	//			firstWallTile = &bt;
	//			break;
	//		}
	//	}

	//	if (firstWallTile == nullptr) {
	//		auto dist = MathHelper::getVectorLength(Vec2Double(bullet.position.x - cross4.x, bullet.position.y - cross4.y));
	//		int shootMeTick = (int)(ceil(dist / bulletVelocity * game.properties.ticksPerSecond));
	//		return make_pair(shootMeTick, shootWallTick);
	//	}
	//}
}


bool isBulletCrossWall(const Vec2Double& bulletPosition, const Vec2Double& bulletVelocity, double time,
                       const Game& game)
{
	const auto targetPosition = Vec2Double(bulletPosition.x + bulletVelocity.x * time,
	                                       bulletPosition.y + bulletVelocity.y * time);

	const auto bulletTiles = MathHelper::getLineSquares(bulletPosition, targetPosition, 1);
	const pair<int, int>* firstWallTile = nullptr;
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

double getShootEnemyProbability(const Unit& me, const Unit& enemy, const Game& game, double spread,
                                Debug* debug = nullptr)
{
	if (me.weapon == nullptr) return 0;
	if (me.weapon->lastAngle == nullptr) return 1;

	const auto bulletCenterPos = Vec2Double(me.position.x, me.position.y + me.size.y / 2);
	const auto deltaAngle = spread / ANGLE_SPLIT_COUNT;

	const auto x1 = enemy.position.x - enemy.size.x / 2;
	const auto x2 = enemy.position.x + enemy.size.x / 2;
	const auto y1 = enemy.position.y;
	const auto y2 = enemy.position.y + enemy.size.y;

	auto shootingCount = 0;

	for (auto i = -ANGLE_SPLIT_COUNT; i <= ANGLE_SPLIT_COUNT; ++i)
	{
		auto isShooting = false;
		double minCrossDist2 = INT_MAX;
		Vec2Double cp = Vec2Double(0, 0);

		const auto bulletVelocityLength = (*me.weapon).params.bullet.speed;

		const auto angle = *me.weapon->lastAngle + deltaAngle * i;
		const auto bulletPos1 = Vec2Double(
			bulletCenterPos.x - me.weapon->params.bullet.size / 2,
			bulletCenterPos.y - me.weapon->params.bullet.size / 2);
		auto bulletVelocity = Vec2Double(bulletVelocityLength * cos(angle), bulletVelocityLength * sin(angle));
		auto shootEnemyCrossPoint = get_shoot_me_bullet_cross_point(
			x1, y1, x2, y2, bulletPos1, bulletVelocity, game);

		if (shootEnemyCrossPoint.hasCrossPoint)
		{
			isShooting = true;
			cp = shootEnemyCrossPoint.crossPoint;
			if (shootEnemyCrossPoint.crossPointDist2 < minCrossDist2)
			{
				minCrossDist2 = shootEnemyCrossPoint.crossPointDist2;
			}
		}


		const auto bulletPos2 = Vec2Double(
			bulletCenterPos.x - me.weapon->params.bullet.size / 2,
			bulletCenterPos.y + me.weapon->params.bullet.size / 2);
		shootEnemyCrossPoint = get_shoot_me_bullet_cross_point(
			x1, y1, x2, y2, bulletPos2, bulletVelocity, game);

		if (shootEnemyCrossPoint.hasCrossPoint)
		{
			isShooting = true;
			cp = shootEnemyCrossPoint.crossPoint;
			if (shootEnemyCrossPoint.crossPointDist2 < minCrossDist2)
			{
				minCrossDist2 = shootEnemyCrossPoint.crossPointDist2;
			}
		}


		const auto bulletPos3 = Vec2Double(
			bulletCenterPos.x + me.weapon->params.bullet.size / 2,
			bulletCenterPos.y - me.weapon->params.bullet.size / 2);
		shootEnemyCrossPoint = get_shoot_me_bullet_cross_point(
			x1, y1, x2, y2, bulletPos3, bulletVelocity, game);

		if (shootEnemyCrossPoint.hasCrossPoint)
		{
			isShooting = true;
			cp = shootEnemyCrossPoint.crossPoint;
			if (shootEnemyCrossPoint.crossPointDist2 < minCrossDist2)
			{
				minCrossDist2 = shootEnemyCrossPoint.crossPointDist2;
			}
		}

		const auto bulletPos4 = Vec2Double(
			bulletCenterPos.x + me.weapon->params.bullet.size / 2,
			bulletCenterPos.y + me.weapon->params.bullet.size / 2);
		shootEnemyCrossPoint = get_shoot_me_bullet_cross_point(
			x1, y1, x2, y2, bulletPos4, bulletVelocity, game);

		if (shootEnemyCrossPoint.hasCrossPoint)
		{
			isShooting = true;
			cp = shootEnemyCrossPoint.crossPoint;
			if (shootEnemyCrossPoint.crossPointDist2 < minCrossDist2)
			{
				minCrossDist2 = shootEnemyCrossPoint.crossPointDist2;
			}
		}


		if (isShooting)
		{
			const auto dist = sqrt(minCrossDist2);
			const auto time = dist / bulletVelocityLength;

			const auto isCrossWall =
				isBulletCrossWall(bulletPos1, bulletVelocity, time, game) ||
				isBulletCrossWall(bulletPos2, bulletVelocity, time, game) ||
				isBulletCrossWall(bulletPos3, bulletVelocity, time, game) ||
				isBulletCrossWall(bulletPos4, bulletVelocity, time, game);

			if (!isCrossWall)
			{
				shootingCount++;
				/*if (debug != nullptr) {
					(*debug).draw(CustomData::Line(
						vec2DoubleToVec2Float(bulletCenterPos),
						vec2DoubleToVec2Float(cp),
						0.1, ColorFloat(100, 100, 100, 0.5)));
				}*/
			}
		}
	}

	return shootingCount * 1.0 / (2 * ANGLE_SPLIT_COUNT + 1.0);
}


map<Bullet, double> getEnemyBulletsShootWallTimes(const Game& game, int meId)
{
	const auto maxX = game.level.tiles.size() * TILE_SIZE;
	const auto maxY = game.level.tiles[0].size() * TILE_SIZE;

	map<Bullet, double> result;
	for (const auto& bullet : game.bullets)
	{
		if (bullet.playerId == meId) continue;

		const auto crossWallPoint = getBulletCrossWallPoint(bullet, maxX, maxY, game);
		const auto wallDist = MathHelper::getVectorLength(
			Vec2Double(bullet.position.x - crossWallPoint.x, bullet.position.y - crossWallPoint.y));

		const auto bulletVelocity = MathHelper::getVectorLength(bullet.velocity);

		const double shootWallTime = wallDist / bulletVelocity;
		result[bullet] = shootWallTime;
	}

	return result;
}

int getShootMeBulletTick(const Unit& me, const Bullet& bullet, const Game& game)
{
	auto x1 = me.position.x - me.size.x / 2;
	auto x2 = me.position.x + me.size.x / 2;
	auto y1 = me.position.y;
	auto y2 = me.position.y + me.size.y;

	if (bullet.velocity.x > 0 && bullet.position.x - bullet.size/2 > x2) return -1;
	if (bullet.velocity.x < 0 && bullet.position.x + bullet.size/2 < x1) return -1;
	if (bullet.velocity.y > 0 && bullet.position.y - bullet.size/2 > y2) return -1;
	if (bullet.velocity.y < 0 && bullet.position.y + bullet.size/2 < y1) return -1;


	double minCrossDist2 = INT_MAX;
	const Vec2Double* minCrossDist2Point = nullptr;

	const auto bulletPosition1 = Vec2Double(bullet.position.x - bullet.size / 2, bullet.position.y - bullet.size / 2);
	const auto smbcp1 = get_shoot_me_bullet_cross_point(x1, y1, x2, y2, bulletPosition1, bullet.velocity, game);
	if (smbcp1.hasCrossPoint)
	{
		if (smbcp1.crossPointDist2 < minCrossDist2)
		{
			minCrossDist2 = smbcp1.crossPointDist2;
			minCrossDist2Point = &smbcp1.crossPoint;
		}
	}

	const auto bulletPosition2 = Vec2Double(bullet.position.x + bullet.size / 2, bullet.position.y - bullet.size / 2);
	const auto smbcp2 = get_shoot_me_bullet_cross_point(x1, y1, x2, y2, bulletPosition2, bullet.velocity, game);
	if (smbcp2.hasCrossPoint)
	{
		if (smbcp2.crossPointDist2 < minCrossDist2)
		{
			minCrossDist2 = smbcp2.crossPointDist2;
			minCrossDist2Point = &smbcp2.crossPoint;
		}
	}

	const auto bulletPosition3 = Vec2Double(bullet.position.x + bullet.size / 2, bullet.position.y + bullet.size / 2);
	const auto smbcp3 = get_shoot_me_bullet_cross_point(x1, y1, x2, y2, bulletPosition3, bullet.velocity, game);
	if (smbcp3.hasCrossPoint)
	{
		if (smbcp3.crossPointDist2 < minCrossDist2)
		{
			minCrossDist2 = smbcp3.crossPointDist2;
			minCrossDist2Point = &smbcp3.crossPoint;
		}
	}


	const auto bulletPosition4 = Vec2Double(bullet.position.x + bullet.size / 2, bullet.position.y + bullet.size / 2);
	const auto smbcp4 = get_shoot_me_bullet_cross_point(x1, y1, x2, y2, bulletPosition4, bullet.velocity, game);
	if (smbcp4.hasCrossPoint)
	{
		if (smbcp4.crossPointDist2 < minCrossDist2)
		{
			minCrossDist2 = smbcp4.crossPointDist2;
			minCrossDist2Point = &smbcp4.crossPoint;
		}
	}

	if (minCrossDist2Point != nullptr)
	{
		auto bulletVelocityLength = MathHelper::getVectorLength(bullet.velocity);
		auto minCrossDist = sqrt(minCrossDist2);

		const auto time = minCrossDist / bulletVelocityLength;
		const auto isCrossWall =
			isBulletCrossWall(bulletPosition1, bullet.velocity, time, game) ||
			isBulletCrossWall(bulletPosition2, bullet.velocity, time, game) ||
			isBulletCrossWall(bulletPosition3, bullet.velocity, time, game) ||
			isBulletCrossWall(bulletPosition4, bullet.velocity, time, game);

		if (isCrossWall) return -1;

		int shootMeTick = static_cast<int>(ceil(minCrossDist / bulletVelocityLength * game.properties.ticksPerSecond));
		return shootMeTick;
	}


	return -1;
}

Vec2Double getBulletPosition(const Bullet& bullet, double time, const Game& game)
{
	return Vec2Double(bullet.position.x + bullet.velocity.x * time, bullet.position.y + bullet.velocity.y * time);
}

vector<ShootMeBullet> getShootMeBullets(const Unit& unit, const Game& game)
{
	vector<ShootMeBullet> result;
	for (const auto& bullet : game.bullets)
	{
		if (bullet.playerId == unit.playerId) continue;
		const auto smbt = getShootMeBulletTick(unit, bullet, game);
		if (smbt == -1) continue;
		result.emplace_back(ShootMeBullet(bullet, smbt));
	}

	return result;
}

//TODO: учесть особенности карты
Vec2Double getUnitGoSidePosition(
	const Unit& unit, int startGoTick, int stopGoTick, double time, int coeff, const Game& game)
{
	if (time * game.properties.ticksPerSecond <= startGoTick)
	{
		return unit.position;
	}

	const auto startGoTime = startGoTick / game.properties.ticksPerSecond;

	if (time * game.properties.ticksPerSecond <= stopGoTick)
	{
		const auto goTime = time - startGoTime;
		const auto dx = game.properties.unitMaxHorizontalSpeed * goTime * coeff;
		return Vec2Double(unit.position.x + dx, unit.position.y);
	}

	const auto stopGoTime = stopGoTick / game.properties.ticksPerSecond;
	const auto goTime = stopGoTime - startGoTime;
	const auto dx = game.properties.unitMaxHorizontalSpeed * goTime * coeff;
	return Vec2Double(unit.position.x + dx, unit.position.y);
}

//TODO: учесть особенности карты
Vec2Double getFallingUnitPosition(
	const Unit& unit, int startGoTick, int stopGoTick, double time, const Game& game)
{
	if (time * game.properties.ticksPerSecond <= startGoTick)
	{
		return unit.position;
	}

	const auto startGoTime = startGoTick / game.properties.ticksPerSecond;

	if (time * game.properties.ticksPerSecond <= stopGoTick || stopGoTick == -1)
	{
		//stopGoTick == -1 - для платформы, не можем остановиться
		const auto goTime = time - startGoTime;
		return Vec2Double(unit.position.x, unit.position.y - game.properties.unitFallSpeed * goTime);
	}

	const auto stopGoTime = stopGoTick / game.properties.ticksPerSecond;
	const auto goTime = stopGoTime - startGoTime;
	return Vec2Double(unit.position.x, unit.position.y - game.properties.unitFallSpeed * goTime);
}


//TODO: учесть максимальное время прыжка
Vec2Double getJumpingUnitPosition(const Unit& unit, int startJumpTick, int stopJumpTick, double time, const Game& game)
{
	if (time * game.properties.ticksPerSecond <= startJumpTick)
	{
		return unit.position;
	}

	const auto startJumpTime = startJumpTick / game.properties.ticksPerSecond;

	if (time * game.properties.ticksPerSecond <= stopJumpTick)
	{
		double jumpTime = time - startJumpTime;
		return Vec2Double(unit.position.x, unit.position.y + game.properties.unitJumpSpeed * jumpTime);
	}

	const auto stopJumpTime = stopJumpTick / game.properties.ticksPerSecond;

	const auto jumpTime = stopJumpTime - startJumpTime;
	auto fallTime = time - stopJumpTime;
	if (fallTime > jumpTime) fallTime = jumpTime; //TODO: случай, когда прыгаем не с земли, можем провалиться ниже
	return Vec2Double(unit.position.x,
	                  unit.position.y + game.properties.unitJumpSpeed * jumpTime - game.properties.unitFallSpeed *
	                  fallTime);
}


bool isBulletMoveCrossUnitMove(
	const Vec2Double& unitPos0, const Vec2Double& unitPos1,
	const Vec2Double& bulletPos0, const Vec2Double& bulletPos1,
	const Vec2Double& unitSize, double halfBulletSize)
{
	const auto unitLeftDown0 = Vec2Double(unitPos0.x - unitSize.x / 2, unitPos0.y);
	const auto unitRightDown0 = Vec2Double(unitPos0.x + unitSize.x / 2, unitPos0.y);
	const auto unitLeftUp0 = Vec2Double(unitPos0.x - unitSize.x / 2, unitPos0.y + unitSize.y);
	const auto unitRightUp0 = Vec2Double(unitPos0.x + unitSize.x / 2, unitPos0.y + unitSize.y);

	const auto unitLeftDown1 = Vec2Double(unitPos1.x - unitSize.x / 2, unitPos1.y);
	const auto unitRightDown1 = Vec2Double(unitPos1.x + unitSize.x / 2, unitPos1.y);
	const auto unitLeftUp1 = Vec2Double(unitPos1.x - unitSize.x / 2, unitPos1.y + unitSize.y);
	const auto unitRightUp1 = Vec2Double(unitPos1.x + unitSize.x / 2, unitPos1.y + unitSize.y);

	const Segment unitSegments[] = {
		Segment(unitLeftDown0, unitLeftUp0),
		Segment(unitLeftUp0, unitRightUp0),
		Segment(unitRightUp0, unitRightDown0),
		Segment(unitRightDown0, unitLeftDown0),

		Segment(unitLeftDown1, unitLeftUp1),
		Segment(unitLeftUp1, unitRightUp1),
		Segment(unitRightUp1, unitRightDown1),
		Segment(unitRightDown1, unitLeftDown1),

		Segment(unitLeftDown0, unitLeftDown1),
		Segment(unitLeftUp0, unitLeftUp1),
		Segment(unitRightUp0, unitRightUp1),
		Segment(unitRightDown0, unitRightUp1)
	};

	const auto bulletLeftDown0 = Vec2Double(bulletPos0.x - halfBulletSize, bulletPos0.y - halfBulletSize);
	const auto bulletLeftUp0 = Vec2Double(bulletPos0.x - halfBulletSize, bulletPos0.y + halfBulletSize);
	const auto bulletRightUp0 = Vec2Double(bulletPos0.x + halfBulletSize, bulletPos0.y + halfBulletSize);
	const auto bulletRightDown0 = Vec2Double(bulletPos0.x + halfBulletSize, bulletPos0.y - halfBulletSize);

	const auto bulletLeftDown1 = Vec2Double(bulletPos1.x - halfBulletSize, bulletPos1.y - halfBulletSize);
	const auto bulletLeftUp1 = Vec2Double(bulletPos1.x - halfBulletSize, bulletPos1.y + halfBulletSize);
	const auto bulletRightUp1 = Vec2Double(bulletPos1.x + halfBulletSize, bulletPos1.y + halfBulletSize);
	const auto bulletRightDown1 = Vec2Double(bulletPos1.x + halfBulletSize, bulletPos1.y - halfBulletSize);

	const Segment bulletSegments[] = {
		Segment(bulletLeftDown0, bulletLeftDown1),
		Segment(bulletLeftUp0, bulletLeftUp1),
		Segment(bulletRightUp0, bulletRightUp1),
		Segment(bulletRightDown0, bulletRightDown1)
	};

	for (const auto& us : unitSegments)
	{
		for (const auto& bs : bulletSegments)
		{
			const auto cross = MathHelper::areSegmentsCross(us, bs);
			if (cross) return true;
		}
	}
	return false;
}

bool isBulletShootGoSideUnit(
	const Bullet& bullet, const Unit& unit, int startGoTick, int stopGoTick,
	double bulletShootWallTime, int coeff, const Game& game, int& killTick)
{
	const auto shootWallTick = static_cast<int>(ceil(bulletShootWallTime * game.properties.ticksPerSecond));

	auto prevUnitPos = unit.position;
	auto prevBulletPos = bullet.position;
	for (int tick = 1; tick <= shootWallTick; ++tick)
	{
		const auto time = tick / game.properties.ticksPerSecond;
		auto unitPos = getUnitGoSidePosition(unit, startGoTick, stopGoTick, time, coeff, game);
		auto bulletPos = getBulletPosition(bullet, time, game);
		if (!isBulletMoveCrossUnitMove(prevUnitPos, unitPos, prevBulletPos, bulletPos, unit.size, bullet.size / 2))
		{
			prevUnitPos = unitPos;
			prevBulletPos = bulletPos;
			continue;
		}


		//считаем по микротику
		for (int j = 1; j <= game.properties.updatesPerTick; ++j)
		{
			const auto mtTime = (tick - 1 + j * 1.0 / game.properties.updatesPerTick) / game.properties.ticksPerSecond;
			unitPos = getUnitGoSidePosition(unit, startGoTick, stopGoTick, mtTime, coeff, game);
			bulletPos = getBulletPosition(bullet, mtTime, game);
			if (isBulletMoveCrossUnitMove(prevUnitPos, unitPos, prevBulletPos, bulletPos, unit.size, bullet.size / 2))
			{
				killTick = min(killTick, tick);
				return true;
			}


			prevUnitPos = unitPos;
			prevBulletPos = bulletPos;
		}

		prevUnitPos = unitPos;
		prevBulletPos = bulletPos;
	}

	return false;
}

bool isBulletShootJumpingUnit(
	const Bullet& bullet, const Unit& unit, int startJumpTick, int stopJumpTick,
	double bulletShootWallTime, const Game& game, int& killTick)
{
	const auto shootWallTick = static_cast<int>(ceil(bulletShootWallTime * game.properties.ticksPerSecond));

	auto prevUnitPos = unit.position;
	auto prevBulletPos = bullet.position;
	for (int tick = 1; tick <= shootWallTick; ++tick)
	{
		const auto time = tick / game.properties.ticksPerSecond;
		auto unitPos = getJumpingUnitPosition(unit, startJumpTick, stopJumpTick, time, game);
		auto bulletPos = getBulletPosition(bullet, time, game);
		if (!isBulletMoveCrossUnitMove(prevUnitPos, unitPos, prevBulletPos, bulletPos, unit.size, bullet.size / 2))
		{
			prevUnitPos = unitPos;
			prevBulletPos = bulletPos;
			continue;
		}


		//считаем по микротику
		for (int j = 1; j <= game.properties.updatesPerTick; ++j)
		{
			const auto mtTime = (tick - 1 + j * 1.0 / game.properties.updatesPerTick) / game.properties.ticksPerSecond;
			unitPos = getJumpingUnitPosition(unit, startJumpTick, stopJumpTick, mtTime, game);
			bulletPos = getBulletPosition(bullet, mtTime, game);
			if (isBulletMoveCrossUnitMove(prevUnitPos, unitPos, prevBulletPos, bulletPos, unit.size, bullet.size / 2))
			{
				killTick = min(killTick, tick);
				return true;
			}


			prevUnitPos = unitPos;
			prevBulletPos = bulletPos;
		}

		prevUnitPos = unitPos;
		prevBulletPos = bulletPos;
	}

	return false;
}

bool isBulletShootFallingUnit(
	const Bullet& bullet, const Unit& unit, int startJumpTick, int stopJumpTick,
	double bulletShootWallTime, const Game& game, int& killTick)
{
	const auto shootWallTick = static_cast<int>(ceil(bulletShootWallTime * game.properties.ticksPerSecond));

	auto prevUnitPos = unit.position;
	auto prevBulletPos = bullet.position;
	for (int tick = 1; tick <= shootWallTick; ++tick)
	{
		const auto time = tick / game.properties.ticksPerSecond;
		auto unitPos = getFallingUnitPosition(unit, startJumpTick, stopJumpTick, time, game);
		auto bulletPos = getBulletPosition(bullet, time, game);
		if (!isBulletMoveCrossUnitMove(prevUnitPos, unitPos, prevBulletPos, bulletPos, unit.size, bullet.size / 2))
		{
			prevUnitPos = unitPos;
			prevBulletPos = bulletPos;
			continue;
		}


		//считаем по микротику
		for (int j = 1; j <= game.properties.updatesPerTick; ++j)
		{
			const auto mtTime = (tick - 1 + j * 1.0 / game.properties.updatesPerTick) / game.properties.ticksPerSecond;
			unitPos = getFallingUnitPosition(unit, startJumpTick, stopJumpTick, mtTime, game);
			bulletPos = getBulletPosition(bullet, mtTime, game);
			if (isBulletMoveCrossUnitMove(prevUnitPos, unitPos, prevBulletPos, bulletPos, unit.size, bullet.size / 2))
			{
				killTick = min(killTick, tick);
				return true;
			}


			prevUnitPos = unitPos;
			prevBulletPos = bulletPos;
		}

		prevUnitPos = unitPos;
		prevBulletPos = bulletPos;
	}

	return false;
}


bool isUnitOnLadder(const Unit& unit, const Game& game)
{
	/*const auto centerUnitTile = game.level.tiles[size_t(unit.position.x)][size_t(unit.position.y + unit.size.y / 2)];
	const auto downUnitTile = game.level.tiles[size_t(unit.position.x)][size_t(unit.position.y)];
	return centerUnitTile == LADDER || downUnitTile == LADDER;*/

	const auto leftSideDownTile = game.level.tiles[size_t(unit.position.x - unit.size.x / 2)][size_t(
		unit.position.y - 1)];
	const auto rightSideDownTile = game.level.tiles[size_t(unit.position.x + unit.size.x / 2)][size_t(
		unit.position.y - 1)];

	return (leftSideDownTile == LADDER || rightSideDownTile == LADDER) &&
		leftSideDownTile != WALL && rightSideDownTile != WALL;
}


bool isUnitOnPlatform(const Unit& unit, const Game& game)
{
	const auto leftSideDownTile = game.level.tiles[size_t(unit.position.x - unit.size.x / 2)][size_t(
		unit.position.y - 1)];
	const auto rightSideDownTile = game.level.tiles[size_t(unit.position.x + unit.size.x / 2)][size_t(
		unit.position.y - 1)];

	return (leftSideDownTile == PLATFORM || rightSideDownTile == PLATFORM) &&
		leftSideDownTile != WALL && rightSideDownTile != WALL;
}


tuple<RunawayDirection, int, int> getJumpAndStopTicks(
	const Unit& me, const vector<ShootMeBullet>& shootingMeBullets,
	const map<Bullet, double>& enemyBulletsShootWallTimes,
	const Game& game)
{
	if (shootingMeBullets.empty())
	{
		return make_tuple(NONE, -1, -1);
	}
	int minShootMeTick = INT_MAX;
	for (const auto& smb : shootingMeBullets)
	{
		if (smb.shootMeTick < minShootMeTick)
		{
			minShootMeTick = smb.shootMeTick;
		}
	}

	double maxShootWallTime = 0;
	for (const auto& item : enemyBulletsShootWallTimes)
	{
		if (item.second > maxShootWallTime)
		{
			maxShootWallTime = item.second;
		}
	}
	const int maxShootWallTick = static_cast<int>(ceil(maxShootWallTime * game.properties.ticksPerSecond));

	const bool isOnLadder = isUnitOnLadder(me, game);
	const bool isOnPlatform = isUnitOnPlatform(me, game);

	for (int startGoTick = minShootMeTick - 1; startGoTick >= 0; startGoTick--)
	{
		auto killGoUpTick = INT_MAX;
		auto killGoLeftTick = INT_MAX;
		auto killGoRightTick = INT_MAX;
		auto killGoDownTick = INT_MAX;

		for (int stopGoTick = startGoTick + 1; stopGoTick < maxShootWallTick; ++stopGoTick)
		{
			auto canGoUp = true;
			auto canGoLeft = true;
			auto canGoRight = true;
			auto canGoDown = true;


			for (const auto& bullet : game.bullets)
			{
				if (bullet.playerId == me.playerId) continue;

				if (stopGoTick > killGoUpTick)
				{
					canGoUp = false;
				}
				else if (canGoUp && isBulletShootJumpingUnit(
					bullet, me, startGoTick, stopGoTick, enemyBulletsShootWallTimes.at(bullet), game, killGoUpTick))
				{
					canGoUp = false;
				}

				if (stopGoTick > killGoRightTick)
				{
					canGoRight = false;
				}
				else if (canGoRight && isBulletShootGoSideUnit(
					bullet, me, startGoTick, stopGoTick, enemyBulletsShootWallTimes.at(bullet), 1, game,
					killGoRightTick))
				{
					canGoRight = false;
				}

				if (stopGoTick > killGoLeftTick)
				{
					canGoLeft = false;
				}
				else if (canGoLeft && isBulletShootGoSideUnit(
					bullet, me, startGoTick, stopGoTick, enemyBulletsShootWallTimes.at(bullet), -1, game,
					killGoLeftTick))
				{
					canGoLeft = false;
				}

				if (!isOnLadder && !isOnPlatform || stopGoTick > killGoDownTick)
				{
					canGoDown = false;
				}
				else if (canGoDown && isBulletShootFallingUnit(
					bullet, me, startGoTick, isOnLadder ? stopGoTick : -1, enemyBulletsShootWallTimes.at(bullet), game,
					killGoDownTick))
				{
					canGoDown = false;
				}

				if (!canGoUp && !canGoLeft && !canGoRight && !canGoDown) break;
			}


			if (canGoUp)
			{
				return make_tuple(UP, startGoTick, stopGoTick);
			}
			if (canGoDown)
			{
				return make_tuple(DOWN, startGoTick, stopGoTick);
			}
			if (canGoLeft)
			{
				return make_tuple(LEFT, startGoTick, stopGoTick);
			}
			if (canGoRight)
			{
				return make_tuple(RIGHT, startGoTick, stopGoTick);
			}
		}
	}

	return make_tuple(NONE, -1, -1); //нет пуль или нет шансов спастись
}

//TODO: учесть границы
Vec2Double getUnitInTimePosition(const Unit& unit, const UnitAction& action, double time, const Game& game)
{
	auto x = unit.position.x;
	auto y = unit.position.y;
	
	auto velocityY = 0.0;	
	if (action.jump) velocityY = game.properties.unitJumpSpeed;
	else if (action.jumpDown) velocityY = -game.properties.unitFallSpeed;

	const auto velocityX = action.velocity >= 0 ? min(action.velocity, game.properties.unitMaxHorizontalSpeed) :
		max(action.velocity, -game.properties.unitMaxHorizontalSpeed);
	return { x + velocityX * time, y + velocityY * time };
}

bool isSafeMove(const Unit& unit, const UnitAction& action, const map<Bullet, double>& enemyBulletShootWallTimes, const Game& game)
{
	
	for (const auto& bullet: game.bullets)
	{
		if (bullet.playerId == unit.playerId) continue;
		const auto time = min(enemyBulletShootWallTimes.at(bullet), 1.0 / game.properties.ticksPerSecond);

		const auto unitInTimePosition = getUnitInTimePosition(unit, action, time, game);
		const auto bulletNextTickPosition = getBulletPosition(bullet, time, game);
		const auto cross = isBulletMoveCrossUnitMove(
			unit.position, unitInTimePosition,
			bullet.position, bulletNextTickPosition,
			unit.size, bullet.size / 2.0);
		if (cross) return false;
	}
	return true;
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
				distanceSqr(unit.position, other.position) <
				distanceSqr(unit.position, nearestEnemy->position))
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
				distanceSqr(unit.position, lootBox.position) <
				distanceSqr(unit.position, nearestWeapon->position))
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
			needGo = getShootEnemyProbability(unit, *nearestEnemy, game, unit.weapon->params.minSpread) <
				WALKING_PROBABILITY;
			needShoot = getShootEnemyProbability(unit, *nearestEnemy, game, unit.weapon->spread, &debug) >=
				SHOOTING_PROBABILITY;
		}
	}


	auto aim = Vec2Double(nearestEnemy->position.x - unit.position.x,
	                      nearestEnemy->position.y - unit.position.y);

	//bool isBulletShootingMe = false;
	//for (const auto& bullet : game.bullets) {
	// if (isShootingMe(unit, bullet, game)) {
	//  isBulletShootingMe = true;
	//  break;
	// }
	//}

	//if (isBulletShootingMe) {
	// debug.draw(CustomData::Log(
	//  std::string("IS SHOOTING ME")));
	//}
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

	const auto shootMeBullets = getShootMeBullets(unit, game);
	const auto enemyBulletsShootWallTimes = getEnemyBulletsShootWallTimes(game, unit.playerId);
	
	if (getStopRunawayTick() == 0)
	{
		const auto runawayDirection = getRunawayDirection();
		if (runawayDirection == UP)
		{
			jump = false;
			velocity = 0;
		}
		else if (runawayDirection == DOWN)
		{
			jump = false;
			velocity = 0;
			jumpDown = true;
		}
		decreaseStopRunawayTick();
	}
	else if (getStopRunawayTick() > 0)
	{
		const auto runawayDirection = getRunawayDirection();
		if (runawayDirection == UP)
		{
			jump = true;
			velocity = 0;
		}
		else if (runawayDirection == DOWN)
		{
			jump = false;
			velocity = 0;
			jumpDown = true;
		}
		else if (runawayDirection == LEFT)
		{
			jump = false;
			velocity = -INT_MAX;
		}
		else if (runawayDirection == RIGHT)
		{
			jump = false;
			velocity = INT_MAX;
		}
		decreaseStopRunawayTick();
	}
	else
	{
		if (unit.jumpState.canJump)
		{						

			const auto jumpAndStopTicks = getJumpAndStopTicks(unit, shootMeBullets, enemyBulletsShootWallTimes, game);
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
				setRunaway(runawayDirection, stopRunawayTick);

				if (runawayDirection == UP)
				{
					jump = true;
					velocity = 0;
				}
				else if (runawayDirection == DOWN)
				{
					jump = false;
					jumpDown = true;
					velocity = 0;
				}
				else if (runawayDirection == LEFT)
				{
					jump = false;

					velocity = -INT_MAX;
				}
				else if (runawayDirection == RIGHT)
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
	action.swapWeapon = false;
	action.plantMine = false;

	if (shootMeBullets.empty() && !isSafeMove(unit, action, enemyBulletsShootWallTimes, game))
	{
		const auto smb2 = getShootMeBullets(unit, game);
		
		action.jump = false;
		action.jumpDown = false;
		action.velocity = 0;
	}

	return action;
}

int MyStrategy::getRunawayDirection() const
{
	return runaway_direction_;
}

int MyStrategy::getStopRunawayTick() const
{
	return stop_runaway_tick_;
}

void MyStrategy::setRunaway(RunawayDirection runaway_direction, int sjt)
{
	runaway_direction_ = runaway_direction;
	stop_runaway_tick_ = sjt;
}

void MyStrategy::decreaseStopRunawayTick()
{
	if (stop_runaway_tick_ >= 0) stop_runaway_tick_--;
}
