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

using namespace std;


MyStrategy::MyStrategy() {}

inline bool operator< (const Bullet& lhs, const Bullet& rhs)
{
	return lhs.position.x < rhs.position.x;
}

double distanceSqr(Vec2Double a, Vec2Double b) {
  return (a.x - b.x) * (a.x - b.x) + (a.y - b.x) * (a.y - b.y);
}

//TODO: учесть граничный TILE
Vec2Double getBulletCrossBorderPoint(const Vec2Double& bulletPosition, const Vec2Double& bulletVelocity, double maxX, double maxY) {	

	if (abs(bulletVelocity.y) < TOLERACNE) {
		return Vec2Double(bulletVelocity.x > 0 ? maxX : 0, bulletPosition.y);
	}

	if (abs(bulletVelocity.x) < TOLERACNE) {
		return Vec2Double(bulletPosition.x, bulletVelocity.y > 0 ? maxY : 0);
	}	

	const auto x1 = bulletPosition.x;
	const auto y1 = bulletPosition.y;
	const auto x2 = bulletPosition.x + bulletVelocity.x;
	const auto y2 = bulletPosition.y + bulletVelocity.y;

	if (bulletVelocity.x > 0) {
	
		const auto vertCross = MathHelper::getLinesCross(x1, y1, x2, y2, maxX, 0, maxX, maxY);
		if (vertCross.y >= 0 && vertCross.y < maxY) {
			return vertCross;
		}

		return bulletVelocity.y < 0 ?
			MathHelper::getLinesCross(x1, y1, x2, y2, 0, 0, maxX, 0) : 
			MathHelper::getLinesCross(x1, y1, x2, y2, 0, maxY, maxX, maxY);
	}
	else {
		const auto vertCross = MathHelper::getLinesCross(x1, y1, x2, y2, 0, 0, 0, maxY);
		if (vertCross.y >= 0 && vertCross.y < maxY) {
			return vertCross;
		}

		return bulletVelocity.y < 0 ? 
			MathHelper::getLinesCross(x1, y1, x2, y2, 0, 0, maxX, 0) : 
			MathHelper::getLinesCross(x1, y1, x2, y2, 0, maxY, maxX, maxY);
	}
}

Vec2Double getBulletCornerCrossWallPoint(
	const Vec2Double& bulletPosition, const Vec2Double& bulletVelocity, double maxX, double maxY, const Game& game){

	auto crossPoint = getBulletCrossBorderPoint(bulletPosition, bulletVelocity, maxX, maxY);
	const auto bulletTiles = MathHelper::getLineSquares(bulletPosition, crossPoint, 1);
	const pair<int, int>* firstWallTile = nullptr;

	for (const auto& bt : bulletTiles) {
		if (bt.first == 0 || bt.second == 0 || bt.first == game.level.tiles.size() - 1 || bt.second == game.level.tiles[0].size() - 1) {
			break; //игнор крайних стен
		}

		if (game.level.tiles[bt.first][bt.second] == Tile::WALL) {
			firstWallTile = &bt;
			break;
		}
	}

	if (firstWallTile != nullptr) {

		const auto x1 = bulletPosition.x;
		const auto y1 = bulletPosition.y;
		const auto x2 = bulletPosition.x + bulletVelocity.x;
		const auto y2 = bulletPosition.y + bulletVelocity.y;

		int minDist = INT_MAX;
		const Vec2Double* minDistCp = nullptr;

		auto cp1 = MathHelper::getLinesCross(
			x1, y1, x2, y2, firstWallTile->first, firstWallTile->second, firstWallTile->first + TILE_SIZE, firstWallTile->second);
		if (cp1.x >= firstWallTile->first && cp1.x <= firstWallTile->first + TILE_SIZE) {
			const auto dist2 = MathHelper::getVectorLength2(Vec2Double(x1 - cp1.x, y1 - cp1.y));
			if (minDistCp == nullptr || dist2 < minDist) {
				minDist = dist2;
				minDistCp = &cp1;
			}
		}

		auto cp2 = MathHelper::getLinesCross(
			x1, y1, x2, y2, firstWallTile->first, firstWallTile->second + TILE_SIZE, firstWallTile->first + TILE_SIZE, firstWallTile->second + TILE_SIZE);
		if (cp2.x >= firstWallTile->first && cp2.x <= firstWallTile->first + TILE_SIZE) {
			const auto dist2 = MathHelper::getVectorLength2(Vec2Double(x1 - cp2.x, y1 - cp2.y));
			if (minDistCp == nullptr || dist2 < minDist) {
				minDist = dist2;
				minDistCp = &cp2;
			}
		}

		auto cp3 = MathHelper::getLinesCross(
			x1, y1, x2, y2, firstWallTile->first, firstWallTile->second, firstWallTile->first, firstWallTile->second + TILE_SIZE);
		if (cp3.y >= firstWallTile->second && cp3.y <= firstWallTile->second + TILE_SIZE) {
			const auto dist2 = MathHelper::getVectorLength2(Vec2Double(x1 - cp3.x, y1 - cp3.y));
			if (minDistCp == nullptr || dist2 < minDist) {
				minDist = dist2;
				minDistCp = &cp3;
			}
		}

		auto cp4 = MathHelper::getLinesCross(
			x1, y1, x2, y2, firstWallTile->first + TILE_SIZE, firstWallTile->second, firstWallTile->first + TILE_SIZE, firstWallTile->second + TILE_SIZE);
		if (cp4.y >= firstWallTile->second && cp4.y <= firstWallTile->second + TILE_SIZE) {
			const auto dist2 = MathHelper::getVectorLength2(Vec2Double(x1 - cp4.x, y1 - cp4.y));
			if (minDistCp == nullptr || dist2 < minDist) {
				minDist = dist2;
				minDistCp = &cp4;
			}
		}

		if (minDistCp == nullptr) {
			throw runtime_error("no wall tile cross");
		}
		return *minDistCp;

	}

	return crossPoint;
}

Vec2Double getBulletCrossWallPoint(const Bullet& bullet, double maxX, double maxY, const Game& game) {

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


Vec2Double getShootingCrossBorderPoint(const Vec2Double& position, double lastAngle, double maxX, double maxY) {
	
	if (abs(lastAngle) < TOLERACNE) {
		return Vec2Double(maxX, position.y);
	}

	if (abs(lastAngle - M_PI) < TOLERACNE) {
		return Vec2Double(0, position.y);
	}

	if (abs(lastAngle - M_PI/2) < TOLERACNE) {
		return Vec2Double(position.x, 0);
	}

	if (abs(lastAngle + M_PI / 2) < TOLERACNE) {
		return Vec2Double(position.x, maxY);
	}
	
	const auto x1 = position.x;
	const auto y1 = position.y;
	const auto x2 = position.x + cos(lastAngle);
	const auto y2 = position.y + sin(lastAngle);

	if (abs(lastAngle) < M_PI/2) {

		const auto vertCross = MathHelper::getLinesCross(x1, y1, x2, y2, maxX, 0, maxX, maxY);
		if (vertCross.y >= 0 && vertCross.y < maxY) {
			return vertCross;
		}

		return lastAngle > 0 ? 
			MathHelper::getLinesCross(x1, y1, x2, y2, 0, maxY, maxX, maxY) : 
			MathHelper::getLinesCross(x1, y1, x2, y2, 0, 0, maxX, 0);
	}
	else {
		const auto vertCross = MathHelper::getLinesCross(x1, y1, x2, y2, 0, 0, 0, maxY);
		if (vertCross.y >= 0 && vertCross.y < maxY) {
			return vertCross;
		}

		return lastAngle > 0 ? 
			MathHelper::getLinesCross(x1, y1, x2, y2, 0, maxY, maxX, maxY) : 
			MathHelper::getLinesCross(x1, y1, x2, y2, 0, 0, maxX, 0);
	}
}

void drawBullets(Debug& debug, const Game& game, int meId) {
	
	const auto maxX = game.level.tiles.size() * TILE_SIZE;
	const auto maxY = game.level.tiles[0].size() * TILE_SIZE;
	for (const auto& bullet : game.bullets) {		
		auto crossPoint = getBulletCrossWallPoint(bullet, maxX, maxY, game);		

		const auto debugBullet = vec2DoubleToVec2Float(bullet.position);
		const auto debugCrossPoint = vec2DoubleToVec2Float(crossPoint);
		debug.draw(CustomData::Line(debugBullet, debugCrossPoint, 0.1, 
			ColorFloat(bullet.playerId == meId ? 0 : 255, bullet.playerId == meId ? 255 : 0, 0, 0.25)));
	}
}

void drawShootingLine(
	Debug& debug, const Game& game, const Vec2Double& weaponPoistion, double angle, double maxX, double maxY, ColorFloat color) {
	auto crossPoint = getShootingCrossBorderPoint(
		weaponPoistion,
		angle, maxX, maxY);

	const auto bulletTiles = MathHelper::getLineSquares(weaponPoistion, crossPoint, 1);
	const pair<int, int> *firstWallTile = nullptr;
	for (const auto& bt : bulletTiles) {
		if (bt.first == 0 || bt.second == 0 || bt.first == game.level.tiles.size() - 1 || bt.second == game.level.tiles[0].size() - 1) {
			break; //игнор крайних стен
		}

		if (game.level.tiles[bt.first][bt.second] == Tile::WALL) {
			firstWallTile = &bt;
			break;
		}
	}

	if (firstWallTile != nullptr) { //TODO: брать нормальное пересечение со стеной
		crossPoint = Vec2Double(firstWallTile->first + 0.5, firstWallTile->second + 0.5);
	}

	const auto debugUnit = vec2DoubleToVec2Float(weaponPoistion);
	const auto debugCrossPoint = vec2DoubleToVec2Float(crossPoint);
	debug.draw(CustomData::Line(debugUnit, debugCrossPoint, 0.1, color));
}


void drawShootingSector(Debug& debug, const Unit& unit, const Game& game) {
	if (unit.weapon == nullptr || (*unit.weapon).lastAngle == nullptr) return;
	
	const auto maxX = game.level.tiles.size() * TILE_SIZE;
	const auto maxY = game.level.tiles[0].size() * TILE_SIZE;


	const auto weaponPoistion = Vec2Double(unit.position.x, unit.position.y + unit.size.y / 2);
	//drawShootingLine(debug, game, weaponPoistion, *(*unit.weapon).lastAngle, maxX, maxY, ColorFloat(0, 0, 255, 0.5));
	drawShootingLine(debug, game, weaponPoistion, *(*unit.weapon).lastAngle - (*unit.weapon).spread, maxX, maxY, ColorFloat(100, 100, 255, 0.5));
	drawShootingLine(debug, game, weaponPoistion, *(*unit.weapon).lastAngle + (*unit.weapon).spread, maxX, maxY, ColorFloat(100, 100, 255, 0.5));
	

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

	double minCrossDist2 = INT_MAX;
	Vec2Double* minCrossDist2Point = nullptr;	

	if (cross1.y >= y1 && cross1.y <= y2)
	{
		auto dist2 = MathHelper::getVectorLength2(Vec2Double(cross1.x - bulletX1, cross1.y - bulletY1));
		if (dist2 < minCrossDist2)
		{
			minCrossDist2 = dist2;
			minCrossDist2Point = &cross1;
		}
	}

	if (cross2.x >= x1 && cross2.x <= x2) {
		auto dist2 = MathHelper::getVectorLength2(Vec2Double(cross2.x - bulletX1, cross2.y - bulletY1));
		if (dist2 < minCrossDist2)
		{
			minCrossDist2 = dist2;
			minCrossDist2Point = &cross2;
		}
	}

	if (cross3.y >= y1 && cross3.y <= y2) {
		auto dist2 = MathHelper::getVectorLength2(Vec2Double(cross3.x - bulletX1, cross3.y - bulletY1));
		if (dist2 < minCrossDist2)
		{
			minCrossDist2 = dist2;
			minCrossDist2Point = &cross3;
		}
	}

	if (cross4.x >= x1 && cross4.x <= x2) {
		auto dist2 = MathHelper::getVectorLength2(Vec2Double(cross4.x - bulletX1, cross4.y - bulletY1));
		if (dist2 < minCrossDist2)
		{
			minCrossDist2 = dist2;
			minCrossDist2Point = &cross4;
		}
	}


	if (minCrossDist2Point != nullptr)
	{
		const auto bulletTiles = MathHelper::getLineSquares(bulletPosition, *minCrossDist2Point, 1);
		const pair<int, int>* firstWallTile = nullptr;
		for (const auto& bt : bulletTiles) {
			if (bt.first == 0 || bt.second == 0 || bt.first == game.level.tiles.size() - 1 || bt.second == game.level.tiles[0].size() - 1) {
				break; //игнор крайних стен
			}

			if (game.level.tiles[bt.first][bt.second] == Tile::WALL) {
				firstWallTile = &bt;
				break;
			}
		}

		if (firstWallTile != nullptr)
		{
			return {Vec2Double(0, 0), true, true, 0};
		}
	}

	return {
		minCrossDist2Point != nullptr ? *minCrossDist2Point : Vec2Double(0,0),
		minCrossDist2Point != nullptr,
		false,
		minCrossDist2
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

double getShootEnemyProbability(const Unit& me, const Unit& enemy, const Game& game, double spread) {
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
		
		const auto angle = *me.weapon->lastAngle + deltaAngle * i;
		auto bulletPos = Vec2Double(
			bulletCenterPos.x - me.weapon->params.bullet.size / 2,
			bulletCenterPos.y - me.weapon->params.bullet.size / 2);
		auto bulletVelocity = Vec2Double(cos(angle), sin(angle));
		auto shootEnemyCrossPoint = get_shoot_me_bullet_cross_point(
			x1, y1, x2, y2, bulletPos, bulletVelocity, game);

		if (shootEnemyCrossPoint.hasWallBefore) continue;
		if (shootEnemyCrossPoint.hasCrossPoint) isShooting = true;


		bulletPos = Vec2Double(
			bulletCenterPos.x - me.weapon->params.bullet.size / 2,
			bulletCenterPos.y + me.weapon->params.bullet.size / 2);
		bulletVelocity = Vec2Double(cos(angle), sin(angle));
		shootEnemyCrossPoint = get_shoot_me_bullet_cross_point(
			x1, y1, x2, y2, bulletPos, bulletVelocity, game);

		if (shootEnemyCrossPoint.hasWallBefore) continue;
		if (shootEnemyCrossPoint.hasCrossPoint) isShooting = true;


		bulletPos = Vec2Double(
			bulletCenterPos.x + me.weapon->params.bullet.size / 2,
			bulletCenterPos.y - me.weapon->params.bullet.size / 2);
		bulletVelocity = Vec2Double(cos(angle), sin(angle));
		shootEnemyCrossPoint = get_shoot_me_bullet_cross_point(
			x1, y1, x2, y2, bulletPos, bulletVelocity, game);

		if (shootEnemyCrossPoint.hasWallBefore) continue;
		if (shootEnemyCrossPoint.hasCrossPoint) isShooting = true;

		bulletPos = Vec2Double(
			bulletCenterPos.x + me.weapon->params.bullet.size / 2,
			bulletCenterPos.y + me.weapon->params.bullet.size / 2);
		bulletVelocity = Vec2Double(cos(angle), sin(angle));
		shootEnemyCrossPoint = get_shoot_me_bullet_cross_point(
			x1, y1, x2, y2, bulletPos, bulletVelocity, game);

		if (shootEnemyCrossPoint.hasWallBefore) continue;
		if (shootEnemyCrossPoint.hasCrossPoint) isShooting = true;

		if (isShooting) shootingCount++;
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

int getShootMeBulletTick(const Unit& me, const Bullet& bullet, const Game& game) {	

	auto x1 = me.position.x - me.size.x / 2;
	auto x2 = me.position.x + me.size.x / 2;
	auto y1 = me.position.y;
	auto y2 = me.position.y + me.size.y;

	if (bullet.velocity.x > 0 && bullet.position.x > x2) return -1;
	if (bullet.velocity.x < 0 && bullet.position.x < x1) return -1;
	if (bullet.velocity.y > 0 && bullet.position.y > y2) return -1;
	if (bullet.velocity.y < 0 && bullet.position.y < y1) return -1;

	
	double minCrossDist2 = INT_MAX;
	const Vec2Double* minCrossDist2Point = nullptr;
	
	const auto bulletPosition1 = Vec2Double(bullet.position.x - bullet.size / 2, bullet.position.y - bullet.size / 2);
	const auto smbcp1 = get_shoot_me_bullet_cross_point(x1, y1, x2, y2, bulletPosition1, bullet.velocity, game);
	if (smbcp1.hasWallBefore)
	{
		return -1;
	}
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
	if (smbcp2.hasWallBefore)
	{
		return -1;
	}
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
	if (smbcp3.hasWallBefore)
	{
		return -1;
	}
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
	if (smbcp4.hasWallBefore)
	{
		return -1;
	}
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
		auto bulletVelocity = MathHelper::getVectorLength(bullet.velocity);
		
		auto minCrossDist = sqrt(minCrossDist2);
		int shootMeTick = static_cast<int>(ceil(minCrossDist / bulletVelocity * game.properties.ticksPerSecond));

		/*const auto maxX = game.level.tiles.size() * TILE_SIZE;
		const auto maxY = game.level.tiles[0].size() * TILE_SIZE;
		const auto crossWallPoint = getBulletCrossWallPoint(bullet, maxX, maxY, game);
		auto wallDist = MathHelper::getVectorLength(
			Vec2Double(bullet.position.x - crossWallPoint.x, bullet.position.y - crossWallPoint.y));		
		int shootWallTick = static_cast<int>(ceil(wallDist / bulletVelocity * game.properties.ticksPerSecond));*/
		
		return shootMeTick;
	}
	
	
	return -1;
}

Vec2Double getBulletPosition(const Bullet& bullet, int tick, const Game& game) {
	double time = 1.0 * tick / game.properties.ticksPerSecond;
	return Vec2Double(bullet.position.x + bullet.velocity.x * time, bullet.position.y + bullet.velocity.y * time);
}

vector<ShootMeBullet> getShootMeBullets(const Unit& unit, const Game& game) {
	
	vector<ShootMeBullet> result;
	for (const auto& bullet : game.bullets) {
		if (bullet.playerId == unit.playerId) continue;
		const auto smbt = getShootMeBulletTick(unit, bullet, game);
		if (smbt == -1) continue;
		result.emplace_back(ShootMeBullet(bullet, smbt));
	}

	return result;
}



//TODO: учесть максимальное время прыжка
Vec2Double getJumpUnitPosition(const Unit& unit, int startJumpTick, int stopJumpTick, int tick, const Game& game) {
	if (tick <= startJumpTick) {
		return unit.position;
	}	

	if (tick <= stopJumpTick) {
		double jumpTime = 1.0*(tick - startJumpTick) / game.properties.ticksPerSecond;
		return Vec2Double(unit.position.x, unit.position.y + game.properties.unitJumpSpeed * jumpTime);
	}

	double jumpTime = 1.0*(stopJumpTick-startJumpTick) / game.properties.ticksPerSecond;
	double fallTime = 1.0*(tick - stopJumpTick) / game.properties.ticksPerSecond;
	if (fallTime > jumpTime) fallTime = jumpTime;//TODO: случай, когда прыгаем не с земли, а продолжаем прыжок
	return Vec2Double(unit.position.x, unit.position.y + game.properties.unitJumpSpeed * jumpTime - game.properties.unitFallSpeed * fallTime);
}


bool isBulletInUnit(const Vec2Double& unitPosition, const Vec2Double& unitSize, const Vec2Double& bulletPostion, double bulletSize) {
	double bulletX1 = bulletPostion.x - bulletSize / 2;
	double bulletX2 = bulletPostion.x + bulletSize / 2;
	double bulletY1 = bulletPostion.y - bulletSize / 2;
	double bulletY2 = bulletPostion.y + bulletSize / 2;

	double unitX1 = unitPosition.x - unitSize.x / 2;
	double unitX2 = unitPosition.x + unitSize.x / 2;
	double unitY1 = unitPosition.y;
	double unitY2 = unitPosition.y + unitSize.y;

	if (bulletX1 >= unitX1 && bulletX1 <= unitX2) {
		if (bulletY1 >= unitY1 && bulletY1 <= unitY2) return true;
		if (bulletY2 >= unitY1 && bulletY2 <= unitY2) return true;
	}

	if (bulletX2 >= unitX1 && bulletX2 <= unitX2) {
		if (bulletY1 >= unitY1 && bulletY1 <= unitY2) return true;
		if (bulletY2 >= unitY1 && bulletY2 <= unitY2) return true;
	}

	return false;
}




pair<int, int> getJumpAndStopTicks(
	const Unit& me, const vector<ShootMeBullet>& shootingMeBullets, const map<Bullet, double>& enemyBulletsShootWallTimes,
	const Game& game) {

	if (shootingMeBullets.empty()) {
		return make_pair(-1, -1);
	}
	int minShootMeTick = INT_MAX;
	for (const auto& smb : shootingMeBullets) {
		if (smb.shootMeTick < minShootMeTick) {
			minShootMeTick = smb.shootMeTick;
		}		
	}

	double maxShootWallTime = 0;
	for (const auto& item:enemyBulletsShootWallTimes)
	{
		if (item.second > maxShootWallTime)
		{
			maxShootWallTime = item.second;
		}
	}
	const int maxShootWallTick = static_cast<int>(ceil(maxShootWallTime * game.properties.ticksPerSecond));
	
	for (int startJumpTick = minShootMeTick - 1; startJumpTick >= 0; startJumpTick--) {
		for (int stopJumpTick = startJumpTick + 1; stopJumpTick < maxShootWallTick; ++stopJumpTick) {

			auto isGoodJump = true;
			for (auto tick = 1; tick < maxShootWallTick; ++tick) {
				const auto mePosition = getJumpUnitPosition(me, startJumpTick, stopJumpTick, tick, game);
				for (const auto& smb : shootingMeBullets) {
					if (enemyBulletsShootWallTimes.at(smb.bullet) * game.properties.ticksPerSecond <= tick) continue;				
					

					const auto bulletPosition0 = getBulletPosition(smb.bullet, tick, game);
					//считаем пулю на 1 тик вперед, чтобы не анализовать коллизии по микротикам
					const auto bulletPosition1 = getBulletPosition(smb.bullet, tick + 1, game);

					if (isBulletInUnit(mePosition, me.size, bulletPosition0, smb.bullet.size) ||
						isBulletInUnit(mePosition, me.size, bulletPosition1, smb.bullet.size)) {
						isGoodJump = false;
						break;
					}
				}

				if (!isGoodJump) break;
			}

			if (isGoodJump) {
				return make_pair(startJumpTick, stopJumpTick);
			}
		}
	}

	return make_pair(-1, -1);//нет пуль или нет шансов спастись
}



UnitAction MyStrategy::getAction(const Unit &unit, const Game &game,
                                 Debug &debug) {
  const Unit *nearestEnemy = nullptr;
  for (const Unit &other : game.units) {
    if (other.playerId != unit.playerId) {
      if (nearestEnemy == nullptr ||
          distanceSqr(unit.position, other.position) <
              distanceSqr(unit.position, nearestEnemy->position)) {
        nearestEnemy = &other;
      }
    }
  }
  const LootBox *nearestWeapon = nullptr;
  for (const LootBox &lootBox : game.lootBoxes) {
    if (std::dynamic_pointer_cast<Item::Weapon>(lootBox.item)) {
      if (nearestWeapon == nullptr ||
          distanceSqr(unit.position, lootBox.position) <
              distanceSqr(unit.position, nearestWeapon->position)) {
        nearestWeapon = &lootBox;
      }
    }
  }
  Vec2Double targetPos = unit.position;
  if (unit.weapon == nullptr && nearestWeapon != nullptr) {
    targetPos = nearestWeapon->position;
  } else if (nearestEnemy != nullptr) {
    targetPos = nearestEnemy->position;
  }
  debug.draw(CustomData::Log(
      std::string("Target pos: ") + targetPos.toString() + "\n"));

  drawBullets(debug, game, unit.playerId);
  drawShootingSector(debug, unit, game);

  //debug.draw(CustomData::Line(Vec2Float(10, 10), Vec2Float(500, 500), 10, ColorFloat(255, 0, 0, 1)));
  Vec2Double aim = Vec2Double(0, 0);
  auto needGo = false;
  auto needShoot = false;
	
  if (nearestEnemy != nullptr) {
	  if (unit.weapon != nullptr) {
		  needGo = getShootEnemyProbability(unit, *nearestEnemy, game, unit.weapon->params.minSpread) < 
			  SHOOTING_PROBABILITY;
		  needShoot = getShootEnemyProbability(unit, *nearestEnemy, game, unit.weapon->spread) >=
			  SHOOTING_PROBABILITY;	  	
	  }	
  }

  aim = Vec2Double(nearestEnemy->position.x - unit.position.x,
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
  if (unit.weapon != nullptr && !needGo) {
	  velocity = 0;
  }
  else {
	  if (targetPos.x > unit.position.x) {
		  velocity = INT_MAX;
	  }
	  else {
		  velocity = -INT_MAX;
	  }	  
  }

  if (getStopJumpTick() == 0) {
	  jump = false;
	  decreaseStopJumpTick();
	  velocity = 0;
  }
  else if (getStopJumpTick() > 0) {
	  jump = true;
	  decreaseStopJumpTick();
	  velocity = 0;
  }
  else {
	  jump = unit.weapon == nullptr && targetPos.y > unit.position.y;	  

	  if (unit.jumpState.canJump) {
		  const auto shootMeBullet = getShootMeBullets(unit, game);
		  const auto enemyBulletsShootWallTimes = getEnemyBulletsShootWallTimes(game, unit.playerId);
	  	
		  const auto jumpAndStopTicks = getJumpAndStopTicks(unit, shootMeBullet, enemyBulletsShootWallTimes, game);
		  debug.draw(CustomData::Log(to_string(jumpAndStopTicks.first) + " " + to_string(jumpAndStopTicks.second)));

		  if (jumpAndStopTicks.first == 0) {
			  jump = true;
			  setStopJumpTick(jumpAndStopTicks.second);
			  velocity = 0;
		  }
	  } 
  }
  
  
  if ((unit.weapon == nullptr || unit.weapon != nullptr && needGo) &&
	  targetPos.x > unit.position.x &&
      game.level.tiles[size_t(unit.position.x + 1)][size_t(unit.position.y)] ==
          Tile::WALL) {
    jump = true;
  }
  if ((unit.weapon == nullptr || unit.weapon != nullptr && needGo) &&
	  targetPos.x < unit.position.x &&
      game.level.tiles[size_t(unit.position.x - 1)][size_t(unit.position.y)] ==
          Tile::WALL) {
    jump = true;
  }

  auto jumpDown = unit.weapon != nullptr ? false : !jump;

  debug.draw(CustomData::Log("SHOOT: " + to_string(needShoot)));
 

  UnitAction action;
  action.velocity = velocity;
  action.jump = jump;
  action.jumpDown = jumpDown;
  action.aim = aim;
  action.shoot = needShoot;
  action.swapWeapon = false;
  action.plantMine = false;
  return action;
}

int MyStrategy::getStopJumpTick()
{
	return stopJumpTick;
}

void MyStrategy::setStopJumpTick(int sjt)
{
	stopJumpTick = sjt;
}

void MyStrategy::decreaseStopJumpTick()
{
	if (stopJumpTick >= 0) stopJumpTick--;
}
