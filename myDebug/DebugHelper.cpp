#define _USE_MATH_DEFINES

#include <cmath>
#include "DebugHelper.h"
#include "../mathcalc/MathHelper.h"
#include "../common/Helper.h"
#include "../model/Game.hpp"
#include "../model/ColorFloat.hpp"
#include "../Debug.hpp"
#include "../simulation/Simulator.h"
#include <climits>
#include <map>


inline bool operator<(const Bullet& lhs, const Bullet& rhs)
{
	return lhs.position.x < rhs.position.x;
}


Vec2Double getShootingCrossBorderPoint(const Vec2Double& position, double lastAngle, double maxX, double maxY)
{
	if (std::abs(lastAngle) < TOLERANCE)
	{
		return Vec2Double(maxX, position.y);
	}

	if (std::abs(lastAngle - M_PI) < TOLERANCE)
	{
		return Vec2Double(0, position.y);
	}

	if (std::abs(lastAngle - M_PI / 2) < TOLERANCE)
	{
		return Vec2Double(position.x, 0);
	}

	if (std::abs(lastAngle + M_PI / 2) < TOLERANCE)
	{
		return Vec2Double(position.x, maxY);
	}

	const auto x1 = position.x;
	const auto y1 = position.y;
	const auto x2 = position.x + cos(lastAngle);
	const auto y2 = position.y + sin(lastAngle);

	if (std::abs(lastAngle) < M_PI / 2)
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








void drawBullets(Debug& debug, const Game& game, const std::map<Bullet, BulletSimulation>& bulletsSimulations, int mePlayerId)
{
	const auto maxX = game.level.tiles.size() * TILE_SIZE;
	const auto maxY = game.level.tiles[0].size() * TILE_SIZE;
	for (const auto& bullet : game.bullets)
	{		
		if (bullet.playerId == mePlayerId) continue;
		const auto debugBullet = vec2DoubleToVec2Float(bullet.position);
		const auto debugCrossPoint = vec2DoubleToVec2Float(bulletsSimulations.at(bullet).targetCrossPoint);
		debug.draw(CustomData::Line(debugBullet, debugCrossPoint, 0.1,
			ColorFloat(bullet.playerId == mePlayerId ? 0 : 255, bullet.playerId == mePlayerId ? 255 : 0, 0,
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

	const auto bulletTiles = MathHelper::getLineSquares2(weaponPoistion, crossPoint);
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
