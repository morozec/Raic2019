#include "MyStrategy.hpp"
#include "MathHelper.h"
#include "Helper.h"
#include <utility>

using namespace std;


MyStrategy::MyStrategy() {}

double distanceSqr(Vec2Double a, Vec2Double b) {
  return (a.x - b.x) * (a.x - b.x) + (a.y - b.x) * (a.y - b.y);
}


Vec2Double getBulletCrossBorderPoint(const Bullet& bullet, double maxX, double maxY) {	

	if (abs(bullet.velocity.y) < TOLERACNE) {
		return Vec2Double(bullet.velocity.x > 0 ? maxX : 0, bullet.position.y);
	}

	if (abs(bullet.velocity.x) < TOLERACNE) {
		return Vec2Double(bullet.position.x, bullet.velocity.y > 0 ? maxY : 0);
	}	

	const auto x1 = bullet.position.x;
	const auto y1 = bullet.position.y;
	const auto x2 = bullet.position.x + bullet.velocity.x;
	const auto y2 = bullet.position.y + bullet.velocity.y;

	if (bullet.velocity.x > 0) {
	
		const auto vertCross = MathHelper::getLinesCross(x1, y1, x2, y2, maxX, 0, maxX, maxY);
		if (vertCross.y >= 0 && vertCross.y < maxY) {
			return vertCross;
		}

		return bullet.velocity.y < 0 ? MathHelper::getLinesCross(x1, y1, x2, y2, 0, 0, maxX, 0) : MathHelper::getLinesCross(x1, y1, x2, y2, 0, maxY, maxX, maxY);
	}
	else {
		const auto vertCross = MathHelper::getLinesCross(x1, y1, x2, y2, 0, 0, 0, maxY);
		if (vertCross.y >= 0 && vertCross.y < maxY) {
			return vertCross;
		}

		return bullet.velocity.y < 0 ? MathHelper::getLinesCross(x1, y1, x2, y2, 0, 0, maxX, 0) : MathHelper::getLinesCross(x1, y1, x2, y2, 0, maxY, maxX, maxY);
	}
}

void drawBullets(Debug& debug, const Game& game) {
	const auto maxX = game.level.tiles.size() * TILE_SIZE;
	const auto maxY = game.level.tiles[0].size() * TILE_SIZE;
	for (const auto& bullet : game.bullets) {		
		auto crossPoint = getBulletCrossBorderPoint(bullet, maxX, maxY);

		const auto bulletTiles = MathHelper::getLineSquares(bullet.position, crossPoint, 1);
		const pair<int, int> *firstWallTile = nullptr;
		for (const auto& bt : bulletTiles) {
			if (game.level.tiles[bt.first][bt.second] == Tile::WALL) {
				firstWallTile = &bt;
				break;
			}
		}

		if (firstWallTile != nullptr) {
			crossPoint = Vec2Double(firstWallTile->first + TILE_SIZE/2, firstWallTile->second + TILE_SIZE/2);
		}

		const auto debugBullet = vec2DoubleToVec2Float(bullet.position);
		const auto debugCrossPoint = vec2DoubleToVec2Float(crossPoint);
		debug.draw(CustomData::Line(debugBullet, debugCrossPoint, 0.5, ColorFloat(0, 255, 0, 1)));
	}
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
      std::string("Target pos: ") + targetPos.toString()));

  drawBullets(debug, game);

  //debug.draw(CustomData::Line(Vec2Float(10, 10), Vec2Float(500, 500), 10, ColorFloat(255, 0, 0, 1)));
  Vec2Double aim = Vec2Double(0, 0);
  if (nearestEnemy != nullptr) {
    aim = Vec2Double(nearestEnemy->position.x - unit.position.x,
                     nearestEnemy->position.y - unit.position.y);
  }
  bool jump = targetPos.y > unit.position.y;
  if (targetPos.x > unit.position.x &&
      game.level.tiles[size_t(unit.position.x + 1)][size_t(unit.position.y)] ==
          Tile::WALL) {
    jump = true;
  }
  if (targetPos.x < unit.position.x &&
      game.level.tiles[size_t(unit.position.x - 1)][size_t(unit.position.y)] ==
          Tile::WALL) {
    jump = true;
  }
  UnitAction action;
  action.velocity = targetPos.x - unit.position.x;
  action.jump = jump;
  action.jumpDown = !action.jump;
  action.aim = aim;
  action.shoot = true;
  action.swapWeapon = false;
  action.plantMine = false;
  return action;
}