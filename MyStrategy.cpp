#define _USE_MATH_DEFINES

#include "MyStrategy.hpp"
#include "MathHelper.h"
#include "Helper.h"
#include <utility>
#include <algorithm>
#include <math.h>
#include "ShootMeBullet.h"

using namespace std;


MyStrategy::MyStrategy() {}

double distanceSqr(Vec2Double a, Vec2Double b) {
  return (a.x - b.x) * (a.x - b.x) + (a.y - b.x) * (a.y - b.y);
}

//TODO: учесть граничный TILE
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

		return bullet.velocity.y < 0 ?
			MathHelper::getLinesCross(x1, y1, x2, y2, 0, 0, maxX, 0) : 
			MathHelper::getLinesCross(x1, y1, x2, y2, 0, maxY, maxX, maxY);
	}
	else {
		const auto vertCross = MathHelper::getLinesCross(x1, y1, x2, y2, 0, 0, 0, maxY);
		if (vertCross.y >= 0 && vertCross.y < maxY) {
			return vertCross;
		}

		return bullet.velocity.y < 0 ? 
			MathHelper::getLinesCross(x1, y1, x2, y2, 0, 0, maxX, 0) : 
			MathHelper::getLinesCross(x1, y1, x2, y2, 0, maxY, maxX, maxY);
	}
}

Vec2Double getBulletCrossWallPoint(const Bullet& bullet, double maxX, double maxY, const Game& game) {
	auto crossPoint = getBulletCrossBorderPoint(bullet, maxX, maxY);
	const auto bulletTiles = MathHelper::getLineSquares(bullet.position, crossPoint, 1);
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

	return crossPoint;
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

void drawBullets(Debug& debug, const Game& game) {
	
	const auto maxX = game.level.tiles.size() * TILE_SIZE;
	const auto maxY = game.level.tiles[0].size() * TILE_SIZE;
	for (const auto& bullet : game.bullets) {		
		auto crossPoint = getBulletCrossWallPoint(bullet, maxX, maxY, game);		

		const auto debugBullet = vec2DoubleToVec2Float(bullet.position);
		const auto debugCrossPoint = vec2DoubleToVec2Float(crossPoint);
		debug.draw(CustomData::Line(debugBullet, debugCrossPoint, 0.1, ColorFloat(0, 255, 0, 0.5)));
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
	drawShootingLine(debug, game, weaponPoistion, *(*unit.weapon).lastAngle, maxX, maxY, ColorFloat(0, 0, 255, 0.5));
	drawShootingLine(debug, game, weaponPoistion, *(*unit.weapon).lastAngle - (*unit.weapon).spread, maxX, maxY, ColorFloat(100, 100, 255, 0.5));
	drawShootingLine(debug, game, weaponPoistion, *(*unit.weapon).lastAngle + (*unit.weapon).spread, maxX, maxY, ColorFloat(100, 100, 255, 0.5));
	

}

bool isVisibleEnemy(const Unit& me, const Unit& enemy, const Game& game) {
	const auto weaponPoistion = Vec2Double(me.position.x, me.position.y + me.size.y / 2);
	const auto enemyCenterPosition = Vec2Double(enemy.position.x, enemy.position.y + enemy.size.y / 2);
	auto squares = MathHelper::getLineSquares(weaponPoistion, enemyCenterPosition, 1);
	const auto wall = find_if(squares.begin(), squares.end(), [game](const auto& p) {return game.level.tiles[p.first][p.second] == Tile::WALL; });
	return wall == squares.end();
}

//TODO: учесть размер пули
pair<int, int> getShootMeBulletTick(const Unit& me, const Bullet& bullet, const Game& game) {

	const auto maxX = game.level.tiles.size() * TILE_SIZE;
	const auto maxY = game.level.tiles[0].size() * TILE_SIZE;

	auto x1 = me.position.x - me.size.x / 2;
	auto x2 = me.position.x + me.size.x / 2;
	auto y1 = me.position.y;
	auto y2 = me.position.y + me.size.y;


	if (bullet.velocity.x > 0 && bullet.position.x > x2) return make_pair(-1,-1);
	if (bullet.velocity.x < 0 && bullet.position.x < x1) return make_pair(-1, -1);
	if (bullet.velocity.y > 0 && bullet.position.y > y2) return make_pair(-1, -1);
	if (bullet.velocity.y < 0 && bullet.position.y < y1) return make_pair(-1, -1);

	auto bulletX1 = bullet.position.x;
	auto bulletY1 = bullet.position.y;
	auto bulletX2 = bullet.position.x + bullet.velocity.x;
	auto bulletY2 = bullet.position.y + bullet.velocity.y;

	auto cross1 = MathHelper::getLinesCross(bulletX1, bulletY1, bulletX2, bulletY2,
		x1, y1, x1, y2);
	auto cross2 = MathHelper::getLinesCross(bulletX1, bulletY1, bulletX2, bulletY2,
		x1, y2, x2, y2);
	auto cross3 = MathHelper::getLinesCross(bulletX1, bulletY1, bulletX2, bulletY2,
		x2, y2, x2, y1);
	auto cross4 = MathHelper::getLinesCross(bulletX1, bulletY1, bulletX2, bulletY2,
		x2, y1, x1, y1);

	const auto crossWallPoint = getBulletCrossWallPoint(bullet, maxX, maxY, game);
	auto wallDist = MathHelper::getVectorLength(Vec2Double(bullet.position.x - crossWallPoint.x, bullet.position.y - crossWallPoint.y));
	auto bulletVelocity = MathHelper::getVectorLength(bullet.velocity);
	int shootWallTick = (int)(ceil(wallDist / bulletVelocity * game.properties.ticksPerSecond));

	if (cross1.y >= y1 && cross1.y <= y2) {
		const auto bulletTiles = MathHelper::getLineSquares(bullet.position, cross1, 1);
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

		if (firstWallTile == nullptr) {
			auto dist = MathHelper::getVectorLength(Vec2Double(bullet.position.x - cross1.x, bullet.position.y - cross1.y));		
			int shootMeTick = (int)(ceil(dist / bulletVelocity * game.properties.ticksPerSecond));
			return make_pair(shootMeTick, shootWallTick);
		}
	}

	if (cross2.x >= x1 && cross2.x <= x2) {
		const auto bulletTiles = MathHelper::getLineSquares(bullet.position, cross2, 1);
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

		if (firstWallTile == nullptr) { 
			auto dist = MathHelper::getVectorLength(Vec2Double(bullet.position.x - cross2.x, bullet.position.y - cross2.y));
			int shootMeTick = (int)(ceil(dist / bulletVelocity * game.properties.ticksPerSecond));
			return make_pair(shootMeTick, shootWallTick);
		}
	}

	if (cross3.y >= y1 && cross3.y <= y2) {
		const auto bulletTiles = MathHelper::getLineSquares(bullet.position, cross3, 1);
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

		if (firstWallTile == nullptr) { 
			auto dist = MathHelper::getVectorLength(Vec2Double(bullet.position.x - cross3.x, bullet.position.y - cross3.y));
			int shootMeTick = (int)(ceil(dist / bulletVelocity * game.properties.ticksPerSecond));
			return make_pair(shootMeTick, shootWallTick);
		}
	}

	if (cross4.x >= x1 && cross4.x <= x2) {
		const auto bulletTiles = MathHelper::getLineSquares(bullet.position, cross4, 1);
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

		if (firstWallTile == nullptr) {
			auto dist = MathHelper::getVectorLength(Vec2Double(bullet.position.x - cross4.x, bullet.position.y - cross4.y));
			int shootMeTick = (int)(ceil(dist / bulletVelocity * game.properties.ticksPerSecond));
			return make_pair(shootMeTick, shootWallTick);
		}
	}
	
	return make_pair(-1,-1);
}

Vec2Double getBulletPosition(const Bullet& bullet, int tick, const Game& game) {
	double time = 1.0 * tick / game.properties.ticksPerSecond;
	return Vec2Double(bullet.position.x + bullet.velocity.x * time, bullet.position.y + bullet.velocity.y * time);
}

vector<ShootMeBullet> getShootMeBullets(const Unit& unit, const Game& game) {
	
	vector<ShootMeBullet> result;
	for (const auto& bullet : game.bullets) {
		//if (bullet.playerId == unit.playerId) continue;
		const auto smbt = getShootMeBulletTick(unit, bullet, game);
		if (smbt.first == -1) continue;
		result.emplace_back(ShootMeBullet(bullet, smbt.first, smbt.second));
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




pair<int, int> getJumpAndStopTicks(const Unit& me, const vector<ShootMeBullet>& shootingMeBullets, const Game& game) {

	if (shootingMeBullets.empty()) {
		return make_pair(-1, -1);
	}
	int minShootMeTick = INT_MAX;
	int maxShootWallTick = 0;

	for (const auto& smb : shootingMeBullets) {
		if (smb.shootMeTick < minShootMeTick) {
			minShootMeTick = smb.shootMeTick;
		}

		if (smb.shootWallTick > maxShootWallTick) {
			maxShootWallTick = smb.shootWallTick;
		}
	}

	
	for (int startJumpTick = minShootMeTick - 1; startJumpTick >= 0; startJumpTick--) {
		for (int stopJumpTick = startJumpTick + 1; stopJumpTick < maxShootWallTick; ++stopJumpTick) {

			auto isGoodJump = true;
			for (auto tick = 1; tick < maxShootWallTick; ++tick) {
				const auto mePosition = getJumpUnitPosition(me, startJumpTick, stopJumpTick, tick, game);
				for (const auto& smb : shootingMeBullets) {
					if (smb.shootWallTick <= tick) continue;
					const auto bulletPosition = getBulletPosition(smb.bullet, tick, game);

					if (isBulletInUnit(mePosition, me.size, bulletPosition, smb.bullet.size)) {
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

  drawBullets(debug, game);
  drawShootingSector(debug, unit, game);

  //debug.draw(CustomData::Line(Vec2Float(10, 10), Vec2Float(500, 500), 10, ColorFloat(255, 0, 0, 1)));
  Vec2Double aim = Vec2Double(0, 0);
  auto isVisible = false;
  if (nearestEnemy != nullptr) {  
	isVisible = isVisibleEnemy(unit, *nearestEnemy, game);
	if (isVisible) {
		aim = Vec2Double(nearestEnemy->position.x - unit.position.x,
			nearestEnemy->position.y - unit.position.y);
	}
  }

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
  if (getStopJumpTick() == 0) {
	  jump = false;
	  decreaseStopJumpTick();
  }
  else if (getStopJumpTick() > 0) {
	  jump = true;
	  decreaseStopJumpTick();
  }
  else {
	  jump = unit.weapon == nullptr && targetPos.y > unit.position.y;	  

	  if (unit.jumpState.canJump) {
		  const auto shootMeBullet = getShootMeBullets(unit, game);
		  const auto jumpAndStopTicks = getJumpAndStopTicks(unit, shootMeBullet, game);
		  debug.draw(CustomData::Log(to_string(jumpAndStopTicks.first) + " " + to_string(jumpAndStopTicks.second)));

		  if (jumpAndStopTicks.first == 0) {
			  jump = true;
			  setStopJumpTick(jumpAndStopTicks.second);
		  }
	  } 
  }
  
  
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
  action.velocity = isVisible && unit.weapon != nullptr ? 0 : targetPos.x - unit.position.x;
  action.jump = jump;
  action.jumpDown = !action.jump;
  action.aim = aim;
  action.shoot = isVisible;
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
