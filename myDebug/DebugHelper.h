#pragma once
#include "../model/Vec2Double.hpp"
#include "../simulation/BulletSimulation.h"
#include <map>

class ColorFloat;
class Debug;
class Unit;
class Game;
Vec2Double getShootingCrossBorderPoint(const Vec2Double& position, double lastAngle, double maxX, double maxY);

void drawBullets(Debug& debug, const Game& game, const std::map<Bullet, BulletSimulation>& bulletsSimulations, int mePlayerId);
void drawShootingLine(
	Debug& debug, const Game& game, const Vec2Double& weaponPoistion, double angle, double maxX, double maxY,
	const ColorFloat& color);
void drawShootingSector(Debug& debug, const Unit& unit, const Game& game);
