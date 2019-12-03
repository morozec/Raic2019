#pragma once
#include "../strategy/ShootMeBulletCrossPoint.h"
#include "../model/Bullet.hpp"
#include "../model/Game.hpp"
#include "../model/UnitAction.hpp"


class Simulator
{
public:
	static Vec2Double getBulletCrossBorderPoint(const Vec2Double& bulletPosition, const Vec2Double& bulletVelocity, double maxX,
		double maxY);	
	static Vec2Double getBulletCrossWallPoint(const Bullet& bullet, double maxX, double maxY, const Game& game);
	static ShootMeBulletCrossPoint get_shoot_me_bullet_cross_point(
		double x1, double y1, double x2, double y2,
		const Vec2Double& bulletPosition, const Vec2Double& bulletVelocity,
		const Game& game);
	static bool isBulletCrossWall(const Vec2Double& bulletPosition, const Vec2Double& bulletVelocity, double time,
		const Game& game);
	static Vec2Double getBulletPosition(const Bullet& bullet, double time, const Game& game);

	static Vec2Double getUnitGoSidePosition(
		const Unit& unit, int startGoTick, int stopGoTick, double time, int coeff, const Game& game);

	static Vec2Double getFallingUnitPosition(
		const Unit& unit, int startGoTick, int stopGoTick, double time, const Game& game);

	static Vec2Double getJumpingUnitPosition(const Unit& unit, int startJumpTick, int stopJumpTick, double time, const Game& game);
	static Vec2Double getUnitInTimePosition(const Unit& unit, const UnitAction& action, double time, const Game& game);

	static bool isUnitOnLadder(const Unit& unit, const Game& game);
	static bool isUnitOnPlatform(const Unit& unit, const Game& game);
	
private:
	static Vec2Double getBulletCornerCrossWallPoint(
		const Vec2Double& bulletPosition, const Vec2Double& bulletVelocity, double maxX, double maxY, const Game& game);
};
