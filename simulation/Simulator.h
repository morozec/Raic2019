#pragma once
#include "../strategy/ShootMeBulletCrossPoint.h"
#include "../model/Bullet.hpp"
#include "../model/Game.hpp"
#include "../model/UnitAction.hpp"


class Simulator
{
public:
	//bullet simulation
	static Vec2Double getBulletCrossBorderPoint(const Vec2Double& bulletPosition, const Vec2Double& bulletVelocity, double maxX,
		double maxY);	
	static Vec2Double getBulletCrossWallPoint(const Bullet& bullet, double maxX, double maxY, const Game& game);
	static ShootMeBulletCrossPoint get_shoot_me_bullet_cross_point(
		double x1, double y1, double x2, double y2,
		const Vec2Double& bulletPosition, const Vec2Double& bulletVelocity,
		const Game& game);
	static bool isBulletCrossWall(const Vec2Double& bulletPosition, const Vec2Double& bulletVelocity, double time,
		const Game& game);
	static Vec2Double getBulletInTimePosition(const Bullet& bullet, double time, const Game& game);
		
	//unit simulation	
	static Vec2Double getUnitNextTickPosition(const Vec2Double& unitPosition, const Vec2Double& unitSize, const UnitAction& action, const Game& game);

	//unit positioning
	static bool isUnitOnWall(const Vec2Double& unitPosition, const Vec2Double& unitSize, const Game& game);
	static bool isUnitOnLadder(const Vec2Double& unitPosition, const Vec2Double& unitSize, const Game& game);
	static bool isUnitOnPlatform(const Vec2Double& unitPosition, const Vec2Double& unitSize, const Game& game);
	static bool isUnitOnAir(const Vec2Double& unitPosition, const Vec2Double& unitSize, const Game& game);
	
private:
	static Vec2Double getBulletCornerCrossWallPoint(
		const Vec2Double& bulletPosition, const Vec2Double& bulletVelocity, double maxX, double maxY, const Game& game);
	static bool canGoThroughTile(const Tile& tile, bool jumpDown);
};
