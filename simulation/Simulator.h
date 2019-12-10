#pragma once
#include "../strategy/ShootMeBulletCrossPoint.h"
#include "../model/Bullet.hpp"
#include "../model/Game.hpp"
#include "../model/UnitAction.hpp"
#include "BulletSimulation.h"
#include <map>


class Simulator
{
public:

	//bullet simulation
	static bool getBulletInTimePosition(
		const Bullet& bullet, double time, const BulletSimulation& bulletSimulation, const Game& game, Vec2Double& position);
	static BulletSimulation getBulletSimulation(const Vec2Double& bulletPosition, const Vec2Double& bulletVelocity, double halfBulletSize, const Game& game);	
	static bool getBulletRectangleFirstCrossPoint(const Vec2Double& bulletPos, const Vec2Double& bulletVelocity, double halfBulletSize,
		double xLeft, double yDown, double xRight, double yUp,
		Vec2Double& crossPoint, Vec2Double& bulletCorner);
	
	//unit simulation	
	static Vec2Double getUnitInTimePosition(
		const Vec2Double& unitPosition, const Vec2Double& unitSize, const UnitAction& action, double time, JumpState& jumpState, const Game& game);
	static void getPolygon(const Vec2Double& unitPos, const Vec2Double& newUnitPos, const Vec2Double& unitSize, Vec2Double polygon[6]);
	static void getPolygon(const Vec2Double& bulletPos, const Vec2Double& newBulletPos, double halfBulletSize,
		Vec2Double polygon[6]);
	

	//unit positioning
	static bool isUnitOnWall(const Vec2Double& unitPosition, const Vec2Double& unitSize, const Game& game);
	static bool isUnitOnLadder(const Vec2Double& unitPosition, const Vec2Double& unitSize, const Game& game);
	static bool isUnitOnPlatform(const Vec2Double& unitPosition, const Vec2Double& unitSize, const Game& game);
	static bool isUnitOnAir(const Vec2Double& unitPosition, const Vec2Double& unitSize, const Game& game);
	static bool isUnitOnJumpPad(const Vec2Double& unitPosition, const Vec2Double& unitSize, const Game& game);

private:
	
	static bool canGoThroughTile(const Tile& tile, bool jumpDown);
	static Vec2Double getBulletBorderCross(const Vec2Double& bulletPos, const Vec2Double& bulletVelocity, const Game& game);
	static bool getBulletPointRectangleFirstCrossPoint(const Vec2Double& bulletPos, const Vec2Double& bulletVelocity,
		double xLeft, double yDown, double xRight, double yUp,
		Vec2Double& crossPoint, double& minDist2);

	static inline void getPolygon(const Vec2Double rect[4], const Vec2Double newRect[4], Vec2Double polygon[6]);

	static void updateJumpState(JumpState& jumpState, double time,
		const Vec2Double& unitPosition, const Vec2Double& unitSize,
		bool isPadJump, bool wasJump, bool isJump, bool isFall, const Game& game);

};
