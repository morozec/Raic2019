#pragma once
#include <map>
#include "../model/Bullet.hpp"
#include "../model/Game.hpp"
#include "ShootMeBullet.h"
#include "RunawayDirection.h"
#include "../Debug.hpp"
#include "../model/UnitAction.hpp"


class Strategy
{
public:
	double getShootEnemyProbability(const Unit& me, const Unit& enemy, const Game& game, double spread,
		Debug* debug = nullptr) const;
	static std::map<Bullet, double> getEnemyBulletsShootWallTimes(const Game& game, int meId);
	int getShootMeBulletTick(const Unit& me, const Bullet& bullet, const Game& game);
	std::vector<ShootMeBullet> getShootMeBullets(const Unit& unit, const Game& game);
	static bool isBulletMoveCrossUnitMove(
		const Vec2Double& unitPos0, const Vec2Double& unitPos1,
		const Vec2Double& bulletPos0, const Vec2Double& bulletPos1,
		const Vec2Double& unitSize, double halfBulletSize);
	bool isBulletShootGoSideUnit(
		const Bullet& bullet, const Unit& unit, int startGoTick, int stopGoTick,
		double bulletShootWallTime, int coeff, const Game& game, int& killTick);
	bool isBulletShootJumpingUnit(
		const Bullet& bullet, const Unit& unit, int startJumpTick, int stopJumpTick,
		double bulletShootWallTime, const Game& game, int& killTick);

	bool isBulletShootFallingUnit(
		const Bullet& bullet, const Unit& unit, int startJumpTick, int stopJumpTick,
		double bulletShootWallTime, const Game& game, int& killTick) const;

	std::tuple<RunawayDirection, int, int> getRunawayAction(
		const Unit& me, const std::vector<ShootMeBullet>& shootingMeBullets,
		const std::map<Bullet, double>& enemyBulletsShootWallTimes,
		const Game& game);

	static bool isSafeMove(const Unit& unit, const UnitAction& action, const std::map<Bullet, double>& enemyBulletShootWallTimes, const Game& game);
};
