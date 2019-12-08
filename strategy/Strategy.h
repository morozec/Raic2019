#pragma once
#include <map>
#include <vector>
#include "../model/Bullet.hpp"
#include "../model/Game.hpp"
#include "RunawayDirection.h"
#include "../Debug.hpp"
#include "../model/UnitAction.hpp"
#include "../simulation/BulletSimulation.h"


class Strategy
{
public:
	double getShootEnemyProbability(const Unit& me, const Unit& enemy, const Game& game, double spread,
		Debug* debug = nullptr) const;

	static std::map<Bullet, BulletSimulation> getEnemyBulletsSimulation(const Game& game, int meId);
	std::map<Bullet, int>  getShootMeBullets(
		const Unit& me, 
		const std::map<Bullet, BulletSimulation>& enemyBulletsSimulations, int addTicks, const UnitAction& unitAction,
		const Game& game) const;
	
	static bool isBulletMoveCrossUnitMove(
		const Vec2Double& unitPos, const Vec2Double& newUnitPos, const Vec2Double& unitSize,
		const Vec2Double& bulletPos, const Vec2Double& newBulletPos, double halfBulletSize);

	std::tuple<RunawayDirection, int, int, int> getRunawayAction(
		const Vec2Double& unitPosition, const Vec2Double& unitSize, int unitPlayerId,
		const std::map<Bullet, int>& shootingMeBullets,
		const std::map<Bullet, BulletSimulation>& enemyBulletsSimulations, int addTicks,
		bool checkUp, bool checkDown, bool checkLeft, bool checkRight,
		bool canJump,
		const Game& game) const;

	static bool isSafeMove(const Unit& unit, const UnitAction& action, const std::map<Bullet, BulletSimulation>& enemyBulletsSimulations, const Game& game);

	static bool isBulletExplosionShootUnit(const Bullet& bullet, const Vec2Double& bulletCrossWallCenter,
		const Vec2Double& unitPosition, const Vec2Double& unitSize);

	int getRunawayDirection() const;
	int getStopRunawayTick() const;
	void setRunaway(RunawayDirection runaway_direction, int sjt);
	void decreaseStopRunawayTick();

private:
	int stop_runaway_tick_ = -1;
	RunawayDirection runaway_direction_ = GoNONE;
};
