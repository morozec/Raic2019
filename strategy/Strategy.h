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
	std::map<Bullet, BulletSimulation>  getShootMeBullets(const Unit& me, const std::map<Bullet, BulletSimulation>& enemyBulletsSimulations, const Game& game) const;
	
	static bool isBulletMoveCrossUnitMove(
		const Vec2Double& unitPos0, const Vec2Double& unitPos1,
		const Vec2Double& bulletPos0, const Vec2Double& bulletPos1,
		const Vec2Double& unitSize, double halfBulletSize);

	std::tuple<RunawayDirection, int, int> getRunawayAction(
		const Unit& me,
		const std::map<Bullet, BulletSimulation>& shootingMeBullets,
		const std::map<Bullet, BulletSimulation>& enemyBulletsSimulations,
		const Game& game);

	static bool isSafeMove(const Unit& unit, const UnitAction& action, const std::map<Bullet, BulletSimulation>& enemyBulletsSimulations, const Game& game);

	int getRunawayDirection() const;
	int getStopRunawayTick() const;
	void setRunaway(RunawayDirection runaway_direction, int sjt);
	void decreaseStopRunawayTick();

private:
	int stop_runaway_tick_ = -1;
	RunawayDirection runaway_direction_ = GoNONE;
};
