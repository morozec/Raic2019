#pragma once
#include <map>
#include <vector>
#include "../model/Bullet.hpp"
#include "../model/Game.hpp"
#include "RunawayDirection.h"
#include "../model/UnitAction.hpp"
#include "../simulation/BulletSimulation.h"
#include <set>


class Strategy
{
public:
	static double getShootEnemyProbability(const Unit& me, const Unit& enemy, const Game& game, double spread);
	static double getShootEnemyProbability(
		const Vec2Double& mePosition, const Vec2Double& meSize,
		const Vec2Double& enemyPosition, const Vec2Double& enemySize,
		const Weapon& weapon, double spread, double shootingAngle,
		const Game& game);
	static double getShootEnemyProbability(
		const Vec2Double& meShootingPosition, const Vec2Double& meSize,
		double shootingAngle, double spread,
		const WeaponParams& weaponParams,
		const std::vector<Vec2Double>& enemyPositions, const Vec2Double& enemySize,
		const Game& game
	);

	static std::map<Bullet, BulletSimulation> getEnemyBulletsSimulation(const Game& game, int mePlayerId);
	std::map<Bullet, int>  getShootMeBullets(
		const Vec2Double& mePosition, const Vec2Double& meSize, const JumpState& meJumpState, int mePlayerId,
		const std::map<Bullet, BulletSimulation>& enemyBulletsSimulations, int addTicks,
		const Game& game) const;
	
	static bool isBulletMoveCrossUnitMove(
		const Vec2Double& unitPos, const Vec2Double& newUnitPos, const Vec2Double& unitSize,
		const Vec2Double& bulletPos, const Vec2Double& newBulletPos, double halfBulletSize);

	std::tuple<RunawayDirection, int, int, int> getRunawayAction(
		const Vec2Double& unitPosition, const Vec2Double& unitSize, int unitPlayerId,
		const JumpState& jumpState,
		const std::map<Bullet, int>& shootingMeBullets,
		const std::map<Bullet, BulletSimulation>& enemyBulletsSimulations, int addTicks,
		bool checkUp, bool checkDown, bool checkLeft, bool checkRight,
		const Game& game) const;

	static std::set<Bullet> isSafeMove(const Unit& unit, const UnitAction& action, const std::map<Bullet, BulletSimulation>& enemyBulletsSimulations, const Game& game);

	static bool isBulletExplosionShootUnit(
		const std::shared_ptr<ExplosionParams>& explosionParams, const Vec2Double& bulletCrossWallCenter,
		const Vec2Double& unitPosition, const Vec2Double& unitSize);

	int getRunawayDirection() const;
	int getStopRunawayTick() const;
	void setRunaway(RunawayDirection runaway_direction, int sjt);
	void decreaseStopRunawayTick();

	size_t getStartedJumpY() const;
	void setStartedJumpY(size_t newStartedJumpY);
	

private:
	int stop_runaway_tick_ = -1;
	RunawayDirection runaway_direction_ = GoNONE;

	size_t startedJumpY_ = 0;
};
