#pragma once
#include <map>
#include <vector>
#include "../model/Bullet.hpp"
#include "../model/Game.hpp"
#include "RunawayDirection.h"
#include "../model/UnitAction.hpp"
#include "../simulation/BulletSimulation.h"
#include <set>
#include "../common/AStar.h"


class Strategy
{
public:
	static bool isDangerousRocketShooting(const Vec2Double& shootingPos, const Vec2Double& unitSize,
		double shootingAngle,
		double spread, double halfBulletSize,
		const Game& game);
	
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

	static std::vector<std::pair<int, int>> getShootMeMines(const Vec2Double& mePosition, const Vec2Double& meSize,
		int addTicks,
		const Game& game);

	static std::map<Bullet, BulletSimulation> getEnemyBulletsSimulation(const Game& game, int mePlayerId, int meId);
	std::vector<std::pair<int, int>>  getShootMeBullets(
		const Vec2Double& mePosition, const Vec2Double& meSize, const JumpState& meJumpState, int mePlayerId, int meUnitId,
		const std::map<Bullet, BulletSimulation>& enemyBulletsSimulations, int addTicks,
		const Game& game) const; //результат: тик - дамаг
	
	static bool isBulletMoveCrossUnitMove(
		const Vec2Double& unitPos, const Vec2Double& newUnitPos, const Vec2Double& unitSize,
		const Vec2Double& bulletPos, const Vec2Double& newBulletPos, double halfBulletSize);

	std::tuple<RunawayDirection, int, int, int> getRunawayAction(
		const Vec2Double& unitPosition, const Vec2Double& unitSize, int unitPlayerId, int unitId,
		const JumpState& jumpState,
		const std::vector<std::pair<int, int>>& shootingMeBullets,
		const std::vector<std::pair<int, int>>& shootMeMines,
		const std::map<Bullet, BulletSimulation>& enemyBulletsSimulations, int addTicks,
		bool checkUp, bool checkDown, bool checkLeft, bool checkRight,
		const Game& game) const;

	static std::map<Bullet, int> isSafeMove(
		const Unit& unit, const UnitAction& action, const std::map<Bullet, BulletSimulation>& enemyBulletsSimulations, const Game& game,
		int& minesDamage);

	static bool isBulletExplosionShootUnit(
		const std::shared_ptr<ExplosionParams>& explosionParams, const Vec2Double& bulletCrossWallCenter,
		const Vec2Double& unitPosition, const Vec2Double& unitSize);

	static bool isMineExplosionShootUnit(const Vec2Double& minePosition, const Vec2Double& mineSize,
		double mineExplosionRadius, const Vec2Double& unitPosition, const Vec2Double& unitSize,
		double xRunDist, double yRunDist);

	int getRunawayDirection(int id) const;
	int getStopRunawayTick(int id) const;
	void setRunaway(int id, RunawayDirection runaway_direction, int srt);
	void decreaseStopRunawayTick(int id);

	size_t getStartedJumpY(int id) const;
	void setStartedJumpY(int id, size_t newStartedJumpY);

	bool getIsMonkeyMode(int id) const;
	void setIsMonkeyMode(int id, bool isMonkeyMode);

	bool isInit = false;

	int getJumpingUnitId() const;
	void setJumpingUnitId(int id);

	std::map<int, Vec2Double> heal_boxes_;
	std::map<int, Vec2Double> lootboxes_;

	int** grid;

	// Create a closed list and initialise it to false which means 
	// that no cell has been included yet 
	// This closed list is implemented as a boolean 2D array

	std::vector<std::vector<std::vector<std::vector<bool>>>> closedList;

	// Declare a 2D array of structure to hold the details 
	//of that cell
	std::vector<std::vector<std::vector<std::vector<cell>>>> cellDetails;

private:
	std::map<int, int> stop_runaway_ticks_;
	std::map<int, RunawayDirection> runaway_directions_;
	std::map<int, size_t> startedJumpYs_;
	std::map<int, bool> isMonkeyMode_;
	int jumpingUnitId_ = -1;
	
};
