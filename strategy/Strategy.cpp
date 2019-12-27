#define _USE_MATH_DEFINES

#include "Strategy.h"
#include "../common/Helper.h"
#include "../simulation/Simulator.h"
#include "../model/Unit.hpp"
#include "../mathcalc/MathHelper.h"
#include <map>
#include "RunawayDirection.h"
#include <climits>
#include <cmath>
#include <algorithm>
#include <map>
#include <set>


inline bool operator<(const Bullet& lhs, const Bullet& rhs)
{
	return lhs.position.x < rhs.position.x;
}

bool Strategy::isDangerousRocketShooting(const Vec2Double& shootingPos, const Vec2Double& unitSize,
	double shootingAngle,
	double spread, double halfBulletSize,
	const Game& game)
{
	const auto bulletCenterPos = Vec2Double(shootingPos.x, shootingPos.y + unitSize.y / 2);
	const auto deltaAngle = spread / ANGLE_SPLIT_COUNT;

	int dangerousShootingCount = 0;

	for (auto i = -ANGLE_SPLIT_COUNT; i <= ANGLE_SPLIT_COUNT; ++i)
	{
		const auto angle = shootingAngle + deltaAngle * i;
		auto bulletVelocity = Vec2Double(cos(angle), sin(angle));

		const auto bulletSimulation = Simulator::getBulletSimulation(
			bulletCenterPos, bulletVelocity, halfBulletSize, game);

		if (std::abs(shootingPos.y - bulletSimulation.targetCrossPoint.y) < SAFE_ROCKET_DISTANCE1 ||
			std::abs(shootingPos.y + unitSize.y - bulletSimulation.targetCrossPoint.y) < SAFE_ROCKET_DISTANCE1)
		{
			if (std::abs(shootingPos.x + unitSize.x/2 - bulletSimulation.targetCrossPoint.x) < SAFE_ROCKET_DISTANCE2 ||
				std::abs(shootingPos.x - unitSize.x / 2 - bulletSimulation.targetCrossPoint.x) < SAFE_ROCKET_DISTANCE2)
			{
				dangerousShootingCount++;
			}
		}
		else if (std::abs(shootingPos.x + unitSize.x/2 - bulletSimulation.targetCrossPoint.x) < SAFE_ROCKET_DISTANCE1 ||
			std::abs(shootingPos.x - unitSize.x / 2 - bulletSimulation.targetCrossPoint.x) < SAFE_ROCKET_DISTANCE1)
		{
			if (std::abs(shootingPos.y - bulletSimulation.targetCrossPoint.y) < SAFE_ROCKET_DISTANCE2 ||
				std::abs(shootingPos.y + unitSize.y - bulletSimulation.targetCrossPoint.y) < SAFE_ROCKET_DISTANCE2)
			{
				dangerousShootingCount++;
			}
		}
	}

	return dangerousShootingCount * 1.0 / (2 * ANGLE_SPLIT_COUNT + 1.0) > SAFE_ROCKET_PROBABILITY;
}

double Strategy::getShootEnemyProbability(const Unit& me, const Unit& enemy, const Game& game, double spread)
{
	if (me.weapon == nullptr) return 0;
	if (me.weapon->lastAngle == nullptr) return 1;
	return getShootEnemyProbability(
		me.position, me.size, enemy.position, enemy.size, *me.weapon, spread, *(me.weapon->lastAngle), game);
}

double Strategy::getShootEnemyProbability(
	const Vec2Double& mePosition, const Vec2Double& meSize,
	const Vec2Double& enemyPosition, const Vec2Double& enemySize,
	const Weapon& weapon, double spread, double shootingAngle,
	const Game& game)
{

	const auto bulletCenterPos = Vec2Double(mePosition.x, mePosition.y + meSize.y / 2);
	const auto deltaAngle = spread / ANGLE_SPLIT_COUNT;

	const auto xLeft = enemyPosition.x - enemySize.x / 2;
	const auto xRight = enemyPosition.x + enemySize.x / 2;
	const auto yDown = enemyPosition.y;
	const auto yUp = enemyPosition.y + enemySize.y;

	auto shootingCount = 0;

	const auto bulletVelocityLength = weapon.params.bullet.speed;
	const auto halfBulletSize = weapon.params.bullet.size / 2;

	for (auto i = -ANGLE_SPLIT_COUNT; i <= ANGLE_SPLIT_COUNT; ++i)
	{
		const auto angle = shootingAngle + deltaAngle * i;
		auto bulletVelocity = Vec2Double(bulletVelocityLength * cos(angle), bulletVelocityLength * sin(angle));

		Vec2Double shootingCrossPoint;
		Vec2Double bulletCornerPoint;
		const auto isShooting = Simulator::getBulletRectangleFirstCrossPoint(bulletCenterPos, bulletVelocity, halfBulletSize,
			xLeft, yDown, xRight, yUp, shootingCrossPoint, bulletCornerPoint);

		if (isShooting)
		{
			const auto bulletSimulation = Simulator::getBulletSimulation(bulletCenterPos, bulletVelocity, halfBulletSize, game);

			if (MathHelper::getVectorLength2(bulletCornerPoint, shootingCrossPoint) <
				MathHelper::getVectorLength2(bulletSimulation.bulletCrossCorner, bulletSimulation.targetCrossPoint))
			{
				shootingCount++;				
			}
		}
	}

	return shootingCount * 1.0 / (2 * ANGLE_SPLIT_COUNT + 1.0);
}

double Strategy::getShootEnemyProbability(
	const Vec2Double& meShootingPosition, const Vec2Double& meSize, 
	double shootingAngle, double spread, 
	const WeaponParams& weaponParams,
	const std::vector<Vec2Double>& enemyPositions, const Vec2Double& enemySize, 
	const Game& game)
{
	const auto tickTime = 1.0 / game.properties.ticksPerSecond;
	const auto deltaAngle = spread / ANGLE_SPLIT_COUNT;
	const auto halfBulletSize = weaponParams.bullet.size / 2;
	const auto startBulletPosition = Vec2Double(meShootingPosition.x, meShootingPosition.y + meSize.y / 2);
	
	auto shootingCount = 0;
	
	for (auto i = -ANGLE_SPLIT_COUNT; i <= ANGLE_SPLIT_COUNT; ++i)
	{
		const auto angle = shootingAngle + deltaAngle * i;
		auto bulletVelocity = Vec2Double(weaponParams.bullet.speed * cos(angle), weaponParams.bullet.speed * sin(angle));

		const auto bulletSimulation = Simulator::getBulletSimulation(
			startBulletPosition, bulletVelocity, halfBulletSize, game);
		const auto bulletPositions = Simulator::getBulletPositions(
			startBulletPosition, bulletVelocity, bulletSimulation.targetCrossTime, game);
		
		const auto shootWallTick = static_cast<size_t>(ceil(bulletSimulation.targetCrossTime * game.properties.ticksPerSecond));



		
		
		
		
		for (size_t j = 0; j < shootWallTick; ++j)
		{
			const auto& bp0 = bulletPositions.at(j);
			const auto& bp1 = bulletPositions.at(j < shootWallTick - 1 ? j + 1 : -1);

			const auto& ep0 = j < enemyPositions.size() ? enemyPositions[j] : enemyPositions.back();
			const auto& ep1 = j + 1 < enemyPositions.size() ? enemyPositions[j + 1] : enemyPositions.back();

			
			if (j == shootWallTick - 1) // тик удара о стену
			{
				const auto thisTickTime = bulletSimulation.targetCrossTime - j / game.properties.ticksPerSecond;
				const auto thisTickPart = thisTickTime / tickTime;
				const auto shootWallEp1 = ep0 + (ep1 - ep0) * thisTickPart;

				if (isBulletExplosionShootUnit(
					weaponParams.explosion, bulletPositions.at(-1), weaponParams.bullet.size/2.0,
					shootWallEp1, enemySize))
				{
					shootingCount++;
					break;
				}

				if (isBulletMoveCrossUnitMove(
					ep0, shootWallEp1, enemySize, bp0, bp1, halfBulletSize))
				{
					shootingCount++;
					break;
				}
			}		
			
			else
			{
				if (isBulletMoveCrossUnitMove(
					ep0, ep1, enemySize, bp0, bp1, halfBulletSize))
				{
					shootingCount++;
					break;
				}
			}
				
			
		}
	}
	return shootingCount * 1.0 / (ANGLE_SPLIT_COUNT * 2 + 1);
}

std::map<Bullet, BulletSimulation> Strategy::getEnemyBulletsSimulation(const Game& game, int mePlayerId, int meId)
{
	std::map<Bullet, BulletSimulation> simulations;
	for (const auto& bullet: game.bullets)
	{
		if (bullet.unitId == meId && bullet.explosionParams == nullptr) continue;
		auto simulation = Simulator::getBulletSimulation(bullet.position, bullet.velocity, bullet.size / 2, game);

		if (bullet.explosionParams != nullptr)//проверим взрывающуюся пулю на столкновение с врагом
		{
			for (const auto& unit: game.units)
			{
				if (bullet.unitId == unit.id) continue;//пуля не попадет в стреляющего
				if (unit.id == meId) continue;//попадание в себя не интересно

				Vec2Double crossPoint;
				Vec2Double bulletCorner;
				const auto isShooting = Simulator::getBulletRectangleFirstCrossPoint(
					bullet.position, bullet.velocity, bullet.size / 2,
					unit.position.x - unit.size.x / 2, unit.position.y, unit.position.x + unit.size.x / 2, unit.position.y + unit.size.y,
					crossPoint, bulletCorner);
				if (!isShooting) continue; //промахнемся
				
				if (MathHelper::getVectorLength2(bulletCorner, crossPoint) >
					MathHelper::getVectorLength2(simulation.bulletCrossCorner, simulation.targetCrossPoint))
					continue; //пуля раньше ударится в стену

				simulation.targetCrossPoint = crossPoint;
				simulation.bulletCrossCorner = bulletCorner;
				simulation.targetCrossTime = MathHelper::getVectorLength(bulletCorner, crossPoint) /
					MathHelper::getVectorLength(bullet.velocity);
				simulation.bulletTarget = EnemyUnit;
			}
		}

		//проверим, что пуля попадет в мину
		for (const auto& mine: game.mines)
		{
			Vec2Double crossPoint;
			Vec2Double bulletCorner;
			const auto isShooting = Simulator::getBulletRectangleFirstCrossPoint(
				bullet.position, bullet.velocity, bullet.size / 2,
				mine.position.x - mine.size.x / 2, mine.position.y, mine.position.x + mine.size.x / 2, mine.position.y + mine.size.y,
				crossPoint, bulletCorner);
			if (!isShooting) continue;

			if (MathHelper::getVectorLength2(bulletCorner, crossPoint) >
				MathHelper::getVectorLength2(simulation.bulletCrossCorner, simulation.targetCrossPoint))
				continue; //пуля раньше ударится в стену или в юнита

			simulation.targetCrossPoint = crossPoint;
			simulation.bulletCrossCorner = bulletCorner;
			simulation.targetCrossTime = MathHelper::getVectorLength(bulletCorner, crossPoint) /
				MathHelper::getVectorLength(bullet.velocity);
			simulation.bulletTarget = GameMine;
			simulation.minePosition = mine.position;
		}
		
		simulation.bulletPositions = Simulator::getBulletPositions(bullet.position, bullet.velocity, simulation.targetCrossTime, game);
		simulations[bullet] = simulation;
	}
	return simulations;
}


std::vector<std::pair<int, int>> Strategy::getShootMeMines(
	const Vec2Double& mePosition, const Vec2Double& meSize, const JumpState& meJumpState, int meId,
	int addTicks,
	const Game& game)
{
	UnitAction action;
	action.velocity = 0;
	action.jump = false;
	action.jumpDown = false;
	
	const auto tickTime = 1.0 / game.properties.ticksPerSecond;
	std::vector<std::pair<int, int>> shootMeMines;
	for (const auto& mine:game.mines)
	{
		if (mine.state != TRIGGERED) continue;
		if (addTicks == 1 && mine.timer != nullptr && *mine.timer < tickTime) continue; //мина взорволась в тик 0-1. ее мы учли заранее

		auto expTick = static_cast<int>((*mine.timer) / tickTime);
		expTick -= addTicks;
		auto jumpState = meJumpState;
		auto pos = mePosition;
		
		for (int i = 0; i < expTick; ++i)
		{
			pos = Simulator::getUnitInTimePosition(pos, meSize, meId, action, tickTime, jumpState, game);
		}
		const auto leftTime = (*mine.timer) - addTicks * tickTime - expTick * tickTime;
		pos = Simulator::getUnitInTimePosition(pos, meSize, meId, action, leftTime, jumpState, game);

		const auto shoot = isMineExplosionShootUnit(mine.position, mine.size, mine.explosionParams.radius,
			pos, meSize, 0, 0);
				
		if (shoot)
		{
			const auto tick = static_cast<int>(ceil(*mine.timer / tickTime)) - addTicks;
			shootMeMines.emplace_back(tick, mine.explosionParams.damage);
		}
	}
	return shootMeMines;
}


std::vector<std::pair<int, int>> Strategy::getShootMeBullets(
	const Vec2Double& mePosition, const Vec2Double& meSize, const JumpState& meJumpState, int mePlayerId, int meUnitId,
	const std::map<Bullet, BulletSimulation>& enemyBulletsSimulations, int addTicks,
	const Game& game) const
{
	//if (addTicks != 0 && addTicks != 1) throw std::runtime_error("addTicks is not 0 or 1");
	
	const auto tickTime = 1.0 / game.properties.ticksPerSecond;
	std::vector<std::pair<int, int>> shootMeBullets;
	std::map<int, Vec2Double> mePositions;
	std::map<int, JumpState> meJumpStates;
	
	mePositions[0] = mePosition;
	meJumpStates[0] = meJumpState;

	UnitAction action;
	action.velocity = 0;
	action.jump = false;
	action.jumpDown = false;

	for (const auto& item:enemyBulletsSimulations)
	{
		const auto& bullet = item.first;
		const auto& bulletSimulation = item.second;
		
		std::map<int, Vec2Double> bulletPositions;
		bulletPositions[0] = bullet.position;		
	
		const auto bulletCrossWallTime = bulletSimulation.targetCrossTime;
		const int bulletCrossWallTick = static_cast<int>(ceil(bulletCrossWallTime*game.properties.ticksPerSecond));

		if (addTicks == 1 && bulletCrossWallTick <= 1)
		{
			//пуля ударилась в тик 0-1. ее мы учли заранее
			continue;
		}
		
		for (int tick = 1; tick <= bulletCrossWallTick - addTicks; ++tick)
		{
			Vec2Double bulletInTimePosition;
			const bool exists = Simulator::getBulletInTimePosition(
				bullet.position, bullet.velocity, (tick+addTicks)* tickTime, 
				bulletSimulation.targetCrossTime, game, bulletInTimePosition);
						

			if (exists)
			{
				bulletPositions[tick] = bulletInTimePosition;
				
				Vec2Double unitInTimePosition;
				if (mePositions.count(tick) > 0) unitInTimePosition = mePositions[tick];//уже посчитали для другой пули
				else
				{
					auto unitInTimeJumpState = meJumpStates.at(tick - 1);
					unitInTimePosition =
						Simulator::getUnitInTimePosition(
							mePositions.at(tick - 1), meSize, meUnitId, action, tickTime, unitInTimeJumpState, game);
					mePositions[tick] = unitInTimePosition;
					meJumpStates[tick] = unitInTimeJumpState;
				}
				const bool isShooting = bullet.unitId != meUnitId && isBulletMoveCrossUnitMove(
					mePositions.at(tick - 1), unitInTimePosition, meSize,
					bulletPositions[tick - 1], bulletInTimePosition, bullet.size / 2.0);
				if (isShooting)
				{
					int damage = bullet.damage;
					if (bullet.explosionParams != nullptr) damage += bullet.explosionParams->damage;
					shootMeBullets.emplace_back(std::make_pair(tick, damage));
					break;
				}
			}
			else
			{
				
				const auto thisTickBulletTime = bulletSimulation.targetCrossTime - (tick + addTicks - 1) * tickTime;

				auto unitInTimeJumpState = meJumpStates.at(tick - 1);
				const auto unitInTimePosition =
					Simulator::getUnitInTimePosition(
						mePositions.at(tick - 1), meSize, meUnitId, action, thisTickBulletTime, unitInTimeJumpState, game);
				auto isShooting = bullet.unitId != meUnitId && isBulletMoveCrossUnitMove(
					mePositions.at(tick - 1), unitInTimePosition, meSize,
					bulletPositions[tick - 1], bulletInTimePosition, bullet.size / 2.0);
				if (isShooting)
				{
					int damage = bullet.damage;
					if (bullet.explosionParams != nullptr) damage += bullet.explosionParams->damage;
					shootMeBullets.emplace_back(std::make_pair(tick, damage));
					break;
				}

				int damage = 0;

				if (bulletSimulation.bulletTarget == GameMine)
				{
					const auto isMineShoot = isMineExplosionShootUnit(bulletSimulation.minePosition, game.properties.mineSize,
						game.properties.mineExplosionParams.radius, unitInTimePosition, meSize, 0.0, 0.0);
					if (isMineShoot)
					{
						damage += game.properties.mineExplosionParams.damage;
					}
				}
				
				if (bullet.explosionParams != nullptr)
				{
					const auto bulletCrossWallCenter = Vec2Double(
						bullet.position.x + bullet.velocity.x * bulletSimulation.targetCrossTime,
						bullet.position.y + bullet.velocity.y * bulletSimulation.targetCrossTime);
					if (isBulletExplosionShootUnit(
						bullet.explosionParams, bulletCrossWallCenter, bullet.size/2.0, unitInTimePosition, meSize))
					{
						damage += bullet.explosionParams->damage;						
					}
				}

				if (damage > 0)
				{
					shootMeBullets.emplace_back(std::make_pair(tick, damage));
					break;
				}
				
			}			
		}
				
	}
	return shootMeBullets;
	
}


bool Strategy::isBulletMoveCrossUnitMove(
	const Vec2Double& unitPos, const Vec2Double& newUnitPos, const Vec2Double& unitSize,
	const Vec2Double& bulletPos, const Vec2Double& newBulletPos, double halfBulletSize)
{	
	Vec2Double bulletPolygon[6];
	Simulator::getPolygon(bulletPos, newBulletPos, halfBulletSize, bulletPolygon);

	Segment bulletSegments[6];
	for (int i = 0; i < 6; ++i)
	{
		const int endIndex = i < 5 ? i + 1 : 0;
		bulletSegments[i] = Segment(bulletPolygon[i], bulletPolygon[endIndex]);
	}

	const auto isStaticUnit = std ::abs(unitPos.x - newUnitPos.x) < TOLERANCE && 
		std ::abs(unitPos.y - newUnitPos.y) < TOLERANCE;	

	if (isStaticUnit)
	{
		Segment unitSegments[4];
		unitSegments[0] = Segment(
			{ unitPos.x - unitSize.x / 2, unitPos.y }, 
			{ unitPos.x - unitSize.x / 2,unitPos.y + unitSize.y });
		unitSegments[1] = Segment(
			{ unitPos.x - unitSize.x / 2, unitPos.y + unitSize.y },
			{ unitPos.x + unitSize.x / 2,unitPos.y + unitSize.y });
		unitSegments[2] = Segment(
			{ unitPos.x + unitSize.x / 2, unitPos.y + unitSize.y },
			{ unitPos.x + unitSize.x / 2,unitPos.y });
		unitSegments[3] = Segment(
			{ unitPos.x + unitSize.x / 2, unitPos.y },
			{ unitPos.x - unitSize.x / 2,unitPos.y });
		
		for (const auto& us : unitSegments)
		{
			for (const auto& bs : bulletSegments)
			{
				const auto cross = MathHelper::areSegmentsCross(us, bs);
				if (cross) return true;
			}
		}
	}else
	{
		Vec2Double unitPolygon[6];
		Simulator::getPolygon(unitPos, newUnitPos, unitSize, unitPolygon);

		Segment unitSegments[6];
		for (int i = 0; i < 6; ++i)
		{
			const int endIndex = i < 5 ? i + 1 : 0;
			unitSegments[i] = Segment(unitPolygon[i], unitPolygon[endIndex]);
		}

		for (const auto& us : unitSegments)
		{
			for (const auto& bs : bulletSegments)
			{
				const auto cross = MathHelper::areSegmentsCross(us, bs);
				if (cross) return true;
			}
		}
	}	
	
	return false;
}


std::tuple<RunawayDirection, int, int, int> Strategy::getRunawayAction(
	const Vec2Double& unitPosition, const Vec2Double& unitSize, int unitPlayerId, int unitId,
	const JumpState& jumpState,
	const std::vector<std::pair<int, int>>& shootingMeBullets,	
	const std::vector<std::pair<int, int>>& shootMeMines,	
	const std::map<Bullet, BulletSimulation>& enemyBulletsSimulations, int addTicks,
	bool checkUp, bool checkDown, bool checkLeft, bool checkRight,
	const Game& game) const
{
	//if (addTicks != 0 && addTicks != 1) throw std::runtime_error("addTicks is not 0 or 1");
	
	if (shootingMeBullets.empty() && shootMeMines.empty())
	{
		return std::make_tuple(GoNONE, -1, -1, 0);
	}

	int maxFirstShootMeTick = INT_MAX;
	int minShootMeDamage = 0;
	
	for (const auto& item: shootingMeBullets)
	{
		if (item.first < maxFirstShootMeTick) maxFirstShootMeTick = item.first;
		minShootMeDamage += item.second;		
	}
	for (const auto& item:shootMeMines)
	{
		if (item.first < maxFirstShootMeTick) maxFirstShootMeTick = item.first;
		minShootMeDamage += item.second;
	}
	

	auto bestRunAwayDirection = GoNONE;
	auto bestStartGoTick = -1;
	auto bestStopGoTick = -1;
	
	const auto tickTime = 1.0 / game.properties.ticksPerSecond;
	
	auto minShootMeTick = INT_MAX;
	for (const auto& smb : shootingMeBullets)
	{
		if (smb.first < minShootMeTick)
		{
			minShootMeTick = smb.first;
		}
	}
	for (const auto& smm : shootMeMines)
	{
		if (smm.first < minShootMeTick)
		{
			minShootMeTick = smm.first;
		}
	}
	

	double maxShootWallTime = 0;
	for (const auto& item : enemyBulletsSimulations)
	{
		if (item.second.targetCrossTime > maxShootWallTime)
		{
			maxShootWallTime = item.second.targetCrossTime;
		}
	}	
	int maxShootWallTick = static_cast<int>(ceil(maxShootWallTime * game.properties.ticksPerSecond));

	double maxMineExplosionTime = 0;
	for (const auto& mine:game.mines)
	{
		if (mine.state != TRIGGERED) continue;
		if (mine.timer == nullptr) continue;
		if (*mine.timer > maxMineExplosionTime) maxMineExplosionTime = *mine.timer;
	}

	int maxMineExplosionTick = static_cast<int>(ceil(maxMineExplosionTime * game.properties.ticksPerSecond));

	int finalBulletMineTick = std::max(maxShootWallTick, maxMineExplosionTick);
	
	finalBulletMineTick -= addTicks;

	
	UnitAction action;
	action.jump = false;
	action.jumpDown = false;
	action.velocity = false;

	/*std::map<int, int> beforeStartGoUpDamage;
	std::map<int, int> beforeStartGoDownDamage;
	std::map<int, int> beforeStartGoLeftDamage;
	std::map<int, int> beforeStartGoRightDamage;

	std::map<int, std::map<int, int>> beforeStopGoUpDamage;
	std::map<int, std::map<int, int>> beforeStopGoDownDamage;
	std::map<int, std::map<int, int>> beforeStopGoLeftDamage;
	std::map<int, std::map<int, int>> beforeStopGoRightDamage;*/
		

	for (int startGoTick = minShootMeTick - 1; startGoTick >= 0; startGoTick--)
	{	
		
		for (int stopGoTick = startGoTick + 1; stopGoTick < finalBulletMineTick; ++stopGoTick)
		{			
			auto canGoUp = checkUp;
			auto upDamage = 0;
			int upFirstMeTick = INT_MAX;
			int upShootMeCount = 0;
			
			auto canGoLeft = checkLeft;
			auto leftDamage = 0;
			int leftFirstMeTick = INT_MAX;
			int leftShootMeCount = 0;
			
			auto canGoRight = checkRight;
			auto rightDamage = 0;
			int rightFirstMeTick = INT_MAX;
			int rightShootMeCount = 0;
						
			auto canGoDown = checkDown;
			auto downDamage = 0;
			int downFirstMeTick = INT_MAX;
			int downShootMeCount = 0;

			auto jumpUnitPosition = unitPosition;
			auto fallUnitPosition = unitPosition;
			auto goLeftUnitPosition = unitPosition;
			auto goRightUnitPosition = unitPosition;

			auto goUpJumpState = jumpState;
			auto goDownJumpState = jumpState;
			auto goLeftJumpState = jumpState;
			auto goRightJumpState = jumpState;

			std::map<Bullet, bool> gotUpBullets;
			std::map<Bullet, bool> gotDownBullets;
			std::map<Bullet, bool> gotLeftBullets;
			std::map<Bullet, bool> gotRightBullets;

			for (const auto& item : enemyBulletsSimulations)
			{
				const auto& bullet = item.first;				
				gotUpBullets[bullet] = false;
				gotDownBullets[bullet] = false;
				gotLeftBullets[bullet] = false;
				gotRightBullets[bullet] = false;
			}

			for (int tick = 1; tick <= finalBulletMineTick; ++tick)
			{
				auto thisTickUpDamage = 0;
				auto thisTickDownDamage = 0;
				auto thisTickLeftDamage = 0;
				auto thisTickRightDamage = 0;
				
				
				if (tick > stopGoTick)
				{
					action.jump = false;
				}
				else if (tick > startGoTick)
				{
					action.jump = true;
				}
				
				auto thisTickGoUpJumpState = goUpJumpState;
				const auto thisTickJumpUnitPosition = Simulator::getUnitInTimePosition(
					jumpUnitPosition, unitSize, unitId, action, tickTime, thisTickGoUpJumpState, game);
				action.jump = false;				
				
				//fall
				if (tick > stopGoTick)
				{
					action.jumpDown = false;
				}
				else if (tick > startGoTick)
				{
					action.jumpDown = true;
				}

				auto thisTickGoDownJumpState = goDownJumpState;
				const auto thisTickFallUnitPosition = Simulator::getUnitInTimePosition(
					fallUnitPosition, unitSize, unitId, action, tickTime, thisTickGoDownJumpState, game);
				action.jumpDown = false;

				//left
				if (tick > stopGoTick)
				{
					action.velocity = 0;
				}
				else if (tick > startGoTick)
				{
					action.velocity = -INT_MAX;
				}
				auto thisTickGoLeftJumpState = goLeftJumpState;
				const auto thisTickGoLeftUnitPosition = Simulator::getUnitInTimePosition(
					goLeftUnitPosition, unitSize, unitId, action, tickTime, thisTickGoLeftJumpState, game);
				action.velocity = 0;

				//right
				if (tick > stopGoTick)
				{
					action.velocity = 0;
				}
				else if (tick > startGoTick)
				{
					action.velocity = INT_MAX;
				}
				auto thisTickGoRightJumpState = goRightJumpState;
				const auto thisTickGoRightUnitPosition = Simulator::getUnitInTimePosition(
					goRightUnitPosition, unitSize, unitId, action, tickTime, thisTickGoRightJumpState, game);
				action.velocity = 0;

				/*if (tick <= startGoTick && 
					beforeStartGoUpDamage.count(tick) > 0 && 
					beforeStartGoDownDamage.count(tick) > 0 && 
					beforeStartGoLeftDamage.count(tick) > 0 && 
					beforeStartGoRightDamage.count(tick) > 0)
				{
					thisTickUpDamage = beforeStartGoUpDamage[tick];
					thisTickDownDamage = beforeStartGoDownDamage[tick];
					thisTickLeftDamage = beforeStartGoLeftDamage[tick];
					thisTickRightDamage = beforeStartGoRightDamage[tick];
				}
				else if (tick <= stopGoTick && tick > startGoTick && 
					beforeStopGoUpDamage.count(tick) > 0 && beforeStopGoUpDamage[tick].count(startGoTick) > 0 &&
					beforeStopGoDownDamage.count(tick) > 0 && beforeStopGoDownDamage[tick].count(startGoTick) > 0 &&
					beforeStopGoLeftDamage.count(tick) > 0 && beforeStopGoLeftDamage[tick].count(startGoTick) > 0 &&
					beforeStopGoRightDamage.count(tick) > 0 && beforeStopGoRightDamage[tick].count(startGoTick) > 0)
				{
					thisTickUpDamage = beforeStopGoUpDamage[tick][startGoTick];
					thisTickDownDamage = beforeStopGoDownDamage[tick][startGoTick];
					thisTickLeftDamage = beforeStopGoLeftDamage[tick][startGoTick];
					thisTickRightDamage = beforeStopGoRightDamage[tick][startGoTick];
				}
				else
				{*/
					for (const auto& item : enemyBulletsSimulations)
					{
						const auto& bullet = item.first;
						const auto& bulletSimulation = item.second;
						
						const auto bulletCrossWallCenter = Vec2Double(
							bullet.position.x + bullet.velocity.x * bulletSimulation.targetCrossTime,
							bullet.position.y + bullet.velocity.y * bulletSimulation.targetCrossTime
						);
						const auto halfBulletSize = bullet.size / 2;

						auto shootWallTick = static_cast<int>(ceil(bulletSimulation.targetCrossTime * game.properties.ticksPerSecond));
						if (shootWallTick < tick + addTicks) continue;//ударилась в стену раньше

						
						const auto bulletExists = bulletSimulation.bulletPositions.count(tick + addTicks) > 0;
						const auto notExistsUnitTime = bulletSimulation.targetCrossTime - (tick + addTicks - 1) / game.properties.ticksPerSecond;
						const auto& prevTickBulletPosition = bulletSimulation.bulletPositions.at(tick + addTicks - 1);
						const auto& thisTickBulletPosition = bulletSimulation.bulletPositions.at( bulletExists ? tick + addTicks : -1);

						//jump
						if (canGoUp && !gotUpBullets[bullet]) {
							/*if (!thisTickCanJump)
							{
								action.jump = false;
							}
							else
							{*/
								if (tick > stopGoTick)
								{
									action.jump = false;
								}
								else if (tick > startGoTick)
								{
									action.jump = true;
								}
							//}

							auto newGoUpJumpState = goUpJumpState;
							const auto newJumpUnitPosition = bulletExists ? thisTickJumpUnitPosition :
								Simulator::getUnitInTimePosition(
									jumpUnitPosition,
									unitSize,
									unitId,
									action,
									notExistsUnitTime,
									newGoUpJumpState,
									game);

							action.jump = false;

							if (bullet.unitId != unitId && isBulletMoveCrossUnitMove(
								jumpUnitPosition,
								newJumpUnitPosition,
								unitSize,
								prevTickBulletPosition,
								thisTickBulletPosition,
								halfBulletSize))
							{
								thisTickUpDamage += bullet.damage;
								if (bullet.explosionParams != nullptr) thisTickUpDamage += bullet.explosionParams->damage;								
								gotUpBullets[bullet] = true;
							}
							else if (!bulletExists)
							{
								if (isBulletExplosionShootUnit(
									bullet.explosionParams, bulletCrossWallCenter, bullet.size/2.0, newJumpUnitPosition, unitSize))
								{
									thisTickUpDamage += bullet.explosionParams->damage;
									gotUpBullets[bullet] = true;
								}
								if (bulletSimulation.bulletTarget == GameMine &&
									isMineExplosionShootUnit(
										bulletSimulation.minePosition, game.properties.mineSize, game.properties.mineExplosionParams.radius,
										newJumpUnitPosition, unitSize, 0,0))
								{
									thisTickUpDamage += game.properties.mineExplosionParams.damage;
									gotUpBullets[bullet] = true;
								}
							}						
											
						}


						//fall
						if (canGoDown && !gotDownBullets[bullet]) {
							if (tick > stopGoTick)
							{
								action.jumpDown = false;
							}
							else if (tick > startGoTick)
							{
								action.jumpDown = true;
							}

							auto newGoDownJumpState = goDownJumpState;
							const auto newFallUnitPosition = bulletExists ? thisTickFallUnitPosition :
								Simulator::getUnitInTimePosition(
									fallUnitPosition,
									unitSize,
									unitId,
									action,
									notExistsUnitTime,
									newGoDownJumpState,
									game);

							action.jumpDown = false;

							if (bullet.unitId != unitId && isBulletMoveCrossUnitMove(
								fallUnitPosition,
								newFallUnitPosition,
								unitSize,
								prevTickBulletPosition,
								thisTickBulletPosition,
								halfBulletSize)) 
							{
								thisTickDownDamage += bullet.damage;
								if (bullet.explosionParams != nullptr) thisTickDownDamage += bullet.explosionParams->damage;
								gotDownBullets[bullet] = true;
							}
							else if (!bulletExists)
							{
								if (isBulletExplosionShootUnit(
									bullet.explosionParams, bulletCrossWallCenter, bullet.size/2.0, newFallUnitPosition, unitSize))
								{
									thisTickDownDamage += bullet.explosionParams->damage;
									gotDownBullets[bullet] = true;
								}
								if (bulletSimulation.bulletTarget == GameMine &&
									isMineExplosionShootUnit(
										bulletSimulation.minePosition, game.properties.mineSize, game.properties.mineExplosionParams.radius,
										newFallUnitPosition, unitSize, 0, 0))
								{
									thisTickDownDamage += game.properties.mineExplosionParams.damage;
									gotDownBullets[bullet] = true;
								}
							}
						}

						//left
						if (canGoLeft && !gotLeftBullets[bullet])
						{
							if (tick > stopGoTick)
							{
								action.velocity = 0;
							}
							else if (tick > startGoTick)
							{
								action.velocity = -INT_MAX;
							}

							auto newGoLeftJumpState = goLeftJumpState;
							const auto newGoLeftUnitPosition = bulletExists ? thisTickGoLeftUnitPosition :
								Simulator::getUnitInTimePosition(
									goLeftUnitPosition,
									unitSize,
									unitId,
									action,
									notExistsUnitTime,
									newGoLeftJumpState,
									game);

							action.velocity = 0;

							if (bullet.unitId != unitId && isBulletMoveCrossUnitMove(
								goLeftUnitPosition,
								newGoLeftUnitPosition,
								unitSize,
								prevTickBulletPosition,
								thisTickBulletPosition,
								halfBulletSize))
							{
								thisTickLeftDamage += bullet.damage;
								if (bullet.explosionParams != nullptr) thisTickLeftDamage += bullet.explosionParams->damage;
								gotLeftBullets[bullet] = true;
							}
							else if (!bulletExists)
							{
								if (isBulletExplosionShootUnit(
									bullet.explosionParams, bulletCrossWallCenter, bullet.size / 2.0, newGoLeftUnitPosition, unitSize))
								{
									thisTickLeftDamage += bullet.explosionParams->damage;
									gotLeftBullets[bullet] = true;
								}
								if (bulletSimulation.bulletTarget == GameMine &&
									isMineExplosionShootUnit(
										bulletSimulation.minePosition, game.properties.mineSize, game.properties.mineExplosionParams.radius,
										newGoLeftUnitPosition, unitSize, 0, 0))
								{
									thisTickLeftDamage += game.properties.mineExplosionParams.damage;
									gotLeftBullets[bullet] = true;
								}
							}
						}

						//right
						if (canGoRight && !gotRightBullets[bullet])
						{
							if (tick > stopGoTick)
							{
								action.velocity = 0;
							}
							else if (tick > startGoTick)
							{
								action.velocity = INT_MAX;
							}

							auto newGoRightJumpState = goRightJumpState;
							const auto newGoRightUnitPosition = bulletExists ? thisTickGoRightUnitPosition :
								Simulator::getUnitInTimePosition(
									goRightUnitPosition,
									unitSize,
									unitId,
									action,
									notExistsUnitTime,
									newGoRightJumpState,
									game);

							action.velocity = 0;

							if (bullet.unitId != unitId && isBulletMoveCrossUnitMove(
								goRightUnitPosition,
								newGoRightUnitPosition,
								unitSize,
								prevTickBulletPosition,
								thisTickBulletPosition,
								halfBulletSize))
							{
								thisTickRightDamage += bullet.damage;
								if (bullet.explosionParams != nullptr) thisTickRightDamage += bullet.explosionParams->damage;
								gotRightBullets[bullet] = true;
							}
							else if (!bulletExists)
							{
								if (isBulletExplosionShootUnit(
									bullet.explosionParams, bulletCrossWallCenter, bullet.size / 2.0, newGoRightUnitPosition, unitSize))
								{
									thisTickRightDamage += bullet.explosionParams->damage;
									gotRightBullets[bullet] = true;
								}
								if (bulletSimulation.bulletTarget == GameMine &&
									isMineExplosionShootUnit(
										bulletSimulation.minePosition, game.properties.mineSize, game.properties.mineExplosionParams.radius,
										newGoRightUnitPosition, unitSize, 0, 0))
								{
									thisTickRightDamage += game.properties.mineExplosionParams.damage;
									gotRightBullets[bullet] = true;
								}
							}
						}

					}

				//цикл по минам
				for (const auto& mine:game.mines)
				{
					if (mine.state != TRIGGERED) continue;
					const auto isExplodingMine = mine.timer != nullptr &&
						*mine.timer > (tick + addTicks - 1) * tickTime && *mine.timer <= (tick + addTicks) * tickTime;

					if (!isExplodingMine) continue;
					const auto time = *mine.timer - (tick + addTicks - 1) / game.properties.ticksPerSecond;

					auto newGoUpJumpState = goUpJumpState;
					const auto newJumpUnitPosition =
						Simulator::getUnitInTimePosition(
							jumpUnitPosition,
							unitSize,
							unitId,
							action,
							time,
							newGoUpJumpState,
							game);

					if (isMineExplosionShootUnit(mine.position, mine.size, mine.explosionParams.radius,
						newJumpUnitPosition, unitSize, 0 ,0))
					{
						thisTickUpDamage += mine.explosionParams.damage;
					}
					

					auto newGoDownJumpState = goDownJumpState;
					const auto newFallUnitPosition =
						Simulator::getUnitInTimePosition(
							fallUnitPosition,
							unitSize,
							unitId,
							action,
							time,
							newGoDownJumpState,
							game);

					if (isMineExplosionShootUnit(mine.position, mine.size, mine.explosionParams.radius,
						newFallUnitPosition, unitSize, 0, 0))
					{
						thisTickDownDamage += mine.explosionParams.damage;
					}
					

					auto newGoLeftJumpState = goLeftJumpState;
					const auto newGoLeftUnitPosition = 
						Simulator::getUnitInTimePosition(
							goLeftUnitPosition,
							unitSize,
							unitId,
							action,
							time,
							newGoLeftJumpState,
							game);

					if (isMineExplosionShootUnit(mine.position, mine.size, mine.explosionParams.radius,
						newGoLeftUnitPosition, unitSize, 0, 0))
					{
						thisTickLeftDamage += mine.explosionParams.damage;
					}


					auto newGoRightJumpState = goRightJumpState;
					const auto newGoRightUnitPosition =
						Simulator::getUnitInTimePosition(
							goRightUnitPosition,
							unitSize,
							unitId,
							action,
							time,
							newGoRightJumpState,
							game);

					if (isMineExplosionShootUnit(mine.position, mine.size, mine.explosionParams.radius,
						newGoRightUnitPosition, unitSize, 0, 0))
					{
						thisTickRightDamage += mine.explosionParams.damage;
					}
				}
										
				
					//if (tick <= startGoTick) {
					//	beforeStartGoUpDamage[tick] = thisTickUpDamage;
					//	//beforeStartGoDownDamage[tick] = thisTickDownDamage;
					//	beforeStartGoLeftDamage[tick] = thisTickLeftDamage;
					//	beforeStartGoRightDamage[tick] = thisTickRightDamage;
					//}
					//else if (tick <= stopGoTick && tick > startGoTick)
					//{
					//	beforeStopGoUpDamage[tick][startGoTick] = thisTickUpDamage;
					//	//beforeStopGoDownDamage[tick][startGoTick] = thisTickDownDamage;
					//	beforeStopGoLeftDamage[tick][startGoTick] = thisTickLeftDamage;
					//	beforeStopGoRightDamage[tick][startGoTick] = thisTickRightDamage;
					//}
					
				//}
				jumpUnitPosition = thisTickJumpUnitPosition;
				fallUnitPosition = thisTickFallUnitPosition;
				goLeftUnitPosition = thisTickGoLeftUnitPosition;
				goRightUnitPosition = thisTickGoRightUnitPosition;

				goUpJumpState = thisTickGoUpJumpState;
				goDownJumpState = thisTickGoDownJumpState;
				goLeftJumpState = thisTickGoLeftJumpState;
				goRightJumpState = thisTickGoRightJumpState;

				upDamage += thisTickUpDamage;
				if (thisTickUpDamage > 0) {
					upShootMeCount++;
					if (tick < upFirstMeTick) upFirstMeTick = tick;
				}
					
				
				downDamage += thisTickDownDamage;
				if (thisTickDownDamage > 0) {
					downShootMeCount++;
					if (tick < downFirstMeTick) downFirstMeTick = tick;
				}
				
				leftDamage += thisTickLeftDamage;
				if (thisTickLeftDamage > 0) {
					leftShootMeCount++;
					if (tick < leftFirstMeTick) leftFirstMeTick = tick;
				}
				
				rightDamage += thisTickRightDamage;
				if (thisTickRightDamage > 0)
				{
					rightShootMeCount++;
					if (tick < rightFirstMeTick) rightFirstMeTick = tick;
				} 

				if ((!canGoUp || upDamage >= minShootMeDamage) &&
					(!canGoDown || downDamage >= minShootMeDamage) &&
					(!canGoLeft || leftDamage >= minShootMeDamage) &&
					(!canGoRight || rightDamage >= minShootMeDamage))
					break;
			}

			if (canGoUp && upDamage == 0)
				return std::make_tuple(GoUP, startGoTick, stopGoTick, 0);
			if (canGoDown && downDamage == 0)
				return std::make_tuple(GoDOWN, startGoTick, stopGoTick, 0);
			if (canGoLeft && leftDamage == 0)
				return std::make_tuple(GoLEFT, startGoTick, stopGoTick, 0);
			if (canGoRight && rightDamage == 0)
				return std::make_tuple(GoRIGHT, startGoTick, stopGoTick, 0);

			if (canGoUp && (upDamage < minShootMeDamage || 
				upDamage == minShootMeDamage && upFirstMeTick > maxFirstShootMeTick && upShootMeCount >= 2))
			{
				minShootMeDamage = upDamage;
				bestRunAwayDirection = GoUP;
				bestStartGoTick = startGoTick;
				bestStopGoTick = stopGoTick;
				maxFirstShootMeTick = upFirstMeTick;
			}
			if (canGoDown && (downDamage < minShootMeDamage || 
				downDamage == minShootMeDamage && downFirstMeTick > maxFirstShootMeTick && downShootMeCount >= 2))
			{
				minShootMeDamage = downDamage;
				bestRunAwayDirection = GoDOWN;
				bestStartGoTick = startGoTick;
				bestStopGoTick = stopGoTick;
				maxFirstShootMeTick = downFirstMeTick;
			}
			if (canGoLeft && (leftDamage < minShootMeDamage || 
				leftDamage == minShootMeDamage && leftFirstMeTick > maxFirstShootMeTick && leftShootMeCount >= 2))
			{
				minShootMeDamage = leftDamage;
				bestRunAwayDirection = GoLEFT;
				bestStartGoTick = startGoTick;
				bestStopGoTick = stopGoTick;
				maxFirstShootMeTick = leftFirstMeTick;
			}
			if (canGoRight && (rightDamage < minShootMeDamage || 
				rightDamage == minShootMeDamage && rightFirstMeTick > maxFirstShootMeTick && rightShootMeCount >= 2))
			{
				minShootMeDamage = rightDamage;
				bestRunAwayDirection = GoRIGHT;
				bestStartGoTick = startGoTick;
				bestStopGoTick = stopGoTick;
				maxFirstShootMeTick = rightFirstMeTick;
			}					
		
		}
	}

	return std::make_tuple(bestRunAwayDirection, bestStartGoTick, bestStopGoTick, minShootMeDamage); 
}


std::map<Bullet, int> Strategy::isSafeMove(
	const Unit& unit, const UnitAction& action, const std::map<Bullet, BulletSimulation>& enemyBulletsSimulations, const Game& game,
	int& minesDamage)
{
	minesDamage = 0;
	
	std::map<Bullet, int> thisTickShootMeBullets;
	const auto tickTime = 1.0 / game.properties.ticksPerSecond;
	for (const auto& item : enemyBulletsSimulations)
	{
		const auto& bullet = item.first;
		const auto& bulletSimulation = item.second;
		
		double unitTime = tickTime;
		
		Vec2Double bulletInTimePosition;
		const auto exists = Simulator::getBulletInTimePosition(
			bullet.position, bullet.velocity, tickTime, bulletSimulation.targetCrossTime, game, bulletInTimePosition);
		if (!exists)
		{
			unitTime = bulletSimulation.targetCrossTime;
		}
		auto jumpState = unit.jumpState;
		const auto unitInTimePosition = Simulator::getUnitInTimePosition(
			unit.position, unit.size, unit.id, action, unitTime, jumpState, game);			
		
		const auto cross = bullet.unitId != unit.id && isBulletMoveCrossUnitMove(
			unit.position,
			unitInTimePosition,
			unit.size,
			bullet.position,
			bulletInTimePosition,
			bullet.size / 2.0);
		if (cross)
		{
			int damage = bullet.damage;
			if (bullet.explosionParams != nullptr) damage += bullet.explosionParams->damage;
			thisTickShootMeBullets[bullet] = damage;
			continue;
		}
		if (!exists)
		{
			int damage = 0;
			if (bulletSimulation.bulletTarget == GameMine)
			{
				const auto isMineShoot = isMineExplosionShootUnit(bulletSimulation.minePosition, game.properties.mineSize,
					game.properties.mineExplosionParams.radius, unitInTimePosition, unit.size, 0.0, 0.0);
				if (isMineShoot)
				{
					damage += game.properties.mineExplosionParams.damage;
				}
			}
			
			if (bullet.explosionParams != nullptr)
			{
				const auto bulletCrossWallCenter = Vec2Double(
					bullet.position.x + bullet.velocity.x * bulletSimulation.targetCrossTime,
					bullet.position.y + bullet.velocity.y * bulletSimulation.targetCrossTime);
				if (isBulletExplosionShootUnit(bullet.explosionParams, bulletCrossWallCenter, bullet.size / 2.0, unitInTimePosition, unit.size))
				{
					damage += bullet.explosionParams->damage;					
				}
			}

			if (damage > 0) thisTickShootMeBullets[bullet] = damage;
		}
	}

	for (const auto& mine:game.mines)
	{
		if(mine.state != TRIGGERED) continue;
		if (mine.timer == nullptr || *mine.timer > tickTime) continue;

		auto jumpState = unit.jumpState;
		const auto unitInTimePosition = Simulator::getUnitInTimePosition(
			unit.position, unit.size, unit.id, action, *mine.timer, jumpState, game);

		if (isMineExplosionShootUnit(mine.position, mine.size, mine.explosionParams.radius, 
			unitInTimePosition, unit.size, 0, 0))
		{
			minesDamage += mine.explosionParams.damage;
		}
	}
	
	return thisTickShootMeBullets;
}

bool Strategy::isBulletExplosionShootUnit(
	const std::shared_ptr<ExplosionParams>& explosionParams, 
	const Vec2Double& bulletCrossWallCenter,
	double halfBulletSize,
	const Vec2Double& unitPosition, const Vec2Double& unitSize)
{
	if (explosionParams == nullptr) return false;

	const auto radius = explosionParams->radius;

	/*const auto unitLeft = unitPosition.x - unitSize.x / 2;
	const auto unitRight = unitPosition.x + unitSize.x / 2;
	const auto unitBottom = unitPosition.y;
	const auto unitTop = unitPosition.y + unitSize.y;

	if (std::abs(bulletCrossWallCenter.x - unitLeft) <= radius + TOLERANCE && 
		std::abs(bulletCrossWallCenter.y - unitBottom) <= radius + TOLERANCE) return true;
	if (std::abs(bulletCrossWallCenter.x - unitLeft) <= radius + TOLERANCE && 
		std::abs(bulletCrossWallCenter.y - unitTop) <= radius + TOLERANCE) return true;
	if (std::abs(bulletCrossWallCenter.x - unitRight) <= radius + TOLERANCE && 
		std::abs(bulletCrossWallCenter.y - unitTop) <= radius + TOLERANCE) return true;
	if (std::abs(bulletCrossWallCenter.x - unitRight) <= radius + TOLERANCE && 
		std::abs(bulletCrossWallCenter.y - unitBottom) <= radius + TOLERANCE) return true;
	return false;*/

	double xDist;
	if (bulletCrossWallCenter.x - halfBulletSize > unitPosition.x + unitSize.x / 2) 
		xDist = bulletCrossWallCenter.x - halfBulletSize - (unitPosition.x + unitSize.x / 2);
	else if (bulletCrossWallCenter.x + halfBulletSize < unitPosition.x - unitSize.x / 2)
		xDist = unitPosition.x - unitSize.x / 2 - (bulletCrossWallCenter.x + halfBulletSize);
	else xDist = 0.0;

	double yDist;
	if (bulletCrossWallCenter.y - halfBulletSize > unitPosition.y + unitSize.y)
		yDist = bulletCrossWallCenter.y - halfBulletSize - (unitPosition.y + unitSize.y);
	else if (bulletCrossWallCenter.y + halfBulletSize < unitPosition.y) 
		yDist = unitPosition.y - (bulletCrossWallCenter.y + halfBulletSize);
	else yDist = 0.0;

	return xDist <= radius - TOLERANCE && yDist <= radius - TOLERANCE;
}

bool Strategy::isMineExplosionShootUnit(const Vec2Double& minePosition, const Vec2Double& mineSize,
	double mineExplosionRadius, const Vec2Double& unitPosition, const Vec2Double& unitSize,
	double xRunDist, double yRunDist)
{
	double xDist;
	if (minePosition.x > unitPosition.x + unitSize.x / 2) 
		xDist = minePosition.x - (unitPosition.x + unitSize.x / 2);
	else if (minePosition.x < unitPosition.x - unitSize.x / 2) 
		xDist = unitPosition.x - (minePosition.x + mineSize.x / 2);
	else xDist = 0.0;
	
	double yDist;
	if (minePosition.y + mineSize.y/2.0 > unitPosition.y + unitSize.y) 
		yDist = minePosition.y + mineSize.y / 2.0 - (unitPosition.y + unitSize.y);
	else if (minePosition.y + mineSize.y/2.0 < unitPosition.y) 
		yDist = unitPosition.y - (minePosition.y + mineSize.y/2.0);
	else yDist = 0.0;

	return xDist + xRunDist <= mineExplosionRadius - TOLERANCE && yDist + yRunDist <= mineExplosionRadius - TOLERANCE;
}


int Strategy::getRunawayDirection(int id) const
{
	return runaway_directions_.at(id);
}

int Strategy::getStopRunawayTick(int id) const
{
	return stop_runaway_ticks_.at(id);
}

void Strategy::setRunaway(int id, RunawayDirection runaway_direction, int srt)
{
	runaway_directions_[id] = runaway_direction;
	stop_runaway_ticks_[id] = srt;
}

void Strategy::decreaseStopRunawayTick(int id)
{
	if (stop_runaway_ticks_.at(id) >= 0) stop_runaway_ticks_[id]--;
}

size_t Strategy::getStartedJumpY(int id) const
{
	return startedJumpYs_.at(id);
}

void Strategy::setStartedJumpY(int id,size_t newStartedJumpY)
{
	startedJumpYs_[id] = newStartedJumpY;
}

bool Strategy::getIsMonkeyMode(int id) const
{
	return isMonkeyMode_.at(id);
}

void Strategy::setIsMonkeyMode(int id, bool isMonkeyMode)
{
	isMonkeyMode_[id] = isMonkeyMode;
}

int Strategy::getJumpingUnitId() const
{
	return jumpingUnitId_;
}

void Strategy::setJumpingUnitId(int id)
{
	jumpingUnitId_ = id;
}



