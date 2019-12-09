#pragma once
#include "../model/Bullet.hpp"
#include <map>

struct BulletSimulation
{
	//Bullet bullet;
	Vec2Double targetCrossPoint;
	Vec2Double bulletCrossCorner;
	double targetCrossTime;

	std::map<int, Vec2Double> bulletPositions;

	BulletSimulation(): targetCrossTime(0)
	{
	}

	BulletSimulation(const Vec2Double& target_cross_point, const Vec2Double& bullet_cross_corner,
		double target_cross_time)
		: targetCrossPoint(target_cross_point),
		  bulletCrossCorner(bullet_cross_corner),
		  targetCrossTime(target_cross_time)
	{
	}
};
