#pragma once
#include "../model/Vec2Double.hpp"

struct ShootMeBulletCrossPoint
{
	Vec2Double crossPoint;
	bool hasCrossPoint;
	double crossPointDist2;

	ShootMeBulletCrossPoint(
		Vec2Double cp, bool hcp, double cpDist2):crossPoint(cp),hasCrossPoint(hcp),crossPointDist2(cpDist2)
	{
		
	}
};
