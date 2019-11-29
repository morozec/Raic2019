#pragma once
#include "model/Vec2Double.hpp"

struct ShootMeBulletCrossPoint
{
	Vec2Double crossPoint;
	bool hasCrossPoint;
	bool hasWallBefore;
	double crossPointDist2;

	ShootMeBulletCrossPoint(
		Vec2Double cp, bool hcp, bool hwb, double cpDist2):crossPoint(cp),hasCrossPoint(hcp),hasWallBefore(hwb),crossPointDist2(cpDist2)
	{
		
	}
};