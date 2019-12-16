//#ifndef _MATHHELPER_HPP_
//#define _MATHHELPER_HPP_
#pragma once

#include "Segment.h"
#include <utility>
#include <vector>

class MathHelper {
public:
	static std::vector<std::pair<int, int>> getLineSquares2(const Vec2Double& start, const Vec2Double& end);
	
	static Vec2Double getLinesCross(double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4);
	static bool areSegmentsCross(const Segment& s1, const Segment& s2);
	//static std::vector<std::pair<int, int>>getLineSquares(const Vec2Double& start, const Vec2Double& end, int squareSide);
	
	static double getVectorLength(double x0, double y0, double x1, double y1);
	static double getVectorLength(const Vec2Double& v);
	static double getVectorLength(const Vec2Double& start, const Vec2Double& end);
	
	static double getVectorLength2(double x0, double y0, double x1, double y1);
	static double getVectorLength2(const Vec2Double& v);
	static double getVectorLength2(const Vec2Double& start, const Vec2Double& end);
	
	static double getScalarMult(const Vec2Double& v1, const Vec2Double& v2);
	static bool IsBetween(const Vec2Double& p0, const Vec2Double& p1, const Vec2Double& p2);
	static bool IsBetween(double x0, double y0, double x1, double y1, double x2, double y2);

	static double getMHDist(const Vec2Double& p0, const Vec2Double& p1);
};


//#endif