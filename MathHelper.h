//#ifndef _MATHHELPER_HPP_
//#define _MATHHELPER_HPP_
#pragma once

#include "model/Vec2Double.hpp"
#include <utility>
#include <vector>

class MathHelper {
public:
	static Vec2Double getLinesCross(double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4);
	static std::vector<std::pair<int, int>>getLineSquares(const Vec2Double& start, const Vec2Double& end, int squareSide);
	static double getVectorLength(const Vec2Double& v);
	static double getVectorLength2(const Vec2Double& v);
};


//#endif