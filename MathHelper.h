//#ifndef _MATHHELPER_HPP_
//#define _MATHHELPER_HPP_
#pragma once

#include "model/Vec2Double.hpp"

class MathHelper {
public:
	static Vec2Double getLinesCross(double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4);
};

//#endif