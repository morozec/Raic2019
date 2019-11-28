#include "MathHelper.h"

Vec2Double MathHelper::getLinesCross(double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4) {
	auto ua = ((x4 - x3)*(y1 - y3) - (y4 - y3)*(x1 - x3)) / ((y4 - y3)*(x2 - x1) - (x4 - x3)*(y2 - y1));
	auto ub = ((x2 - x1)*(y1 - y3) - (y2 - y1)*(x1 - x3)) / ((y4 - y3)*(x2 - x1) - (x4 - x3)*(y2 - y1));
	auto x = x1 + ua * (x2 - x1);
	auto y = y1 + ua * (y2 - y1);
	return Vec2Double(x, y);
}