#include "MathHelper.h"
#include "Helper.h"
#include <math.h>
#include <algorithm>

using namespace std;

Vec2Double MathHelper::getLinesCross(double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4) {
	auto ua = ((x4 - x3)*(y1 - y3) - (y4 - y3)*(x1 - x3)) / ((y4 - y3)*(x2 - x1) - (x4 - x3)*(y2 - y1));
	auto ub = ((x2 - x1)*(y1 - y3) - (y2 - y1)*(x1 - x3)) / ((y4 - y3)*(x2 - x1) - (x4 - x3)*(y2 - y1));
	auto x = x1 + ua * (x2 - x1);
	auto y = y1 + ua * (y2 - y1);
	return Vec2Double(x, y);
}

bool MathHelper::areSegmentsCross(
	const Segment& s1, const Segment& s2)
{
	const auto lineCross = MathHelper::getLinesCross(s1.start.x, s1.start.y, s1.end.x, s1.end.y, 
		s2.start.x, s2.start.y, s2.end.x, s2.end.y);
	return IsBetween(lineCross, s1.start, s1.end) && IsBetween(lineCross, s2.start, s2.end);
}


/// <summary>
/// Get containg value square index
/// </summary>
/// <param name="value">value</param>
/// <param name="squareSide">sqaure side</param>
/// <returns></returns>
int GetSquareIndex(double value, int squareSide)
{
	return (int)floor(value / squareSide);
}


/// <summary>
/// Main method to get all anchor crossing squares
/// </summary>
/// <param name="start">Start anchor point</param>
/// <param name="end">End anchor point</param>
/// <param name="squareSide">square side</param>
/// <returns></returns>
vector<pair<int, int>> MathHelper::getLineSquares(const Vec2Double& start, const Vec2Double& end, int squareSide)
{
	bool isInverse = false;
	Vec2Double realStart, realEnd;

	if (start.x <= end.x) {
		isInverse = false;
		realStart = start;
		realEnd = end;
	}
	else {//invert anchor if start.X > end.X 
		isInverse = true;
		realStart = end;
		realEnd = start;
	}

	//get step coeffs depending on start and end points position
	auto xCoeff = realEnd.x > realStart.x ? 1 : -1;
	auto yCoeff = realEnd.y > realStart.y ? 1 : -1;

	//get left and top coords of containing start point square
	auto startX = GetSquareIndex(realStart.x, squareSide) * squareSide;
	auto startY = GetSquareIndex(realStart.y, squareSide) * squareSide;

	//get left and top coords of containing end point square
	auto endX = GetSquareIndex(realEnd.x, squareSide) * squareSide;
	auto endY = GetSquareIndex(realEnd.y, squareSide) * squareSide;

	//add containing start point square 
	vector <pair<int, int>> result;
	result.emplace_back(make_pair(GetSquareIndex(startX, squareSide), GetSquareIndex(startY, squareSide)));
	
	auto x = startX;
	auto y = startY;

	while (x != endX) //while anchor is not finished
	{
		auto newX = x + squareSide * xCoeff;//x coord of next x border
		auto yReal = abs(end.x - start.x) < TOLERACNE ? start.y : start.y + (end.y - start.y) * (newX - start.x) / (end.x - start.x);//y coord of next x border 
		auto newY = GetSquareIndex(yReal, squareSide) * squareSide;//top coord of containing yReal square 
		while (y != newY) //look through current anchor [(x, y), (newX, newY)] 
		{
			y += squareSide * yCoeff;//y coord of next y border
			result.emplace_back(make_pair(GetSquareIndex(x, squareSide), GetSquareIndex(y, squareSide)));//add all squares while y != newY  
		}
		result.emplace_back(make_pair(GetSquareIndex(newX, squareSide), GetSquareIndex(newY, squareSide)));//add final square for current anchor 
		x = newX;
	}

	{//final step to add squares with x: endX - squareSide < x <= endX 
		auto newY = GetSquareIndex(endY, squareSide) * squareSide;
		while (y != newY)
		{
			y += squareSide * yCoeff;
			result.emplace_back(make_pair(GetSquareIndex(x, squareSide), GetSquareIndex(y, squareSide)));
		}
	}

	if (isInverse) {
		reverse(result.begin(), result.end());
	}

	return result;
}

double MathHelper::getVectorLength(const Vec2Double & v)
{
	return sqrt(MathHelper::getVectorLength2(v));
}

double MathHelper::getVectorLength2(const Vec2Double & v)
{
	return v.x * v.x + v.y * v.y;
}

double MathHelper::getScalarMult(const Vec2Double & v1, const Vec2Double & v2)
{
	return v1.x * v2.x + v1.y * v2.y;
}

bool MathHelper::IsBetween(const Vec2Double& p0, const Vec2Double& p1, const Vec2Double& p2)
{
	return (p1.x - p0.x) * (p2.x - p0.x) <= 0 && (p1.y - p0.y) * (p2.y - p0.y) <= 0;
}
