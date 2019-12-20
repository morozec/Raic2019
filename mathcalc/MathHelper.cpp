#include "MathHelper.h"
#include "../common/Helper.h"
#include <cmath>
#include <algorithm>

using namespace std;

std::vector<std::pair<int, int>> MathHelper::getLineSquares2(const Vec2Double& start, const Vec2Double& end)
{
	std::vector<std::pair<int, int>> result;

	if (std::abs(end.x - start.x) < 2 * TOLERANCE && std::abs(end.y - start.y) < 2 * TOLERANCE)
	{
		result.emplace_back(make_pair(static_cast<int>(start.x), static_cast<int>(start.y)));
		return result;
	}
	if (std::abs(end.y - start.y) < 2 * TOLERANCE)
	{
		const int y = static_cast<int>(start.y);
		const int stepX = end.x >= start.x ? 1 : -1;		
		int x = static_cast<int>(start.x + TOLERANCE * stepX);
		const int endX = static_cast<int>(end.x - TOLERANCE * stepX);
		result.emplace_back(make_pair(x, y));

		while (x != endX)
		{
			x += stepX;
			result.emplace_back(make_pair(x, y));
		}
		return result;
	}
	if (std::abs(end.x - start.x) < 2 * TOLERANCE)
	{
		const int x = static_cast<int>(start.x);
		const int stepY = end.y >= start.y ? 1 : -1;
		int y = static_cast<int>(start.y + TOLERANCE * stepY);
		const int endY = static_cast<int>(end.y - TOLERANCE * stepY);
		result.emplace_back(make_pair(x, y));
		
		while (y != endY)
		{
			y += stepY;
			result.emplace_back(make_pair(x, y));
		}
		return result;
	}
	
	
	const Vec2Double v = end - start;	

	const int stepX = end.x >= start.x ? 1 : -1;
	const int stepY = end.y >= start.y ? 1 : -1;

	////double intPart;
	//const auto roundStartX = round(start.x);
	//auto correctStartX = abs(start.x - roundStartX) <= TOLERANCE ? roundStartX : start.x;
	////correctStartX += TOLERANCE * stepX;
	//
	//const auto roundStartY = round(start.y);
	//auto correctStartY = abs(start.y - roundStartY) <= TOLERANCE ? roundStartY : start.y;
	////correctStartY += TOLERANCE * stepY;

	//const auto roundEndX = round(end.x);
	//auto correctEndX = abs(end.x - roundEndX) <= TOLERANCE ? roundEndX : end.x;
	////correctEndX -= TOLERANCE * stepX;

	//const auto roundEndY = round(end.y);
	//auto correctEndY = abs(end.y - roundEndY) <= TOLERANCE ? roundEndY : end.y;
	////correctEndY -= TOLERANCE * stepY;

	const auto correctStartX = start.x + 2*TOLERANCE * stepY;
	//if (std::modf(correctStartX, &intPart) == 0.0) correctStartX += TOLERANCE * stepX;
	const auto correctStartY = start.y + 2*TOLERANCE * stepY;
	//if (std::modf(correctStartY, &intPart) == 0.0) correctStartY += TOLERANCE * stepY;
	const auto correctEndX = end.x - 2*TOLERANCE * stepX;
	//if (std::modf(correctEndX, &intPart) == 0.0) correctEndX -= TOLERANCE * stepX;
	const auto correctEndY = end.y - 2*TOLERANCE * stepY;
	//if (std::modf(correctEndY, &intPart) == 0.0) correctEndY -= TOLERANCE * stepY;
	
	
	int x = static_cast<int>(correctStartX);
	int y = static_cast<int>(correctStartY);
	result.emplace_back(make_pair(x, y));

	const int endX = static_cast<int>(correctEndX);
	const int endY = static_cast<int>(correctEndY);

	

	const auto voxelBoundaryX = stepX > 0 ? trunc(correctStartX + stepX) : ceil(correctStartX + stepX);
	const auto voxelBoundaryY = stepY > 0 ? trunc(correctStartY + stepY) : ceil(correctStartY + stepY);

	auto tMaxX = (voxelBoundaryX - start.x) / v.x;
	auto tMaxY = (voxelBoundaryY - start.y) / v.y;

	const auto tDeltaX = stepX/v.x;
	const auto tDeltaY = stepY/v.y;

	while (x != endX || y != endY)
	{
		if (x != endX && y != endY)
		{
			if (tMaxX < tMaxY)
			{
				tMaxX += tDeltaX;
				x += stepX;
			}
			else
			{
				tMaxY += tDeltaY;
				y += stepY;
			}
		}
		else if (x != endX)
		{
			tMaxX += tDeltaX;
			x += stepX;
		}
		else// y != endY
		{
			tMaxY += tDeltaY;
			y += stepY;
		}
		result.emplace_back(make_pair(x, y));
	}

	return result;
}

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
//vector<pair<int, int>> MathHelper::getLineSquares(const Vec2Double& start, const Vec2Double& end, int squareSide)
//{
//	bool isInverse = false;
//	Vec2Double realStart, realEnd;
//
//	if (start.x <= end.x) {
//		isInverse = false;
//		realStart = start;
//		realEnd = end;
//	}
//	else {//invert anchor if start.X > end.X 
//		isInverse = true;
//		realStart = end;
//		realEnd = start;
//	}
//
//	//get step coeffs depending on start and end points position
//	auto xCoeff = realEnd.x > realStart.x ? 1 : -1;
//	auto yCoeff = realEnd.y > realStart.y ? 1 : -1;
//
//	//get left and top coords of containing start point square
//	auto startX = GetSquareIndex(realStart.x, squareSide) * squareSide;
//	auto startY = GetSquareIndex(realStart.y, squareSide) * squareSide;
//
//	//get left and top coords of containing end point square
//	auto endX = GetSquareIndex(realEnd.x, squareSide) * squareSide;
//	auto endY = GetSquareIndex(realEnd.y, squareSide) * squareSide;
//
//	//add containing start point square 
//	vector <pair<int, int>> result;
//	result.emplace_back(make_pair(GetSquareIndex(startX, squareSide), GetSquareIndex(startY, squareSide)));
//	
//	auto x = startX;
//	auto y = startY;
//
//	while (x != endX) //while anchor is not finished
//	{
//		auto newX = x + squareSide * xCoeff;//x coord of next x border
//		auto yReal = abs(end.x - start.x) < TOLERANCE ? start.y : start.y + (end.y - start.y) * (newX - start.x) / (end.x - start.x);//y coord of next x border 
//		auto newY = GetSquareIndex(yReal, squareSide) * squareSide;//top coord of containing yReal square 
//		while (y != newY) //look through current anchor [(x, y), (newX, newY)] 
//		{
//			y += squareSide * yCoeff;//y coord of next y border
//			result.emplace_back(make_pair(GetSquareIndex(x, squareSide), GetSquareIndex(y, squareSide)));//add all squares while y != newY  
//		}
//		result.emplace_back(make_pair(GetSquareIndex(newX, squareSide), GetSquareIndex(newY, squareSide)));//add final square for current anchor 
//		x = newX;
//	}
//
//	{//final step to add squares with x: endX - squareSide < x <= endX 
//		auto newY = GetSquareIndex(endY, squareSide) * squareSide;
//		while (y != newY)
//		{
//			y += squareSide * yCoeff;
//			result.emplace_back(make_pair(GetSquareIndex(x, squareSide), GetSquareIndex(y, squareSide)));
//		}
//	}
//
//	if (isInverse) {
//		reverse(result.begin(), result.end());
//	}
//
//	return result;
//}

double MathHelper::getVectorLength(double x0, double y0, double x1, double y1)
{
	return sqrt(getVectorLength2(x0, y0, x1, y1));
}

double MathHelper::getVectorLength(const Vec2Double & v)
{
	return sqrt(MathHelper::getVectorLength2(v));
}

double MathHelper::getVectorLength(const Vec2Double& start, const Vec2Double& end)
{
	return sqrt(getVectorLength2(start, end));
}

double MathHelper::getVectorLength2(double x0, double y0, double x1, double y1)
{
	return (x1 - x0)*(x1 - x0) + (y1 - y0)*(y1 - y0);
}

double MathHelper::getVectorLength2(const Vec2Double & v)
{
	return v.x * v.x + v.y * v.y;
}

double MathHelper::getVectorLength2(const Vec2Double& start, const Vec2Double& end)
{
	return getVectorLength2(end.x, end.y, start.x, start.y);
}

double MathHelper::getScalarMult(const Vec2Double & v1, const Vec2Double & v2)
{
	return v1.x * v2.x + v1.y * v2.y;
}

bool MathHelper::IsBetween(const Vec2Double& p0, const Vec2Double& p1, const Vec2Double& p2)
{
	return IsBetween(p0.x, p0.y, p1.x, p1.y, p2.x, p2.y);
}

bool MathHelper::IsBetween(double x0, double y0, double x1, double y1, double x2, double y2)
{
	return (x1 - x0) * (x2 - x0) <= TOLERANCE && (y1 - y0) * (y2 - y0) <= TOLERANCE;
}

double MathHelper::getMHDist(const Vec2Double& p0, const Vec2Double& p1)
{
	return std::abs(p1.x - p0.x) + std::abs(p1.y - p0.y);
}

