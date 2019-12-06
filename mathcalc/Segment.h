#pragma once
#include "../model/Vec2Double.hpp"

struct Segment
{
public:
	Vec2Double start;
	Vec2Double end;


	Segment(const Vec2Double& start, const Vec2Double& end)
		: start(start),
		  end(end)
	{
	}

	Segment() = default;
};
