#pragma once

#include "../model/Vec2Float.hpp"
#include "../model/Vec2Double.hpp"

class Bullet;
constexpr auto TOLERANCE = 1E-6;
constexpr auto TILE_SIZE = 1;
constexpr auto ANGLE_SPLIT_COUNT = 50;
constexpr auto SHOOTING_PROBABILITY = 0.33;
constexpr auto WALKING_PROBABILITY = 0.5;


Vec2Float vec2DoubleToVec2Float(const Vec2Double& vec2Double);
//inline bool operator<(const Bullet& lhs, const Bullet& rhs);