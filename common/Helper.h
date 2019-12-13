#pragma once

#include "../model/Vec2Float.hpp"
#include "../model/Vec2Double.hpp"

class Bullet;
constexpr auto TOLERANCE = 1E-6;
constexpr auto TILE_SIZE = 1;
constexpr auto ANGLE_SPLIT_COUNT = 10;
constexpr auto SHOOTING_PROBABILITY = 0.33;
constexpr auto WALKING_PROBABILITY = 0.5;

const double OK_SHOOTING_PROBABILITY = 0.85;
const int MAX_SIMULATIONS = 100;

Vec2Float vec2DoubleToVec2Float(const Vec2Double& vec2Double);
//inline bool operator<(const Bullet& lhs, const Bullet& rhs);