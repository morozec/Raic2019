#pragma once

#include "../model/Vec2Float.hpp"
#include "../model/Vec2Double.hpp"

constexpr auto TOLERANCE = 1E-6;
constexpr auto TILE_SIZE = 1;
constexpr auto ANGLE_SPLIT_COUNT = 10;
constexpr auto WALKING_PROBABILITY = 0.5;
constexpr auto SAFE_SHOOTING_DIST = 7.0;
constexpr auto SAFE_SHOOTING_PROBABILITY = 0.33;

constexpr auto MONKEY_FIRE_TICK = 10;
constexpr auto MONKEY_DIST = 5.0;

const double OK_SHOOTING_PROBABILITY = 0.85;
const double NOT_BAD_SHOOTING_PROBABILITY = 0.5;
const int MAX_SIMULATIONS = 100;
const double SAFE_ROCKET_PROBABILITY = 0.1;
const double SAFE_ROCKET_DISTANCE1 = 1.0;
const double SAFE_ROCKET_DISTANCE2 = 1.0;

const auto MIN_SET_MINE_DIST = 7.0;

const auto SAFE_OTHER_UNIT_PLANT_MINE_DIST = 5.0;

const auto SAFE_ATTACK_DIST = 7.0;
const auto SAFE_DIST_TO_BORDER = 5.0;

Vec2Float vec2DoubleToVec2Float(const Vec2Double& vec2Double);
//inline bool operator<(const Bullet& lhs, const Bullet& rhs);