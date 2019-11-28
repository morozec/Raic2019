#pragma once

#include "model/Vec2Float.hpp"
#include "model/Vec2Double.hpp"

constexpr auto TOLERACNE = 0.001;
constexpr auto TILE_SIZE = 1;

Vec2Float vec2DoubleToVec2Float(const Vec2Double& vec2Double);
