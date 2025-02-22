#pragma once

// Creating a shortcut for int, int pair type 
#include <utility>
#include <vector>
class Game;
typedef std::pair<int, int> Pair;
typedef std::tuple<int, int, int, int> Four;

// A structure to hold the neccesary parameters 
struct cell
{
	// Row and Column index of its parent 
	// Note that 0 <= i <= ROW-1 & 0 <= j <= COL-1 
	int parent_i, parent_j, parent_k, parent_l;
	// f = g + h 
	double f, g, h;
};

std::vector<Four> aStarSearch(
	int**& grid, int COL, int ROW,
	bool****& closedList,
	cell****& cellDetails,
	const Four& src, const Pair& dest,
	int maxJumpTiles, int maxJumpPadJumpTiles,
	const std::vector<std::pair<int, int>>& unitWalls,
	const Game& game);