#pragma once

// Creating a shortcut for int, int pair type 
#include <utility>
#include <vector>
class Game;
typedef std::pair<int, int> Pair;
typedef std::tuple<int, int, int, int> Four;

std::vector<Pair>  aStarSearch(std::vector<std::vector<int>> grid, Four src, Pair dest,
	int maxJumpTicks,
	const Game& game);