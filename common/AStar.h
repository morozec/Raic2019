#pragma once

// Creating a shortcut for int, int pair type 
#include <utility>
#include <vector>
#include <stack>
class Game;
typedef std::pair<int, int> Pair;
typedef std::tuple<int, int, int> Triple;

std::stack<Pair> aStarSearch(std::vector<std::vector<int>> grid, Triple src, Pair dest,
	const Game& game);