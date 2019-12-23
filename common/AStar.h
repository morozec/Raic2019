#pragma once

// Creating a shortcut for int, int pair type 
#include <utility>
#include <vector>
typedef std::pair<int, int> Pair;

void aStarSearch(std::vector<std::vector<int>> grid, Pair src, Pair dest);