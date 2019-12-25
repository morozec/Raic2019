#define _USE_MATH_DEFINES

#include "AStar.h"

// A C++ Program to implement A* Search Algorithm 
#include <utility>
#include <cstdio>
#include <stack>
#include <set>
#include <vector>
#include <stdexcept>
#include <tuple>
#include <climits>
#include <algorithm>
#include "../model/Game.hpp"
using namespace std;

//#define ROW 9 
//#define COL 10 



// Creating a shortcut for pair<int, pair<int, int>> type 
typedef pair<double, Four> pPair;



// A Utility Function to check whether given cell (row, col) 
// is a valid cell or not. 
bool isValid(int col, int row, int COL, int ROW)
{
	// Returns true if row number and column number 
	// is in range 
	return (row >= 0) && (row < ROW) &&
		(col >= 0) && (col < COL);
}

// A Utility Function to check whether the given cell is 
// blocked or not 
bool isUnBlocked(vector<vector<int>> grid, int col, int row)
{
	// Returns true if the cell is not blocked else false 
	if (grid[col][row] == 1)
		return (true);
	else
		return (false);
}

// A Utility Function to check whether destination cell has 
// been reached or not 
bool isDestination(int col, int row, Pair dest)
{
	if (col == dest.first && row == dest.second)
		return (true);
	else
		return (false);
}

// A Utility Function to calculate the 'h' heuristics. 
int calculateHValue(int col, int row, Pair dest)
{
	// Return using the distance formula 
	return std::abs(col - dest.first) + std::abs(row - dest.second);
}

// A Utility Function to trace the path from the source 
// to destination 
vector<Pair> tracePath(vector<vector<vector<vector<cell>>>> cellDetails, Pair dest, int foundDestK, int foundDestL)
{
	int col = dest.first;
	int row = dest.second;
	int k = foundDestK;
	int l = foundDestL;

	vector<Pair> Path;

	while (!(cellDetails[col][row][k][l].parent_i == col
		&& cellDetails[col][row][k][l].parent_j == row))
	{
		Path.emplace_back(make_pair(col, row));
		int temp_col = cellDetails[col][row][k][l].parent_i;
		int temp_row = cellDetails[col][row][k][l].parent_j;
		int temp_k = cellDetails[col][row][k][l].parent_k;
		int temp_l = cellDetails[col][row][k][l].parent_l;
		col = temp_col;
		row = temp_row;		
		k = temp_k;
		l = temp_l;
	}

	Path.emplace_back(make_pair(col, row));
	std::reverse(Path.begin(), Path.end());

	return Path;
}

// A Function to find the shortest path between 
// a given source cell to a destination cell according 
// to A* Search Algorithm 
vector<Pair> aStarSearch(
	const std::vector<std::vector<int>>& grid, vector<vector<vector<vector<bool>>>>& closedList,
	vector<vector<vector<vector<cell>>>>& cellDetails,
	const Four& src, const Pair& dest,
	int maxJumpTiles, int maxJumpPadJumpTiles,
	const Game& game)
{
	// z-index:
	// -1 - falling
	//  0 - standing
	// >0 - jumping
	
	const auto ROW = grid.size();
	const auto COL = grid[0].size();
	const auto Z_SIZE = maxJumpPadJumpTiles + 2; //+1 - Ì‡ Ô‡‰ÂÌËÂ, +1 - Ì‡ ÒÚÓˇÌËÂ
	const auto PAD_JUMP_STATE_SIZE = 2;

	const auto start_x = get<0>(src);
	const auto start_y = get<1>(src);
	const auto start_z = get<2>(src);
	const auto start_pad_jump_state = get<3>(src);
	
	// If the source is out of range 
	if (isValid(start_x, start_y, ROW, COL) == false)
	{
		throw runtime_error("Source is invalid\n");
	}

	// If the destination is out of range 
	if (isValid(dest.first, dest.second, ROW, COL) == false)
	{
		throw runtime_error("Destination is invalid\n");
	}

	// Either the source or the destination is blocked 
	if (isUnBlocked(grid, start_x, start_y) == false ||
		isUnBlocked(grid, dest.first, dest.second) == false)
	{
		throw runtime_error("Source or the destination is blocked\n");
	}

	// If the destination cell is the same as source cell 
	if (isDestination(start_x, start_y, dest) == true)
	{
		return vector<Pair>(0);
	}	

	int i, j, k, l;

	for (i = 0; i < ROW; i++)
	{
		for (j = 0; j < COL; j++)
		{
			for (k = 0; k < Z_SIZE; ++k) {
				for (l = 0; l < PAD_JUMP_STATE_SIZE; ++l) {
					cellDetails[i][j][k][l].f = INT_MAX;
					cellDetails[i][j][k][l].g = INT_MAX;
					cellDetails[i][j][k][l].h = INT_MAX;
					cellDetails[i][j][k][l].parent_i = -1;
					cellDetails[i][j][k][l].parent_j = -1;
					cellDetails[i][j][k][l].parent_k = -1;
					cellDetails[i][j][k][l].parent_l = -1;

					closedList[i][j][k][l] = false;
				}
			}
		}
	}

	// Initialising the parameters of the starting node 
	i = start_x, j = start_y; k = start_z; l = start_pad_jump_state;
	cellDetails[i][j][k][l].f = 0.0;
	cellDetails[i][j][k][l].g = 0.0;
	cellDetails[i][j][k][l].h = 0.0;
	cellDetails[i][j][k][l].parent_i = i;
	cellDetails[i][j][k][l].parent_j = j;
	cellDetails[i][j][k][l].parent_k = k;
	cellDetails[i][j][k][l].parent_l = l;

	/*
	 Create an open list having information as-
	 <f, <i, j>>
	 where f = g + h,
	 and i, j are the row and column index of that cell
	 Note that 0 <= i <= ROW-1 & 0 <= j <= COL-1
	 This open list is implenented as a set of pair of pair.*/
	set<pPair> openList;
	const auto diagonalAddWeight = 0.001;

	// Put the starting cell on the open list and set its 
	// 'f' as 0
	
	openList.insert(make_pair(0.0, 
		make_tuple(i, j, start_z, start_pad_jump_state)));

	// We set this boolean value as false as initially 
	// the destination is not reached. 
	bool foundDest = false;
	int foundDestK = -1;
	int foundDestL = -1;

	while (!openList.empty())
	{
		pPair p = *openList.begin();

		// Remove this vertex from the open list 
		openList.erase(openList.begin());

		// Add this vertex to the closed list 
		i = get<0>(p.second);
		j = get<1>(p.second);
		k = get<2>(p.second);
		l = get<3>(p.second);
		const auto kValue = k == maxJumpPadJumpTiles + 1 ? -1 : k;
		
		closedList[i][j][k][l] = true;

		/*
		 Generating all the 8 successor of this cell

			 N.W   N   N.E
			   \   |   /
				\  |  /
			 W----Cell----E
				  / | \
				/   |  \
			 S.W    S   S.E

		 Cell-->Popped Cell (i, j)
		 N -->  North       (i-1, j)
		 S -->  South       (i+1, j)
		 E -->  East        (i, j+1)
		 W -->  West           (i, j-1)
		 N.E--> North-East  (i-1, j+1)
		 N.W--> North-West  (i-1, j-1)
		 S.E--> South-East  (i+1, j+1)
		 S.W--> South-West  (i+1, j-1)*/

		 // To store the 'g', 'h' and 'f' of the 8 successors 
		double gNew, hNew, fNew;
		int kNew, lNew;
		int jpJ;

		//----------- 1st Successor (WEST) ------------ 

		// Only process this cell if this is a valid one 
		if (isValid(i - 1, j, ROW, COL) == true &&
			isUnBlocked(grid, i - 1, j) == true &&
			(l ==0 || l == 1 && k == maxJumpPadJumpTiles) &&
			game.level.tiles[i - 1][j - 1] != EMPTY &&
			game.level.tiles[i - 1][j - 1] != JUMP_PAD)
		{
			kNew = 0;
			if (game.level.tiles[i - 1][j] == JUMP_PAD)
			{
				lNew = 1;
				jpJ = j + 1;
			}
			else
			{
				lNew = 0;
				jpJ = j;
			}
			// If the destination cell is the same as the 
			// current successor 
			if (isDestination(i - 1, jpJ, dest) == true)
			{
				// Set the Parent of the destination cell 
				cellDetails[i - 1][jpJ][kNew][lNew].parent_i = i;
				cellDetails[i - 1][jpJ][kNew][lNew].parent_j = j;
				cellDetails[i - 1][jpJ][kNew][lNew].parent_k = k;
				cellDetails[i - 1][jpJ][kNew][lNew].parent_l = l;
				foundDest = true;
				foundDestK = kNew;
				foundDestL = lNew;
				break;
			}
			// If the successor is already on the closed 
			// list or if it is blocked, then ignore it. 
			// Else do the following 
			else if (closedList[i - 1][jpJ][kNew][lNew] == false)
			{
				gNew = cellDetails[i][j][k][l].g + 1.0;
				hNew = calculateHValue(i - 1, jpJ, dest);
				fNew = gNew + hNew;

				// If it isnít on the open list, add it to 
				// the open list. Make the current square 
				// the parent of this square. Record the 
				// f, g, and h costs of the square cell 
				//                OR 
				// If it is on the open list already, check 
				// to see if this path to that square is better, 
				// using 'f' cost as the measure. 
				if (cellDetails[i - 1][jpJ][kNew][lNew].f == INT_MAX ||
					cellDetails[i - 1][jpJ][kNew][lNew].f > fNew)
				{
					openList.insert(make_pair(fNew,
						make_tuple(i - 1, jpJ, kNew, lNew)));

					// Update the details of this cell 
					cellDetails[i - 1][jpJ][kNew][lNew].f = fNew;
					cellDetails[i - 1][jpJ][kNew][lNew].g = gNew;
					cellDetails[i - 1][jpJ][kNew][lNew].h = hNew;
					cellDetails[i - 1][jpJ][kNew][lNew].parent_i = i;
					cellDetails[i - 1][jpJ][kNew][lNew].parent_j = j;
					cellDetails[i - 1][jpJ][kNew][lNew].parent_k = k;
					cellDetails[i - 1][jpJ][kNew][lNew].parent_l = l;
				}
			}
		}

		//----------- 2nd Successor (EAST) ------------ 

		// Only process this cell if this is a valid one 
		if (isValid(i + 1, j, ROW, COL) == true &&
			isUnBlocked(grid, i + 1, j) == true &&
			(l ==0 || l == 1 && k == maxJumpPadJumpTiles) &&
			game.level.tiles[i + 1][j - 1] != EMPTY &&
			game.level.tiles[i + 1][j - 1] != JUMP_PAD)
		{
			kNew = 0;
			if (game.level.tiles[i + 1][j] == JUMP_PAD)
			{
				lNew = 1;
				jpJ = j + 1;
			}
			else
			{
				lNew = 0;
				jpJ = j;
			}
			// If the destination cell is the same as the 
			// current successor 
			if (isDestination(i + 1, jpJ, dest) == true)
			{
				// Set the Parent of the destination cell 
				cellDetails[i + 1][jpJ][kNew][lNew].parent_i = i;
				cellDetails[i + 1][jpJ][kNew][lNew].parent_j = j;
				cellDetails[i + 1][jpJ][kNew][lNew].parent_k = k;
				cellDetails[i + 1][jpJ][kNew][lNew].parent_l = l;
				foundDest = true;
				foundDestK = kNew;
				foundDestL = lNew;
				break;
			}
			// If the successor is already on the closed 
			// list or if it is blocked, then ignore it. 
			// Else do the following 
			else if (closedList[i + 1][jpJ][kNew][lNew] == false )
			{
				gNew = cellDetails[i][j][k][l].g + 1.0;
				hNew = calculateHValue(i + 1, jpJ, dest);
				fNew = gNew + hNew;

				// If it isnít on the open list, add it to 
				// the open list. Make the current square 
				// the parent of this square. Record the 
				// f, g, and h costs of the square cell 
				//                OR 
				// If it is on the open list already, check 
				// to see if this path to that square is better, 
				// using 'f' cost as the measure. 
				if (cellDetails[i + 1][jpJ][kNew][lNew].f == INT_MAX ||
					cellDetails[i + 1][jpJ][kNew][lNew].f > fNew)
				{
					openList.insert(make_pair(fNew, make_tuple(i + 1, jpJ, kNew, lNew)));
					// Update the details of this cell 
					cellDetails[i + 1][jpJ][kNew][lNew].f = fNew;
					cellDetails[i + 1][jpJ][kNew][lNew].g = gNew;
					cellDetails[i + 1][jpJ][kNew][lNew].h = hNew;
					cellDetails[i + 1][jpJ][kNew][lNew].parent_i = i;
					cellDetails[i + 1][jpJ][kNew][lNew].parent_j = j;
					cellDetails[i + 1][jpJ][kNew][lNew].parent_k = k;
					cellDetails[i + 1][jpJ][kNew][lNew].parent_l = l;
				}
			}
		}

		const auto addNorth = l == 0 || l == 1 && kValue == maxJumpPadJumpTiles - 1 ? 1 : 2;

		//----------- 3.0 Successor (NORTH-2) ------------ 

		// Only process this cell if this is a valid one 
		if (isValid(i, j + addNorth, ROW, COL) == true &&
			kValue >= 0 &&
			(l == 0 && kValue < maxJumpTiles ||
				l == 1 && kValue < maxJumpPadJumpTiles) &&
			isUnBlocked(grid, i, j + addNorth) == true)
		{
			kNew = k+addNorth;
			lNew = l;
			// If the destination cell is the same as the 
			// current successor 
			if (isDestination(i, j + addNorth, dest) == true)
			{
				// Set the Parent of the destination cell 
				cellDetails[i][j + addNorth][kNew][lNew].parent_i = i;
				cellDetails[i][j + addNorth][kNew][lNew].parent_j = j;
				cellDetails[i][j + addNorth][kNew][lNew].parent_k = k;
				cellDetails[i][j + addNorth][kNew][lNew].parent_l = l;
				foundDest = true;
				foundDestK = kNew;
				foundDestL = lNew;
				break;
			}

			// If the successor is already on the closed 
			// list or if it is blocked, then ignore it. 
			// Else do the following 
			else if (closedList[i][j + addNorth][kNew][lNew] == false)
			{
				gNew = cellDetails[i][j][k][l].g + 1.0;
				hNew = calculateHValue(i, j + addNorth, dest);
				fNew = gNew + hNew;

				// If it isnít on the open list, add it to 
				// the open list. Make the current square 
				// the parent of this square. Record the 
				// f, g, and h costs of the square cell 
				//                OR 
				// If it is on the open list already, check 
				// to see if this path to that square is better, 
				// using 'f' cost as the measure. 
				if (cellDetails[i][j + addNorth][kNew][lNew].f == INT_MAX ||
					cellDetails[i][j + addNorth][kNew][lNew].f > fNew)
				{
					openList.insert(make_pair(fNew,
						make_tuple(i, j + addNorth, kNew, lNew)));

					// Update the details of this cell 
					cellDetails[i][j + addNorth][kNew][lNew].f = fNew;
					cellDetails[i][j + addNorth][kNew][lNew].g = gNew;
					cellDetails[i][j + addNorth][kNew][lNew].h = hNew;
					cellDetails[i][j + addNorth][kNew][lNew].parent_i = i;
					cellDetails[i][j + addNorth][kNew][lNew].parent_j = j;
					cellDetails[i][j + addNorth][kNew][lNew].parent_k = k;
					cellDetails[i][j + addNorth][kNew][lNew].parent_l = l;
				}
			}
		}

		//----------- 3rd Successor (NORTH-0) ------------ 

		// Only process this cell if this is a valid one 
		if (isValid(i, j + addNorth, ROW, COL) == true &&
			(l == 0 && kValue >=0 && kValue < maxJumpTiles ||
				l == 1 && kValue == maxJumpPadJumpTiles - addNorth) &&
			isUnBlocked(grid, i, j + addNorth) == true &&
			(game.level.tiles[i][j + addNorth - 1] == PLATFORM || game.level.tiles[i][j + addNorth - 1] == LADDER))
		{
			kNew = 0;
			if (game.level.tiles[i][j + addNorth] == JUMP_PAD)
			{
				lNew = 1;
				jpJ = j + addNorth + 1;
			}
			else
			{
				lNew = 0;
				jpJ = j + addNorth;
			}
			// If the destination cell is the same as the 
			// current successor 
			if (isDestination(i, jpJ, dest) == true)
			{
				// Set the Parent of the destination cell 
				cellDetails[i][jpJ][kNew][lNew].parent_i = i;
				cellDetails[i][jpJ][kNew][lNew].parent_j = j;
				cellDetails[i][jpJ][kNew][lNew].parent_k = k;
				cellDetails[i][jpJ][kNew][lNew].parent_l = l;
				foundDest = true;
				foundDestK = kNew;
				foundDestL = lNew;
				break;
			}

			// If the successor is already on the closed 
			// list or if it is blocked, then ignore it. 
			// Else do the following 
			else if (closedList[i][jpJ][kNew][lNew] == false)
			{
				gNew = cellDetails[i][j][k][l].g + 1.0;
				hNew = calculateHValue(i, jpJ, dest);
				fNew = gNew + hNew;

				// If it isnít on the open list, add it to 
				// the open list. Make the current square 
				// the parent of this square. Record the 
				// f, g, and h costs of the square cell 
				//                OR 
				// If it is on the open list already, check 
				// to see if this path to that square is better, 
				// using 'f' cost as the measure. 
				if (cellDetails[i][jpJ][kNew][lNew].f == INT_MAX ||
					cellDetails[i][jpJ][kNew][lNew].f > fNew)
				{
					openList.insert(make_pair(fNew,
						make_tuple(i, jpJ, kNew, lNew)));

					// Update the details of this cell 
					cellDetails[i][jpJ][kNew][lNew].f = fNew;
					cellDetails[i][jpJ][kNew][lNew].g = gNew;
					cellDetails[i][jpJ][kNew][lNew].h = hNew;
					cellDetails[i][jpJ][kNew][lNew].parent_i = i;
					cellDetails[i][jpJ][kNew][lNew].parent_j = j;
					cellDetails[i][jpJ][kNew][lNew].parent_k = k;
					cellDetails[i][jpJ][kNew][lNew].parent_l = l;
				}
			}
		}

		//TODO: NORTH-1
				

		//----------- 7th Successor (NORTH-WEST-2) ------------ 

	// Only process this cell if this is a valid one 
		if (isValid(i - 1, j + addNorth, ROW, COL) == true &&
			kValue >= 0 && 
			(l ==0 && kValue < maxJumpTiles ||
				l == 1 && kValue < maxJumpPadJumpTiles) &&
			isUnBlocked(grid, i - 1, j + addNorth) == true &&
			isUnBlocked(grid, i, j + addNorth) == true)
		{						
			if (game.level.tiles[i - 1][j + addNorth - 1] == JUMP_PAD)
			{
				kNew = 0;
				lNew = 1;
			}
			else
			{
				kNew = k + addNorth;
				lNew = l;
			}
			
			// If the destination cell is the same as the 
			// current successor 
			if (isDestination(i - 1, j + addNorth, dest) == true)
			{
				// Set the Parent of the destination cell 
				cellDetails[i - 1][j + addNorth][kNew][lNew].parent_i = i;
				cellDetails[i - 1][j + addNorth][kNew][lNew].parent_j = j;
				cellDetails[i - 1][j + addNorth][kNew][lNew].parent_k = k;
				cellDetails[i - 1][j + addNorth][kNew][lNew].parent_l = l;
				foundDest = true;
				foundDestK = kNew;
				foundDestL = lNew;
				break;
			}

			// If the successor is already on the closed 
			// list or if it is blocked, then ignore it. 
			// Else do the following 
			else if (closedList[i - 1][j + addNorth][kNew][lNew] == false &&
				isUnBlocked(grid, i - 1, j + addNorth) == true)
			{
				gNew = cellDetails[i][j][k][l].g + 1.0 + diagonalAddWeight;
				hNew = calculateHValue(i - 1, j + addNorth, dest);
				fNew = gNew + hNew;

				// If it isnít on the open list, add it to 
				// the open list. Make the current square 
				// the parent of this square. Record the 
				// f, g, and h costs of the square cell 
				//                OR 
				// If it is on the open list already, check 
				// to see if this path to that square is better, 
				// using 'f' cost as the measure. 
				if (cellDetails[i - 1][j + addNorth][kNew][lNew].f == INT_MAX ||
					cellDetails[i - 1][j + addNorth][kNew][lNew].f > fNew)
				{
					openList.insert(make_pair(fNew,
						make_tuple(i - 1, j + addNorth, kNew, lNew)));

					// Update the details of this cell 
					cellDetails[i - 1][j + addNorth][kNew][lNew].f = fNew;
					cellDetails[i - 1][j + addNorth][kNew][lNew].g = gNew;
					cellDetails[i - 1][j + addNorth][kNew][lNew].h = hNew;
					cellDetails[i - 1][j + addNorth][kNew][lNew].parent_i = i;
					cellDetails[i - 1][j + addNorth][kNew][lNew].parent_j = j;
					cellDetails[i - 1][j + addNorth][kNew][lNew].parent_k = k;
					cellDetails[i - 1][j + addNorth][kNew][lNew].parent_l = l;
				}
			}
		}

		//----------- 7th Successor (NORTH-WEST-0) ------------ 

	// Only process this cell if this is a valid one 
		if (isValid(i - 1, j + addNorth, ROW, COL) == true &&
			(l == 0 && kValue >= 0 && kValue < maxJumpTiles ||
			l == 1 && kValue == maxJumpPadJumpTiles - addNorth) &&
			isUnBlocked(grid, i - 1, j + addNorth) == true &&
			isUnBlocked(grid, i, j + addNorth) == true &&
			(game.level.tiles[i - 1][j + addNorth - 1] == PLATFORM || game.level.tiles[i - 1][j + addNorth - 1] == LADDER || game.level.tiles[i - 1][j + addNorth - 1] == WALL))
		{
			kNew = 0;
			if (game.level.tiles[i - 1][j + addNorth] == JUMP_PAD)
			{
				lNew = 1;
				jpJ = j + addNorth + 1;
			}
			else
			{
				lNew = 0;
				jpJ = j + addNorth;
			}
			// If the destination cell is the same as the 
			// current successor 
			if (isDestination(i - 1, jpJ, dest) == true)
			{
				// Set the Parent of the destination cell 
				cellDetails[i - 1][jpJ][kNew][lNew].parent_i = i;
				cellDetails[i - 1][jpJ][kNew][lNew].parent_j = j;
				cellDetails[i - 1][jpJ][kNew][lNew].parent_k = k;
				cellDetails[i - 1][jpJ][kNew][lNew].parent_l = l;
				foundDest = true;
				foundDestK = kNew;
				foundDestL = lNew;
				break;
			}

			// If the successor is already on the closed 
			// list or if it is blocked, then ignore it. 
			// Else do the following 
			else if (closedList[i - 1][jpJ][kNew][lNew] == false &&
				isUnBlocked(grid, i - 1, jpJ) == true)
			{
				gNew = cellDetails[i][j][k][l].g + 1.0 + diagonalAddWeight;
				hNew = calculateHValue(i - 1, jpJ, dest);
				fNew = gNew + hNew;

				// If it isnít on the open list, add it to 
				// the open list. Make the current square 
				// the parent of this square. Record the 
				// f, g, and h costs of the square cell 
				//                OR 
				// If it is on the open list already, check 
				// to see if this path to that square is better, 
				// using 'f' cost as the measure. 
				if (cellDetails[i - 1][jpJ][kNew][lNew].f == INT_MAX ||
					cellDetails[i - 1][jpJ][kNew][lNew].f > fNew)
				{
					openList.insert(make_pair(fNew,
						make_tuple(i - 1, jpJ, kNew, lNew)));

					// Update the details of this cell 
					cellDetails[i - 1][jpJ][kNew][lNew].f = fNew;
					cellDetails[i - 1][jpJ][kNew][lNew].g = gNew;
					cellDetails[i - 1][jpJ][kNew][lNew].h = hNew;
					cellDetails[i - 1][jpJ][kNew][lNew].parent_i = i;
					cellDetails[i - 1][jpJ][kNew][lNew].parent_j = j;
					cellDetails[i - 1][jpJ][kNew][lNew].parent_k = k;
					cellDetails[i - 1][jpJ][kNew][lNew].parent_l = l;
				}
			}
		}

		//TODO: NORTH-WEST-1				

		//----------- 8th Successor (NORTH-EAST-2) ------------ 

		// Only process this cell if this is a valid one 
		if (isValid(i + 1, j + addNorth, ROW, COL) == true &&
			kValue >= 0 && 
			(l == 0 && kValue < maxJumpTiles ||
				l == 1 && kValue < maxJumpPadJumpTiles) &&
			isUnBlocked(grid, i + 1, j + addNorth) == true &&
			isUnBlocked(grid, i, j + addNorth) == true)
		{
			if (game.level.tiles[i + 1][j + addNorth - 1] == JUMP_PAD)
			{
				kNew = 0;
				lNew = 1;
			}
			else
			{
				kNew = k + addNorth;
				lNew = l;
			}			
		
			// If the destination cell is the same as the 
			// current successor 
			if (isDestination(i + 1, j + addNorth, dest) == true)
			{
				// Set the Parent of the destination cell 
				cellDetails[i + 1][j + addNorth][kNew][lNew].parent_i = i;
				cellDetails[i + 1][j + addNorth][kNew][lNew].parent_j = j;
				cellDetails[i + 1][j + addNorth][kNew][lNew].parent_k = k;
				cellDetails[i + 1][j + addNorth][kNew][lNew].parent_l = l;
				foundDest = true;
				foundDestK = kNew;
				foundDestL = lNew;
				break;
			}

			// If the successor is already on the closed 
			// list or if it is blocked, then ignore it. 
			// Else do the following 
			else if (closedList[i + 1][j + addNorth][kNew][lNew] == false &&
				isUnBlocked(grid, i + 1, j + addNorth) == true)
			{
				gNew = cellDetails[i][j][k][l].g + 1.0 + diagonalAddWeight;
				hNew = calculateHValue(i + 1, j + addNorth, dest);
				fNew = gNew + hNew;

				// If it isnít on the open list, add it to 
				// the open list. Make the current square 
				// the parent of this square. Record the 
				// f, g, and h costs of the square cell 
				//                OR 
				// If it is on the open list already, check 
				// to see if this path to that square is better, 
				// using 'f' cost as the measure. 
				if (cellDetails[i + 1][j + addNorth][kNew][lNew].f == INT_MAX ||
					cellDetails[i + 1][j + addNorth][kNew][lNew].f > fNew)
				{
					openList.insert(make_pair(fNew,
						make_tuple(i + 1, j + addNorth, kNew, lNew)));

					// Update the details of this cell 
					cellDetails[i + 1][j + addNorth][kNew][lNew].f = fNew;
					cellDetails[i + 1][j + addNorth][kNew][lNew].g = gNew;
					cellDetails[i + 1][j + addNorth][kNew][lNew].h = hNew;
					cellDetails[i + 1][j + addNorth][kNew][lNew].parent_i = i;
					cellDetails[i + 1][j + addNorth][kNew][lNew].parent_j = j;
					cellDetails[i + 1][j + addNorth][kNew][lNew].parent_k = k;
					cellDetails[i + 1][j + addNorth][kNew][lNew].parent_l = l;
				}
			}
		}

		//----------- 8th Successor (NORTH-EAST-0) ------------ 

		// Only process this cell if this is a valid one 
		if (isValid(i + 1, j + addNorth, ROW, COL) == true &&
			(l == 0 && kValue >= 0 && kValue < maxJumpTiles ||
				l == 1 && kValue == maxJumpPadJumpTiles - addNorth) &&
			isUnBlocked(grid, i + 1, j + addNorth) == true &&
			isUnBlocked(grid, i, j + addNorth) == true &&
			(game.level.tiles[i + 1][j + addNorth - 1] == PLATFORM || game.level.tiles[i + 1][j + addNorth - 1] == LADDER || game.level.tiles[i + 1][j + addNorth - 1] == WALL))
		{
			kNew = 0;

			if (game.level.tiles[i + 1][j + addNorth] == JUMP_PAD)
			{
				lNew = 1;
				jpJ = j + addNorth + 1;
			}
			else
			{
				lNew = 0;
				jpJ = j + addNorth;
			}

			// If the destination cell is the same as the 
			// current successor 
			if (isDestination(i + 1, jpJ, dest) == true)
			{
				// Set the Parent of the destination cell 
				cellDetails[i + 1][jpJ][kNew][lNew].parent_i = i;
				cellDetails[i + 1][jpJ][kNew][lNew].parent_j = j;
				cellDetails[i + 1][jpJ][kNew][lNew].parent_k = k;
				cellDetails[i + 1][jpJ][kNew][lNew].parent_l = l;
				foundDest = true;
				foundDestK = kNew;
				foundDestL = lNew;
				break;
			}

			// If the successor is already on the closed 
			// list or if it is blocked, then ignore it. 
			// Else do the following 
			else if (closedList[i + 1][jpJ][kNew][lNew] == false &&
				isUnBlocked(grid, i + 1, jpJ) == true)
			{
				gNew = cellDetails[i][j][k][l].g + 1.0 + diagonalAddWeight;
				hNew = calculateHValue(i + 1, jpJ, dest);
				fNew = gNew + hNew;

				// If it isnít on the open list, add it to 
				// the open list. Make the current square 
				// the parent of this square. Record the 
				// f, g, and h costs of the square cell 
				//                OR 
				// If it is on the open list already, check 
				// to see if this path to that square is better, 
				// using 'f' cost as the measure. 
				if (cellDetails[i + 1][jpJ][kNew][lNew].f == INT_MAX ||
					cellDetails[i + 1][jpJ][kNew][lNew].f > fNew)
				{
					openList.insert(make_pair(fNew,
						make_tuple(i + 1, jpJ, kNew, lNew)));

					// Update the details of this cell 
					cellDetails[i + 1][jpJ][kNew][lNew].f = fNew;
					cellDetails[i + 1][jpJ][kNew][lNew].g = gNew;
					cellDetails[i + 1][jpJ][kNew][lNew].h = hNew;
					cellDetails[i + 1][jpJ][kNew][lNew].parent_i = i;
					cellDetails[i + 1][jpJ][kNew][lNew].parent_j = j;
					cellDetails[i + 1][jpJ][kNew][lNew].parent_k = k;
					cellDetails[i + 1][jpJ][kNew][lNew].parent_l = l;
				}
			}
		}


		//TODO: NORTH-EAST-1


		//----------- 4th Successor (SOUTH) ------------ 

		// Only process this cell if this is a valid one 
		if (isValid(i, j - 1, ROW, COL) == true &&
			kValue != 0 &&
			(l == 0 || l == 1 && kValue == maxJumpPadJumpTiles) &&
			isUnBlocked(grid, i, j - 1) == true)
		{
			kNew = game.level.tiles[i][j - 2] == EMPTY ||
				game.level.tiles[i][j - 2] == JUMP_PAD ?
				maxJumpPadJumpTiles + 1 : 0;

			if (game.level.tiles[i][j - 1] == JUMP_PAD)
			{
				lNew = 1;
				jpJ = j;
			}
			else
			{
				lNew = 0;
				jpJ = j - 1;
			}

			// If the destination cell is the same as the 
			// current successor 
			if (isDestination(i, jpJ, dest) == true)
			{
				// Set the Parent of the destination cell 
				cellDetails[i][jpJ][kNew][lNew].parent_i = i;
				cellDetails[i][jpJ][kNew][lNew].parent_j = j;
				cellDetails[i][jpJ][kNew][lNew].parent_k = k;
				cellDetails[i][jpJ][kNew][lNew].parent_l = l;
				foundDest = true;
				foundDestK = kNew;
				foundDestL = lNew;
				break;
			}

			// If the successor is already on the closed 
			// list or if it is blocked, then ignore it. 
			// Else do the following 
			else if (closedList[i][jpJ][kNew][lNew] == false)
			{
				gNew = cellDetails[i][j][k][l].g + 1.0;
				hNew = calculateHValue(i, jpJ, dest);
				fNew = gNew + hNew;

				// If it isnít on the open list, add it to 
				// the open list. Make the current square 
				// the parent of this square. Record the 
				// f, g, and h costs of the square cell 
				//                OR 
				// If it is on the open list already, check 
				// to see if this path to that square is better, 
				// using 'f' cost as the measure. 
				if (cellDetails[i][jpJ][kNew][lNew].f == INT_MAX ||
					cellDetails[i][jpJ][kNew][lNew].f > fNew)
				{
					openList.insert(make_pair(fNew,
						make_tuple(i, jpJ, kNew, lNew)));

					// Update the details of this cell 
					cellDetails[i][jpJ][kNew][lNew].f = fNew;
					cellDetails[i][jpJ][kNew][lNew].g = gNew;
					cellDetails[i][jpJ][kNew][lNew].h = hNew;
					cellDetails[i][jpJ][kNew][lNew].parent_i = i;
					cellDetails[i][jpJ][kNew][lNew].parent_j = j;
					cellDetails[i][jpJ][kNew][lNew].parent_k = k;
					cellDetails[i][jpJ][kNew][lNew].parent_l = l;
				}
			}
		}



		
		//----------- 5th Successor (SOUTH-WEST) ------------ 

		// Only process this cell if this is a valid one 
		if (isValid(i - 1, j - 1, ROW, COL) == true &&
			isUnBlocked(grid, i - 1, j - 1) == true &&
			(l ==0 || l == 1 && kValue == maxJumpPadJumpTiles) &&
			(kValue != 0 || kValue == 0 && isUnBlocked(grid, i - 1, j)))
		{
			if (game.level.tiles[i][j - 1] == JUMP_PAD)
			{
				kNew = 0;
				lNew = 1;
				jpJ = j;
			}

			else
			{		
				if (game.level.tiles[i - 1][j - 1] == JUMP_PAD)
				{
					kNew = 0;
					lNew = 1;
					jpJ = j;
				}
				else
				{
					kNew = game.level.tiles[i - 1][j - 2] == EMPTY ||
						game.level.tiles[i - 1][j - 2] == JUMP_PAD ?
						maxJumpPadJumpTiles + 1 : 0;
					lNew = 0;
					jpJ = j - 1;
				}
			}
			
			// If the destination cell is the same as the 
			// current successor 
			if (isDestination(i - 1, jpJ, dest) == true)
			{
				// Set the Parent of the destination cell 
				cellDetails[i - 1][jpJ][kNew][lNew].parent_i = i;
				cellDetails[i - 1][jpJ][kNew][lNew].parent_j = j;
				cellDetails[i - 1][jpJ][kNew][lNew].parent_k = k;
				cellDetails[i - 1][jpJ][kNew][lNew].parent_l = l;
				foundDest = true;
				foundDestK = kNew;
				foundDestL = lNew;
				break;
			}

			// If the successor is already on the closed 
			// list or if it is blocked, then ignore it. 
			// Else do the following 
			else if (closedList[i - 1][jpJ][kNew][lNew] == false)
			{
				gNew = cellDetails[i][j][k][l].g + 1.0 + diagonalAddWeight;
				hNew = calculateHValue(i - 1, jpJ, dest);
				fNew = gNew + hNew;

				// If it isnít on the open list, add it to 
				// the open list. Make the current square 
				// the parent of this square. Record the 
				// f, g, and h costs of the square cell 
				//                OR 
				// If it is on the open list already, check 
				// to see if this path to that square is better, 
				// using 'f' cost as the measure. 
				if (cellDetails[i - 1][jpJ][kNew][lNew].f == INT_MAX ||
					cellDetails[i - 1][jpJ][kNew][lNew].f > fNew)
				{
					openList.insert(make_pair(fNew,
						make_tuple(i - 1, j - 1, kNew, lNew)));
					// Update the details of this cell 
					cellDetails[i - 1][jpJ][kNew][lNew].f = fNew;
					cellDetails[i - 1][jpJ][kNew][lNew].g = gNew;
					cellDetails[i - 1][jpJ][kNew][lNew].h = hNew;
					cellDetails[i - 1][jpJ][kNew][lNew].parent_i = i;
					cellDetails[i - 1][jpJ][kNew][lNew].parent_j = j;
					cellDetails[i - 1][jpJ][kNew][lNew].parent_k = k;
					cellDetails[i - 1][jpJ][kNew][lNew].parent_l = l;
				}
			}
		}

		//----------- 6th Successor (SOUTH-EAST) ------------ 

		// Only process this cell if this is a valid one 
		if (isValid(i + 1, j - 1, ROW, COL) == true &&
			isUnBlocked(grid, i + 1, j - 1) == true &&
			(l == 0 || l == 1 && kValue == maxJumpPadJumpTiles) &&
			(kValue != 0 || kValue == 0 && isUnBlocked(grid, i + 1, j)))
		{
			if (game.level.tiles[i][j - 1] == JUMP_PAD)
			{
				kNew = 0;
				lNew = 1;
				jpJ = j;
			}

			else
			{				
				if (game.level.tiles[i + 1][j - 1] == JUMP_PAD)
				{
					kNew = 0;
					lNew = 1;
					jpJ = j;
				}
				else
				{
					kNew = game.level.tiles[i + 1][j - 2] == EMPTY ||
						game.level.tiles[i + 1][j - 2] == JUMP_PAD ?
						maxJumpPadJumpTiles + 1 : 0;
					lNew = 0;
					jpJ = j - 1;
				}
			}
			
			// If the destination cell is the same as the 
			// current successor 
			if (isDestination(i + 1, jpJ, dest) == true)
			{
				// Set the Parent of the destination cell 
				cellDetails[i + 1][jpJ][kNew][lNew].parent_i = i;
				cellDetails[i + 1][jpJ][kNew][lNew].parent_j = j;
				cellDetails[i + 1][jpJ][kNew][lNew].parent_k = k;
				cellDetails[i + 1][jpJ][kNew][lNew].parent_l = l;
				foundDest = true;
				foundDestK = kNew;
				foundDestL = lNew;
				break;
			}

			// If the successor is already on the closed 
			// list or if it is blocked, then ignore it. 
			// Else do the following 
			else if (closedList[i + 1][jpJ][kNew][lNew] == false)
			{
				gNew = cellDetails[i][j][k][l].g + 1.0 + diagonalAddWeight;
				hNew = calculateHValue(i + 1, jpJ, dest);
				fNew = gNew + hNew;

				// If it isnít on the open list, add it to 
				// the open list. Make the current square 
				// the parent of this square. Record the 
				// f, g, and h costs of the square cell 
				//                OR 
				// If it is on the open list already, check 
				// to see if this path to that square is better, 
				// using 'f' cost as the measure. 
				if (cellDetails[i + 1][jpJ][kNew][lNew].f == INT_MAX ||
					cellDetails[i + 1][jpJ][kNew][lNew].f > fNew)
				{
					openList.insert(make_pair(fNew,
						make_tuple(i + 1, jpJ, kNew, lNew)));

					// Update the details of this cell 
					cellDetails[i + 1][jpJ][kNew][lNew].f = fNew;
					cellDetails[i + 1][jpJ][kNew][lNew].g = gNew;
					cellDetails[i + 1][jpJ][kNew][lNew].h = hNew;
					cellDetails[i + 1][jpJ][kNew][lNew].parent_i = i;
					cellDetails[i + 1][jpJ][kNew][lNew].parent_j = j;
					cellDetails[i + 1][jpJ][kNew][lNew].parent_k = k;
					cellDetails[i + 1][jpJ][kNew][lNew].parent_l = l;
				}
			}
		}

	}

	// When the destination cell is not found and the open 
	// list is empty, then we conclude that we failed to 
	// reach the destiantion cell. This may happen when the 
	// there is no way to destination cell (due to blockages) 
	if (!foundDest) throw runtime_error("Failed to find the Destination Cell\n");
	
	return tracePath(cellDetails, dest, foundDestK, foundDestL);
}


// Driver program to test above function 
//int main()
//{
//	/* Description of the Grid-
//	 1--> The cell is not blocked
//	 0--> The cell is blocked    */
//	int grid[ROW][COL] =
//	{
//		{ 1, 0, 1, 1, 1, 1, 0, 1, 1, 1 },
//		{ 1, 1, 1, 0, 1, 1, 1, 0, 1, 1 },
//		{ 1, 1, 1, 0, 1, 1, 0, 1, 0, 1 },
//		{ 0, 0, 1, 0, 1, 0, 0, 0, 0, 1 },
//		{ 1, 1, 1, 0, 1, 1, 1, 0, 1, 0 },
//		{ 1, 0, 1, 1, 1, 1, 0, 1, 0, 0 },
//		{ 1, 0, 0, 0, 0, 1, 0, 0, 0, 1 },
//		{ 1, 0, 1, 1, 1, 1, 0, 1, 1, 1 },
//		{ 1, 1, 1, 0, 0, 0, 1, 0, 0, 1 }
//	};
//
//	// Source is the left-most bottom-most corner 
//	Pair src = make_pair(8, 0);
//
//	// Destination is the left-most top-most corner 
//	Pair dest = make_pair(0, 0);
//
//	aStarSearch(grid, src, dest);
//
//	return(0);
//}