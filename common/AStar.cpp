
#include "AStar.h"

// A C++ Program to implement A* Search Algorithm 
#include <utility>
#include <cstdio>
#include <stack>
#include <set>
#include <vector>
#include <stdexcept>
#include <tuple>
#include "../model/Game.hpp"
using namespace std;

//#define ROW 9 
//#define COL 10 



// Creating a shortcut for pair<int, pair<int, int>> type 
typedef pair<double, Triple> pPair;

// A structure to hold the neccesary parameters 
struct cell
{
	// Row and Column index of its parent 
	// Note that 0 <= i <= ROW-1 & 0 <= j <= COL-1 
	int parent_i, parent_j, parent_k;
	// f = g + h 
	double f, g, h;
};

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
bool isDestination(int row, int col, Pair dest)
{
	if (row == dest.first && col == dest.second)
		return (true);
	else
		return (false);
}

// A Utility Function to calculate the 'h' heuristics. 
double calculateHValue(int row, int col, Pair dest)
{
	// Return using the distance formula 
	return ((double)sqrt((row - dest.first) * (row - dest.first)
		+ (col - dest.second) * (col - dest.second)));
}

// A Utility Function to trace the path from the source 
// to destination 
stack<Pair> tracePath(vector<vector<vector<cell>>> cellDetails, Pair dest, int foundDestK)
{
	int col = dest.first;
	int row = dest.second;
	int k = foundDestK;

	stack<Pair> Path;

	while (!(cellDetails[col][row][k].parent_i == col
		&& cellDetails[col][row][k].parent_j == row))
	{
		Path.push(make_pair(col, row));
		int temp_col = cellDetails[col][row][k].parent_i;
		int temp_row = cellDetails[col][row][k].parent_j;
		int temp_k = cellDetails[col][row][k].parent_k;
		col = temp_col;
		row = temp_row;		
		k = temp_k;
	}

	Path.push(make_pair(col, row));	

	return Path;
}

// A Function to find the shortest path between 
// a given source cell to a destination cell according 
// to A* Search Algorithm 
stack<Pair> aStarSearch(vector<vector<int>> grid, Triple src, Pair dest,
	const Game& game)
{
	// z-index:
	// 0 - standing
	// 1 - falling
	// 2 - jumping
	
	const auto ROW = grid.size();
	const auto COL = grid[0].size();
	const auto Z_SIZE = 3;

	const auto start_x = get<0>(src);
	const auto start_y = get<1>(src);
	const auto start_z = get<2>(src);
	
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
		throw runtime_error("We are already at the destination\n");
	}

	// Create a closed list and initialise it to false which means 
	// that no cell has been included yet 
	// This closed list is implemented as a boolean 2D array

	vector<vector<vector<bool>>> closedList(
		ROW, vector<vector<bool>>(
			COL, vector<bool>(Z_SIZE, false)));

	// Declare a 2D array of structure to hold the details 
	//of that cell
	vector<vector<vector<cell>>> cellDetails(
		ROW, vector<vector<cell>>(COL, 
			vector<cell>(Z_SIZE)));

	int i, j, k;

	for (i = 0; i < ROW; i++)
	{
		for (j = 0; j < COL; j++)
		{
			for (k = 0; k < Z_SIZE; ++k) {
				cellDetails[i][j][k].f = FLT_MAX;
				cellDetails[i][j][k].g = FLT_MAX;
				cellDetails[i][j][k].h = FLT_MAX;
				cellDetails[i][j][k].parent_i = -1;
				cellDetails[i][j][k].parent_j = -1;
				cellDetails[i][j][k].parent_k = -1;
			}
		}
	}

	// Initialising the parameters of the starting node 
	i = start_x, j = start_y; k = start_z;
	cellDetails[i][j][k].f = 0.0;
	cellDetails[i][j][k].g = 0.0;
	cellDetails[i][j][k].h = 0.0;
	cellDetails[i][j][k].parent_i = i;
	cellDetails[i][j][k].parent_j = j;
	cellDetails[i][j][k].parent_k = k;

	/*
	 Create an open list having information as-
	 <f, <i, j>>
	 where f = g + h,
	 and i, j are the row and column index of that cell
	 Note that 0 <= i <= ROW-1 & 0 <= j <= COL-1
	 This open list is implenented as a set of pair of pair.*/
	set<pPair> openList;

	// Put the starting cell on the open list and set its 
	// 'f' as 0
	
	openList.insert(make_pair(0.0, 
		make_tuple(i, j, start_z)));

	// We set this boolean value as false as initially 
	// the destination is not reached. 
	bool foundDest = false;
	int foundDestK = -1;

	while (!openList.empty())
	{
		pPair p = *openList.begin();

		// Remove this vertex from the open list 
		openList.erase(openList.begin());

		// Add this vertex to the closed list 
		i = get<0>(p.second);
		j = get<1>(p.second);
		k = get<2>(p.second);
		closedList[i][j][k] = true;

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
		int kNew;

		//----------- 1st Successor (WEST) ------------ 

		// Only process this cell if this is a valid one 
		if (isValid(i - 1, j, ROW, COL) == true && k == 0)
		{
			kNew = game.level.tiles[i - 1][j - 1] == EMPTY ||
				game.level.tiles[i - 1][j - 1] == JUMP_PAD ?
				1 :	0;
			// If the destination cell is the same as the 
			// current successor 
			if (isDestination(i - 1, j, dest) == true)
			{
				// Set the Parent of the destination cell 
				cellDetails[i - 1][j][kNew].parent_i = i;
				cellDetails[i - 1][j][kNew].parent_j = j;
				cellDetails[i - 1][j][kNew].parent_k = k;
				foundDest = true;
				foundDestK = kNew;
				break;
			}
			// If the successor is already on the closed 
			// list or if it is blocked, then ignore it. 
			// Else do the following 
			else if (closedList[i - 1][j][kNew] == false &&
				isUnBlocked(grid, i - 1, j) == true)
			{
				gNew = cellDetails[i][j][k].g + 1.0;
				hNew = calculateHValue(i - 1, j, dest);
				fNew = gNew + hNew;

				// If it isn’t on the open list, add it to 
				// the open list. Make the current square 
				// the parent of this square. Record the 
				// f, g, and h costs of the square cell 
				//                OR 
				// If it is on the open list already, check 
				// to see if this path to that square is better, 
				// using 'f' cost as the measure. 
				if (cellDetails[i - 1][j][kNew].f == FLT_MAX ||
					cellDetails[i - 1][j][kNew].f > fNew)
				{
					openList.insert(make_pair(fNew,
						make_tuple(i - 1, j, kNew)));

					// Update the details of this cell 
					cellDetails[i - 1][j][kNew].f = fNew;
					cellDetails[i - 1][j][kNew].g = gNew;
					cellDetails[i - 1][j][kNew].h = hNew;
					cellDetails[i - 1][j][kNew].parent_i = i;
					cellDetails[i - 1][j][kNew].parent_j = j;
					cellDetails[i - 1][j][kNew].parent_k = k;
				}
			}
		}

		//----------- 2nd Successor (EAST) ------------ 

		// Only process this cell if this is a valid one 
		if (isValid(i + 1, j, ROW, COL) == true && k == 0)
		{
			kNew = game.level.tiles[i + 1][j - 1] == EMPTY ||
				game.level.tiles[i + 1][j - 1] == JUMP_PAD ?
				1 : 0;
			// If the destination cell is the same as the 
			// current successor 
			if (isDestination(i + 1, j, dest) == true)
			{
				// Set the Parent of the destination cell 
				cellDetails[i + 1][j][kNew].parent_i = i;
				cellDetails[i + 1][j][kNew].parent_j = j;
				cellDetails[i + 1][j][kNew].parent_k = k;
				foundDest = true;
				foundDestK = kNew;
				break;
			}
			// If the successor is already on the closed 
			// list or if it is blocked, then ignore it. 
			// Else do the following 
			else if (closedList[i + 1][j][kNew] == false &&
				isUnBlocked(grid, i + 1, j) == true)
			{
				gNew = cellDetails[i][j][k].g + 1.0;
				hNew = calculateHValue(i + 1, j, dest);
				fNew = gNew + hNew;

				// If it isn’t on the open list, add it to 
				// the open list. Make the current square 
				// the parent of this square. Record the 
				// f, g, and h costs of the square cell 
				//                OR 
				// If it is on the open list already, check 
				// to see if this path to that square is better, 
				// using 'f' cost as the measure. 
				if (cellDetails[i + 1][j][kNew].f == FLT_MAX ||
					cellDetails[i + 1][j][kNew].f > fNew)
				{
					openList.insert(make_pair(fNew, make_tuple(i + 1, j, kNew)));
					// Update the details of this cell 
					cellDetails[i + 1][j][kNew].f = fNew;
					cellDetails[i + 1][j][kNew].g = gNew;
					cellDetails[i + 1][j][kNew].h = hNew;
					cellDetails[i + 1][j][kNew].parent_i = i;
					cellDetails[i + 1][j][kNew].parent_j = j;
					cellDetails[i + 1][j][kNew].parent_k = k;
				}
			}
		}

		//----------- 3.0 Successor (NORTH-2) ------------ 

		// Only process this cell if this is a valid one 
		if (isValid(i, j + 1, ROW, COL) == true && k != 1)
		{
			kNew = 2;
			// If the destination cell is the same as the 
			// current successor 
			if (isDestination(i, j + 1, dest) == true)
			{
				// Set the Parent of the destination cell 
				cellDetails[i][j + 1][kNew].parent_i = i;
				cellDetails[i][j + 1][kNew].parent_j = j;
				cellDetails[i][j + 1][kNew].parent_k = k;
				foundDest = true;
				foundDestK = kNew;
				break;
			}

			// If the successor is already on the closed 
			// list or if it is blocked, then ignore it. 
			// Else do the following 
			else if (closedList[i][j + 1][kNew] == false &&
				isUnBlocked(grid, i, j + 1) == true)
			{
				gNew = cellDetails[i][j][k].g + 1.0;
				hNew = calculateHValue(i, j + 1, dest);
				fNew = gNew + hNew;

				// If it isn’t on the open list, add it to 
				// the open list. Make the current square 
				// the parent of this square. Record the 
				// f, g, and h costs of the square cell 
				//                OR 
				// If it is on the open list already, check 
				// to see if this path to that square is better, 
				// using 'f' cost as the measure. 
				if (cellDetails[i][j + 1][kNew].f == FLT_MAX ||
					cellDetails[i][j + 1][kNew].f > fNew)
				{
					openList.insert(make_pair(fNew,
						make_tuple(i, j + 1, kNew)));

					// Update the details of this cell 
					cellDetails[i][j + 1][kNew].f = fNew;
					cellDetails[i][j + 1][kNew].g = gNew;
					cellDetails[i][j + 1][kNew].h = hNew;
					cellDetails[i][j + 1][kNew].parent_i = i;
					cellDetails[i][j + 1][kNew].parent_j = j;
					cellDetails[i][j + 1][kNew].parent_k = k;
				}
			}
		}

		//----------- 3rd Successor (NORTH-0) ------------ 

		// Only process this cell if this is a valid one 
		if (isValid(i, j + 1, ROW, COL) == true && k == 2 &&
			(game.level.tiles[i][j] == PLATFORM || game.level.tiles[i][j] == LADDER))
		{
			kNew = 0;
			// If the destination cell is the same as the 
			// current successor 
			if (isDestination(i, j + 1, dest) == true)
			{
				// Set the Parent of the destination cell 
				cellDetails[i][j + 1][kNew].parent_i = i;
				cellDetails[i][j + 1][kNew].parent_j = j;
				cellDetails[i][j + 1][kNew].parent_k = k;
				foundDest = true;
				foundDestK = kNew;
				break;
			}

			// If the successor is already on the closed 
			// list or if it is blocked, then ignore it. 
			// Else do the following 
			else if (closedList[i][j + 1][kNew] == false &&
				isUnBlocked(grid, i, j + 1) == true)
			{
				gNew = cellDetails[i][j][k].g + 1.0;
				hNew = calculateHValue(i, j + 1, dest);
				fNew = gNew + hNew;

				// If it isn’t on the open list, add it to 
				// the open list. Make the current square 
				// the parent of this square. Record the 
				// f, g, and h costs of the square cell 
				//                OR 
				// If it is on the open list already, check 
				// to see if this path to that square is better, 
				// using 'f' cost as the measure. 
				if (cellDetails[i][j + 1][kNew].f == FLT_MAX ||
					cellDetails[i][j + 1][kNew].f > fNew)
				{
					openList.insert(make_pair(fNew,
						make_tuple(i, j + 1, kNew)));

					// Update the details of this cell 
					cellDetails[i][j + 1][kNew].f = fNew;
					cellDetails[i][j + 1][kNew].g = gNew;
					cellDetails[i][j + 1][kNew].h = hNew;
					cellDetails[i][j + 1][kNew].parent_i = i;
					cellDetails[i][j + 1][kNew].parent_j = j;
					cellDetails[i][j + 1][kNew].parent_k = k;
				}
			}
		}

		//TODO: NORTH-1

		//----------- 4th Successor (SOUTH) ------------ 

		// Only process this cell if this is a valid one 
		if (isValid(i, j - 1, ROW, COL) == true && (k == 1 || k == 2))
		{
			kNew = game.level.tiles[i][j - 2] == EMPTY ||
				game.level.tiles[i][j - 2] == JUMP_PAD ?
				1 : 0;
			// If the destination cell is the same as the 
			// current successor 
			if (isDestination(i, j - 1, dest) == true)
			{
				// Set the Parent of the destination cell 
				cellDetails[i][j - 1][kNew].parent_i = i;
				cellDetails[i][j - 1][kNew].parent_j = j;
				cellDetails[i][j - 1][kNew].parent_k = k;
				foundDest = true;
				foundDestK = kNew;
				break;
			}

			// If the successor is already on the closed 
			// list or if it is blocked, then ignore it. 
			// Else do the following 
			else if (closedList[i][j - 1][kNew] == false &&
				isUnBlocked(grid, i, j - 1) == true)
			{
				gNew = cellDetails[i][j][k].g + 1.0;
				hNew = calculateHValue(i, j - 1, dest);
				fNew = gNew + hNew;

				// If it isn’t on the open list, add it to 
				// the open list. Make the current square 
				// the parent of this square. Record the 
				// f, g, and h costs of the square cell 
				//                OR 
				// If it is on the open list already, check 
				// to see if this path to that square is better, 
				// using 'f' cost as the measure. 
				if (cellDetails[i][j - 1][kNew].f == FLT_MAX ||
					cellDetails[i][j - 1][kNew].f > fNew)
				{
					openList.insert(make_pair(fNew,
						make_tuple(i, j - 1, kNew)));

					// Update the details of this cell 
					cellDetails[i][j - 1][kNew].f = fNew;
					cellDetails[i][j - 1][kNew].g = gNew;
					cellDetails[i][j - 1][kNew].h = hNew;
					cellDetails[i][j - 1][kNew].parent_i = i;
					cellDetails[i][j - 1][kNew].parent_j = j;
					cellDetails[i][j - 1][kNew].parent_k = k;
				}
			}
		}

	//	//----------- 5th Successor (North-East) ------------ 

	//	// Only process this cell if this is a valid one 
	//	if (isValid(i - 1, j + 1, ROW, COL) == true)
	//	{
	//		// If the destination cell is the same as the 
	//		// current successor 
	//		if (isDestination(i - 1, j + 1, dest) == true)
	//		{
	//			// Set the Parent of the destination cell 
	//			cellDetails[i - 1][j + 1].parent_i = i;
	//			cellDetails[i - 1][j + 1].parent_j = j;
	//			foundDest = true;
	//			break;
	//		}

	//		// If the successor is already on the closed 
	//		// list or if it is blocked, then ignore it. 
	//		// Else do the following 
	//		else if (closedList[i - 1][j + 1] == false &&
	//			isUnBlocked(grid, i - 1, j + 1) == true)
	//		{
	//			gNew = cellDetails[i][j].g + 1.414;
	//			hNew = calculateHValue(i - 1, j + 1, dest);
	//			fNew = gNew + hNew;

	//			// If it isn’t on the open list, add it to 
	//			// the open list. Make the current square 
	//			// the parent of this square. Record the 
	//			// f, g, and h costs of the square cell 
	//			//                OR 
	//			// If it is on the open list already, check 
	//			// to see if this path to that square is better, 
	//			// using 'f' cost as the measure. 
	//			if (cellDetails[i - 1][j + 1].f == FLT_MAX ||
	//				cellDetails[i - 1][j + 1].f > fNew)
	//			{
	//				openList.insert(make_pair(fNew,
	//					make_pair(i - 1, j + 1)));

	//				// Update the details of this cell 
	//				cellDetails[i - 1][j + 1].f = fNew;
	//				cellDetails[i - 1][j + 1].g = gNew;
	//				cellDetails[i - 1][j + 1].h = hNew;
	//				cellDetails[i - 1][j + 1].parent_i = i;
	//				cellDetails[i - 1][j + 1].parent_j = j;
	//			}
	//		}
	//	}

	//	//----------- 6th Successor (North-West) ------------ 

	//	// Only process this cell if this is a valid one 
	//	if (isValid(i - 1, j - 1, ROW, COL) == true)
	//	{
	//		// If the destination cell is the same as the 
	//		// current successor 
	//		if (isDestination(i - 1, j - 1, dest) == true)
	//		{
	//			// Set the Parent of the destination cell 
	//			cellDetails[i - 1][j - 1].parent_i = i;
	//			cellDetails[i - 1][j - 1].parent_j = j;
	//			foundDest = true;
	//			break;
	//		}

	//		// If the successor is already on the closed 
	//		// list or if it is blocked, then ignore it. 
	//		// Else do the following 
	//		else if (closedList[i - 1][j - 1] == false &&
	//			isUnBlocked(grid, i - 1, j - 1) == true)
	//		{
	//			gNew = cellDetails[i][j].g + 1.414;
	//			hNew = calculateHValue(i - 1, j - 1, dest);
	//			fNew = gNew + hNew;

	//			// If it isn’t on the open list, add it to 
	//			// the open list. Make the current square 
	//			// the parent of this square. Record the 
	//			// f, g, and h costs of the square cell 
	//			//                OR 
	//			// If it is on the open list already, check 
	//			// to see if this path to that square is better, 
	//			// using 'f' cost as the measure. 
	//			if (cellDetails[i - 1][j - 1].f == FLT_MAX ||
	//				cellDetails[i - 1][j - 1].f > fNew)
	//			{
	//				openList.insert(make_pair(fNew, make_pair(i - 1, j - 1)));
	//				// Update the details of this cell 
	//				cellDetails[i - 1][j - 1].f = fNew;
	//				cellDetails[i - 1][j - 1].g = gNew;
	//				cellDetails[i - 1][j - 1].h = hNew;
	//				cellDetails[i - 1][j - 1].parent_i = i;
	//				cellDetails[i - 1][j - 1].parent_j = j;
	//			}
	//		}
	//	}

	//	//----------- 7th Successor (South-East) ------------ 

	//	// Only process this cell if this is a valid one 
	//	if (isValid(i + 1, j + 1, ROW, COL) == true)
	//	{
	//		// If the destination cell is the same as the 
	//		// current successor 
	//		if (isDestination(i + 1, j + 1, dest) == true)
	//		{
	//			// Set the Parent of the destination cell 
	//			cellDetails[i + 1][j + 1].parent_i = i;
	//			cellDetails[i + 1][j + 1].parent_j = j;
	//			foundDest = true;
	//			break;
	//		}

	//		// If the successor is already on the closed 
	//		// list or if it is blocked, then ignore it. 
	//		// Else do the following 
	//		else if (closedList[i + 1][j + 1] == false &&
	//			isUnBlocked(grid, i + 1, j + 1) == true)
	//		{
	//			gNew = cellDetails[i][j].g + 1.414;
	//			hNew = calculateHValue(i + 1, j + 1, dest);
	//			fNew = gNew + hNew;

	//			// If it isn’t on the open list, add it to 
	//			// the open list. Make the current square 
	//			// the parent of this square. Record the 
	//			// f, g, and h costs of the square cell 
	//			//                OR 
	//			// If it is on the open list already, check 
	//			// to see if this path to that square is better, 
	//			// using 'f' cost as the measure. 
	//			if (cellDetails[i + 1][j + 1].f == FLT_MAX ||
	//				cellDetails[i + 1][j + 1].f > fNew)
	//			{
	//				openList.insert(make_pair(fNew,
	//					make_pair(i + 1, j + 1)));

	//				// Update the details of this cell 
	//				cellDetails[i + 1][j + 1].f = fNew;
	//				cellDetails[i + 1][j + 1].g = gNew;
	//				cellDetails[i + 1][j + 1].h = hNew;
	//				cellDetails[i + 1][j + 1].parent_i = i;
	//				cellDetails[i + 1][j + 1].parent_j = j;
	//			}
	//		}
	//	}

	//	//----------- 8th Successor (South-West) ------------ 

	//	// Only process this cell if this is a valid one 
	//	if (isValid(i + 1, j - 1, ROW, COL) == true)
	//	{
	//		// If the destination cell is the same as the 
	//		// current successor 
	//		if (isDestination(i + 1, j - 1, dest) == true)
	//		{
	//			// Set the Parent of the destination cell 
	//			cellDetails[i + 1][j - 1].parent_i = i;
	//			cellDetails[i + 1][j - 1].parent_j = j;
	//			foundDest = true;
	//			break;
	//		}

	//		// If the successor is already on the closed 
	//		// list or if it is blocked, then ignore it. 
	//		// Else do the following 
	//		else if (closedList[i + 1][j - 1] == false &&
	//			isUnBlocked(grid, i + 1, j - 1) == true)
	//		{
	//			gNew = cellDetails[i][j].g + 1.414;
	//			hNew = calculateHValue(i + 1, j - 1, dest);
	//			fNew = gNew + hNew;

	//			// If it isn’t on the open list, add it to 
	//			// the open list. Make the current square 
	//			// the parent of this square. Record the 
	//			// f, g, and h costs of the square cell 
	//			//                OR 
	//			// If it is on the open list already, check 
	//			// to see if this path to that square is better, 
	//			// using 'f' cost as the measure. 
	//			if (cellDetails[i + 1][j - 1].f == FLT_MAX ||
	//				cellDetails[i + 1][j - 1].f > fNew)
	//			{
	//				openList.insert(make_pair(fNew,
	//					make_pair(i + 1, j - 1)));

	//				// Update the details of this cell 
	//				cellDetails[i + 1][j - 1].f = fNew;
	//				cellDetails[i + 1][j - 1].g = gNew;
	//				cellDetails[i + 1][j - 1].h = hNew;
	//				cellDetails[i + 1][j - 1].parent_i = i;
	//				cellDetails[i + 1][j - 1].parent_j = j;
	//			}
	//		}
	//	}
	}

	// When the destination cell is not found and the open 
	// list is empty, then we conclude that we failed to 
	// reach the destiantion cell. This may happen when the 
	// there is no way to destination cell (due to blockages) 
	if (foundDest == false)
		printf("Failed to find the Destination Cell\n");

	return tracePath(cellDetails, dest, foundDestK);
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