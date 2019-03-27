//Zhaoran Wang Head files for path finding

#pragma once
#ifndef _BUILDPATH_H_
#define _BUILDPATH_H_

#include <vector>
#include "Angel.h"
#include "Recast\Recast.h"

using namespace std;

struct AStarNode
{
	int iIndex;
	int iF_score;
};

//Area
void CalculateArea(rcPolyMesh *PolyMesh, int *PolyArea);

//Contiguous
void CalculateContiguous(rcPolyMesh *PolyMesh, int ** con);

//Center and Contiguous
void CalculateContiguousAndCentre(rcPolyMesh *PolyMesh, bool ** con, vec3* centre);

//Dijkstra
void Dijkstra(int **con, int nploys, int v0, int *distance, int *path);

//Next point
bool FindNextPoint(rcPolyMesh *PolyMesh,int *StartPoint, int StartIndex, int *EndPoint, int EndIndex, 
				   int *Path, int *NextPoint, int &NextIndex);

//Best Path
bool FindPath(rcPolyMesh *PolyMesh, int **con, vec3 StartPoint, vec3 EndPoint, vector<vec3> &Vect);

//A* alg
void AStar(bool **con, int npolys, int iStart, int iEnd, vec3 *centre, int *came_from);

//A* path finding
bool FindPathOfAStar(rcPolyMesh *PolyMesh, bool **con, vec3 *centre, vec3 StartPoint, vec3 EndPoint, vector<vec3> &Vect);

#endif