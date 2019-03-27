//Zhaoran Wang Path finding

#pragma once
#include "BuildPath.h"
#include "NavigationMesh.h"
#include <cmath>
#include <queue>
using namespace std;

#ifndef MAXWEIGHT
#define MAXWEIGHT 0xfffffff          //Max number define
#endif

//Get poly index
int GetPolyIndex(rcPolyMesh *PolyMesh, vec3 vPoint)
{
	for (int i = 0; i < PolyMesh->npolys; i ++)
	{
		//get poly
		unsigned short *poly = &PolyMesh->polys[i * PolyMesh->nvp * 2];
		int n = countPolyVerts(poly, PolyMesh->nvp);
		int j = 0;
		int pa[3] = {0}, pb[3] = {0}, pc[3] = {0};
		pc[0] = vPoint.x;
		pc[1] = vPoint.y;
		pc[2] = vPoint.z;
		for (j = 0; j < n; j ++)
		{
			unsigned short va0 = poly[j];
			unsigned short va1 = poly[(j+1) % n];
			pa[0] = PolyMesh->verts[va0 * 3 + 0];
			pa[1] = PolyMesh->verts[va0 * 3 + 1];
			pa[2] = PolyMesh->verts[va0 * 3 + 2];
			pb[0] = PolyMesh->verts[va1 * 3 + 0];
			pb[1] = PolyMesh->verts[va1 * 3 + 1];
			pb[2] = PolyMesh->verts[va1 * 3 + 2];
			if(!leftOn(pa, pb, pc))
				break;
		}
		if (j == n) return i;
	}
	return -1;
}

//Get poly area
void CalculateArea(rcPolyMesh *PolyMesh, int *PolyArea)
{
	int minx, maxx, miny, maxy;
	for (int i = 0; i < PolyMesh->npolys; i ++)
	{
		unsigned short* poly = &PolyMesh->polys[i * PolyMesh->nvp * 2];
		minx = maxx = PolyMesh->verts[poly[0] * 3 + 0];
		miny = maxy = PolyMesh->verts[poly[0] * 3 + 2];
		for (int j = 0; j < PolyMesh->nvp; j ++)
		{
			if (poly[j] == RC_MESH_NULL_IDX)
				break;
			minx = minx > PolyMesh->verts[poly[j] * 3 + 0] ? PolyMesh->verts[poly[j] * 3 + 0] : minx;
			maxx = maxx < PolyMesh->verts[poly[j] * 3 + 0] ? PolyMesh->verts[poly[j] * 3 + 0] : maxx;
			miny = miny > PolyMesh->verts[poly[j] * 3 + 2] ? PolyMesh->verts[poly[j] * 3 + 2] : miny;
			maxy = maxy < PolyMesh->verts[poly[j] * 3 + 2] ? PolyMesh->verts[poly[j] * 3 + 2] : maxy;
		}

		PolyArea[i] = sqrt(double((maxx - minx) * (maxy - miny)));
	}
}

//Get center
void CalculateCentre(rcPolyMesh *PolyMesh, vec3 *PolyCentre)
{
	int iSumX = 0, iSumZ = 0;
	for (int i = 0; i < PolyMesh->npolys; i ++)
	{
		iSumX = 0;
		iSumZ = 0;
		unsigned short* poly = &PolyMesh->polys[i * PolyMesh->nvp * 2];
		//Cal vertex Number
		int n = countPolyVerts(poly, PolyMesh->nvp);
		for (int j = 0; j < n; j ++)
		{
			iSumX += PolyMesh->verts[poly[j] * 3 + 0];
			iSumZ += PolyMesh->verts[poly[j] * 3 + 2];
		}
		PolyCentre[i].x = iSumX / n;
		PolyCentre[i].y = 0.0f;
		PolyCentre[i].z = iSumZ / n;
	}
}

//Check poly if share a same edge
bool CheackShareEdge(unsigned short* pa, unsigned short* pb, int nvp, int& ea, int& eb)
{
	const int na = countPolyVerts(pa, nvp);
	const int nb = countPolyVerts(pb, nvp);
	ea = -1;
	eb = -1;
	for (int i = 0; i < na; ++i)
	{
		unsigned short va0 = pa[i];
		unsigned short va1 = pa[(i+1) % na]; 
		if (va0 > va1)
			rcSwap(va0, va1);
		for (int j = 0; j < nb; ++j)
		{
			unsigned short vb0 = pb[j];
			unsigned short vb1 = pb[(j+1) % nb];
			if (vb0 > vb1)
				rcSwap(vb0, vb1);
			if (va0 == vb0 && va1 == vb1)
			{
				ea = i;
				eb = j;
				break;
			}
		}
	}
	if (ea == -1 || eb == -1)
		return false;

	return true;
}

//Calculate Contiguous
void CalculateContiguous(rcPolyMesh *PolyMesh, int ** con)
{
	int ea = -1, eb = -1;
	int *iArea = new int[PolyMesh->npolys];
	CalculateArea(PolyMesh, iArea);

	for (int i = 0; i < PolyMesh->npolys; i ++)
	{
		con[i][i] = 0;
		for (int j = i + 1; j < PolyMesh->npolys; j ++)
		{
			if (CheackShareEdge(&PolyMesh->polys[i * PolyMesh->nvp * 2],
				&PolyMesh->polys[j * PolyMesh->nvp * 2], PolyMesh->nvp, ea, eb))
			{
				con[i][j] = con[j][i] = iArea[j];
			}
		}
	}
	delete [] iArea;
}

//Calculate Contiguous and Centre
void CalculateContiguousAndCentre(rcPolyMesh *PolyMesh, bool ** con, vec3* centre)
{
	int ea = -1, eb = -1;
	CalculateCentre(PolyMesh, centre);

	for (int i = 0; i < PolyMesh->npolys; i ++)
	{
		con[i][i] = true;
		for (int j = i + 1; j < PolyMesh->npolys; j ++)
		{
			if (CheackShareEdge(&PolyMesh->polys[i * PolyMesh->nvp * 2],
				&PolyMesh->polys[j * PolyMesh->nvp * 2], PolyMesh->nvp, ea, eb))
			{
				con[i][j] = con[j][i] = true;
			}
		}
	}
}


//Dijkstra
void Dijkstra(int **con, int npolys, int v0, int *distance, int *path)
{
	int *s = new int[npolys];
	int minDis, i, j, u;
	//inital
	for (i = 0; i < npolys; ++i)
	{
		distance[i] = con[v0][i];
		s[i] = 0;
		if(i != v0 && distance[i] < MAXWEIGHT)
		{
			path[i] = v0;
		}
		else 
		{
			path[i] = -1;
		}
	}
	s[v0] = 1;

	//Find all nearst vertex
	for (i = 1; i < npolys; i ++)
	{
		minDis = MAXWEIGHT;
		for (j = 0; j < npolys; j ++)
		{
			if (s[j] == 0 && distance[j] < minDis)
			{
				u = j;
				minDis = distance[j];
			}
		}

		//Can not find a path
		if (minDis == MAXWEIGHT) return;

		s[u] = 1;

		for (j = 0; j < npolys; j ++)
		{
			if (s[j] == 0 && con[u][j] < MAXWEIGHT && 
				distance[u] + con[u][j] < distance[j])
			{
				distance[j] = distance[u] + con[u][j];
				path[j] = u;
			}
		}
	}
}

//Using Mesh find next point
bool FindNextPoint(rcPolyMesh *PolyMesh,int *StartPoint, int StartIndex, int *EndPoint, int EndIndex, 
				   int *Path, int *NextPoint, int &NextIndex)
{
	int ea = -1, eb = -1;	
	int iArrLastEndPoint[3] = {StartPoint[0], StartPoint[1], StartPoint[2]};
	int iArrLeftPoint[3] = {0};
	int iArrRightPoint[3] = {0};
	int iArrNewLeftPoint[3] = {0};
	int iArrNewRightPoint[3] = {0};

	//Nearst edge
	unsigned short *pa = &PolyMesh->polys[Path[StartIndex] * PolyMesh->nvp * 2];
	unsigned short *pb = &PolyMesh->polys[StartIndex * PolyMesh->nvp * 2];
	CheackShareEdge(pa, pb, PolyMesh->nvp, ea, eb);

	iArrLeftPoint[0] = PolyMesh->verts[pa[ea] * 3 + 0];
	iArrLeftPoint[1] = 0.0f;
	iArrLeftPoint[2] = PolyMesh->verts[pa[ea] * 3 + 2];
	iArrRightPoint[0] = PolyMesh->verts[pb[eb] * 3 + 0];
	iArrRightPoint[1] = 0.0f;
	iArrRightPoint[2] = PolyMesh->verts[pb[eb] * 3 + 2];

	StartIndex = Path[StartIndex];
	int iLeftIndex = StartIndex, iRightIndex = StartIndex;

	while(Path[StartIndex] != -1)
	{
		//next edge
		pa = &PolyMesh->polys[Path[StartIndex] * PolyMesh->nvp * 2];
		pb = &PolyMesh->polys[StartIndex * PolyMesh->nvp * 2];
		CheackShareEdge(pa, pb, PolyMesh->nvp, ea, eb);
		iArrNewLeftPoint[0] = PolyMesh->verts[pa[ea] * 3 + 0];
		iArrNewLeftPoint[1] = 0.0f;
		iArrNewLeftPoint[2] = PolyMesh->verts[pa[ea] * 3 + 2];
		iArrNewRightPoint[0] = PolyMesh->verts[pb[eb] * 3 + 0];
		iArrNewRightPoint[1] = 0.0f;
		iArrNewRightPoint[2] = PolyMesh->verts[pb[eb] * 3 + 2];
		if( !left(iArrLastEndPoint, iArrLeftPoint, iArrNewLeftPoint)
			&& leftOn(iArrLastEndPoint, iArrRightPoint, iArrNewLeftPoint))
		{
			iArrLeftPoint[0] = iArrNewLeftPoint[0];
			iArrLeftPoint[1] = iArrNewLeftPoint[1];
			iArrLeftPoint[2] = iArrNewLeftPoint[2];
			iLeftIndex = Path[StartIndex];
		}
		else if( !leftOn(iArrLastEndPoint, iArrLeftPoint, iArrNewLeftPoint)
			&& !leftOn(iArrLastEndPoint, iArrRightPoint, iArrNewLeftPoint))
		{
			//Next new point
			NextPoint[0] = iArrRightPoint[0];
			NextPoint[1] = iArrRightPoint[1];
			NextPoint[2] = iArrRightPoint[2];
			//update index
			NextIndex = iRightIndex;
			return true;
		}
		if( !left(iArrLastEndPoint, iArrLeftPoint, iArrNewRightPoint)
			&& leftOn(iArrLastEndPoint, iArrRightPoint, iArrNewRightPoint))
		{
			for(int i = 0; i < 3; i ++)
			{
				iArrRightPoint[i] = iArrNewRightPoint[i];
			}
			iRightIndex = Path[StartIndex];
		}
		else if( left(iArrLastEndPoint, iArrLeftPoint, iArrNewRightPoint)
			&& left(iArrLastEndPoint, iArrRightPoint, iArrNewRightPoint))
		{
			NextPoint[0] = iArrLeftPoint[0];
			NextPoint[1] = iArrLeftPoint[1];
			NextPoint[2] = iArrLeftPoint[2];
			NextIndex = iLeftIndex;
			return true;
		}
		StartIndex = Path[StartIndex];
	}

	//Compare with last point
	if( !left(iArrLastEndPoint, iArrLeftPoint, EndPoint)
		&& leftOn(iArrLastEndPoint, iArrRightPoint, EndPoint))
	{
		NextIndex = EndIndex;
		return false;
	}
	else if( !leftOn(iArrLastEndPoint, iArrLeftPoint, EndPoint)
		&& !leftOn(iArrLastEndPoint, iArrRightPoint, EndPoint))
	{
		NextPoint[0] = iArrRightPoint[0];
		NextPoint[1] = iArrRightPoint[1];
		NextPoint[2] = iArrRightPoint[2];
		NextIndex = iRightIndex;
		return true;
	}
	if( !left(iArrLastEndPoint, iArrLeftPoint, EndPoint)
		&& leftOn(iArrLastEndPoint, iArrRightPoint, EndPoint))
	{
		NextIndex = EndIndex;
		return false;
	}
	else if( left(iArrLastEndPoint, iArrLeftPoint, EndPoint)
		&& left(iArrLastEndPoint, iArrRightPoint, EndPoint))
	{
		NextPoint[0] = iArrLeftPoint[0];
		NextPoint[1] = iArrLeftPoint[1];
		NextPoint[2] = iArrLeftPoint[2];
		NextIndex = iLeftIndex;
		return true;
	}

	return false;
}

//Find best path
bool FindPath(rcPolyMesh *PolyMesh, int **con, vec3 StartPoint, vec3 EndPoint, vector<vec3> &Vect)
{
	int iStart = GetPolyIndex(PolyMesh, StartPoint);
	int iEnd = GetPolyIndex(PolyMesh, EndPoint);
	printf("Start Index Point:  End Index Point: ", iStart, iEnd);
	if (iStart == iEnd)
	{
		Vect.push_back(EndPoint);
		Vect.push_back(StartPoint);
	}
#if 1
	else 
	{
		//Cal path
		int *distance = new int[PolyMesh->npolys];
		int *path = new int[PolyMesh->npolys];
		Dijkstra(con, PolyMesh->npolys, iStart, distance, path);
		//cant reach path
		if (path[iEnd] == -1)
			return false; 
		int temp = iEnd;
		unsigned short *pa, *pb;
		int ea = -1, eb = -1;
		printf("", iEnd, EndPoint.x, 0.0f, EndPoint.z);
		while(path[temp] != -1)
		{
			pa = &PolyMesh->polys[path[temp] * PolyMesh->nvp * 2];
			pb = &PolyMesh->polys[temp * PolyMesh->nvp * 2];
			CheackShareEdge(pa, pb, PolyMesh->nvp, ea, eb);
			int x = PolyMesh->verts[pa[ea] * 3 + 0] + PolyMesh->verts[pb[eb] * 3 + 0]; 
			int z = PolyMesh->verts[pa[ea] * 3 + 2] + PolyMesh->verts[pb[eb] * 3 + 2];
			printf("", temp, x / 2, 0, z / 2);
			printf("", 
				PolyMesh->verts[pa[ea] * 3 + 0], 
				PolyMesh->verts[pa[ea] * 3 + 1],
				PolyMesh->verts[pa[ea] * 3 + 2],
				PolyMesh->verts[pb[eb] * 3 + 0],
				PolyMesh->verts[pb[eb] * 3 + 1],
				PolyMesh->verts[pb[eb] * 3 + 2]
			);
			temp = path[temp];
		}
		printf("", iStart, StartPoint.x, 0.0f, StartPoint.z);
		int iArrStartPoint[3] = {EndPoint.x, EndPoint.y, EndPoint.z};
		int iArrEndPoint[3] = {StartPoint.x, StartPoint.y, StartPoint.z};
		int iArrNextPoint[3] = {0};
		int StartIndex = iEnd;
		int EndIndex = iStart;
		int NextIndex = -1;
		Vect.push_back(EndPoint);
		cout << Vect[Vect.size() - 1] << endl;
		while(path[StartIndex] != -1)
		{
			//Cant find new point
			if (!FindNextPoint(PolyMesh, iArrStartPoint, StartIndex, iArrEndPoint, 
				EndIndex, path, iArrNextPoint, NextIndex))
			{
				break;
			}

			vec3 newPoint(iArrNextPoint[0], iArrNextPoint[1], iArrNextPoint[2]);
			Vect.push_back(newPoint);
			cout << Vect[Vect.size() - 1] << endl;
			//Update start point
			for (int i = 0; i < 3; i ++)
			{
				iArrStartPoint[i] = iArrNextPoint[i];
			}
			StartIndex = NextIndex;
		}
		Vect.push_back(StartPoint);
		cout << Vect[Vect.size() - 1] << endl;

		delete [] path;
		delete [] distance;
	}
#endif

	return true;
}

static int Math_DistPointPoint(int dx, int dy)
{
	if (dx < 0)
	{
		dx = -dx;
	}
	if (dy < 0)
	{
		dy = -dy;
	}

	int min, max;
	if (dx < dy)
	{
		min = dx;
		max = dy;
	}
	else
	{
		min = dy;
		max = dx;
	}
	return ((max << 8) - (max << 3) - (max << 1) + (min << 6) + (min << 5)
		+ (min << 2) + (min << 1)) >> 8;
}

int HeuristicEstimateOfDistance(vec3 vStart, vec3 vEnd)
{
	int dx = vEnd.x - vStart.x;
	int dy = vEnd.z - vStart.z;
	return Math_DistPointPoint(dx, dy);
}

int DistBetween(vec3 a, vec3 b)
{
	int len = sqrt((a.x - b.x)*(a.x - b.x) + (a.z - b.z)*(a.z - b.z));
	return len;
}

struct cmp1{
	bool operator ()(AStarNode &a, AStarNode &b){
		return a.iF_score > b.iF_score;
	}
};

void UpdateFScore(priority_queue<AStarNode, vector<AStarNode>, cmp1> &que, int index, int len)
{
	queue<AStarNode> temp;
	AStarNode tempNode;
	while(!que.empty())
	{
		tempNode = que.top();
		que.pop();
		if (tempNode.iIndex == index)
		{
			tempNode.iF_score = len;
			que.push(tempNode);
			break;
		}
		temp.push(tempNode);
	}
	while(!temp.empty())
	{
		tempNode = temp.front();
		temp.pop();
		que.push(tempNode);
	}
}


//A*  Alg
void AStar(bool **con, int npolys, int iStart, int iEnd, vec3 *centre, int *came_from)
{
	priority_queue<AStarNode, vector<AStarNode>, cmp1> OpenList;      //Open Lish
	int* iG_score = new int[npolys];          //G
	int* iH_score = new int[npolys];          //H
	int* iInList = new int[npolys];      
	AStarNode* node = new AStarNode[npolys];
	AStarNode tempNode;
	//Inital Vertex
	for(int i = 0; i < npolys; i ++)
	{
		node[i].iIndex = i;
		node[i].iF_score = MAXWEIGHT;
		came_from[i] = -1;
		iInList[i] = 0;
	}
	iG_score[iStart] = 0;
	iH_score[iStart] = HeuristicEstimateOfDistance(centre[iStart], centre[iEnd]);
	node[iStart].iF_score = iH_score[iStart];	
	OpenList.push(node[iStart]);

//Not empty open list
	while(!OpenList.empty())
	{
		tempNode = OpenList.top();//First
		iInList[tempNode.iIndex] = -1; //Into close list
		//End point
		if(tempNode.iIndex == iEnd)
		{
			return;
		}
		OpenList.pop();
		//Search cloest edge
		for (int i = 0; i < npolys; i ++)
		{
			if (con[tempNode.iIndex][i] && tempNode.iIndex != i)
			{
				if (iInList[i] == -1)
					continue;
				int tentative_g_score = iG_score[tempNode.iIndex] + 
					DistBetween(centre[tempNode.iIndex], centre[i]);
				if(iInList[i] != 1)
				{
					came_from[i] = tempNode.iIndex; 
					//Update parent node
					iG_score[i] = tentative_g_score;
					iH_score[i] = HeuristicEstimateOfDistance(centre[i], centre[iEnd]);
					AStarNode NewNode;
					NewNode.iIndex = i;
					NewNode.iF_score = iG_score[i] + iH_score[i];
					OpenList.push(NewNode);
					iInList[i] = 1; 
				}
				else if (tentative_g_score < iG_score[i])
				{
					//Update F
					UpdateFScore(OpenList, i, tentative_g_score);
				}
			}
		}
	}
}


//A* Path finding
bool FindPathOfAStar(rcPolyMesh *PolyMesh, bool **con, vec3 *centre, vec3 StartPoint, vec3 EndPoint, vector<vec3> &Vect)
{
	int iStart = GetPolyIndex(PolyMesh, StartPoint);
	int iEnd = GetPolyIndex(PolyMesh, EndPoint);
	printf("  A* Start Index:  End Index: ", iStart, iEnd);
	if (iStart == iEnd)
	{
		Vect.push_back(EndPoint);
		Vect.push_back(StartPoint);
	}
#if 1
	else 
	{
		//Calculate Path
		int *path = new int[PolyMesh->npolys];
		AStar(con, PolyMesh->npolys, iStart, iEnd, centre, path);
		//Can not reach
		if (path[iEnd] == -1)
			return false; 
		int temp = iEnd;
		unsigned short *pa, *pb;
		int ea = -1, eb = -1;
		printf("", iEnd, EndPoint.x, 0.0f, EndPoint.z);
		while(path[temp] != -1)
		{
			pa = &PolyMesh->polys[path[temp] * PolyMesh->nvp * 2];
			pb = &PolyMesh->polys[temp * PolyMesh->nvp * 2];
			CheackShareEdge(pa, pb, PolyMesh->nvp, ea, eb);
			int x = PolyMesh->verts[pa[ea] * 3 + 0] + PolyMesh->verts[pb[eb] * 3 + 0]; 
			int z = PolyMesh->verts[pa[ea] * 3 + 2] + PolyMesh->verts[pb[eb] * 3 + 2];
			printf("", temp, x / 2, 0, z / 2);
			printf("", 
				PolyMesh->verts[pa[ea] * 3 + 0], 
				PolyMesh->verts[pa[ea] * 3 + 1],
				PolyMesh->verts[pa[ea] * 3 + 2],
				PolyMesh->verts[pb[eb] * 3 + 0],
				PolyMesh->verts[pb[eb] * 3 + 1],
				PolyMesh->verts[pb[eb] * 3 + 2]
			);
			temp = path[temp];
		}
		printf("", iStart, StartPoint.x, 0.0f, StartPoint.z);
		int iArrStartPoint[3] = {EndPoint.x, EndPoint.y, EndPoint.z};
		int iArrEndPoint[3] = {StartPoint.x, StartPoint.y, StartPoint.z};
		int iArrNextPoint[3] = {0};
		int StartIndex = iEnd;
		int EndIndex = iStart;
		int NextIndex = -1;
		//End point
		Vect.push_back(EndPoint);
		cout << Vect[Vect.size() - 1] << endl;
		while(path[StartIndex] != -1)
		{
			//Cant find new point
			if (!FindNextPoint(PolyMesh, iArrStartPoint, StartIndex, iArrEndPoint, 
				EndIndex, path, iArrNextPoint, NextIndex))
			{
				break;
			}
			vec3 newPoint(iArrNextPoint[0], iArrNextPoint[1], iArrNextPoint[2]);
			Vect.push_back(newPoint);
			cout << Vect[Vect.size() - 1] << endl;
			//Update for new point
			for (int i = 0; i < 3; i ++)
			{
				iArrStartPoint[i] = iArrNextPoint[i];
			}
			StartIndex = NextIndex;
		}
		Vect.push_back(StartPoint);
		cout << Vect[Vect.size() - 1] << endl;

		delete [] path;
	}
#endif

	return true;
}