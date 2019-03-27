//Zhaoran Wang Head files for Navigation Mesh

#pragma once
#ifndef _NAVMESH_H_
#define _NAVMESH_H_

#include "Recast\Recast.h"
#include "Recast\RecastAlloc.h"
#include "Recast\RecastAssert.h"


struct rcEdge
{
	unsigned short vert[2];
	unsigned short polyEdge[2];
	unsigned short poly[2];
};

bool LoadConSet(fstream *f, rcContourSet *cset, int &sum);
//Number of Vertex
static void pushBack(int v, int* arr, int& an);
static void pushFront(int v, int* arr, int& an);

//Next index
inline int next(int i, int n);
//Last index
inline int prev(int i, int n);
inline int area2(const int* a, const int* b, const int* c);
static bool vequal(const int* a, const int* b);

//Number of vertex
int countPolyVerts(const unsigned short* p, const int nvp);
inline bool xorb(bool x, bool y);
inline bool uleft(const unsigned short* a, const unsigned short* b, const unsigned short* c);
static bool between(const int* a, const int* b, const int* c);
static int getPolyMergeValue(unsigned short* pa, unsigned short* pb,
							 const unsigned short* verts, int& ea, int& eb,
							 const int nvp);
static void mergePolys(unsigned short* pa, unsigned short* pb, int ea, int eb,
					   unsigned short* tmp, const int nvp);


//Cal Hash
inline int computeVertexHash(int x, int y, int z);

//Add vertex
static unsigned short addVertex(unsigned short x, unsigned short y, unsigned short z,
								unsigned short* verts, int* firstVert, int* nextVert, int& nv);


static bool buildMeshAdjacency(unsigned short* polys, const int npolys,
							   const int nverts, const int vertsPerPoly);

static bool intersect(const int* a, const int* b, const int* c, const int* d);

inline bool left(const int* a, const int* b, const int* c);
inline bool leftOn(const int* a, const int* b, const int* c);
inline bool collinear(const int* a, const int* b, const int* c);
static bool intersectProp(const int* a, const int* b, const int* c, const int* d);
static bool diagonal(int i, int j, int n, const int* verts, int* indices);
static bool	inCone(int i, int j, int n, const int* verts, int* indices);
static bool diagonalie(int i, int j, int n, const int* verts, int* indices);

//remove index vertext
static bool canRemoveVertex(rcPolyMesh& mesh, const unsigned short rem);

static bool removeVertex(rcPolyMesh& mesh, const unsigned short rem, const int maxTris);

//create mesh poly
bool BuildPolyMesh(rcContourSet& cset, const int nvp, rcPolyMesh& mesh);

static int triangulate(int n, const int* verts, int* indices, int* tris);

//out put mesh
void OutputPolyMesh(fstream *f, rcPolyMesh *rcPmesh, int &iSumOfIndexNum);

#endif