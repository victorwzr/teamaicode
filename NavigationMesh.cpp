//Main class for implementation of Navigation Mesh system based on recast and detour
//Origin code ref to recast and detour
//Zhaoran Wang

#pragma once
#include "NavigationMesh.h"

static const int VERTEX_BUCKET_COUNT = (1<<12);

//read in Con Set
bool LoadConSet(fstream *f, rcContourSet *cset, int &sum)
{
	int maxContours;
	sum = 0;
	(*f) >> maxContours;
	cset->conts = (rcContour*)rcAlloc(sizeof(rcContour)*maxContours, RC_ALLOC_PERM);
	if (!cset->conts)
		return false;
	cset->nconts = 0;
	for (int i = 0; i < maxContours; i ++)
	{
		rcContour* cont = &cset->conts[cset->nconts++];
		(*f) >> cont->nverts;
		sum += cont->nverts;
		cont->verts = (int*)rcAlloc(sizeof(int)*cont->nverts*4, RC_ALLOC_PERM);
		if (!cont->verts)
		{
			printf("rcBuildContours: Out of memory 'verts' (%d).", cont->nverts);
			return false;
		}

	  //Vertex information
		for (int j = 0; j < cont->nverts*4; j ++)
		{
			(*f) >> cont->verts[j];
		}
		cont->reg = i;
		cont->area = i;
	}
	return true;
}

//Get Poly vertex number
int countPolyVerts(const unsigned short* p, const int nvp)
{
	for (int i = 0; i < nvp; ++i)
		if (p[i] == RC_MESH_NULL_IDX)
			return i;
	return nvp;
}

static void pushBack(int v, int* arr, int& an)
{
	arr[an] = v;
	an++;
}

static void pushFront(int v, int* arr, int& an)
{
	an++;
	for (int i = an-1; i > 0; --i) arr[i] = arr[i-1];
	arr[0] = v;
}

//Calculation

inline int next(int i, int n) { return i+1 < n ? i+1 : 0; }
inline int prev(int i, int n) { return i-1 >= 0 ? i-1 : n-1; }

//product
inline int area2(const int* a, const int* b, const int* c)
{
	return (b[0] - a[0]) * (c[2] - a[2]) - (c[0] - a[0]) * (b[2] - a[2]);
}

inline bool left(const int* a, const int* b, const int* c)
{
	return area2(a, b, c) < 0;
}


inline bool leftOn(const int* a, const int* b, const int* c)
{
	return area2(a, b, c) <= 0;
}

inline bool collinear(const int* a, const int* b, const int* c)
{
	return area2(a, b, c) == 0;
}

static bool vequal(const int* a, const int* b)
{
	return a[0] == b[0] && a[2] == b[2];
}

inline bool xorb(bool x, bool y)
{
	return !x ^ !y;
}

inline bool uleft(const unsigned short* a, const unsigned short* b, const unsigned short* c)
{
	return ((int)b[0] - (int)a[0]) * ((int)c[2] - (int)a[2]) -
		((int)c[0] - (int)a[0]) * ((int)b[2] - (int)a[2]) < 0;
}

static bool between(const int* a, const int* b, const int* c)
{
	if (!collinear(a, b, c))
		return false;

	if (a[0] != b[0])
		return	((a[0] <= c[0]) && (c[0] <= b[0])) || ((a[0] >= c[0]) && (c[0] >= b[0]));
	else
		return	((a[2] <= c[2]) && (c[2] <= b[2])) || ((a[2] >= c[2]) && (c[2] >= b[2]));
}



static int getPolyMergeValue(unsigned short* pa, unsigned short* pb,
							 const unsigned short* verts, int& ea, int& eb,
							 const int nvp)
{
	//Calculate Vertex number
	const int na = countPolyVerts(pa, nvp);
	const int nb = countPolyVerts(pb, nvp);

	//If the merged polygon would be too big do not merge.
	if (na+nb-2 > nvp)
		return -1;

	//Check polygons share an edge.
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

	//No common edge
	if (ea == -1 || eb == -1)
		return -1;

	//Check if the merged polygon convex.
	unsigned short va, vb, vc;
	va = pa[(ea+na-1) % na];
	vb = pa[ea];
	vc = pb[(eb+2) % nb];
	if (!uleft(&verts[va*3], &verts[vb*3], &verts[vc*3]))
		return -1;
	va = pb[(eb+nb-1) % nb];
	vb = pb[eb];
	vc = pa[(ea+2) % na];
	if (!uleft(&verts[va*3], &verts[vb*3], &verts[vc*3]))
		return -1;
	va = pa[ea];
	vb = pa[(ea+1)%na];
	int dx = (int)verts[va*3+0] - (int)verts[vb*3+0];
	int dy = (int)verts[va*3+2] - (int)verts[vb*3+2];
	return dx*dx + dy*dy;
}

static void mergePolys(unsigned short* pa, unsigned short* pb, int ea, int eb,
					   unsigned short* tmp, const int nvp)
{
	//Poly vertex number
	const int na = countPolyVerts(pa, nvp);
	const int nb = countPolyVerts(pb, nvp);

	//Merge polygons.
	memset(tmp, 0xff, sizeof(unsigned short)*nvp);
	int n = 0;
	//Add pa
	for (int i = 0; i < na-1; ++i)
		tmp[n++] = pa[(ea+1+i) % na];
	//Add pb
	for (int i = 0; i < nb-1; ++i)
		tmp[n++] = pb[(eb+1+i) % nb];
	memcpy(pa, tmp, sizeof(unsigned short)*nvp);
}


//Vertex hash
inline int computeVertexHash(int x, int y, int z)
{
	const unsigned int h1 = 0x8da6b343;
	const unsigned int h2 = 0xd8163841;
	const unsigned int h3 = 0xcb1ab31f;
	unsigned int n = h1 * x + h2 * y + h3 * z;
	return (int)(n & (VERTEX_BUCKET_COUNT-1));
}

//Add Vertex
static unsigned short addVertex(unsigned short x, unsigned short y, unsigned short z,
								unsigned short* verts, int* firstVert, int* nextVert, int& nv)
{
	int bucket = computeVertexHash(x, 0, z);
	int i = firstVert[bucket];

	while (i != -1)
	{
		const unsigned short* v = &verts[i*3];
		if (v[0] == x && (rcAbs(v[1] - y) <= 2) && v[2] == z)
			return (unsigned short)i; 
		i = nextVert[i];
	}

	i = nv; nv++;
	unsigned short* v = &verts[i*3];
	v[0] = x;
	v[1] = y;
	v[2] = z;
	nextVert[i] = firstVert[bucket];
	firstVert[bucket] = i;
	return (unsigned short)i;
}


static bool buildMeshAdjacency(unsigned short* polys, const int npolys,
							   const int nverts, const int vertsPerPoly)
{
	//Based on code by Eric Lengyel from:
	int maxEdgeCount = npolys*vertsPerPoly;
	unsigned short* firstEdge = (unsigned short*)rcAlloc(sizeof(unsigned short)*(nverts + maxEdgeCount), RC_ALLOC_TEMP);
	if (!firstEdge)
		return false;
	unsigned short* nextEdge = firstEdge + nverts;
	int edgeCount = 0;

	rcEdge* edges = (rcEdge*)rcAlloc(sizeof(rcEdge)*maxEdgeCount, RC_ALLOC_TEMP);
	if (!edges)
	{
		rcFree(firstEdge);
		return false;
	}

	for (int i = 0; i < nverts; i++)
		firstEdge[i] = RC_MESH_NULL_IDX;

	for (int i = 0; i < npolys; ++i)
	{
		unsigned short* t = &polys[i*vertsPerPoly*2];
		for (int j = 0; j < vertsPerPoly; ++j)
		{
			if (t[j] == RC_MESH_NULL_IDX) break;
			unsigned short v0 = t[j];
			unsigned short v1 = (j+1 >= vertsPerPoly || t[j+1] == RC_MESH_NULL_IDX) ? t[0] : t[j+1];
			if (v0 < v1)
			{
				rcEdge& edge = edges[edgeCount];
				edge.vert[0] = v0;
				edge.vert[1] = v1;
				edge.poly[0] = (unsigned short)i;
				edge.polyEdge[0] = (unsigned short)j;
				edge.poly[1] = (unsigned short)i;
				edge.polyEdge[1] = 0;
				//Insert edge
				nextEdge[edgeCount] = firstEdge[v0];
				firstEdge[v0] = (unsigned short)edgeCount;
				edgeCount++;
			}
		}
	}

	for (int i = 0; i < npolys; ++i)
	{
		unsigned short* t = &polys[i*vertsPerPoly*2];
		for (int j = 0; j < vertsPerPoly; ++j)
		{
			if (t[j] == RC_MESH_NULL_IDX) break;
			unsigned short v0 = t[j];
			unsigned short v1 = (j+1 >= vertsPerPoly || t[j+1] == RC_MESH_NULL_IDX) ? t[0] : t[j+1];
			if (v0 > v1)
			{
				for (unsigned short e = firstEdge[v1]; e != RC_MESH_NULL_IDX; e = nextEdge[e])
				{
					rcEdge& edge = edges[e];
					if (edge.vert[1] == v0 && edge.poly[0] == edge.poly[1])
					{
						edge.poly[1] = (unsigned short)i;
						edge.polyEdge[1] = (unsigned short)j;
						break;
					}
				}
			}
		}
	}

	//Store adjacency
	for (int i = 0; i < edgeCount; ++i)
	{
		const rcEdge& e = edges[i];
		if (e.poly[0] != e.poly[1])
		{
			unsigned short* p0 = &polys[e.poly[0]*vertsPerPoly*2];
			unsigned short* p1 = &polys[e.poly[1]*vertsPerPoly*2];
			p0[vertsPerPoly + e.polyEdge[0]] = e.poly[1];
			p1[vertsPerPoly + e.polyEdge[1]] = e.poly[0];
		}
	}

	rcFree(firstEdge);
	rcFree(edges);

	return true;
}


//ab prop intersects cd
static bool intersectProp(const int* a, const int* b, const int* c, const int* d)
{
	if (collinear(a,b,c) || collinear(a,b,d) ||
		collinear(c,d,a) || collinear(c,d,b))
		return false;
	return xorb(left(a,b,c), left(a,b,d)) && xorb(left(c,d,a), left(c,d,b));
}

//ab and cd intersect
static bool intersect(const int* a, const int* b, const int* c, const int* d)
{
	if (intersectProp(a, b, c, d))
		return true;
	else if (between(a, b, c) || between(a, b, d) ||
		between(c, d, a) || between(c, d, b))
		return true;
	else
		return false;
}


//(v_i, v_j) is a proper internal

static bool diagonal(int i, int j, int n, const int* verts, int* indices)
{
	return inCone(i, j, n, verts, indices) && diagonalie(i, j, n, verts, indices);
}

static bool	inCone(int i, int j, int n, const int* verts, int* indices)
{
	const int* pi = &verts[(indices[i] & 0x0fffffff) * 4];
	const int* pj = &verts[(indices[j] & 0x0fffffff) * 4];
	const int* pi1 = &verts[(indices[next(i, n)] & 0x0fffffff) * 4];
	const int* pin1 = &verts[(indices[prev(i, n)] & 0x0fffffff) * 4];

	if (leftOn(pin1, pi, pi1))
		return left(pi, pj, pin1) && left(pj, pi, pi1);
	return !(leftOn(pi, pj, pi1) && leftOn(pj, pi, pin1));
}

static bool diagonalie(int i, int j, int n, const int* verts, int* indices)
{
	const int* d0 = &verts[(indices[i] & 0x0fffffff) * 4];
	const int* d1 = &verts[(indices[j] & 0x0fffffff) * 4];

	// For each edge
	for (int k = 0; k < n; k++)
	{
		int k1 = next(k, n);//next node
		// Skip edges
		if (!((k == i) || (k1 == i) || (k == j) || (k1 == j)))
		{
			const int* p0 = &verts[(indices[k] & 0x0fffffff) * 4];
			const int* p1 = &verts[(indices[k1] & 0x0fffffff) * 4];
			if (vequal(d0, p0) || vequal(d1, p0) || vequal(d0, p1) || vequal(d1, p1))
				continue;
			if (intersect(d0, d1, p0, p1))
				return false;
		}
	}
	return true;
}

static int triangulate(int n, const int* verts, int* indices, int* tris)
{
	int ntris = 0;
	int* dst = tris;
	
	for (int i = 0; i < n; i++)
	{
		int i1 = next(i, n); //next node
		int i2 = next(i1, n);
		if (diagonal(i, i2, n, verts, indices))
			indices[i1] |= 0x80000000;
	}
	
	while (n > 3)
	{
		int minLen = -1;
		int mini = -1;
		for (int i = 0; i < n; i++)
		{
			int i1 = next(i, n);
			if (indices[i1] & 0x80000000)
			{
				const int* p0 = &verts[(indices[i] & 0x0fffffff) * 4];
				const int* p2 = &verts[(indices[next(i1, n)] & 0x0fffffff) * 4];
				int dx = p2[0] - p0[0];
				int dy = p2[2] - p0[2];
				int len = dx*dx + dy*dy;
				
				//smallest point
				if (minLen < 0 || len < minLen)
				{
					minLen = len;
					mini = i;
				}
			}
		}
		
		if (mini == -1)
		{
			return -ntris;
		}
		
		int i = mini;
		int i1 = next(i, n);
		int i2 = next(i1, n);

		*dst++ = indices[i] & 0x0fffffff;
		*dst++ = indices[i1] & 0x0fffffff;
		*dst++ = indices[i2] & 0x0fffffff;
		ntris++;
		n--;
		for (int k = i1; k < n; k++)
			indices[k] = indices[k+1];
		
		if (i1 >= n) i1 = 0;
		i = prev(i1,n);
		if (diagonal(prev(i, n), i1, n, verts, indices))
			indices[i] |= 0x80000000;
		else
			indices[i] &= 0x0fffffff;
		if (diagonal(i, next(i1, n), n, verts, indices))
			indices[i1] |= 0x80000000;
		else
			indices[i1] &= 0x0fffffff;
	}
	*dst++ = indices[0] & 0x0fffffff;
	*dst++ = indices[1] & 0x0fffffff;
	*dst++ = indices[2] & 0x0fffffff;
	ntris++;
	
	return ntris;
}

//remove vertex
static bool canRemoveVertex(rcPolyMesh& mesh, const unsigned short rem)
{
	const int nvp = mesh.nvp;
	//Number to remove
	int numRemovedVerts = 0;
	int numTouchedVerts = 0;
	int numRemainingEdges = 0;
	for (int i = 0; i < mesh.npolys; ++i)
	{
		unsigned short* p = &mesh.polys[i*nvp*2];
		//total point
		const int nv = countPolyVerts(p, nvp);
		int numRemoved = 0;
		int numVerts = 0;
		for (int j = 0; j < nv; ++j)
		{
			if (p[j] == rem)
			{
				numTouchedVerts++;
				numRemoved++;
			}
			numVerts++;
		}
		if (numRemoved)
		{
			numRemovedVerts += numRemoved;
			numRemainingEdges += numVerts-(numRemoved+1);
		}
	}
	if (numRemainingEdges <= 2)
		return false;

	const int maxEdges = numTouchedVerts*2;
	int nedges = 0;
	rcScopedDelete<int> edges = (int*)rcAlloc(sizeof(int)*maxEdges*3, RC_ALLOC_TEMP);
	if (!edges)
	{
		return false;
	}

	for (int i = 0; i < mesh.npolys; ++i)
	{
		unsigned short* p = &mesh.polys[i*nvp*2];
		const int nv = countPolyVerts(p, nvp);
		for (int j = 0, k = nv-1; j < nv; k = j++)
		{
			if (p[j] == rem || p[k] == rem)
			{

				int a = p[j], b = p[k];
				if (b == rem)
					rcSwap(a,b);
				bool exists = false;
				for (int m = 0; m < nedges; ++m)
				{
					int* e = &edges[m*3];
					if (e[1] == b)
					{
						e[2]++;
						exists = true;
					}
				}
				if (!exists)
				{
					int* e = &edges[nedges*3];
					e[0] = a;
					e[1] = b;
					e[2] = 1;
					nedges++;
				}
			}
		}
	}
	int numOpenEdges = 0;
	for (int i = 0; i < nedges; ++i)
	{
		if (edges[i*3+2] < 2)
			numOpenEdges++;
	}
	if (numOpenEdges > 2)
		return false;

	return true;
}

static bool removeVertex(rcPolyMesh& mesh, const unsigned short rem, const int maxTris)
{
	const int nvp = mesh.nvp;
	int numRemovedVerts = 0;
	for (int i = 0; i < mesh.npolys; ++i)
	{
		unsigned short* p = &mesh.polys[i*nvp*2];
		const int nv = countPolyVerts(p, nvp);
		for (int j = 0; j < nv; ++j)
		{
			if (p[j] == rem)
				numRemovedVerts++;
		}
	}

	int nedges = 0;
	rcScopedDelete<int> edges = (int*)rcAlloc(sizeof(int)*numRemovedVerts*nvp*4, RC_ALLOC_TEMP);
	if (!edges)
	{
		return false;
	}

	int nhole = 0;
	rcScopedDelete<int> hole = (int*)rcAlloc(sizeof(int)*numRemovedVerts*nvp, RC_ALLOC_TEMP);
	if (!hole)
	{
		return false;
	}

	int nhreg = 0;
	rcScopedDelete<int> hreg = (int*)rcAlloc(sizeof(int)*numRemovedVerts*nvp, RC_ALLOC_TEMP);
	if (!hreg)
	{
		return false;
	}

	int nharea = 0;
	rcScopedDelete<int> harea = (int*)rcAlloc(sizeof(int)*numRemovedVerts*nvp, RC_ALLOC_TEMP);
	if (!harea)
	{
		return false;
	}

	for (int i = 0; i < mesh.npolys; ++i)
	{
		unsigned short* p = &mesh.polys[i*nvp*2];
		const int nv = countPolyVerts(p, nvp);
		bool hasRem = false;
		for (int j = 0; j < nv; ++j)
		{
			if (p[j] == rem) hasRem = true;
		}
		if (hasRem)
		{
			for (int j = 0, k = nv-1; j < nv; k = j++)
			{
				if (p[j] != rem && p[k] != rem)
				{
					int* e = &edges[nedges*4];
					e[0] = p[k];
					e[1] = p[j];
					e[2] = mesh.regs[i];
					e[3] = mesh.areas[i];
					nedges++;
				}
			}
			// Remove the polygon.
			unsigned short* p2 = &mesh.polys[(mesh.npolys-1)*nvp*2];
			memcpy(p,p2,sizeof(unsigned short)*nvp);
			memset(p+nvp,0xff,sizeof(unsigned short)*nvp);
			mesh.regs[i] = mesh.regs[mesh.npolys-1];
			mesh.areas[i] = mesh.areas[mesh.npolys-1];
			mesh.npolys--;
			--i;
		}
	}

	// Remove vertex.
	for (int i = (int)rem; i < mesh.nverts; ++i)
	{
		mesh.verts[i*3+0] = mesh.verts[(i+1)*3+0];
		mesh.verts[i*3+1] = mesh.verts[(i+1)*3+1];
		mesh.verts[i*3+2] = mesh.verts[(i+1)*3+2];
	}
	mesh.nverts--;

	//Adjust indices to match the removed vertex layout.
	for (int i = 0; i < mesh.npolys; ++i)
	{
		unsigned short* p = &mesh.polys[i*nvp*2];
		const int nv = countPolyVerts(p, nvp);
		for (int j = 0; j < nv; ++j)
			if (p[j] > rem) p[j]--;
	}
	for (int i = 0; i < nedges; ++i)
	{
		if (edges[i*4+0] > rem) edges[i*4+0]--;
		if (edges[i*4+1] > rem) edges[i*4+1]--;
	}

	if (nedges == 0)
		return true;
	pushBack(edges[0], hole, nhole);
	pushBack(edges[2], hreg, nhreg);
	pushBack(edges[3], harea, nharea);

	while (nedges)
	{
		bool match = false;

		for (int i = 0; i < nedges; ++i)
		{
			const int ea = edges[i*4+0];
			const int eb = edges[i*4+1];
			const int r = edges[i*4+2];
			const int a = edges[i*4+3];
			bool add = false;
			if (hole[0] == eb)
			{
				pushFront(ea, hole, nhole);
				pushFront(r, hreg, nhreg);
				pushFront(a, harea, nharea);
				add = true;
			}
			else if (hole[nhole-1] == ea)
			{
				pushBack(eb, hole, nhole);
				pushBack(r, hreg, nhreg);
				pushBack(a, harea, nharea);
				add = true;
			}
			if (add)
			{
				edges[i*4+0] = edges[(nedges-1)*4+0];
				edges[i*4+1] = edges[(nedges-1)*4+1];
				edges[i*4+2] = edges[(nedges-1)*4+2];
				edges[i*4+3] = edges[(nedges-1)*4+3];
				--nedges;
				match = true;
				--i;
			}
		}

		if (!match)
			break;
	}

	rcScopedDelete<int> tris = (int*)rcAlloc(sizeof(int)*nhole*3, RC_ALLOC_TEMP);
	if (!tris)
	{
		return false;
	}

	rcScopedDelete<int> tverts = (int*)rcAlloc(sizeof(int)*nhole*4, RC_ALLOC_TEMP);
	if (!tverts)
	{
		return false;
	}

	rcScopedDelete<int> thole = (int*)rcAlloc(sizeof(int)*nhole, RC_ALLOC_TEMP);
	if (!tverts)
	{
		return false;
	}

	for (int i = 0; i < nhole; ++i)
	{
		const int pi = hole[i];
		tverts[i*4+0] = mesh.verts[pi*3+0];
		tverts[i*4+1] = mesh.verts[pi*3+1];
		tverts[i*4+2] = mesh.verts[pi*3+2];
		tverts[i*4+3] = 0;
		thole[i] = i;
	}

	int ntris = triangulate(nhole, &tverts[0], &thole[0], tris);
	if (ntris < 0)
	{
		ntris = -ntris;
	}

	rcScopedDelete<unsigned short> polys = (unsigned short*)rcAlloc(sizeof(unsigned short)*(ntris+1)*nvp, RC_ALLOC_TEMP);
	if (!polys)
	{
		return false;
	}
	rcScopedDelete<unsigned short> pregs = (unsigned short*)rcAlloc(sizeof(unsigned short)*ntris, RC_ALLOC_TEMP);
	if (!pregs)
	{
		return false;
	}
	rcScopedDelete<unsigned char> pareas = (unsigned char*)rcAlloc(sizeof(unsigned char)*ntris, RC_ALLOC_TEMP);
	if (!pregs)
	{
		return false;
	}

	unsigned short* tmpPoly = &polys[ntris*nvp];
	int npolys = 0;
	memset(polys, 0xff, ntris*nvp*sizeof(unsigned short));
	for (int j = 0; j < ntris; ++j)
	{
		int* t = &tris[j*3];
		if (t[0] != t[1] && t[0] != t[2] && t[1] != t[2])
		{
			polys[npolys*nvp+0] = (unsigned short)hole[t[0]];
			polys[npolys*nvp+1] = (unsigned short)hole[t[1]];
			polys[npolys*nvp+2] = (unsigned short)hole[t[2]];
			pregs[npolys] = (unsigned short)hreg[t[0]];
			pareas[npolys] = (unsigned char)harea[t[0]];
			npolys++;
		}
	}
	if (!npolys)
		return true;

	//Combine polygons.
	if (nvp > 3)
	{
		for (;;)
		{
			int bestMergeVal = 0;
			int bestPa = 0, bestPb = 0, bestEa = 0, bestEb = 0;

			for (int j = 0; j < npolys-1; ++j)
			{
				unsigned short* pj = &polys[j*nvp];
				for (int k = j+1; k < npolys; ++k)
				{
					unsigned short* pk = &polys[k*nvp];
					int ea, eb;
					int v = getPolyMergeValue(pj, pk, mesh.verts, ea, eb, nvp);
					if (v > bestMergeVal)
					{
						bestMergeVal = v;
						bestPa = j;
						bestPb = k;
						bestEa = ea;
						bestEb = eb;
					}
				}
			}

			if (bestMergeVal > 0)
			{
				unsigned short* pa = &polys[bestPa*nvp];
				unsigned short* pb = &polys[bestPb*nvp];
				mergePolys(pa, pb, bestEa, bestEb, tmpPoly, nvp);
				memcpy(pb, &polys[(npolys-1)*nvp], sizeof(unsigned short)*nvp);
				pregs[bestPb] = pregs[npolys-1];
				pareas[bestPb] = pareas[npolys-1];
				npolys--;
			}
			else
			{
				break;
			}
		}
	}
	for (int i = 0; i < npolys; ++i)
	{
		if (mesh.npolys >= maxTris) break;
		unsigned short* p = &mesh.polys[mesh.npolys*nvp*2];
		memset(p,0xff,sizeof(unsigned short)*nvp*2);
		for (int j = 0; j < nvp; ++j)
			p[j] = polys[i*nvp+j];
		mesh.regs[mesh.npolys] = pregs[i];
		mesh.areas[mesh.npolys] = pareas[i];
		mesh.npolys++;
		if (mesh.npolys > maxTris)
		{
			return false;
		}
	}

	return true;
}

//Max Poly Mesh
bool BuildPolyMesh(rcContourSet& cset, const int nvp, rcPolyMesh& mesh)
{
	rcVcopy(mesh.bmin, cset.bmin);
	rcVcopy(mesh.bmax, cset.bmax);
	mesh.cs = cset.cs;
	mesh.ch = cset.ch;
	mesh.borderSize = cset.borderSize;
	
	int maxVertices = 0;
	int maxTris = 0;
	int maxVertsPerCont = 0;
	//Cal contours number
	for (int i = 0; i < cset.nconts; ++i)
	{
		//Skip
		if (cset.conts[i].nverts < 3) continue;
		maxVertices += cset.conts[i].nverts;
		//Cal tri number
		maxTris += cset.conts[i].nverts - 2; 
		//Cal Max vertex for one
		maxVertsPerCont = rcMax(maxVertsPerCont, cset.conts[i].nverts);
	}
	
	if (maxVertices >= 0xfffe)//Max value
	{
		return false;
	}
		
	rcScopedDelete<unsigned char> vflags = (unsigned char*)rcAlloc(sizeof(unsigned char)*maxVertices, RC_ALLOC_TEMP);
	if (!vflags)
	{
		return false;
	}
	memset(vflags, 0, maxVertices);
	

	mesh.verts = (unsigned short*)rcAlloc(sizeof(unsigned short)*maxVertices*3, RC_ALLOC_PERM);
	if (!mesh.verts)
	{
		return false;
	}
	mesh.polys = (unsigned short*)rcAlloc(sizeof(unsigned short)*maxTris*nvp*2, RC_ALLOC_PERM);
	if (!mesh.polys)
	{
		return false;
	}
	mesh.regs = (unsigned short*)rcAlloc(sizeof(unsigned short)*maxTris, RC_ALLOC_PERM);
	if (!mesh.regs)
	{
		return false;
	}
	mesh.areas = (unsigned char*)rcAlloc(sizeof(unsigned char)*maxTris, RC_ALLOC_PERM);
	if (!mesh.areas)
	{

		return false;
	}
	
	mesh.nverts = 0;
	mesh.npolys = 0;
	mesh.nvp = nvp;
	mesh.maxpolys = maxTris;
	memset(mesh.verts, 0, sizeof(unsigned short)*maxVertices*3);
	memset(mesh.polys, 0xff, sizeof(unsigned short)*maxTris*nvp*2);
	memset(mesh.regs, 0, sizeof(unsigned short)*maxTris);
	memset(mesh.areas, 0, sizeof(unsigned char)*maxTris);
	
	rcScopedDelete<int> nextVert = (int*)rcAlloc(sizeof(int)*maxVertices, RC_ALLOC_TEMP);
	if (!nextVert)
	{
		return false;
	}
	memset(nextVert, 0, sizeof(int)*maxVertices);

	rcScopedDelete<int> firstVert = (int*)rcAlloc(sizeof(int)*VERTEX_BUCKET_COUNT, RC_ALLOC_TEMP);
	if (!firstVert)
	{
		return false;
	}
	for (int i = 0; i < VERTEX_BUCKET_COUNT; ++i)
		firstVert[i] = -1;
	
	rcScopedDelete<int> indices = (int*)rcAlloc(sizeof(int)*maxVertsPerCont, RC_ALLOC_TEMP);
	if (!indices)
	{
		return false;
	}
	rcScopedDelete<int> tris = (int*)rcAlloc(sizeof(int)*maxVertsPerCont*3, RC_ALLOC_TEMP);
	if (!tris)
	{
		return false;
	}
	rcScopedDelete<unsigned short> polys = (unsigned short*)rcAlloc(sizeof(unsigned short)*(maxVertsPerCont+1)*nvp, RC_ALLOC_TEMP);
	if (!polys)
	{
		return false;
	}
	unsigned short* tmpPoly = &polys[maxVertsPerCont*nvp];

	for (int i = 0; i < cset.nconts; ++i)
	{
		rcContour& cont = cset.conts[i];
		if (cont.nverts < 3)
			continue;
		for (int j = 0; j < cont.nverts; ++j)
			indices[j] = j;
			
		//CAl tri number
		int ntris = triangulate(cont.nverts, cont.verts, &indices[0], &tris[0]);
		if (ntris <= 0)
		{
			ntris = -ntris;
		}
				
		//Add vertices.
		for (int j = 0; j < cont.nverts; ++j)
		{
			const int* v = &cont.verts[j*4];
			indices[j] = addVertex((unsigned short)v[0], (unsigned short)v[1], (unsigned short)v[2],
								   mesh.verts, firstVert, nextVert, mesh.nverts);
			if (v[3] & RC_BORDER_VERTEX)
			{
				vflags[indices[j]] = 1;
			}
		}

		int npolys = 0;
		memset(polys, 0xff, maxVertsPerCont*nvp*sizeof(unsigned short));

		for (int j = 0; j < ntris; ++j)
		{
			int* t = &tris[j*3];
			if (t[0] != t[1] && t[0] != t[2] && t[1] != t[2])
			{
				polys[npolys*nvp+0] = (unsigned short)indices[t[0]];
				polys[npolys*nvp+1] = (unsigned short)indices[t[1]];
				polys[npolys*nvp+2] = (unsigned short)indices[t[2]];
				npolys++;
			}
		}
		if (!npolys)
			continue;



#if 0  //debug
		fstream myf;
		myf.open("polys_test.txt", ios::out);
		for (int i = 0; i < npolys; i ++)
		{
			myf << polys[i*nvp+0] << " " << polys[i*nvp+1] 
			<< " " << polys[i*nvp+2] << endl;
		}
		myf.close();
		
#endif
   	//Merge polygons.
		if (nvp > 3)
		{
			for(;;)
			{
				int bestMergeVal = 0;
				int bestPa = 0, bestPb = 0, bestEa = 0, bestEb = 0;
				
				for (int j = 0; j < npolys-1; ++j)
				{
					unsigned short* pj = &polys[j*nvp];
					for (int k = j+1; k < npolys; ++k)
					{
						unsigned short* pk = &polys[k*nvp];
						int ea, eb;
						int v = getPolyMergeValue(pj, pk, mesh.verts, ea, eb, nvp);
						if (v > bestMergeVal)
						{
							bestMergeVal = v;
							bestPa = j;
							bestPb = k;
							bestEa = ea;
							bestEb = eb;
						}
					}
				}
				
				if (bestMergeVal > 0)
				{
					unsigned short* pa = &polys[bestPa*nvp];
					unsigned short* pb = &polys[bestPb*nvp];
					mergePolys(pa, pb, bestEa, bestEb, tmpPoly, nvp);
					unsigned short* lastPoly = &polys[(npolys-1)*nvp];
					if (pb != lastPoly)
						memcpy(pb, lastPoly, sizeof(unsigned short)*nvp);
					npolys--;
				}
				else
				{
					break;
				}
			}
		}
		
		//Store polygons.
		for (int j = 0; j < npolys; ++j)
		{
			unsigned short* p = &mesh.polys[mesh.npolys*nvp*2];
			unsigned short* q = &polys[j*nvp];
			for (int k = 0; k < nvp; ++k)
				p[k] = q[k];
			mesh.regs[mesh.npolys] = cont.reg;
			mesh.areas[mesh.npolys] = cont.area;
			mesh.npolys++;
			if (mesh.npolys > maxTris)
			{
				return false;
			}
		}

#if 0//debug
	myf.open("Save_Polys_test.txt", ios::out);
	for (int i = 0; i < mesh.npolys; i ++)
	{
		for (int j = 0; j < nvp; j ++)
		{
			myf << mesh.polys[i * nvp + j] << " ";
		}
		myf << endl;
	}
	myf.close();

#endif

	}
	//Remove edge vertices.
	for (int i = 0; i < mesh.nverts; ++i)
	{
		if (vflags[i])
		{
			if (!canRemoveVertex(mesh, (unsigned short)i))
				continue;
			if (!removeVertex(mesh, (unsigned short)i, maxTris))
			{
				return false;
			}
			for (int j = i; j < mesh.nverts; ++j)
				vflags[j] = vflags[j+1];
			--i;
		}
	}
	
	//Calculate adjacency.
	if (!buildMeshAdjacency(mesh.polys, mesh.npolys, mesh.nverts, nvp))
	{
		return false;
	}
	if (mesh.borderSize > 0)
	{
		const int w = cset.width;
		const int h = cset.height;
		for (int i = 0; i < mesh.npolys; ++i)
		{
			unsigned short* p = &mesh.polys[i*2*nvp];
			for (int j = 0; j < nvp; ++j)
			{
				if (p[j] == RC_MESH_NULL_IDX) break;
				if (p[nvp+j] != RC_MESH_NULL_IDX)
					continue;
				int nj = j+1;
				if (nj >= nvp || p[nj] == RC_MESH_NULL_IDX) nj = 0;
				const unsigned short* va = &mesh.verts[p[j]*3];
				const unsigned short* vb = &mesh.verts[p[nj]*3];

				if ((int)va[0] == 0 && (int)vb[0] == 0)
					p[nvp+j] = 0x8000 | 0;
				else if ((int)va[2] == h && (int)vb[2] == h)
					p[nvp+j] = 0x8000 | 1;
				else if ((int)va[0] == w && (int)vb[0] == w)
					p[nvp+j] = 0x8000 | 2;
				else if ((int)va[2] == 0 && (int)vb[2] == 0)
					p[nvp+j] = 0x8000 | 3;
			}
		}
	}

	mesh.flags = (unsigned short*)rcAlloc(sizeof(unsigned short)*mesh.npolys, RC_ALLOC_PERM);
	if (!mesh.flags)
	{
		return false;
	}
	memset(mesh.flags, 0, sizeof(unsigned short) * mesh.npolys);
	
	if (mesh.nverts > 0xffff)
	{
	}
	if (mesh.npolys > 0xffff)
	{
	}

	return true;
}

//Output poly mesh
#if 1
void OutputPolyMesh(fstream *f, rcPolyMesh *rcPmesh, int &iSumOfIndexNum)
{
	(*f) << rcPmesh->nverts <<endl;
	for (int i = 0; i < rcPmesh->nverts * 3; i ++)
	{
		(*f) << rcPmesh->verts[i] << " ";
		if ((i + 1) % 3 == 0)
		{
			(*f) << endl;
		}
	}

	(*f) << "Num of Poly" << rcPmesh->npolys << endl;
	(*f) << "Max Num of Poly" << rcPmesh->maxpolys << endl;
	(*f) << "Max num of Poly Vertices" << rcPmesh->nvp << endl;

#if 0   //debug
	(*f) << "test 1" << endl;
	for (int j = 0; j < rcPmesh->nvp * 2; j ++)
	{
		for (int k = 0; k < rcPmesh->nvp; k ++)
		{		
			(*f) << rcPmesh->polys[j * rcPmesh->nvp + k] << " ";
			//<< rcPmesh->regs[j * rcPmesh->nvp + k] << " "
			//<< rcPmesh->flags[j * rcPmesh->nvp + k] << endl;
		}
		if((j + 1) % 2 == 0) 
			(*f) << endl;
		if((j + 1) % 4 == 0) 
			(*f) << endl;
	}
#endif
#if 1  //test
	(*f) << "test2" << endl;
	int index = 0;
	int finish = 0;
	iSumOfIndexNum = 0; 
	while(finish < rcPmesh->npolys)
	{
		bool bFlag = false;
		int sum = 0;
		for (int i = index * rcPmesh->nvp * 2; i < (index + 1) * rcPmesh->nvp * 2; i ++)
		{
			if(rcPmesh->polys[i] == 65535)
				break;
			sum ++;
		}
		if (sum > 2) 
		{
			if (sum > rcPmesh->nvp)
			{
				sum = rcPmesh->nvp;
			}
			(*f) << sum << endl;
			for (int i = index * rcPmesh->nvp * 2; i < (index * 2 + 1) * rcPmesh->nvp; i ++)
			{
				if (rcPmesh->polys[i] == 65535)
					break;

				(*f) << rcPmesh->polys[i] << " ";
			}
			(*f) << endl;
			finish ++;
			iSumOfIndexNum += sum;
		}

		index ++;
	}
	(*f) << "index£º" << index << endl; 
	(*f) << "Sum of Index Num:" << iSumOfIndexNum << endl;
#endif
}
#endif