//This class is for Poly Triangulation Zhaoran Wang
//For visualize independtent test use
//Resourse fils in Poly2tri folder under MIT liences

#include "PolyTriangulation.h"

void BuildS(string filename)
{
	Polygon poly(filename, true);
	//PolyTriangulation
	poly.triangulation();                   
	poly.saveAsMetaPost();
}

bool BuildS(string filename, rcContourSet &rcConSet)
{
	Polygon poly(filename, true);
	poly.triangulation();  
	return poly.ChangeToConset(rcConSet);
}