#include"pbrt.h"
#include"Shape.h"
#include"diffgeom.h"

class Sphere:public Shape {
public:
// sphere public method
	Sphere(const Transform *o2w, const Transform *w2o, bool ro,
		float rad, float z0, float z1, float pm);
	
// build a BBox by two points, include min and max value along each axis.
	BBox ObjectBound() const;

// compute intersection by transform ray from world to object space.
	bool Intersect(const Ray &r, float *tHit, float *rayEpsilon,
		DifferentialGeometry *dg) const;

// check if a ray is intersected by this sphere.
	bool IntersectP(const Ray &ray) const;

// compute area of surface;
	float Area()const;

private:
// sphere private data
	float radius;
	float phiMax;
	float zmin, zmax;
	float thetaMin, thetaMax;
};
