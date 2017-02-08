#include"pbrt.h"
#include"Geometry.h"
#include"Shape.h"


struct DifferentialGeometry {
	DifferentialGeometry() { u = v = 0.; shape = NULL; }

	//--------------------differentialgeometry public method------------
	// ininitialize parameters, compute and inverse the normal.
	DifferentialGeometry(const Point &P,
		const Vector &DPDU, const Vector &DPDV,
		const Normal &DNDU, const Normal &DNDV,
		float uu, float vv, const Shape *sh);

	void ComputeDifferentials(const RayDifferential &ray) const;
	//--------------------differentialgeometry public data-------------
	//  if a point p is in the surface and p =f(u,v)
	//  wo can store some imformation about point p.
	Point p;
	Normal nn;     // normalized
	float u, v;
	const Shape *shape;

	Vector dpdu, dpdv; // partial derivatives
	Normal dndu, dndv;

	mutable Vector dpdx, dpdy;
	mutable float dudx, dvdx, dudy, dvdy;
};
