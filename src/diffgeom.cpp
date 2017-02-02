#include"diffgeom.h"

DifferentialGeometry::DifferentialGeometry(const Point &P,
	const Vector &DPDU, const Vector &DPDV,
	const Normal &DNDU, const Normal &DNDV,
	float uu, float vv, const Shape *sh)
	: p(P), dpdu(DPDU), dpdv(DPDV), dndu(DNDU), dndv(DNDV) {

	//----------initialize,get the normal----------------
	nn = Normal(Normalize(Cross(dpdu, dpdv)));
	u = uu;
	v = vv;
	shape = sh;

	//adjust normal base on handness 
	if (shape && (shape->ReverseOrientation^shape->TransformSwapsHandednessa))
		nn *= -1.f;
}