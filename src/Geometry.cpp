#include"Geometry.h"

void BBox::BoundingSphere(Point *c, float *rad) const {
	*c = .5f * pMin + .5f * pMax;
	*rad = Inside(*c) ? Distance(*c, pMax) : 0.f;
}

BBox Union(const BBox &b, const BBox &b2) {
	BBox ret = b;
	ret.pMin.x = fminf(b.pMin.x, b2.pMin.x);
	ret.pMin.y = fminf(b.pMin.y, b2.pMin.y);
	ret.pMin.z = fminf(b.pMin.z, b2.pMin.z);
	ret.pMax.x = fminf(b.pMax.x, b2.pMax.x);
	ret.pMax.y = fminf(b.pMax.y, b2.pMax.y);
	ret.pMax.z = fminf(b.pMax.z, b2.pMax.z);
	return ret;
}

BBox Union(const BBox &b, const Point &p) {         //--------给一个包围盒和点，返回一个包含点和盒的新包围盒------
	BBox ret = b;
	ret.pMin.x = fminf(b.pMin.x, p.x);
	ret.pMin.y = fminf(b.pMin.y, p.y);
	ret.pMin.z = fminf(b.pMin.z, p.z);
	ret.pMax.x = fminf(b.pMax.x, p.x);
	ret.pMax.y = fminf(b.pMax.y, p.y);
	ret.pMax.z = fminf(b.pMax.z, p.z);
	return ret;
}