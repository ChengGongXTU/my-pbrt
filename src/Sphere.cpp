#include"Sphere.h"

// Sphere method definitions
Sphere::Sphere(const Transform *o2w, const Transform *w2o, bool ro,
	float rad, float z0, float z1, float pm)
	:Shape(o2w, w2o, ro) {
	radius = rad;
	zmin = Clamp(min(z0, z1), -radius, radius);
	zmax = Clamp(max(z0, z1), -radius, radius);
	thetaMin = acosf(Clamp(zmin / radius, -1.f, 1.f));
	thetaMax = acosf(Clamp(zmax / radius, -1.f, 1.f));
	phiMax = Radians(Clamp(pm, 0.0f, 360.0f));
}