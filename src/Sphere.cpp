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

BBox Sphere::ObjectBound()const {
	return BBox(Point(-radius, -radius, zmin),
		Point(radius, radius, zmax));
}

bool Sphere::Intersect(const Ray &r, float *tHit, float *rayEpsilon,
	DifferentialGeometry *dg) const {
	float phi;
	Point phit;

	//  Trasform ray from world to object space.
	Ray ray;
	(*WorldToObject)(r, &ray);  //set operator() in "Transform.h"

	// compute the quadratic equation of intersection with Ray and sphere in object space
	//  At^2+Bt+C = 0
	float A = ray.d.x*ray.d.x + ray.d.y*ray.d.y + ray.d.z*ray.d.z;   //dx^2+dy^2+dz^2
	float B = 2 * (ray.d.x*ray.o.x + ray.d.y*ray.o.y + ray.d.z*ray.o.z);  //2(dx*ox+dy*oy+dz*oz)
	float C = ray.o.x*ray.o.x + ray.o.y*ray.o.y + ray.o.z*ray.o.z - radius*radius;  // ox^2+oy^2+oz^2 -R^2

	// get the t from quadratic equation, t0 < t1.
	float t0, t1;
	if (Quadratic(A, B, C, &t0, &t1))
		return false;

	// Is the intersection(t0 or t1) in the range between maxPoint(maxt) and minPoint(mint) of the Ray?
	if (t0 > ray.maxt || t1 < ray.mint)
		return false;
	float thit = t0;
	if (t0 < ray.mint) {
		thit = t1;
		if (thit > ray.maxt) return false;
	}

	//ignored the intersection in the clipped area.	
	phit = ray(thit);                   //the object space position of intersection 
	if (phit.x == 0.f && phit.y == 0.f) phit.x = 1e-5f * radius;
	phi = atan2f(phit.y, phit.x);	    // angle = arctan(y/x)
	if (phi < 0.) phi += 2.f*M_PI;

	// check if t0 point is out of the sphere.
	if ((zmin > -radius && phit.z <zmin) ||
		(zmax>radius && phit.z>zmax) || phi > phiMax) {
		if (thit == t1) return false;
		if (t1 > ray.maxt) return false;
		
		// compute position and angle of t1 point;		
		thit = t1;
		phit = ray(thit);
		if (phit.x == 0.f && phit.y == 0.f) phit.x = 1e-5f * radius;
		phi = atan2f(phit.y, phit.x); 
		if (phi < 0.) phi += 2.f*M_PI;

		// check t1 point
		if ((zmin > -radius && phit.z <zmin) ||
			(zmax>radius && phit.z > zmax) || phi > phiMax)
			return false;
	}

	// Transform f(phi,theta) to f(u,v) in the intersection
	// parameter:
	float u = phi / phiMax;      // phi = u*phiMax
	float theta = acosf(Clamp(phit.z / radius, -1.f, 1.f));
	float v = (theta - thetaMin) / (thetaMax - thetaMin);  //theta = thetaMin + v*(thetaMax -thetaMin)
	
	// compute parametric partial derivatives of the intersection .
	float zradius = sqrtf(phit.x*phit.x + phit.y*phit.y);
	float invzradius = 1.f / zradius;
	float cosphi = phit.x * invzradius;
	float sinphi = phit.y * invzradius;
	Vector dpdu(-phiMax * phit.y, phiMax * phit.x, 0);
	Vector dpdv = (thetaMax - thetaMin) *
		Vector(phit.z * cosphi, phit.z * sinphi,
			-radius * sinf(theta));
}


inline bool Quadratic(float A, float B, float C, float *t0, float *t1) {
	// B^2 -4AC >0 or <= 0;
	float discrim = B*B - 4.f*A*C;
	if (discrim <= 0.) return false;
	float rootDiscrim = sqrtf(discrim);

	// compute t0 = q /A; t1 = C/q; q = 0.5*(B -/+ sqrt(B^2 -4AC));
	float q;
	if (B < 0) q = -0.5f*(B - rootDiscrim);
	else q = -0.5f*(B + rootDiscrim);

	*t0 = q / A;
	*t1 = C / q;

	if (*t0 >*t1) swap(*t0, *t1);
	return true;
}