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

	//partial derivative of normal vector
	//compute the second partial derivative for second fundamental form
	Vector d2Pduu = -phiMax * phiMax * Vector(phit.x, phit.y, 0);       //  d^2p/du^2 = -phiMax^2(x,y,z)
	Vector d2Pduv = (thetaMax - thetaMin) * phit.z * phiMax *			//  d^2p/dudv = (thetaMax - thetaMin)*z*phiMax(-sin(theta),cos(theta),0)
		Vector(-sinphi, cosphi, 0.);
	Vector d2Pdvv = -(thetaMax - thetaMin) * (thetaMax - thetaMin) *	//  d^2p/dv^2 = -(thetaMax - thetaMin)*(x,y,z)
		Vector(phit.x, phit.y, phit.z);
	
	//coefficients of the first fundamental form
	float E = Dot(dpdu, dpdu);
	float F = Dot(dpdu, dpdv);
	float G = Dot(dpdv, dpdv);

	//coefficients of the second fundamental form
	Vector N = Normalize(Cross(dpdu, dpdv));
	float e = Dot(N, d2Pduu);
	float f = Dot(N, d2Pduv);
	float g = Dot(N, d2Pdvv);

	// compute the dn/du and dm/dv by coefficients: e,f,g and E,F,G.
	float invEGF2 = 1.f / (E*G - F*F);
	Normal dndu = Normal((f*F - e*G) * invEGF2 * dpdu +
		(e*F - f*E) * invEGF2 * dpdv);
	Normal dndv = Normal((g*F - f*G) * invEGF2 * dpdu +
		(f*F - g*E) * invEGF2 * dpdv);

	// differentialgeometry function initialization
	const Transform &o2w = *ObjectToWorld;
	*dg = DifferentialGeometry(o2w(phit), o2w(dpdu), o2w(dpdv),    // save the imformation of intersection in world space
		o2w(dndu), o2w(dndv), u, v, this);
	*tHit = thit;
	*rayEpsilon = 5e-4f * *tHit;

	return true;
}

// much close to Intersect function.
bool Sphere::IntersectP(const Ray &r) const {
	float phi;
	Point phit;
	Ray ray;
	(*WorldToObject)(r, &ray);  
	float A = ray.d.x*ray.d.x + ray.d.y*ray.d.y + ray.d.z*ray.d.z;   
	float B = 2 * (ray.d.x*ray.o.x + ray.d.y*ray.o.y + ray.d.z*ray.o.z);  
	float C = ray.o.x*ray.o.x + ray.o.y*ray.o.y + ray.o.z*ray.o.z - radius*radius;  
	float t0, t1;
	if (Quadratic(A, B, C, &t0, &t1))
		return false;
	if (t0 > ray.maxt || t1 < ray.mint)
		return false;
	float thit = t0;
	if (t0 < ray.mint) {
		thit = t1;
		if (thit > ray.maxt) return false;
	}
	phit = ray(thit);                   
	if (phit.x == 0.f && phit.y == 0.f) phit.x = 1e-5f * radius;
	phi = atan2f(phit.y, phit.x);	   
	if (phi < 0.) phi += 2.f*M_PI;
	if ((zmin > -radius && phit.z <zmin) ||
		(zmax>radius && phit.z>zmax) || phi > phiMax) {
		if (thit == t1) return false;
		if (t1 > ray.maxt) return false;	
		thit = t1;
		phit = ray(thit);
		if (phit.x == 0.f && phit.y == 0.f) phit.x = 1e-5f * radius;
		phi = atan2f(phit.y, phit.x);
		if (phi < 0.) phi += 2.f*M_PI;
		if ((zmin > -radius && phit.z <zmin) ||
			(zmax>radius && phit.z > zmax) || phi > phiMax)
			return false;
	}
	return true;
}


float Sphere::Area() const {
	return phiMax * radius * (zmax - zmin);
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