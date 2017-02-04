#include"pbrt.h"
#include"Shape.h"

class Sphere:public Shape {
public:
// sphere public method
	Sphere(const Transform *o2w, const Transform *w2o, bool ro,
		float rad, float z0, float z1, float pm);

private:
// sphere private data
	float radius;
	float phiMax;
	float zmin, zmax;
	float thetaMin, thetaMax;
};
