#include"pbrt.h"
#include"diffgeom.h"
#include"Transform.h"

struct Intersection {
public:
	// data
	DifferentialGeometry dg;
	const Primitive *primitive;
	Transform WorldToObject, ObjectToWorld;
	uint32_t shapeId, primitiveId;
	float rayEpsilon;
};
