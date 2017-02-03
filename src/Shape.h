#include"pbrt.h"
#include"Memory.h"
#include"Transform.h"


class Shape :public ReferenceCounted {
public:
//--------------------Shape Interface-----------------
	Shape(const Transform *o2w, const Transform *w2o, bool ro);

//---------------------Shape Public Data--------------
	// transform pointer for coordinate mapping, and bool for normal reverse.
	const Transform *ObjectToWorld, *WorldToObject;
	const bool ReverseOrientation, TransformSwapsHandedness;
	
	// numeric id for each shape. 
	const uint32_t shapeId;
	static uint32_t nextshapeId;
};
