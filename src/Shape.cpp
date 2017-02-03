#include"Shape.h"

Shape::Shape(const Transform *o2w, const Transform *w2o, bool ro)
	:ObjectToWorld(o2w), WorldToObject(w2o), ReverseOrientation(ro),
	TransformSwapsHandedness(o2w->SwapsHandedness()),
	shapeId(nextshapeId++) {
}

uint32_t Shape::nextshapeId = 1;