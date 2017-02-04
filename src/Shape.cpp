#include"Shape.h"

Shape::Shape(const Transform *o2w, const Transform *w2o, bool ro)
	:ObjectToWorld(o2w), WorldToObject(w2o), ReverseOrientation(ro),
	TransformSwapsHandedness(o2w->SwapsHandedness()),
	shapeId(nextshapeId++) {
}

uint32_t Shape::nextshapeId = 1;

BBox Shape::WorldBound()const {
	return(*ObjectToWorld)(ObjectBound());
}

bool Shape::CanIntersect()const {
	return true;
}

void Shape::Refine(vector<Reference<Shape>> &refined) const {
	Severe("Unimplemented Shape::Refine() method called");
}

bool Shape::Intersect(const Ray &ray, float *tHit, float *rayEpsilon,
	DifferentialGeometry *dg) const {
	Severe("Unimplemented Shape::Intersect() method called");
	return false;
}

bool Shape::IntersectP(const Ray &ray) const {
	Severe("Unimplemented Shape::IntersectP() method called");
	return false;
}

float Shape::Area()const {
	Severe("Unimplemented Shape::Area() method called");
		return 0.;
}