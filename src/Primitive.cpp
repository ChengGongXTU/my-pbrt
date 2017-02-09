#include"Primitive.h"
#include "light.h"
//-------------------------------------------primitive class--------------------------------------------
uint32_t Primitive::nextprimitiveId = 1;

void Primitive::FullyRefine(vector<Reference<Primitive>> &refine) const {
	vector<Reference<Primitive>> todo;
	todo.push_back(const_cast<Primitive *>(this));
	// save intersectale primitive into refine, else save in todo.
	while (todo.size()) {
		Reference<Primitive> prim = todo.back();
		todo.pop_back();
		if (prim->CanIntersect())
			refined.push_back(prim);
		else
			prim->Refine(todo);
	}
}

bool Primitive::CanIntersect() const {
	return true;
}

void Primitive::Refine(vector<Reference<Primitive> > &refined) const {
	Severe("Unimplemented Primitive::Refine() method called!");
}

const AreaLight *Aggregate::GetAreaLight() const {
	Severe("Aggregate::GetAreaLight() method"
		"called; should have gone to GeometricPrimitive");
	return NULL;
}

BSDF *Aggregate::GetBSDF(const DifferentialGeometry &,
	const Transform &, MemoryArena &) const {
	Severe("Aggregate::GetBSDF() method"
		"called; should have gone to GeometricPrimitive");
	return NULL;
}

BSSRDF *Aggregate::GetBSSRDF(const DifferentialGeometry &,
	const Transform &, MemoryArena &) const {
	Severe("Aggregate::GetBSSRDF() method"
		"called; should have gone to GeometricPrimitive");
	return NULL;
}

//------------------------------------ geometryprimitive class-----------------------------------------
GeometricPrimitive::GeometricPrimitive(const Reference<Shape> &s,
	const Reference<Material> &m, AreaLight *a)
	: shape(s), material(m), areaLight(a) {
}

bool  GeometricPrimitive::Intersect(const Ray &r, Intersection *isect)const {
	float thit, rayEpsilon;
	// call shape::Intersect
	if (!shape->Intersect(r, &thit, &rayEpsilon, &isect->dg))
		return false;
	//stores intersection's imformation.
	isect->primitive = this;
	isect->WorldToObject= *shape->WorldToObject;
	isect->ObjectToWorld = *shape->ObjectToWorld;
	isect->shapeId = shape->shapeId;
	isect->primitiveId = primitiveId;
	isect->rayEpsilon = rayEpsilon;
	r.maxt = thit;
	return true;
}

BBox GeometricPrimitive::WorldBound() const {
	return shape->WorldBound();
}


bool GeometricPrimitive::IntersectP(const Ray &r) const {
	return shape->IntersectP(r);
}


bool GeometricPrimitive::CanIntersect() const {
	return shape->CanIntersect();
}


void GeometricPrimitive::
Refine(vector<Reference<Primitive> > &refined)
const {
	vector<Reference<Shape> > r;
	shape->Refine(r);
	for (uint32_t i = 0; i < r.size(); ++i) {
		GeometricPrimitive *gp = new GeometricPrimitive(r[i],
			material, areaLight);
		refined.push_back(gp);
	}
}

const AreaLight *GeometricPrimitive::GetAreaLight() const {
	return areaLight;
}

BSDF *GeometricPrimitive::GetBSDF(const DifferentialGeometry &dg,
	const Transform &ObjectToWorld,
	MemoryArena &arena) const {
	DifferentialGeometry dgs;
	shape->GetShadingGeometry(ObjectToWorld, dg, &dgs);
	return material->GetBSDF(dg, dgs, arena);
}

BSSRDF *GeometricPrimitive::GetBSSRDF(const DifferentialGeometry &dg,
	const Transform &ObjectToWorld,
	MemoryArena &arena) const {
	DifferentialGeometry dgs;
	shape->GetShadingGeometry(ObjectToWorld, dg, &dgs);
	return material->GetBSSRDF(dg, dgs, arena);
}


//-----------------------------------transformprimitive---------------

bool TransformPrimitive::Intersect(const Ray &r, Intersection *isect)const {
	Transform w2p;
	WorldToPrimitive.Interpolate(r.time, &w2p);
	Ray ray = w2p(r);
	if (!primitive->Intersect(ray, isect))
		return false;
	r.maxt = ray.maxt;
	isect->primitiveId = primitiveId;
	if (!w2p.IsIdentity()) {
		isect->WorldToObject = isect->WorldToObject*w2p;
		isect->ObjectToWorld = Inverse(isect->WorldToObject);

		Transform PrimitiveToWorld = Inverse(w2p);
		isect->dg.p = PrimitiveToWorld(isect->dg.p);
		isect->dg.nn = Normalize(PrimitiveToWorld(isect->dg.nn));
		isect->dg.dpdu = PrimitiveToWorld(isect->dg.dpdu);
		isect->dg.dpdv = PrimitiveToWorld(isect->dg.dpdv);
		isect->dg.dndu = PrimitiveToWorld(isect->dg.dndu);
		isect->dg.dndv = PrimitiveToWorld(isect->dg.dndv);
	}
	return true;
}
