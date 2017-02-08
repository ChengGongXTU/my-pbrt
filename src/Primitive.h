#include"pbrt.h"
#include"Memory.h"
#include"Intersection.h"

class Primitive : public ReferenceCounted {
public:
	// Interface
	Primitive():primitiveId(nextprimitiveId++){}

	// five geometric routines
	// 1st: BBox in world space -- use to acceleration data structure
	virtual BBox WorldBound()const = 0;

	// 2nd and so on:
	virtual bool CanIntersect()const;
	virtual bool Intersect(const Ray &r, Intersection *in)const = 0;
	virtual bool IntersectP(const Ray &r) const = 0;
	virtual void Refine(vector<Reference<Primitive>> &refined) const;

	//repeatedly refine primitive
	void FullyRefine(vector<Reference<Primitive>> &refine) const;

	// three method for  material properties
	// 1st: if the primitive is a light source, return a point to the  emission distribution
	virtual const AreaLight *GetAreaLight() const = 0;

	// 2nd: return a BSDF, describes local light-scattering properties at the intersection point
	virtual BSDF *GetBSDF(const DifferentialGeometry &dg,
		const Transform &ObjectToWorld, MemoryArena &arena) const = 0;

	// 3rd: return a BSSRDF, describes subsurface scattering
	virtual BSSRDF *GetBSSRDF(const DifferentialGeometry &dg,
		const Transform &ObjectToWorld, MemoryArena &arena) const = 0;

	// public data
	const uint32_t primitiveId;						// uniquely identify

protected:
	// data
	static uint32_t nextprimitiveId;
};


class GeometricPrimitive :public Primitive {    // represents a single shape in the scene
public:
	GeometricPrimitive(const Reference<Shape> &s,
		const Reference<Material> &m, AreaLight *a);

	// call shape::Intersect() to handle Intersection
	bool Intersect(const Ray &r, Intersection *isect)const;

	//WorldBound(),IntersectP(), CanIntersect() and Refine() also call shape's function.
	bool CanIntersect() const;
	void Refine(vector<Reference<Primitive> > &refined) const;
	virtual BBox WorldBound() const;
	virtual bool IntersectP(const Ray &r) const;

	// three methods for light source
	const AreaLight *GetAreaLight() const;
	BSDF *GetBSDF(const DifferentialGeometry &dg,
		const Transform &ObjectToWorld, MemoryArena &arena) const;
	BSSRDF *GetBSSRDF(const DifferentialGeometry &dg,
		const Transform &ObjectToWorld, MemoryArena &arena) const;

private:
	Reference<Shape> shape;				// stores a reference to a shape
	Reference<Material> material;		// stores a reference to its material
	Arealight *arealight;				// stores a pointer to arealight
};

class 

