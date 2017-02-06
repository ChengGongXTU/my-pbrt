#include"pbrt.h"
#include"Memory.h"
#include"Transform.h"
#include"diffgeom.h"


class Shape :public ReferenceCounted {
public:
//--------------------Shape Interface-----------------
	Shape(const Transform *o2w, const Transform *w2o, bool ro);

	//  bounding: build shape's Box in world coorinate. 
	virtual BBox ObjectBound() const = 0;
	virtual BBox WorldBound() const;

	// refinement: check if the shape can be intersected by ray
	virtual bool CanIntersect() const;
	virtual void Refine(vector<Reference<Shape>> &refined) const;

	//intersection: 1, compute the imformation of intersection; 2, check if a ray intersect this shape
	// rayEpsilon is the maximum floating-point error,solving the problem of self-intersection.
	virtual bool Intersect(const Ray &ray, float *tHit, float *rayEpsilon,
		DifferentialGeometry *dg) const;
	virtual bool IntersectP(const Ray &ray) const;

	// shading geometry
	virtual virtual void GetShadingGeometry(const Transform &obj2world,
		const DifferentialGeometry &dg, DifferentialGeometry *dgShading)const {
		*dgShading = dg;
	}

	//suface area for area lights
	virtual float Area() const;

//---------------------Shape Public Data--------------
	// transform pointer for coordinate mapping, and bool for normal reverse.
	const Transform *ObjectToWorld, *WorldToObject;
	const bool ReverseOrientation, TransformSwapsHandedness;
	
	// numeric id for each shape. 
	const uint32_t shapeId;
	static uint32_t nextshapeId;
};

