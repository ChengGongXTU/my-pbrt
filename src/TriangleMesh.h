#include"pbrt.h"
#include"Shape.h"
#include"diffgeom.h"

class TriangleMesh : public Shape {
public:
	TriangleMesh(const Transform *o2w, const Transform *w2o,
		bool ro, int nt, int nv, const int *vi, const Point *P,
		const Normal *N, const Vector *S, const float *uv,
		const Reference<Texture<float>> &atex);

	// the BBox of triangle in objecct space
	BBox ObjectBound() const;

	// BBox in world space
	BBox WorldBound()const;

	// trianglemesh can't be intersected directly
	bool CanIntersect()const { return false; }

	// 
	void Refine(vector<Reference<Shape>> &refined) const;

protected:
	int ntris, nverts;
	int *vertexIndex;
	Point *p;				// array of vertex position in world space
	Normal *n;
	Vector *s;
	float *uvs;
	Reference<Texture<float>>alphaTexture;
};


class Triangle :public Shape {
public:
	Triangle(const Transform *o2w, const Transform *w2o, bool ro,
		TriangleMesh *m, int n)
		:Shape(o2w, w2o, ro) {
		mesh = m;
		v = &mesh->vertexIndex[3 * n];  // point to this triangle's first vertex index
	}

	// BBox in object space
	BBox ObjectBound()const;

	// BBox in world space;
	BBox WorldBound()const;

	// ray - intersection
	bool Intersect(const Ray &ray, float *tHit, float *rayEpsilon,
		DifferentialGeometry *dg) const;

	// the (u, v) coordinates of vertices
	void GetUVs(float uv[3][2]) const {
		if (mesh->uvs) {
			uv[0][0] = mesh->uvs[2 * v[0]];
			uv[0][1] = mesh->uvs[2 * v[0] + 1];
			uv[1][0] = mesh->uvs[2 * v[1]];
			uv[1][1] = mesh->uvs[2 * v[1] + 1];
			uv[2][0] = mesh->uvs[2 * v[2]];
			uv[2][1] = mesh->uvs[2 * v[2] + 1];
		}
		else {
			uv[0][0] = 0.; uv[0][1] = 0.;
			uv[1][0] = 1.; uv[1][1] = 0.;
			uv[2][0] = 1.; uv[2][1] = 1.;
		}
	}

	//  suface area 
	float Area()const;

	// shading geometry initialization
	void GetShadingGeometry(const Transform &obj2world,
		const DifferentialGeometry &dg, DifferentialGeometry *dgShading)const;

private:
	Reference<TriangleMesh> mesh;
	int *v;
};