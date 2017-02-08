#include"TriangleMesh.h"

TriangleMesh::TriangleMesh(const Transform *o2w, const Transform *w2o,
	bool ro, int nt, int nv, const int *vi, const Point *P,
	const Normal *N, const Vector *S, const float *uv,
	const Reference<Texture<float>> &atex)
	:Shape(o2w, w2o, ro), alphaTexture(atex) {
	ntris = nt;                                        //number of triangle
	nverts = nv;									   //number of vertex
	vertexIndex = new int[3 * ntris];				   // copy an array of vertex indices from vi to vertexIndex
	memcpy(vertexIndex, vi, 3 * ntris*sizeof(int));

	// Transform triangle mesh to world space
	for (int i = 0; i < nverts; ++i)
		p[i] = (*ObjectToWorld)(P[i]);
}

BBox TriangleMesh::ObjectBound() const {
	BBox objectBounds;
	for (int i = 0; i < nverts; ++i)
		objectBounds = Union(objectBounds, (*WorldToObject)(p[i]));
	return objectBounds;
}

BBox TriangleMesh::WorldBound()const {
	BBox worldBounds;
	for (int i = 0; i < nverts; ++i)
		worldBounds = Union(worldBounds, p[i]);
	return worldBounds;
}

void TriangleMesh::Refine(vector<Reference<Shape>> &refined) const {
	for (int i = 0; i < nverts; ++i)
		refined.push_back(new Triangle(ObjectToWorld, WorldToObject, ReverseOrientation,
			(TriangleMesh *)this, i));
}

BBox Triangle::ObjectBound()const {
	const Point &p1 = mesh->p[v[0]];
	const Point &p2 = mesh->p[v[1]];
	const Point &p3 = mesh->p[v[2]];
	return Union(BBox((*WorldToObject)(p1), (*WorldToObject)(p2)), (*WorldToObject)(p3));
}

BBox Triangle::WorldBound()const {
	const Point &p1 = mesh->p[v[0]];
	const Point &p2 = mesh->p[v[1]];
	const Point &p3 = mesh->p[v[2]];
	return Union(BBox(p1.p2), p3);
}

bool Triangle::Intersect(const Ray &ray, float *tHit, float *rayEpsilon,
	DifferentialGeometry *dg)const {
	
	// get the vertex of triangle				equation:   o + td = (1? b1? b2)*p0+ b1*p1+ b2*p2.
	const Point &p1 = mesh->p[v[0]];		  //transform to matrix style:   (t, b1, b2) ^ T = (s2*e2, s1*s, s2*d) ^ T / (s1*e1)
	const Point &p2 = mesh->p[v[1]];
	const Point &p3 = mesh->p[v[2]];

	// get the parameter S1
	Vector e1 = p2 - p1;
	Vector e2 = p3 - p1;
	Vector s1 = Cross(ray.d, e2);			// s1 = d x e2
	float divisor = Dot(s1, e1);			//  1 / (s1*e1)
	if (divisor == 0)
		return false;
	float invDivisor = 1.f / divisor;

	// get the barycentric coordinate b1
	Vector d = ray.o - p1;					// s = o - p1
	float b1 = Dot(d, s1)*invDivisor;		// b1 = (s1*s) / (s1*e1)        
	if (b1 < 0. || b1>1)
		return false;

	// get the b2
	Vector s2 = Cross(d, e1);				// s2 = s x e1
	float b2 = Dot(ray.d, s2)*invDivisor;	// b2 = (s2*d) / (s1*e1)
	if (b2<0 || b1 + b2>1.)
		return false;

	// get the t 
	float t = Dot(e2, s2)*divisor;			// t = (s2*e2) / (s1*e1)
	if (t<ray.mint || t>ray.maxt)
		return false;

	// compute triangle partial derivatives
	Vector dpdu, dpdv;
	float uvs[3][2];
	GetUVs(uvs);		// transform vertex position to u,v coordinate
	
	float du1 = uvs[0][0] - uvs[2][0];  //  the (u, v) differences matrix
	float du2 = uvs[1][0] - uvs[2][0];	// [ du1, dv1 ]^-1 x [ p1 - p3] = [dpdu]
	float dv1 = uvs[0][1] - uvs[2][1];	// [ du2, dv2 ]      [ p2 - p3]   [dpdv]
	float dv2 = uvs[1][1] - uvs[2][1];	//
	Vector dp1 = p1 - p3, dp2 = p2 - p3;

	float determinant = du1 * dv2 - dv1 * du2;
	if (determinant == 0.f) {
		CoordinateSystem(Normalize(Cross(e2, e1)), &dpdu, &dpdv);
	}
	else {
		float invdet = 1.f / determinant;
		dpdu = (dv2 * dp1 - dv1 * dp2) * invdet;
		dpdv = (-du2 * dp1 + du1 * dp2) * invdet;
	}

	// Interpolate (u, v) triangle parametric coordinates 
	float b0 = 1 - b1 - b2;
	float tu = b0*uvs[0][0] + b1*uvs[1][0] + b2*uvs[2][0];
	float tv = b0*uvs[0][1] + b1*uvs[1][1] + b2*uvs[2][1];

	// alpha texture test for intersection
	if (mesh->alphaTexture) {
		DifferentialGeometry dgLocal(ray(t), dpdu, dpdv,
			Normal(0, 0, 0), Normal(0, 0, 0),
			tu, tv, this);
		if (mesh->alphaTexture->Evaluate(dgLocal) == 0.f)
			return false;
	}

	// fill its partial derivative imformation to the differentialgeometry
	*dg = DifferentialGeometry(ray(t), dpdu, dpdv,
		Normal(0, 0, 0), Normal(0, 0, 0),
		tu, tv, this);

	*tHit = t;
	*rayEpsilon = 1e-3f**tHit;
	return true;
}

float Triangle::Area()const {
	const Point &p1 = mesh->p[v[0]];
	const Point &p2 = mesh->p[v[1]];
	const Point &p3 = mesh->p[v[2]];
	return 0.5f*Cross(p2 - p1, p3 - p1).Length();
}

void Triangle::GetShadingGeometry(const Transform &obj2world,
	const DifferentialGeometry &dg, DifferentialGeometry *dgShading)const {

	if (!mesh->n && mesh->s) {
		*dgShading = dg;
		return;
	}

	// barycentric coordinate,  u = b0*u0+b1*u1+ b2*u2, v = b0*v0+b1*v1+ b2*v2
	float b[3];
	float uv[3][2];
	GetUVs(uv);
	float A[2][2] = {
		{ uv[1][0] - uv[0][0], uv[2][0] - uv[0][0] },
		{ uv[1][1] - uv[0][1], uv[2][1] - uv[0][1] }
	};
	float C[2] = { dg.u - uv[0][0],dg.v - uv[0][1] };
	if (!SolveLinearSystem2x2(A, C, &b[1], &b[2])) {
		b[0] = b[1] = b[2] = 1.f / 3.f;
	}
	else
		b[0] = 1.f - b[1] - b[2];

	// get the shading tangents
	Normal ns;
	Vector ss, ts;
	// get the u
	if(mesh->n) ns = Normalize(obj2world(b[0] * mesh->n[v[0]] +b[1] * mesh->n[v[1]] +b[2] * mesh->n[v[2]]));
	else ns = dg.nn;
	// get the v
	if (mesh->s) ss = Normalize(obj2world(b[0] * mesh->s[v[0]] +b[1] * mesh->s[v[1]] +b[2] * mesh->s[v[2]]));
	else ss = Normalize(dg.dpdu);

	ts = Cross(ss, ns);
	if (ts.LengthSquared() > 0.f) {
		ts = Normalize(ts);
		ss = Cross(ts, ns);
	}
	else
		CoordinateSystem((Vector)ns, &ss, &ts);

	// get the dndu and dndv. similar to computing dpdu and dpdv in Intersectfunction
	Normal dndu, dndv;
	if (mesh->n) {
		float uvs[3][2];
		GetUVs(uvs);
		float du1 = uvs[0][0] - uvs[2][0];
		float du2 = uvs[1][0] - uvs[2][0];
		float dv1 = uvs[0][1] - uvs[2][1];
		float dv2 = uvs[1][1] - uvs[2][1];
		Normal dn1 = mesh->n[v[0]] - mesh->n[v[2]];
		Normal dn2 = mesh->n[v[1]] - mesh->n[v[2]];
		float determinant = du1 * dv2 - dv1 * du2;
		if (determinant == 0.f)
			dndu = dndv = Normal(0, 0, 0);
		else {
			float invdet = 1.f / determinant;
			dndu = (dv2 * dn1 - dv1 * dn2) * invdet;
			dndv = (-du2 * dn1 + du1 * dn2) * invdet;
		}
	}
	else
		dndu = dndv = Normal(0, 0, 0);

	*dgShading = DifferentialGeometry(dg.p, ss, ts,
		(*ObjectToWorld)(dndu), (*ObjectToWorld)(dndv),
		dg.u, dg.v, dg.shape);
	dgShading->dudx = dg.dudx;  dgShading->dvdx = dg.dvdx;
	dgShading->dudy = dg.dudy;  dgShading->dvdy = dg.dvdy;
	dgShading->dpdx = dg.dpdx;  dgShading->dpdy = dg.dpdy;
}
