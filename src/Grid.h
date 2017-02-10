#include"pbrt.h"
#include"Primitive.h"

class GridAccel :public Aggregate {
public:
	GridAccel(const vector<Reference<Primitive>> &p,
		bool refineImmediately);

	bool Intersect(const Ray &ray, Intersection *isect) const;

private:
	mutable RWMutex *rwMutex;      // reader-write mutex

	vector<Reference<Primitive>> primitives;

	BBox bounds;
	int nVoxels[3];

	Vector width, invWidth;
	Voxel **voxels;

	MemoryArena voxelArena;

	// turn world space position into voxel coordinates
	int posToVoxel(const Point &P, int axis)const {
		int v = Float2Int((P[axis] - bounds.pMin[axis]) *invWidth[axis]);
		return Clamp(v, 0, nVoxels[axis] - 1);
	}

	// from voxel-coordinate into world sapce.
	float voxelToPos(int p, int axis) const {
		return bounds.pMin[axis] + p * width[axis];
	}

	inline int offset(int x, int y, int z) const {
		return z*nVoxels[0] * nVoxels[1] + y*nVoxels[0] + x;
	}

};


struct Voxel {
	// method:
	Voxel(Reference<Primitive> op)
	{
		allCanIntersect = false;
		primitives.push_back(op);   // store primitive 
	}

	void AddPrimitive(Reference<Primitive> prim) {
		primitives.push_back(prim);  //store primitive
	}

	bool Intersect(const Ray &ray, Intersection *isect, RWMutexLock &lock);

	bool IntersectP(const Ray &ray) const;

private:
	vector<Reference<Primitive>> primitives;
	bool allCanIntersect;
};
