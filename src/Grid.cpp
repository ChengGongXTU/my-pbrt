#include"Grid.h"

GridAccel::GridAccel(const vector<Reference<Primitive>> &p,
	bool refineImmediately) {

	// Initialize primitive
	if (refineImmediately)
		for (uint32_t i = 0; i < p.size; ++i)
			p[i]->FullyRefine(primitives);
	else primitives = p;

	// compute the bound
	for (uint32_t i = 0; i < primitives.size; ++i)
		bounds = Union(bounds, primitives[i]->WorldBound());
	Vector delta = bounds.pMax - bounds.pMin;

	// compute the number of voxel per unit distance
	int maxAxis = bounds.MaxmumExtent();
	float invMaxWidth = 1.f / delta[maxAxis];		// largest extent
	float cubeRoot = 3.f * powf(float(primitives.size()), 1.f / 3.f);		// voxles' number along largest extent 
	float voxelPerUnitDist = cubeRoot*invMaxWidth;

	// give the number of voxel in each dimension.
	for (int axis = 0; axis < 3; ++axis) {
		nVoxels[axis] = Round2Int(delta[axis] * voxelPerUnitDist);
		nVoxels[axis] = Clamp(nVoxels[axis], 1, 64);
	}

	// comute voxel widths
	for (int axis = 0; axis < 3; ++axis) {
		width[axis] = delta[axis] / nVoxels[axis];
		invWidth[axis] = (width[axis] == 0.f) ? 0.f : 1.f/ width[axis];
	}

	// allocate voxel
	int nv = nVoxels[0] * nVoxels[1] * nVoxels[2];
	voxels = AllocAligned<Voxel *>(nv);
	memset(voxels, 0, nv*sizeof(Voxel *));

	// add primitive into voxel
	for (uint32_t i = 0; i < primitives.size(); ++i)
	{

		BBox pb = primitives[i]->WorldBound();
		int vmin[3], vmax[3];
		for (int axis = 0; axis < 3; ++axis)
		{
			vmin[axis] = posToVoxel(pb.pMin, axis);
			vmax[axis] = posToVoxel(pb.pMax, axis);
		}
		for (int z = vmin[2]; z <= vmax[2]; ++z)
			for (int y = vmin[1]; y <= vmax[1]; ++y)
				for (int x = vmin[0]; x <= vmax[0]; ++x)
				{
					int o = offset(x, y, z);
					if (!voxels[o])
					{
						voxels[o] = voxelArena.Alloc<Voxel>();
						*voxels[o] = Voxel(primitives[i]);
					}
					else { voxels[o]->AddPrimitive(primitives[i]); }
				}
	}

}

bool GridAccel::Intersect(const Ray &ray, Intersection *isect) const{
	//  determine where the ray-box intersection is
	float rayT;
	if (bounds.Inside(ray(ray.mint)))  // if ray-min-point is in the bounds
		rayT = ray.mint;
	else if (!bounds.IntersectP(ray, &rayT))  // check if the ray is intersected by bounds.
		return false;
	Point gridIntersect = ray(rayT);    // the ray-bounds intersection

	// 3D DDA:
	float NextCrossingT[3], DeltaT[3];
	int Step[3], Out[3], Pos[3];
	for (int axis = 0; axis < 3; ++axis) {
		Pos[axis] = posToVoxel(gridIntersect, axis);
		if (ray.d[axis] >= 0) {
			NextCrossingT[axis] = rayT + (voxelToPos(Pos[axis] + 1, axis) - gridIntersect[axis]) / ray.d[axis];
			DeltaT[axis] = width[axis] / ray.d[axis];
			Step[axis] = 1;
			Out[axis] = nVoxels[axis];
		}
		else {
			NextCrossingT[axis] = rayT + (voxelToPos(Pos[axis], axis) - gridIntersect[axis]) / ray.d[axis];
			DeltaT[axis] = -width[axis] / ray.d[axis];
			Step[axis] = -1;
			Out[axis] = -1;
		}
	}

	// 
	RWMutexLock lock(*rwMutex, READ);
	bool hitSomething = false;
	for (;;) {
		Voxel *voxel = voxels[offset(Pos[0], Pos[1], Pos[2])];
		if (voxel != NULL)
			hitSomething |= voxel->Intersect(ray, isect, lock);
		int bits = ((NextCrossingT[0] < NextCrossingT[1]) << 2) +
			((NextCrossingT[0] < NextCrossingT[2]) << 1) +
			((NextCrossingT[1] < NextCrossingT[2]));
		const int cmpToAxis[8] = { 2, 1, 2, 1, 2, 2, 0, 0 };
		int stepAxis = cmpToAxis[bits];

		int bits = ((NextCrossingT[0] < NextCrossingT[1]) << 2) +
			((NextCrossingT[0] < NextCrossingT[2]) << 1) +
			((NextCrossingT[1] < NextCrossingT[2]));
		const int cmpToAxis[8] = { 2, 1, 2, 1, 2, 2, 0, 0 };
		int stepAxis = cmpToAxis[bits];

		if (ray.maxt < NextCrossingT[stepAxis])
			break;
		Pos[stepAxis] += Step[stepAxis];
		if (Pos[stepAxis] == Out[stepAxis])
			break;
		NextCrossingT[stepAxis] += DeltaT[stepAxis];
	}
	return hitSomething;
}


bool Voxel::Intersect(const Ray &ray, Intersection *isect,
	RWMutexLock &lock) {
	if (!allCanIntersect) {
		lock.UpgradeToWrite();
		for (uint32_t i = 0; i < primitives.size(); ++i) {
			Reference<Primitive> &prim = primitives[i];
			if (!prim->CanIntersect()) {
				vector<Reference<Primitive> > p;
				prim->FullyRefine(p);
				if (p.size() == 1)
					primitives[i] = p[0];
				else
					primitives[i] = new GridAccel(p, false);
			}
		}
		allCanIntersect = true;
		lock.DowngradeToRead();
	}
	bool hitSomething = false;
	for (uint32_t i = 0; i < primitives.size(); ++i) {
		Reference<Primitive> &prim = primitives[i];
		if (prim->Intersect(ray, isect))
			hitSomething = true;
	}
	return hitSomething;
}