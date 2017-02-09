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