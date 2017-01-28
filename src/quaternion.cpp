#include "pbrt.h"
#include "quaternion.h"

//   the rotation transform M for quaternion q: "p' = Mp = qpq-1"
Transform Quaternion::ToTransform()const {
	Matrix4x4 m;
	m.m[0][0] = 1.f - 2.f*(v.y*v.y + v.z*v.z);
	m.m[0][1] = 2.f*(v.x*v.y + v.z*w);
	m.m[0][2] = 2.f*(v.x*v.z - v.y*w);

	m.m[1][0] = 2.f*(v.x*v.y - v.z*w);
	m.m[1][1] = 1.f - 2.f*(v.x*v.x + v.z*v.z);
	m.m[1][2] = 2.f*(v.y*v.z + v.x*w);

	m.m[2][0] = 2.f*(v.x*v.z + v.y*w);
	m.m[2][1] = 2.f*(v.y*v.z - v.x*w);
	m.m[2][2] = 1.f - 2.f*(v.x*v.x + v.y*v.y);

	Matrix4x4 n = Transpose(m);
	return Transform(n, m);
}