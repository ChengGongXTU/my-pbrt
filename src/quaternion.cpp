#include "quaternion.h"
#include"Transform.h"

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

// the Quaternion q for Transform M
Quaternion::Quaternion(const Transform &t) {
	// set Quaternion is Transform's firend class, so that you can visit data m;
	const Matrix4x4 &m = t.m;
	// get the xyz 3x3 matrix trace without w;
	float trace = m.m[0][0] + m.m[1][1] + m.m[2][2];

	if (trace > 0.f) {
		float s = sqrtf(trace + 1.f);
		// (2w)^2 = martix trace(include m[3][3] == 1) 
		w = s / 2.f;
		s = 0.5f / s;
		v.x = (m.m[2][1] - m.m[1][2])*s;
		v.y = (m.m[0][2] - m.m[2][0])*s;
		v.z = (m.m[1][0] - m.m[0][1])*s;
	}

	else {
		const int nxt[3] = { 1,2,0 };
		float q[3];
		int i = 0;

		if (m.m[1][1] > m.m[0][0]) i = 1;
		if (m.m[2][2] > m.m[1][1]) i = 2;
		int j = nxt[i];
		int k = nxt[j];

		float s = sqrtf((m.m[i][i] - (m.m[j][j] + m.m[k][k])) + 1.f);
		q[i] = s*0.5f;
		if (s != 0.f) s = 0.5f / s;
		w = (m.m[k][j] - m.m[j][k])*s;
		q[j] = (m.m[j][k] + m.m[i][j])*s;
		q[k] = (m.m[k][i] + m.m[i][k])*s;

		v.x = q[0];
		v.y = q[1];
		v.z = q[2];
	}
}

// spherical linear interpolation
Quaternion Slerp(float t, const Quaternion &q1, const Quaternion &q2) {
	
	// compute angle between q1 and q2 
	float cos = Dot(q1, q2);
	
	// if angle is too small
	if (cos > .9995f)
		return Normalize((1.f - t)*q1 + t*q2);
	
	else {
		//Is the "cos"'s angle closer 180 angle than 0 angle? return the closer one: 180 or 0 angle.		
		float theta = acosf(Clamp(cos, -1.f, 1.f));
		
		// get the interpolate vector's angle. 
		float thetap = theta*t;
		
		//compute the normalize q' vector, which is orthogonal to q1
		Quaternion qperp = Normalize(q2 - q1*cos);

		// compuer the interpolate quaternion
		return q1*cosf(thetap) + qperp*sinf(thetap);
	}
}
