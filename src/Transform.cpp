#include"Transform.h"

//----------------------Matrix Method Definitions---------------------

Matrix4x4::Matrix4x4(float mat[4][4]) {
	memcpy(m, mat, 16 * sizeof(float));
}


Matrix4x4::Matrix4x4(float t00, float t01, float t02, float t03,
	float t10, float t11, float t12, float t13,
	float t20, float t21, float t22, float t23,
	float t30, float t31, float t32, float t33) {
	m[0][0] = t00; m[0][1] = t01; m[0][2] = t02; m[0][3] = t03;
	m[1][0] = t10; m[1][1] = t11; m[1][2] = t12; m[1][3] = t13;
	m[2][0] = t20; m[2][1] = t21; m[2][2] = t22; m[2][3] = t23;
	m[3][0] = t30; m[3][1] = t31; m[3][2] = t32; m[3][3] = t33;
}


			//----------------------2x2 linear ststem--------------------
bool SolveLinearSystem2x2(const float A[2][2],
	const float B[2], float *x0, float *x1) {
	float det = A[0][0] * A[1][1] - A[0][1] * A[1][0];
	if (fabsf(det) < 1e-10f)
		return false;
	*x0 = (A[1][1] * B[0] - A[0][1] * B[1]) / det;
	*x1 = (A[0][0] * B[1] - A[1][0] * B[0]) / det;
	if (isnan(*x0) || isnan(*x1))
		return false;
	return true;
}
			//------------------------------返回转置矩阵---------------------
Matrix4x4 Transpose(const Matrix4x4 &m) {
	return Matrix4x4(m.m[0][0], m.m[1][0], m.m[2][0], m.m[3][0],
		m.m[0][1], m.m[1][1], m.m[2][1], m.m[3][1],
		m.m[0][2], m.m[1][2], m.m[2][2], m.m[3][2],
		m.m[0][3], m.m[1][3], m.m[2][3], m.m[3][3]);
}

Transform Translate(const Vector &delta) {       //--------平移矩阵---------
	Matrix4x4 m(1, 0, 0, delta.x,
		0, 1, 0, delta.y,
		0, 0, 1, delta.z,
		0, 0, 0, 1);
	Matrix4x4 minv(1, 0, 0, -delta.x,
		0, 1, 0, -delta.y,
		0, 0, 1, -delta.z,
		0, 0, 0, 1);
	return Transform(m, minv);
}

Transform Scale(float x, float y, float z) {      //---------比例变换矩阵--------
	Matrix4x4 m(x, 0, 0, 0,
		0, y, 0, 0,
		0, 0, z, 0,
		0, 0, 0, 1);
	Matrix4x4 minv(1.f/x, 0, 0, 0,
		0, 1.f/y, 0, 0,
		0, 0, 1.f/z, 0,
		0, 0, 0, 1);
	return Transform(m, minv);

}

Transform Rotate(float angle, const Vector &axis) {
	Vector a = Normalize(axis);
	float s = sinf(Radians(angle));
	float c = cosf(Radians(angle));
	float m[4][4];

	m[0][0] = a.x*a.x + (1.f - a.x*a.x)*c;
	m[0][1] = a.x*a.y*(1.f - c) - a.z*s;
	m[0][2] = a.x*a.z*(1.f - c) + a.y*s;
	m[0][3] = 0;

	m[1][0] = a.x*a.y*(1.f - c) + a.z*s;
	m[1][1] = a.y*a.y+(1.f - a.y*a.y)*c;
	m[1][2] = a.y*a.z*(1.f - c) - a.x*s;
	m[1][3] = 0;

	m[2][0] = a.x*a.z*(1.f - c) - a.y*s;
	m[2][1] = a.y*a.z*(1.f - c) + a.x*s;
	m[2][2] = a.z*a.z + (1.f - a.z*a.z)*c;
	m[2][3] = 0;

	m[3][0] = 0;
	m[3][1] = 0;
	m[3][2] = 0;
	m[3][3] = 1;

	Matrix4x4 mat(m);
	return Transform(mat,Transpose(mat));
}

            //-----------look-at变换，先求dir视线向量，再用它和已有up向量求出left向量，最后dir和left求出新的up------
Transform LookAt(const Point &pos, const Point &look, const Vector &up) {        
	float m[4][4];
	m[0][3] = pos.x;
	m[1][3] = pos.y;
	m[2][3] = pos.z;
	m[3][3] = 1;
	Vector dir = Normalize(look - pos);
	Vector left = Normalize(Cross(Normalize(up), dir));
	Vector newUp = Cross(dir, left);
	m[0][0] = left.x;
	m[1][0] = left.y;
	m[2][0] = left.z;
	m[3][0] = 0.;
	m[0][0] = newUp.x;
	m[1][0] = newUp.y;
	m[2][0] = newUp.z;
	m[3][0] = 0.;
	m[0][0] = dir.x;
	m[1][0] = dir.y;
	m[2][0] = dir.z;
	m[3][0] = 0.;


	Matrix4x4 camToWorld(m);
	return Transform(Inverse(camToWorld), camToWorld);
}

          //---------------------------求逆矩阵，Error和swap函数暂未引入声明和定义-----------

Matrix4x4 Inverse(const Matrix4x4 &m) {
	int indxc[4], indxr[4];
	int ipiv[4] = { 0, 0, 0, 0 };
	float minv[4][4];
	memcpy(minv, m.m, 4 * 4 * sizeof(float));
	for (int i = 0; i < 4; i++) {
		int irow = -1, icol = -1;
		float big = 0.;
		// Choose pivot
		for (int j = 0; j < 4; j++) {
			if (ipiv[j] != 1) {
				for (int k = 0; k < 4; k++) {
					if (ipiv[k] == 0) {
						if (fabsf(minv[j][k]) >= big) {
							big = float(fabsf(minv[j][k]));
							irow = j;
							icol = k;
						}
					}
					else if (ipiv[k] > 1)
						Error("Singular matrix in MatrixInvert");
				}
			}
		}
		++ipiv[icol];
		// Swap rows _irow_ and _icol_ for pivot
		if (irow != icol) {
			for (int k = 0; k < 4; ++k)
				swap(minv[irow][k], minv[icol][k]);
		}
		indxr[i] = irow;
		indxc[i] = icol;
		if (minv[icol][icol] == 0.)
			Error("Singular matrix in MatrixInvert");

		// Set $m[icol][icol]$ to one by scaling row _icol_ appropriately
		float pivinv = 1.f / minv[icol][icol];
		minv[icol][icol] = 1.f;
		for (int j = 0; j < 4; j++)
			minv[icol][j] *= pivinv;

		// Subtract this row from others to zero out their columns
		for (int j = 0; j < 4; j++) {
			if (j != icol) {
				float save = minv[j][icol];
				minv[j][icol] = 0;
				for (int k = 0; k < 4; k++)
					minv[j][k] -= minv[icol][k] * save;
			}
		}
	}
	// Swap columns to reflect permutation
	for (int j = 3; j >= 0; j--) {
		if (indxr[j] != indxc[j]) {
			for (int k = 0; k < 4; k++)
				swap(minv[k][indxr[j]], minv[k][indxc[j]]);
		}
	}
	return Matrix4x4(minv);
}


//--------------------Transformation decomposition------------------
void AnimatedTransform::Decompose(const Matrix4x4 &m, Vector *T, Quaternion *Rquat, Matrix4x4 *S) {
	
	// Translation T
	T->x = m.m[0][3];
	T->y = m.m[1][3];
	T->z = m.m[2][3];

	// remove translation, create a 4x4 new trasform matrix M
	Matrix4x4 M = m;
	for (int i = 0; i < 3; ++i)
		M.m[i][3] = M.m[3][i] = 0.f;
	M.m[3][3] = 1.f;

	//get the rotation R by a interative equation.
	float norm;
	int count = 0;
	Matrix4x4 R = M;  //M(i)
	do {

		// compute the interative equation: M(i+1) = 0.5*( M(i) + M(i).T.I )
		Matrix4x4 Rnext; //M(i+1)
		Matrix4x4 Rit = Inverse(Transpose(R)); //M(i).T.I
		for (int i = 0; i < 4; ++i)
			for (int j = 0; j < 4; ++j)
				Rnext.m[i][j] = 0.5f*(R.m[i][j] + Rit.m[i][j]);

		//compute the difference between M(i) and M(i+1)
		norm = 0.f;
		for (int i = 0; i < 3; ++i) {
			float n = fabsf(R.m[i][0] - Rnext.m[i][0] +
				R.m[i][1] - Rnext.m[i][1] +
				R.m[i][2] - Rnext.m[i][2]);
			norm = max(norm, n);
		}
	} while (++count < 100 && norm > .0001f);   // set the end of iterative action.
	*Rquat = Quaternion(R);

	// get the scale S: S = R.I x M
	*S = Matrix4x4::Mul(Inverse(R), M);
}

void AnimatedTransform::Interpolate(float time, Transform *t) const {

	// make sure that time is in the range from start to end.
	if (!actuallyAnimated || time <= startTime) {
		*t = *startTransform;
		return;
	}
	if (time >= endTime) {
		*t = *endTransform;
		return;
	}

	// set the relative value of time in the range (0,1)
	float dt = (time - startTime) / (endTime - startTime);

	// interpolate translation
	Vector trans = (1.f - dt)*T[0] + dt*T[1];

	// interpolate rotation
	Quaternion rotation = slerp(dt, R[0], R[1]);

	// interpolate scale
	Matrix4x4 scale;
	for (int i = 0; i < 3; ++i)
		for (int j = 0; j < 3; ++j)
			scale.m[i][j] = Lerp(dt, S[0].m[i][j], S[1].m[i][j]);

	// compute new transform by the product of Trans, Rotation and Scale
	// notice: they should be change into the matrix style.
	*t = Translate(trans)*rotation.ToTransform*Transform(scale);
}

BBox AnimatedTransform::MotionBounds(const BBox &b, bool useInverse)const {
	
	// check the time's range
	if (!actuallyAnimated) return Inverse(*startTransform)(b);

	// compute a bbox include 128 steps in the animated motion 
	BBox ret;
	const int nSteps = 128;
		//In a step, compute the transform "t" and its BBox "t(b)", than merge into the BBox "ret".
	for (int i = 0; i < nSteps; ++i) {
		Transform t;
		float time = Lerp(float(i) / float(nSteps), startTime, endTime);
		Interpolate(time, &t);
		if (useInverse)
			t = Inverse(t);
		ret = Union(ret, t(b));
	}
	return ret;

}

