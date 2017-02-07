#include"pbrt.h"
#include"Geometry.h"
#include"quaternion.h"

struct Matrix4x4 {
	//------------------初始化------------------------------------
	Matrix4x4() {
		m[0][0] = m[2][2] = m[3][3] = m[4][4] = 1.f;
		m[0][1] = m[0][2] = m[0][3] =
			m[1][0] = m[1][2] = m[1][3] =
			m[2][0] = m[2][1] = m[2][3] =
			m[3][0] = m[3][1] = m[3][2] = 0.f;
	}

	//------------------matrix4x4 public methos----------------------------
	Matrix4x4(float mat[4][4]);
	Matrix4x4(float t00, float t01, float t02, float t03,
		float t10, float t11, float t12, float t13,
		float t20, float t21, float t22, float t23,
		float t30, float t31, float t32, float t33);

	static Matrix4x4 Mul(const Matrix4x4 &m1, const Matrix4x4 &m2) {    //----计算矩阵相乘---
		Matrix4x4 r;
		for (int i = 0; i < 4; i++)
			for (int j = 0; j < 4; j++)
				r.m[i][j] = m1.m[i][0] * m2.m[0][j] +
				m1.m[i][1] * m2.m[1][j] +
				m1.m[i][2] * m2.m[2][j] +
				m1.m[i][3] * m2.m[3][j];
		return r;
	}
	friend Matrix4x4 Transpose(const Matrix4x4 &);                     //-------矩阵求转置和求逆（未完成！）-----------
	friend Matrix4x4 Inverse(const Matrix4x4 &);

	bool operator==(const Matrix4x4 &m2) const {					//----------求两个矩阵相同或不同-----------
		for (int i = 0; i < 4; ++i)
			for (int j = 0; j < 4; ++j)
				if (m[i][j] != m2.m[i][j]) return false;
		return true;
	}
	bool operator!=(const Matrix4x4 &m2) const {
		for (int i = 0; i < 4; ++i)
			for (int j = 0; j < 4; ++j)
				if (m[i][j] != m2.m[i][j]) return true;
		return false;
	}
	//---------------------------data---------------------------------------
	float m[4][4];
};





class Transform {
public:
	//------------------------transform public methods------------------------------
	 //---------初始化：Matrix4x4函数已赋默认值-----------
	Transform(){}             
	Transform(const float mat[4][4]) {
		m = Matrix4x4(mat[0][0], mat[0][1], mat[0][2], mat[0][3],
			mat[1][0], mat[1][1], mat[1][2], mat[1][3],
			mat[2][0], mat[2][1], mat[2][2], mat[2][3],
			mat[3][0], mat[3][1], mat[3][2], mat[3][3]);
		mInv = Inverse(m);
	}
	Transform(const Matrix4x4 &mat)
		:m(mat), mInv(Inverse(mat)) {
	}

	Transform(const Matrix4x4 &mat, const Matrix4x4 &minv)    //------直接给出矩阵和逆矩阵----
		:m(mat), mInv(minv) {
	}

	//------矩阵和逆矩阵相互交换------------------
	friend Transform Inverse(const Transform &t) {    
		return Transform(t.mInv, t.m);
	}

	//-----平移矩阵和它的逆矩阵T------------
	friend Transform Translate(const Vector &delta); 

	//--------------------比例变换S-------------------
	friend Transform Scale(float x, float y, float z);

	bool HasScale()const {                                   //------检测是否三项都为1-------
		float la2 = (*this)(Vector(1, 0, 0)).LengthSquared;
		float lb2 = (*this)(Vector(0, 1, 0)).LengthSquared;
		float lc2 = (*this)(Vector(0, 0, 1)).LengthSquared;
	#define NOT_ONE(x)((x)<.999f||(x)>1.001f)                 //-------这里通过宏定义，用条件检测三个项是否是1---------
		return (NOT_ONE(la2) || NOT_ONE(lb2) || NOT_ONE(lc2));
	#undef NOT_ONE
	}

	//-----------------------旋转变换R--------------------------
	friend Transform RotationX(float angle) {
		float sin_t = sinf(Radians(angle));
		float cos_t = cosf(Radians(angle));
		Matrix4x4 m(1, 0, 0, 0,
			0, cos_t, -sin_t, 0,
			0, sin_t, cos_t, 0,
			0, 0, 0, 1);
		return Transform(m, Transpose(m));
	}

	friend Transform Rotate(float angle, const Vector &axis);   //---------任意旋转轴的旋转变换----------

	//----------------look-at变换--------------------------------
	friend Transform LookAt(const Point &pos, const Point &look, const Vector &up);

	//-------------------------------------转换矩阵与点和向量的运算------------------------
	inline Point operator()(const Point &pt) const;
	inline void operator()(const Point &pt, Point *ptrans)const;

	inline Vector operator()(const Vector &v) const;

	inline void operator()(const Vector &v, Vector *vtrans)const;

	inline Normal operator()(const Normal &n)const;

	inline Ray operator()(const Ray &r)const;

	inline void operator()(const Ray &r, Ray *rt) const;

	inline BBox operator()(const BBox &b)const;

	//---------------矩阵之间的运算---------------------------------------
	inline Transform operator*(const Transform &t2)const;        //---右乘t2，用于多重转换的矩阵推导---

	//-------------检测手性变换-----------------
	bool SwapsHandedness()const {
		float det = ((m.m[0][0] * (m.m[1][1] * m.m[2][2] - m.m[1][2] * m.m[2][1])) -
			(m.m[0][1] * (m.m[1][0] * m.m[2][2] - m.m[1][2] * m.m[2][0])) -
			(m.m[0][2] * (m.m[0][1] * m.m[2][1] - m.m[1][1] * m.m[2][0])));
		return det < 0.f;
	}

	//--------------bool operate--------------
	bool operator!=(const Transform &t)const {
		return m != t.m || mInv != t.mInv;
	}
	bool operator==(const Transform &t)const {
		return m == t.m && mInv == t.mInv;
	}

private:
	//-------------------------transform private data--------------------------------
	Matrix4x4 m, mInv;                    //---------含有一个矩阵和相应的逆矩阵------
	friend struct Quaternion;
	friend class AnimatedTransform;

};


class AnimatedTransform {
	//-----------------------------------------------
	AnimatedTransform(const Transform *transform1, float time1,
		const Transform *transform2, float time2)
		:startTime(time1), endTime(time2),
		startTransform(transform1), endTransform(transform2),
		actuallyAnimated(*startTransform != *endTransform) {
		Decompose(startTransform->m, &T[0], &R[0], &S[0]);
		Decompose(endTransform->m, &T[1], &R[1], &S[1]);
	}
	
	//-----------------------publicb method-------------------------
	static void Decompose(const Matrix4x4 &m, Vector *T, Quaternion *Rquat, Matrix4x4 *S);
	void Interpolate(float time, Transform *t)const;
	BBox MotionBounds(const BBox &b, bool useInverse)const;

	//-----------------------private data---------------------------
	const Transform *startTransform;
	const Transform *endTransform;
	const float startTime;
	const float endTime;
	const bool actuallyAnimated;
	Vector T[2];
	Quaternion R[2];
	Matrix4x4 S[2];
};


//-------------------------------------transform inline function----------------------

inline Point Transform::operator()(const Point &pt)const {              //---点坐标转为齐次列向量坐标，左乘转换矩阵，再除以齐次权重wp，得出新的点坐标------
	float x = pt.x, y = pt.y, z = pt.z;
	float xp = m.m[0][0] * x + m.m[0][1] * y + m.m[0][2] * z + m.m[0][3];
	float yp = m.m[1][0] * x + m.m[1][1] * y + m.m[1][2] * z + m.m[1][3];
	float zp = m.m[2][0] * x + m.m[2][1] * y + m.m[2][2] * z + m.m[2][3];
	float wp = m.m[3][0] * x + m.m[3][1] * y + m.m[3][2] * z + m.m[3][3];
	if (wp == 1.) return Point(xp, yp, zp);
	else return Point(xp, yp, zp) / wp;
}

inline void Transform::operator()(const Point &pt, Point *ptrans)const {       //----------同上，将结果储存到指针指向的变量中---------
	float x = pt.x, y = pt.y, z = pt.z;
	ptrans->x = m.m[0][0] * x + m.m[0][1] * y + m.m[0][2] * z + m.m[0][3];
	ptrans->y = m.m[1][0] * x + m.m[1][1] * y + m.m[1][2] * z + m.m[1][3];
	ptrans->z = m.m[2][0] * x + m.m[2][1] * y + m.m[2][2] * z + m.m[2][3];
	float w = m.m[3][0] * x + m.m[3][1] * y + m.m[3][2] * z + m.m[3][3];
	if (w != 1.) *ptrans /= w;
}

inline Vector Transform::operator()(const Vector &v)const{                 //----------vector的齐次权重w=0，所以它的齐次列向量左乘转换矩阵后，可转为一样的行向量------
	float x = v.x, y = v.y, z = v.z;
	return Vector(m.m[0][0]*x+m.m[0][1]*y+m.m[0][2]*z,
		m.m[1][0] * x + m.m[1][1] * y + m.m[1][2] * z,
		m.m[2][0] * x + m.m[2][1] * y + m.m[2][2] * z);
}

inline void Transform::operator()(const Vector &v, Vector *vtrans)const {
	float x = v.x, y = v.y, z = v.z;
	vtrans->x=m.m[0][0] * x + m.m[0][1] * y + m.m[0][2] * z;
	vtrans->y=m.m[1][0] * x + m.m[1][1] * y + m.m[1][2] * z;
	vtrans->z=m.m[2][0] * x + m.m[2][1] * y + m.m[2][2] * z;
}

inline Normal Transform::operator()(const Normal &n)const {                //------------------------法线转换矩阵，需要列向量左乘转置的逆转换矩阵---------------------
	float x = n.x, y = n.y, z = n.z;
	return Normal(mInv.m[0][0] * x + mInv.m[1][0] * y + mInv.m[2][0] * z,
		mInv.m[0][1] * x + mInv.m[1][1] * y + mInv.m[2][1] * z,
		mInv.m[0][2] * x + mInv.m[1][2] * y + mInv.m[2][2] * z);
}

inline Ray Transform::operator()(const Ray &r)const {                   //-----转换射线：分别转换其中的源点和方向向量--------
	Ray ret = r;
	(*this)(ret.o, &ret.o);
	(*this)(ret.d, &ret.d);
	return ret;
}

inline void Transform::operator()(const Ray &r, Ray *rt) const {
	(*this)(r.o, &rt->o);
	(*this)(r.d, &rt->d);
	if (rt != &r) {
		rt->mint = r.mint;
		rt->maxt = r.maxt;
		rt->time = r.time;
		rt->depth = r.depth;
	}
}

inline BBox Transform::operator()(const BBox &b)const {              //----转换包围盒：逐个转换顶点（太麻烦了！）----------
	const Transform &M = *this;
	BBox ret(M(Point(b.pMin.x, b.pMin.y, b.pMin.z)));                //--------可以换个方法：你可以求出最大顶点和最小顶点之间的向量
	ret = Union(ret, M(Point(b.pMax.x, b.pMin.y, b.pMin.z)));		 //--------然后用转换这个向量，把新向量输入BBox函数，构建新BBox
	ret = Union(ret, M(Point(b.pMin.x, b.pMax.y, b.pMin.z)));
	ret = Union(ret, M(Point(b.pMin.x, b.pMin.y, b.pMax.z)));
	ret = Union(ret, M(Point(b.pMax.x, b.pMax.y, b.pMin.z)));
	ret = Union(ret, M(Point(b.pMax.x, b.pMin.y, b.pMax.z)));
	ret = Union(ret, M(Point(b.pMin.x, b.pMax.y, b.pMax.z)));
	ret = Union(ret, M(Point(b.pMax.x, b.pMax.y, b.pMax.z)));
	return ret;
}

inline Transform Transform::operator*(const Transform &t2)const {       //--------当前矩阵右乘一个t2矩阵---------------
	Matrix4x4 m1 = Matrix4x4::Mul(m, t2.m);
	Matrix4x4 m2 = Matrix4x4::Mul(t2.mInv, mInv);
	return Transform(m1, m2);
}
