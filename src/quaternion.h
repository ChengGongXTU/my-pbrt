#include "pbrt.h"
#include "Geometry.h"

class Quaternion {
public:
	//-----------------------quaternion public method--------------------------------
	// initialize 
	Quaternion() { v = Vector(0, 0, 0); w = 1.f; }
	Quaternion(Vector vv,float ww)
		:v(vv),w(ww){}
	Quaternion(const Quaternion &q) {
		v = q.v, w = q.w;
	}
	Quaternion &operator=(const Quaternion &q){
		v = q.v; w = q.w;
		return *this;
	}
	// addition
	Quaternion operator+(const Quaternion &q) const{
		return Quaternion(v + q.v, w + q.w);
	}

	Quaternion &operator+=(const Quaternion &q) {
		v += q.v; w += q.w;
		return *this;
	}

	// subtractio
	Quaternion operator-(const Quaternion &q) const {
		return Quaternion(v - q.v, w - q.w);
	}

	Quaternion &operator-=(const Quaternion &q) {
		v -= q.v; w -= q.w;
		return *this;
	}

	// multiplication by float
	Quaternion operator*(float f)const {
		return Quaternion(v*f, w*f);
	}

	Quaternion &operator*=(float f) {
		v *=f; w *=f;
		return *this;
	}

	// division by float
	Quaternion operator/(float f)const {
		Assert(f != 0);
		float inv = 1.f / f;
		return Quaternion(v / f, w / f);
	}

	Quaternion &operator/=(float f) {
		Assert(f != 0);
		v /= f; w /= f;
		return *this;
	}

	// create a Transform M from Quaternion q
	Transform ToTransform() const;

	// change this quaternion q base on Transform t
	Quaternion(const Transform &t);




	//-----------------------quaternion public data----------------------------------
	Vector v;
	float w;
};

// slerp is a independent funcion, not belong Quaternion class.
Quaternion slerp(float f, const Quaternion &q1, const Quaternion &q2);


//--------------------------Quaternion inline function--------------------------------
// inner product by two Quaternions
inline float Dot(const Quaternion &q1, const Quaternion &q2) {
	return Dot(q1.v, q2.v) + q1.w*q2.w;
}

// multiplicatiopn: f*quaternion
inline Quaternion operator*(float f, const Quaternion &q) {
	return q*f;
}

// normalize
inline Quaternion Normalize(const Quaternion &q) {
	return q / sqrt(Dot(q, q));
}