#include"pbrt.h"
class Vector {
public:

	//------------------------------------<vector public methods>---------------------------------------------
	
	//--------------------------------------------��ʼ��-----------------------------------------
	Vector() { x = y = z = 0.f; } //--------�趨������ʼֵ-------
	
	Vector(float xx, float yy, float zz) //------------��Assert�������������xyz�ǲ��Ǹ���ֵ-----------
		: x(xx), y(yy), z(zz) {         //-----------��xx,yy,zz��ֵ�������ڵ�xyz------------
		Assert(!HasNaNs());
	}
	
	bool HasNaNs() const { return isnan(x) || isnan(y) || isnan(z); } //-----------��isnan���xyz�ǲ���NaNs---

	explicit Vector(const Point &p);

	Vector(const Vector &v) {
		x = v.x, y = v.y, z = v.z;
	}

	Vector &operator=(const Vector &v) {
		x = v.x, y = v.y, z = v.z;
		return *this;
	}
	//-------------------------------------------���üӼ��˳�����--------------------------------------------
	Vector operator+(const Vector &v) const {  //----------����+��������ʹ2�������ܹ���ӣ�����һ���¶���---------------
		return Vector(x + v.x, y + v.y, z + v.z);
	}
	
	Vector& operator+=(const Vector &v){       //-----------����+=�����������ض���������ǰ���õ�Vector-------------
		x += v.x; y += v.y; z += v.z;          
		return *this;                         //----------���ص�ǰ���õĶ���+=��ߵ�Vector��-----
	}

	Vector operator-(const Vector &v) const {  //----------����-������----------------
		return Vector(x - v.x, y - v.y, z - v.z);
	}

	Vector& operator-=(const Vector &v) {       //-----------����-=������--------------
		x -= v.x; y -= v.y; z -= v.z;
		return *this;                         
	}

	Vector operator*(float f) const {         //----------���ء�*��������------------
		return Vector(f*x, f*y, f*z);
	}

	Vector &operator*=(float f) {            //----------����*=������---------------
		x *= f; y *= f; z *= f;
		return *this;
	}

	Vector operator/(float f)const {                 //-----���ء�/������----------
		Assert(f != 0);
		float inv = 1.f / f;
		return Vector(x*inv, y*inv, z*inv);
	}

	Vector &operator/=(float f) {
		Assert(f != 0);
		float inv = 1.f / f;
		x *= inv; y *= inv; z *= inv;
		return *this;
	}

	Vector operator-() const {                        //----------����һ���෴�����-��------------
		return Vector(-x, -y, -z);
	}

	float operator[](int i) const {                   //-----------�����±�--------------
		Assert(i >= 0 && i <= 2);
		return (&x)[i];
	}

	float &operator[](int i) {
		Assert(i >= 0 && i <= 2);
		return (&x)[i];
	}

	explicit Vector(const Normal &n);                 //---------------------����ת����----------------

	//------------------------------------------������׼��----------------------------------------
	float LengthSquared() const { return x*x + y*y + z*z; }    //---------��������ĳ���-------
	float Length() const { return sqrtf(LengthSquared()); }


	//---------------------------------------------<vector public data>------------------------------------


	float x, y, z;
};



class Point {
public:
	//-------------------------------------point public methods----------------------------
	//-------------------------------------��ʼ��-----------------------------------
	Point() { x = y = z = 0.f; }
	Point(float xx, float yy, float zz)
		:x(xx), y(yy), z(zz) {
	}

	Point(const Point &p) {
		x = p.x, y - p.y, z = p.z;
	}

	Point &operator=(const Point &p) {
		x = p.x, y = p.y, z = p.z;
		return *this;
	}

	//-----------------------------------�������-----------------------------------

	Point operator+(const Vector &v) const {                //-----------��+����=�µ�-------------
		return Point(x + v.x, y + v.y, z + v.z);
	}

	Point &operator+=(const Vector &v) {
		x += v.x; y += v.y; z += v.z;
		return *this;
	}

	Vector operator-(const Point &p) const {              //--------------��-��=����---------------
		return Vector(x - p.x, y - p.y, z - p.z);
	}

	Point operator-(const Vector &v) const {              //--------------��-����=��---------------
		return Point(x - v.x, y - v.y, z - v.z);
	}

	Point &operator-=(const Vector &v) {
		x -= v.x; y -= v.y; z -= v.z;
		return *this;
	}

	Point &operator+=(const Point &p) {                  //------------��+��--------------
		x += p.x; y += p.y; z += p.z;
		return *this;
	}

	Point operator+(const Point &p) const {
		return Point(x + p.x, y + p.y, z + p.z);
	}

	Point operator*(float f) const {         //----------���ء�*��������------------
		return Point(f*x, f*y, f*z);
	}

	Point &operator*=(float f) {            //----------����*=������---------------
		x *= f; y *= f; z *= f;
		return *this;
	}

	Point operator/(float f)const {         //-----------����/������-------------
		float inv = 1.f / f;
		return Point(x*inv,y*inv,z*inv);
	}

	Point &operator/=(float f) {
		float inv = 1.f / f;
		x *= inv, y *= inv, z *= inv;
		return *this;
	}
	 
	float operator[](int i) const {        //--------�趨�±�---------------
		Assert(i >= 0 && i <= 2);
		return (&x)[i];
	}

	float &operator[](int i) {
		Assert(i >= 0 && i <= 2);
		return (&x)[i];
	}

	bool operator==(const Point &p)const {
		return x == p.x&&y = p.y&&z = p.z;
	}

	bool operator!=(const Point &p)const {
		return x != p.x || y != p.y || z != p.z;
	}


	//-------------------------------------point public data--------------------------------
	float x, y, z;
};


class Normal {                         //-----------������-------------------------------
public:
	//--------------------------------------------��ʼ��---------------------------------
	Normal() { x = y = z = 0.f; }
	Normal(float xx, float yy, float zz) 
		: x(xx), y(yy), z(zz) {        
		Assert(!HasNaNs());
	}
	bool HasNaNs() const { return isnan(x) || isnan(y) || isnan(z); } 

	Normal(const Normal &n) {
		x = n.x, y = n.y, z = n.z;
	}

	Normal &operator=(const Normal &n) {
		x = n.x, y = n.y, z = n.z;
		return *this;
	}

	//--------------------------------------------normal public methods------------------
	Normal operator+(const Normal &v) const {  //----------����+������---------------
		return Normal(x + v.x, y + v.y, z + v.z);
	}

	Normal &operator+=(const Normal &v) {       //-----------����+=�����������ض�����-------------
		x += v.x; y += v.y; z += v.z;
		return *this;                        
	}

	Normal operator-(const Normal &n)const {        //-----------����-������-----------------
		return Normal(x - n.x, y - n.y, z - n.z);
	}

	Normal &operator-=(const Normal &n) {
		x -= n.x, y -= n.y, z -= n.z;
		return *this;
	}

	Normal operator-() const {                        //----------����һ���෴�����-��------------
		return Normal(-x, -y, -z);
	}

	float operator[](int i) const {					//---------�����±�--------
		Assert(i >= 0 && i <= 2);
		return (&x)[i];
	}

	float &operator[](int i) {
		Assert(i >= 0 && i <= 2);
		return (&x)[i];
	}

	Normal operator*(float f) const {         //----------���ء�*��������------------
		return Normal(f*x, f*y, f*z);
	}

	Normal &operator*=(float f) {            //----------����*=������---------------
		x *= f; y *= f; z *= f;
		return *this;
	}

	Normal operator/(float f) {                 //-----���ء�/������----------
		Assert(f != 0);
		float inv = 1.f / f;
		return Normal(x*inv, y*inv, z*inv);
	}

	Normal &operator/=(float f) {
		Assert(f != 0);
		float inv = 1.f / f;
		x *= f; y *= f; z *= f;
		return *this;
	}

	bool operator==(const Normal &n)const {
		return x == n.x&&y = n.y&&z = n.z;
	}

	bool operator!=(const Normal &p)const {
		return x != n.x || y != n.y || z != n.z;
	}


	//------------------------------------------���߱�׼��----------------------------------------
	float LengthSquared() const { return x*x + y*y + z*z; }    //---------������ߵĳ���-------
	float Length() const { return sqrtf(LengthSquared()); }


	explicit Normal(const Vector &v)          //-----------����ת����--------------------
		:x(v.x), y(v.y), z(v.z) {
	}
	//--------------------------------------------normal public data----------------------
	float x, y, z;
};


class Ray {
public:

	//--------------------------------------Ray public methods-------------------------------------

	//------------------------------------��ʼ��---------------------------------------------------
	Ray():mint(0.f),maxt(INFINITY),time(0.f),depth(0){}              //------3������Ϊ0��һ��Ϊ����󣬵������ҲΪ0������������0��---
	Ray(const Point &origin, const Vector &direction,                //------������ֵ��䣺��Ҫ���㡢��������С��Χ��ֵ---------------
		float start, float end = INFINITY, float t=0.f,int d = 0)
		:o(origin),d(direction),mint(start),maxt(end),time(t),depth(d){}

	Ray(const Point &origin,const Vector &direction,const Ray &parent,   //------�ӽ��������Ķ�������-------
		float start, float end = INFINITY)                               //------�̳и������ߵ�time����depth+1����������+1��-------------
		: o(origin), d(direction), mint(start), maxt(end),
		  time(parent.time), depth(parent.depth+1) {}

	Point operator()(float t) const { return o + d*t; }         //-----����Ray���������㣺��+����=�㣬�����Ǽ���Ray��һ��-------------


	//-------------------------------------Ray pubilic data----------------------------------------
	Point o;                            //------------������ԭ��ͷ��򹹳�-------------------------
	Vector d;

	mutable float mint, maxt;           //------------�������������Զ��-------------------------
	float time;                         //------------ʱ��������붯̬ģ�����---------------------
	int depth;                          //------------�ù������㷨׷�ٹ⵽�׵����˶��ٴ�-----------
};




class RayDifferential :public Ray {        //----------------�����Ƭƽ���ϣ���������ĳһ��������x��y�������������ƫ�ƣ�����������������Ϣ��-------------------
public:
	//----------------------------------------Ray Differential public Methods----------------------------------
	RayDifferential() { hasDifferentials = false; }
	RayDifferential(const Point &org, const Vector &dir, float start, float end = INFINITY, float t = 0.f, int d = 0)    //-------------------RayDiff��Ray���࣬������ֵһ��------
		:Ray(org, dir, start, end, t, d) {
		hasDifferentials = false;
	}
	RayDifferential(const Point &org, const Vector &dir, const Ray &parent,                                     //-------------------��Ray�Ķ������ߣ�depth+1��������RayDiff����--------------------
		float start, float end = INFINITY)
		:Ray(org, dir, start, end, parent.time, parent.depth + 1) {
		hasDifferentials = false;
	}

	explicit RayDifferential(const Ray &ray) :Ray(ray) {           //------��Ray����RayDiff����---------------
		hasDifferentials = false;                                  //------explicitǰ׺�������ú������ܱ���ʽ���ã���RayDiff = Ray������ʽ��ֵ��-----------
	}

	void ScaleDifferentials(float s) {                        //-----------�ӵ�ǰ�����ռ�s����RayDiff----------
		rxOrigin = o + (rxOrigin - o)*s;
		ryOrigin = o + (ryOrigin - o)*s;
		rxDirection = d + (rxDirection - d)*s;
		ryDirection = d + (ryDirection - d)*s;
	}
	//----------------------------------------RayDifferential public dara-------------------------------------

	bool hasDifferentials;
	Point rxOrigin, ryOrigin;
	Vector rxDirection, ryDirection;
};


class BBox {                     //------------------��Χ�������ά��Χ��:axis-aligned bounding boxes------
public:
	//-----------------------------------------bbox public methods------------------------------------------
	BBox() {                                            //---------��ʼ������С�����������С---------
		pMin = Point(INFINITY, INFINITY, INFINITY);
		pMax = Point(-INFINITY, -INFINITY, -INFINITY);
	}

	BBox(const Point &p) : pMin(p), pMax(p) { }         //---------��һ���㣬��С�㶼�������------------

	BBox(const Point &p1, const Point &p2) {            //----------��2���㣬����С��ֱ�ֵ2����������ֵ��Сֵ---
		pMin = Point(fminf(p1.x, p2.x), fminf(p1.y, p2.y), fminf(p1.z, p2.z));
		pMax = Point(fmaxf(p1.x, p2.x), fmaxf(p1.y, p2.y), fmaxf(p1.z, p2.z));
	}


	friend BBox Union(const BBox &b, const Point &p);
	friend BBox Union(const BBox &b, const BBox &b2);

	bool Overlaps(const BBox &b) const {               //-------------��⵱ǰ�����������Ƿ����ص�----------
		bool x = (pMax.x >= b.pMin.x) && (pMin.x <= b.pMax.x);
		bool y = (pMax.y >= b.pMin.y) && (pMin.y <= b.pMax.y);
		bool z = (pMax.z >= b.pMin.z) && (pMin.z <= b.pMax.z);
	}

	bool Inside(const Point &pt) const {               //--------------ȷ�����Ƿ�����ڰ�Χ����--------------
		return (pt.x >= pMin.x && pt.x <= pMax.x &&
			    pt.y >= pMin.y && pt.y <= pMax.y &&
			    pt.z >= pMin.z && pt.z <= pMax.z);
	}

	void Expand(float delta) {							//-----------��һ������չ�еĴ�С-------------
		pMin -= Vector(delta, delta, delta);
		pMax += Vector(delta, delta, delta);
	}

	float SurfaceArea() const {							//-------------�������������----------------
		Vector d = pMax - pMin;
		return 2.f*(d.x*d.y + d.x*d.z + d.y*d.z);
	}

	float Volume() const {
		Vector d = pMax - pMin;
		return d.x*d.y*d.z;
	}

	int MaxmumExtent() const {							//----------------�����ĸ��߳��������kd��-----------
		Vector diag = pMax - pMin;
		if (diag.x > diag.y && diag.x > diag.z)
			return 0;
		else if (diag.y > diag.z)
			return 1;
		else
			return 2;
	}

	const Point &operator[](int i) const;
	Point &operator[](int i);

	Point Lerp(float tx, float ty, float tz) const {			//-------------------������---------------
		return Point(::Lerp(tx, pMin.x, pMax.x), ::Lerp(ty, pMin.y, pMax.y),
			::Lerp(tz, pMin.z, pMax.z));
	}

	Vector Offset(const Point &p) const {						//---------------�������ں������λ��------
		return Vector((p.x - pMin.x) / (pMax.x - pMin.x),
			(p.y - pMin.y) / (pMax.y - pMin.y),
			(p.z - pMin.z) / (pMax.z - pMin.z));
	}

	void BoundingSphere(Point *c, float *rad) const;            //--------------�����һ�����е�����-----

	bool IntersectP(const Ray &ray, float *hitt0, float *hitt1) const;

	//-----------------------------------------bbox public data----------------------------------------------
	Point pMin, pMax;

};


//------------------------------------------Geometry Inline Functions-----------------------------------

//------------------------------------������ֵ-------------------------------
inline Vector::Vector(const Point &p)
	:x(p.x), y(p.y), z(p.z) {
}

//------------------------------------���������˷�---------------------------
inline Vector operator*(float f, const Vector &v) {            //------���������������������صġ�*����������������һ��*����--------
	return v*f;
}

//--------------------------------------------���õ�˺ͽ���˻�------------------------------------
inline float Dot(const Vector &v1, const Vector &v2) {         //-------������ˣ����������������нǵ�����cos-----------------
	return v1.x*v2.x + v1.y*v2.y + v1.z*v2.z;
}

inline float AbsDot(const Vector &v1, const Vector &v2) {      //---------��˵ľ���ֵ------------------
	return fabsf(Dot(v1, v2));
}

inline Vector Cross(const Vector &v1, const Vector &v2) {      //-------��ˣ��õ���2�������ഹֱ��1��������------------------
	return Vector((v1.y*v2.z) - (v1.z*v2.y),                   //-----------2��������˵�ģ����������ɵ�ƽ���ı��ε����------------
		(v1.z*v2.x) - (v1.x - v2.z),
		(v1.x*v2.y) - (v1.y*v2.x));
}

inline Vector Cross(const Vector &v1, const Normal &v2) {      //-------------------�����������Ĳ��-------------
	double v1x = v1.x, v1y = v1.y, v1z = v1.z;
	double v2x = v2.x, v2y = v2.y, v2z = v2.z;
	return Vector((v1y * v2z) - (v1z * v2y),
		(v1z * v2x) - (v1x * v2z),
		(v1x * v2y) - (v1y * v2x));
}


inline Vector Cross(const Normal &v1, const Vector &v2) {
	double v1x = v1.x, v1y = v1.y, v1z = v1.z;
	double v2x = v2.x, v2y = v2.y, v2z = v2.z;
	return Vector((v1y * v2z) - (v1z * v2y),
		(v1z * v2x) - (v1x * v2z),
		(v1x * v2y) - (v1y * v2x));
}

inline Vector Normalize(const Vector &v) { return v / v.Length; }    //-----���������Գ��ȣ��ó���λ����----

//------------------------------------------��һ�������������ֲ�����ϵ-------------------------------------------

inline void CoordinateSystem(const Vector &v1, Vector *v2, Vector *v3) {
	if (fabsf(v1.x) > fabsf(v1.y)) {
		float invLen = 1.f / sqrtf(v1.x*v1.x + v1.z*v1.z);
		*v2 = Vector(-v1.z*invLen, 0.f, v1.x*invLen);
	}
	else {
		float invLen = 1.f / sqrtf(v1.y*v1.y + v1.z*v1.z);
		*v2 = Vector(0.f, v1.z*invLen, -v1.y*invLen);
	}
	*v3 = Cross(v1, *v2);
}

//--------------------------------------�������----------------------------------------
inline float Distance(const Point &p1, const Point &p2) {
	return (p1 - p2).Length;
}

inline float DistanceSquared(const Point &p1, const Point &p2) {
	return (p1 - p2).LengthSquared;
}

inline Point operator*(float f, const Point &p) {
	return p*f;
}

//-----------------------------------��������-------------------------------------------------------

inline Normal Normalize(const Normal &n) { return n / n.Length; }    //--------�õ�λ����---------------

inline Vector::Vector(const Normal &n)               //------------����ת����------------------
	:x(n.x), y(n.y), z(n.z) {
}

inline Normal operator*(float f, const Normal &n) {         //-----------�ó���f������С����------------
	return Normal(f*n.x, f*n.y, f*n.z);
}

inline float Dot(const Normal &n1, const Normal &n2) {         //-------���ߵ�ˣ��������������߼нǵ�����cos-----------------
	return n1.x*n2.x + n1.y*n2.y + n1.z*n2.z;
}

inline float AbsDot(const Normal &n1, const Normal &n2) {      //---------���ߵ�˵ľ���ֵ------------------
	return fabsf(Dot(n1, n2));
}

//-------------------------��������������---------------------------------------------

inline float Dot(const Normal &n, const Vector &v) {         //-------�������������-----------------
	return n.x*v.x + n.y*v.y + n.z*v.z;
}

inline float Dot(const Vector &v1, const Normal &n2) {
	return v1.x * n2.x + v1.y * n2.y + v1.z * n2.z;
}

inline float Dot(const Normal &n1, const Normal &n2) {        //------------�����뷨�ߵ��-------------
	return n1.x * n2.x + n1.y * n2.y + n1.z * n2.z;
}

inline float AbsDot(const Normal &n, const Vector &v) {      //---------��˵ľ���ֵ------------------
	return fabsf(Dot(n,v));
}

inline float AbsDot(const Vector &v1, const Normal &n2) {
	return fabsf(v1.x * n2.x + v1.y * n2.y + v1.z * n2.z);
}

inline float AbsDot(const Normal &n1, const Normal &n2) {
	return fabsf(n1.x * n2.x + n1.y * n2.y + n1.z * n2.z);
}

inline Normal Faceforword(const Normal &n, const Vector &v) {      //-----------����������������ת��ͬһ����-------
	return(Dot(n, v) < 0.f) ? -n : n;
}

inline Normal Faceforward(const Normal &n, const Normal &n2) {    //--------------����ת����һ���ߵ�ͬ����--------
	return (Dot(n, n2) < 0.f) ? -n : n;
}



inline Vector Faceforward(const Vector &v, const Vector &v2) {      //----------����ת����һ������ͬ����--------
	return (Dot(v, v2) < 0.f) ? -v : v;
}



inline Vector Faceforward(const Vector &v, const Normal &n2) {      //-----------����ת����һ���ߵ�ͬ����---------
	return (Dot(v, n2) < 0.f) ? -v : v;
}



inline const Point &BBox::operator[](int i) const {                 //---------------��Χ�е��±�------------------
	Assert(i == 0 || i == 1);
	return (&pMin)[i];
}



inline Point &BBox::operator[](int i) {
	Assert(i == 0 || i == 1);
	return (&pMin)[i];
}



