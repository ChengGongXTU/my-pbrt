#include"pbrt.h"
class Vector {
public:

	//------------------------------------<vector public methods>---------------------------------------------
	
	//--------------------------------------------初始化-----------------------------------------
	Vector() { x = y = z = 0.f; } //--------设定向量初始值-------
	
	Vector(float xx, float yy, float zz) //------------用Assert函数检查向量的xyz是不是浮点值-----------
		: x(xx), y(yy), z(zz) {         //-----------将xx,yy,zz赋值给函数内的xyz------------
		Assert(!HasNaNs());
	}
	
	bool HasNaNs() const { return isnan(x) || isnan(y) || isnan(z); } //-----------用isnan检查xyz是不是NaNs---

	explicit Vector(const Point &p);

	Vector(const Vector &v) {
		x = v.x, y = v.y, z = v.z;
	}

	Vector &operator=(const Vector &v) {
		x = v.x, y = v.y, z = v.z;
		return *this;
	}
	//-------------------------------------------设置加减乘除方法--------------------------------------------
	Vector operator+(const Vector &v) const {  //----------重载+操作符，使2个向量能够相加，返回一个新对象---------------
		return Vector(x + v.x, y + v.y, z + v.z);
	}
	
	Vector& operator+=(const Vector &v){       //-----------重载+=操作符，返回对象本身，即当前调用的Vector-------------
		x += v.x; y += v.y; z += v.z;          
		return *this;                         //----------返回当前调用的对象“+=左边的Vector”-----
	}

	Vector operator-(const Vector &v) const {  //----------重载-操作符----------------
		return Vector(x - v.x, y - v.y, z - v.z);
	}

	Vector& operator-=(const Vector &v) {       //-----------重载-=操作符--------------
		x -= v.x; y -= v.y; z -= v.z;
		return *this;                         
	}

	Vector operator*(float f) const {         //----------重载“*”操作符------------
		return Vector(f*x, f*y, f*z);
	}

	Vector &operator*=(float f) {            //----------重载*=操作符---------------
		x *= f; y *= f; z *= f;
		return *this;
	}

	Vector operator/(float f) {                 //-----重载“/”除法----------
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

	Vector operator-() const {                        //----------重载一个相反算符‘-’------------
		return Vector(-x, -y, -z);
	}

	float operator[](int i) const {                   //-----------建立下标--------------
		Assert(i >= 0 && i <= 2);
		return (&x)[i];
	}

	float &operator[](int i) {
		Assert(i >= 0 && i <= 2);
		return (&x)[i];
	}

	explicit Vector(const Normal &n);                 //---------------------法线转向量----------------

	//------------------------------------------向量标准化----------------------------------------
	float LengthSquared() const { return x*x + y*y + z*z; }    //---------求出向量的长度-------
	float Length() const { return sqrtf(LengthSquared()); }


	//---------------------------------------------<vector public data>------------------------------------


	float x, y, z;
};



class Point {
public:
	//-------------------------------------point public methods----------------------------
	//-------------------------------------初始化-----------------------------------
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

	//-----------------------------------点的运算-----------------------------------

	Point operator+(const Vector &v) const {                //-----------点+向量=新点-------------
		return Point(x + v.x, y + v.y, z + v.z);
	}

	Point &operator+=(const Vector &v) {
		x += v.x; y += v.y; z += v.z;
		return *this;
	}

	Vector operator-(const Point &p) const {              //--------------点-点=向量---------------
		return Vector(x - p.x, y - p.y, z - p.z);
	}

	Point operator-(const Vector &v) const {              //--------------点-向量=点---------------
		return Point(x - v.x, y - v.y, z - v.z);
	}

	Point &operator-=(const Vector &v) {
		x -= v.x; y -= v.y; z -= v.z;
		return *this;
	}

	Point &operator+=(const Point &p) {                  //------------点+点--------------
		x += p.x; y += p.y; z += p.z;
		return *this;
	}

	Point operator+(const Point &p) const {
		return Point(x + p.x, y + p.y, z + p.z);
	}

	Point operator*(float f) const {         //----------重载“*”操作符------------
		return Point(f*x, f*y, f*z);
	}

	Point &operator*=(float f) {            //----------重载*=操作符---------------
		x *= f; y *= f; z *= f;
		return *this;
	}

	Point operator/(float f)const {         //-----------重载/操作符-------------
		float inv = 1.f / f;
		return Point(x*inv,y*inv,z*inv);
	}

	Point &operator/=(float f) {
		float inv = 1.f / f;
		x *= inv, y *= inv, z *= inv;
		return *this;
	}
	 
	float operator[](int i) const {        //--------设定下标---------------
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


class Normal {                         //-----------法线类-------------------------------
public:
	//--------------------------------------------初始化---------------------------------
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
	Normal operator+(const Normal &v) const {  //----------重载+操作符---------------
		return Normal(x + v.x, y + v.y, z + v.z);
	}

	Normal &operator+=(const Normal &v) {       //-----------重载+=操作符，返回对象本身-------------
		x += v.x; y += v.y; z += v.z;
		return *this;                        
	}

	Normal operator-(const Normal &n)const {        //-----------重载-操作符-----------------
		return Normal(x - n.x, y - n.y, z - n.z);
	}

	Normal &operator-=(const Normal &n) {
		x -= n.x, y -= n.y, z -= n.z;
		return *this;
	}

	Normal operator-() const {                        //----------重载一个相反算符‘-’------------
		return Normal(-x, -y, -z);
	}

	float operator[](int i) const {					//---------建立下标--------
		Assert(i >= 0 && i <= 2);
		return (&x)[i];
	}

	float &operator[](int i) {
		Assert(i >= 0 && i <= 2);
		return (&x)[i];
	}

	Normal operator*(float f) const {         //----------重载“*”操作符------------
		return Normal(f*x, f*y, f*z);
	}

	Normal &operator*=(float f) {            //----------重载*=操作符---------------
		x *= f; y *= f; z *= f;
		return *this;
	}

	Normal operator/(float f) {                 //-----重载“/”除法----------
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


	//------------------------------------------法线标准化----------------------------------------
	float LengthSquared() const { return x*x + y*y + z*z; }    //---------求出法线的长度-------
	float Length() const { return sqrtf(LengthSquared()); }


	explicit Normal(const Vector &v)          //-----------向量转法线--------------------
		:x(v.x), y(v.y), z(v.z) {
	}
	//--------------------------------------------normal public data----------------------
	float x, y, z;
};


class Ray {
public:

	//--------------------------------------Ray public methods-------------------------------------

	//------------------------------------初始化---------------------------------------------------
	Ray():mint(0.f),maxt(INFINITY),time(0.f),depth(0){}              //------3个变量为0，一个为无穷大，点和向量也为0（其类中已设0）---
	Ray(const Point &origin, const Vector &direction,                //------函数赋值语句：需要给点、向量和最小范围赋值---------------
		float start, float end = INFINITY, float t=0.f,int d = 0)
		:o(origin),d(direction),mint(start),maxt(end),time(t),depth(d){}

	Ray(const Point &origin,const Vector &direction,const Ray &parent,   //------从交点引出的额外射线-------
		float start, float end = INFINITY)                               //------继承父辈射线的time，且depth+1（弹跳次数+1）-------------
		: o(origin), d(direction), mint(start), maxt(end),
		  time(parent.time), depth(parent.depth+1) {}

	Point operator()(float t) const { return o + d*t; }         //-----重载Ray（），计算：点+向量=点，本质是计算Ray上一点-------------


	//-------------------------------------Ray pubilic data----------------------------------------
	Point o;                            //------------射线由原点和方向构成-------------------------
	Vector d;

	mutable float mint, maxt;           //------------作用域：最近和最远点-------------------------
	float time;                         //------------时间参数：与动态模糊相关---------------------
	int depth;                          //------------让光输运算法追踪光到底弹跳了多少次-----------
};




class RayDifferential :public Ray {        //----------------计算底片平面上，主射线与某一采样点在x和y方向上相机射线偏移：（两个额外射线信息）-------------------
public:
	//----------------------------------------Ray Differential public Methods----------------------------------
	RayDifferential() { hasDifferentials = false; }
	RayDifferential(const Point &org, const Vector &dir, float start, float end = INFINITY, float t = 0.f, int d = 0)    //-------------------RayDiff是Ray子类，变量赋值一样------
		:Ray(org, dir, start, end, t, d) {
		hasDifferentials = false;
	}
	RayDifferential(const Point &org, const Vector &dir, const Ray &parent,                                     //-------------------从Ray的额外射线（depth+1）引出的RayDiff子类--------------------
		float start, float end = INFINITY)
		:Ray(org, dir, start, end, parent.time, parent.depth + 1) {
		hasDifferentials = false;
	}

	explicit RayDifferential(const Ray &ray) :Ray(ray) {           //------从Ray创建RayDiff子类---------------
		hasDifferentials = false;                                  //------explicit前缀表明：该函数不能被隐式调用（如RayDiff = Ray这类隐式赋值）-----------
	}

	void ScaleDifferentials(float s) {                        //-----------从当前采样空间s更新RayDiff----------
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


class BBox {                     //------------------包围对象的三维包围盒:axis-aligned bounding boxes------
public:
	//-----------------------------------------bbox public methods------------------------------------------
	BBox() {                                            //---------初始化：最小店最大，最大点最小---------
		pMin = Point(INFINITY, INFINITY, INFINITY);
		pMax = Point(-INFINITY, -INFINITY, -INFINITY);
	}

	BBox(const Point &p) : pMin(p), pMax(p) { }         //---------给一个点，大小点都是这个点------------

	BBox(const Point &p1, const Point &p2) {            //----------给2个点，给大小点分别赋值2点分量的最大值最小值---
		pMin = Point(fminf(p1.x, p2.x), fminf(p1.y, p2.y), fminf(p1.z, p2.z));
		pMax = Point(fmaxf(p1.x, p2.x), fmaxf(p1.y, p2.y), fmaxf(p1.z, p2.z));
	}


	friend BBox Union(const BBox &b, const Point &p);
	friend BBox Union(const BBox &b, const BBox &b2);

	bool Overlaps(const BBox &b) const {               //-------------检测当前盒与所给盒是否有重叠----------
		bool x = (pMax.x >= b.pMin.x) && (pMin.x <= b.pMax.x);
		bool y = (pMax.y >= b.pMin.y) && (pMin.y <= b.pMax.y);
		bool z = (pMax.z >= b.pMin.z) && (pMin.z <= b.pMax.z);
	}

	bool Inside(const Point &pt) const {               //--------------确定点是否包含在包围盒内--------------
		return (pt.x >= pMin.x && pt.x <= pMax.x &&
			    pt.y >= pMin.y && pt.y <= pMax.y &&
			    pt.z >= pMin.z && pt.z <= pMax.z);
	}

	void Expand(float delta) {							//-----------用一个数扩展盒的大小-------------
		pMin -= Vector(delta, delta, delta);
		pMax += Vector(delta, delta, delta);
	}

	float SurfaceArea() const {							//-------------计算表面积和体积----------------
		Vector d = pMax - pMin;
		return 2.f*(d.x*d.y + d.x*d.z + d.y*d.z);
	}

	float Volume() const {
		Vector d = pMax - pMin;
		return d.x*d.y*d.z;
	}

	int MaxmumExtent() const {							//----------------计算哪个边长最长，用于kd树-----------
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

	Point Lerp(float tx, float ty, float tz) const {			//-------------------不明白---------------
		return Point(::Lerp(tx, pMin.x, pMax.x), ::Lerp(ty, pMin.y, pMax.y),
			::Lerp(tz, pMin.z, pMax.z));
	}

	Vector Offset(const Point &p) const {						//---------------所给点在盒中相对位置------
		return Vector((p.x - pMin.x) / (pMax.x - pMin.x),
			(p.y - pMin.y) / (pMax.y - pMin.y),
			(p.z - pMin.z) / (pMax.z - pMin.z));
	}

	void BoundingSphere(Point *c, float *rad) const;            //--------------求盒中一点相切的球形-----


	//-----------------------------------------bbox public data----------------------------------------------
	Point pMin, pMax;

};


//------------------------------------------Geometry Inline Functions-----------------------------------

//------------------------------------向量赋值-------------------------------
inline Vector::Vector(const Point &p)
	:x(p.x), y(p.y), z(p.z) {
}

//------------------------------------设置向量乘法---------------------------
inline Vector operator*(float f, const Vector &v) {            //------定义内联函数，用已重载的“*”操作符，重载另一种*操作--------
	return v*f;
}

//--------------------------------------------设置点乘和交叉乘积------------------------------------
inline float Dot(const Vector &v1, const Vector &v2) {         //-------向量点乘，本质是两个向量夹角的余弦cos-----------------
	return v1.x*v2.x + v1.y*v2.y + v1.z*v2.z;
}

inline float AbsDot(const Vector &v1, const Vector &v2) {      //---------点乘的绝对值------------------
	return fabsf(Dot(v1, v2));
}

inline Vector Cross(const Vector &v1, const Vector &v2) {      //-------叉乘，得到与2个向量相垂直的1个新向量------------------
	return Vector((v1.y*v2.z) - (v1.z*v2.y),                   //-----------2个向量叉乘的模用来计算组成的平行四边形的面积------------
		(v1.z*v2.x) - (v1.x - v2.z),
		(v1.x*v2.y) - (v1.y*v2.x));
}

inline Vector Cross(const Vector &v1, const Normal &v2) {      //-------------------法线与向量的叉乘-------------
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

inline Vector Normalize(const Vector &v) { return v / v.Length; }    //-----分向量除以长度，得出单位向量----

//------------------------------------------用一个向量，建立局部坐标系-------------------------------------------

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

//--------------------------------------点的运算----------------------------------------
inline float Distance(const Point &p1, const Point &p2) {
	return (p1 - p2).Length;
}

inline float DistanceSquared(const Point &p1, const Point &p2) {
	return (p1 - p2).LengthSquared;
}

inline Point operator*(float f, const Point &p) {
	return p*f;
}

//-----------------------------------法线运算-------------------------------------------------------

inline Normal Normalize(const Normal &n) { return n / n.Length; }    //--------得单位法线---------------

inline Vector::Vector(const Normal &n)               //------------法线转向量------------------
	:x(n.x), y(n.y), z(n.z) {
}

inline Normal operator*(float f, const Normal &n) {         //-----------用常数f扩大缩小法线------------
	return Normal(f*n.x, f*n.y, f*n.z);
}

inline float Dot(const Normal &n1, const Normal &n2) {         //-------法线点乘，本质是两个法线夹角的余弦cos-----------------
	return n1.x*n2.x + n1.y*n2.y + n1.z*n2.z;
}

inline float AbsDot(const Normal &n1, const Normal &n2) {      //---------法线点乘的绝对值------------------
	return fabsf(Dot(n1, n2));
}

//-------------------------法线与向量运算---------------------------------------------

inline float Dot(const Normal &n, const Vector &v) {         //-------法线与向量点乘-----------------
	return n.x*v.x + n.y*v.y + n.z*v.z;
}

inline float Dot(const Vector &v1, const Normal &n2) {
	return v1.x * n2.x + v1.y * n2.y + v1.z * n2.z;
}

inline float Dot(const Normal &n1, const Normal &n2) {        //------------法线与法线点乘-------------
	return n1.x * n2.x + n1.y * n2.y + n1.z * n2.z;
}

inline float AbsDot(const Normal &n, const Vector &v) {      //---------点乘的绝对值------------------
	return fabsf(Dot(n,v));
}

inline float AbsDot(const Vector &v1, const Normal &n2) {
	return fabsf(v1.x * n2.x + v1.y * n2.y + v1.z * n2.z);
}

inline float AbsDot(const Normal &n1, const Normal &n2) {
	return fabsf(n1.x * n2.x + n1.y * n2.y + n1.z * n2.z);
}

inline Normal Faceforword(const Normal &n, const Vector &v) {      //-----------将法线与所给向量转到同一半球-------
	return(Dot(n, v) < 0.f) ? -n : n;
}

inline Normal Faceforward(const Normal &n, const Normal &n2) {    //--------------法线转到另一法线的同半球--------
	return (Dot(n, n2) < 0.f) ? -n : n;
}



inline Vector Faceforward(const Vector &v, const Vector &v2) {      //----------向量转到另一向量的同半球--------
	return (Dot(v, v2) < 0.f) ? -v : v;
}



inline Vector Faceforward(const Vector &v, const Normal &n2) {      //-----------向量转到另一法线的同半球---------
	return (Dot(v, n2) < 0.f) ? -v : v;
}



inline const Point &BBox::operator[](int i) const {                 //---------------包围盒的下标------------------
	Assert(i == 0 || i == 1);
	return (&pMin)[i];
}



inline Point &BBox::operator[](int i) {
	Assert(i == 0 || i == 1);
	return (&pMin)[i];
}



