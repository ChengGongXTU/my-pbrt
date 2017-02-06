#include<math.h>
#include<stdlib.h>
#include<stdio.h>
#include<string.h>

#include<string>
using std::string;
#include<vector>
using std::vector;

#include<algorithm>
using std::min;
using std::max;
using std::swap;
using std::sort;

//----------global macro for memory management--------------------------------
#define ALLOCA(TYPE, COUNT) (TYPE *)alloca((COUNT) * sizeof(TYPE))
#define M_PI 3.1415926

#define PBRT_VERSION "2.0.0"
//-------------------------------类和结构体的声明------------------------------------
class Vector;
class Point;
class Normal;
class Ray;
class RayDifferential;
class BBox;
class Transform;
struct Matrix4x4;

//---------------------Global Inline Functions-----------------------------------------

          //---------------------------------线性插值函数------------------------------
inline float Lerp(float t, float v1, float v2) {
	return (1.f - t)*v1 + t*v2;
}

