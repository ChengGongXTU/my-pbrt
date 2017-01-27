#include<math.h>
#include<stdlib.h>
#include<stdio.h>
#include<string.h>

#include<string>
using std::string;
#include<vector>
using std::vector;

#define PBRT_VERSION "2.0.0"
//-------------------------------��ͽṹ�������------------------------------------
class Vector;
class Point;
class Normal;
class Ray;
class RayDifferential;
class BBox;
class Transform;
struct Matrix4x4;



//---------------------Global Inline Functions-----------------------------------------

          //---------------------------------���Բ�ֵ����------------------------------
inline float Lerp(float t, float v1, float v2) {
	return (1.f - t)*v1 + t*v2;
}

