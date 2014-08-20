#pragma once
#include "nvVector.h"
using namespace nv;

#define PI			3.1415926535897932384626433832795
#define INV_PI		0.31830988618379067153776752674503
#define EPSILON		(1e-5)
#define BUFFERSIZE  1024

// Macros
#define SQR(x)      ((x)*(x))                        // x^2 
#define SQR_ABS(x)  (SQR(creal(x)) + SQR(cimag(x)))  // |x|^2
#define MAX(a, b) a > b ? a : b
#define MIN(a, b) a < b ? a : b
#define CLAMP(v, min_v, max_v) v < min_v ? min_v : v > max_v ? max_v : v 
#define MAX_VEC_COMP(v)  MAX(MAX(v.x, v.y), v.z)
#define MIN_VEC_COMP(v)  MIN(MIN(v.x, v.y), v.z)

//#define MERGE_GLOSSY
typedef unsigned uint;
