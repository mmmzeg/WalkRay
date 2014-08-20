#pragma once
#include "macros.h"
#include "frame.h"
#include "nvMatrix.h"
#include <float.h>

// check float overflow 
static bool isLegalColor(const vec3f &color){
	for(int i = 0; i < 3; i++){
		if(_isnan(color[i]))
			return false;
		if(!_finite(color[i]))
			return false;
		if(color[i] < 0)
			return false;
	}
	return true;
}

inline float Lerp(float t, float v1, float v2){
	return (1.f - t) * v1 + t * v2;
}

inline vec3f Lerp(float t, vec3f v1, vec3f v2){
	vec3f lerpResult;
	for(int i = 0; i < 3; i++)
		lerpResult[i] = Lerp(t, v1[i], v2[i]);
	return lerpResult;
}

// sRGB luminance
static float Luminance(const vec3f &aRGB)
{
    return 0.212671f * aRGB.x +
        0.715160f * aRGB.y +
        0.072169f * aRGB.z;
}

// Matrix rotation
static mat4f RotateMatrix(const vec3f &axis, const float angle){
	float c = cos(angle);
	float s = sin(angle);
	float _c = 1 - c;
	float _s = 1 - s;
	float x = axis.x;
	float y = axis.y;
	float z = axis.z;
	return transpose(
		mat4f(c+_c*x*x, _c*x*y-s*z, _c*x*z+s*y, 0.f,
			  _c*x*y+s*z, c+_c*y*y, _c*y*z-s*x, 0.f,
		      _c*x*z-s*y, _c*y*z+s*x, c+_c*z*z, 0.f,
		      0.f, 0.f, 0.f, 1.f)
	);
}

// compute the eigenvalue decomposition of a symmetric 2x2
// matrix in the form A=[a b;b c], so that
//    A  *  v1 =   v1  *  lambda1
//    A  *  v2 =   v2  *  lambda2
static bool evdecomposesymm(
	double a, double b,double c,
	double & lambda1, double & lambda2,
	double & v1x, double & v1y, 
	double & v2x, double & v2y, bool printer = false)
{
	double disc = sqrt((a-c)*(a-c)+4*b*b)/2;

	lambda1 = (a+c)/2 + disc;
	lambda2 = (a+c)/2 - disc;
	if(printer)
	///printf("\t: lambda1 = %f, lambda2 = %f\n",lambda1,lambda2);
	printf("\tMatrix: a = %f, b = %f, c = %f\n",a,b,c);

	if (fabs(lambda1) < FLT_EPSILON || fabs(lambda2) < FLT_EPSILON)
	{
		printf("\twarning: lambda1 = %f, lambda2 = %f\n",lambda1,lambda2);
		printf("\twarning Matrix: a = %f, b = %f, c = %f\n",a,b,c);



		return false;
	}

	v1x = -b;  v1y = a-lambda1;
	v2x = -b;  v2y = a-lambda2;
	double v1mag = sqrt(v1x*v1x + v1y*v1y);  
	double v2mag = sqrt(v2x*v2x + v2y*v2y);
	v1x/= v1mag;  v1y/=v1mag;
	v2x/= v2mag;  v2y/=v2mag;

	return true;
}


//////////////////////////////////////////////////////////////////////////
// Utilities for converting PDF between Area (A) and Solid angle (W)
// WtoA = PdfW * cosine / distance_squared
// AtoW = PdfA * distance_squared / cosine

static float PdfWtoA(
    const float aPdfW,
    const float aDist,
    const float aCosThere)
{
    return aPdfW * std::abs(aCosThere) / SQR(aDist);
}

static float PdfAtoW(
    const float aPdfA,
    const float aDist,
    const float aCosThere)
{
    return aPdfA * SQR(aDist) / std::abs(aCosThere);
}



// ----------------------------------------------------------------------------
inline static void dsyev2(double A, double B, double C, double *rt1, double *rt2,
                   double *cs, double *sn)
// ----------------------------------------------------------------------------
// Calculates the eigensystem of a real symmetric 2x2 matrix
//    [ A  B ]
//    [ B  C ]
// in the form
//    [ A  B ]  =  [ cs  -sn ] [ rt1   0  ] [  cs  sn ]
//    [ B  C ]     [ sn   cs ] [  0   rt2 ] [ -sn  cs ]
// where rt1 >= rt2. Note that this convention is different from the one used
// in the LAPACK routine DLAEV2, where |rt1| >= |rt2|.
// ----------------------------------------------------------------------------
{
  double sm = A + C;
  double df = A - C;
  double rt = sqrt(SQR(df) + 4.0*B*B);
  double t;

  if (sm > 0.0)
  {
    *rt1 = 0.5 * (sm + rt);
    t = 1.0/(*rt1);
    *rt2 = (A*t)*C - (B*t)*B;
  }
  else if (sm < 0.0)
  {
    *rt2 = 0.5 * (sm - rt);
    t = 1.0/(*rt2);
    *rt1 = (A*t)*C - (B*t)*B;
  }
  else       // This case needs to be treated separately to avoid div by 0
  {
    *rt1 = 0.5 * rt;
    *rt2 = -0.5 * rt;
  }

  // Calculate eigenvectors
  if (df > 0.0)
    *cs = df + rt;
  else
    *cs = df - rt;

  if (fabs(*cs) > 2.0*fabs(B))
  {
    t   = -2.0 * B / *cs;
    *sn = 1.0 / sqrt(1.0 + SQR(t));
    *cs = t * (*sn);
  }
  else if (fabs(B) == 0.0)
  {
    *cs = 1.0;
    *sn = 0.0;
  }
  else
  {
    t   = -0.5 * (*cs) / B;
    *cs = 1.0 / sqrt(1.0 + SQR(t));
    *sn = t * (*cs);
  }

  if (df > 0.0)
  {
    t   = *cs;
    *cs = -(*sn);
    *sn = t;
  }
}

inline static bool inverseMat2X2(double mat[][2], double inverseMat[][2]){
	double a = mat[0][0];
	double b = mat[0][1];
	double c = mat[1][0];
	double d = mat[1][1];
	double det = a*d - b*c;
	if(fabs(det) < EPSILON){
		return false;
	}
	inverseMat[0][0] = d / det;
	inverseMat[0][1] = -b / det;
	inverseMat[1][0] = -c / det;
	inverseMat[1][1] = a / det;
	return true;
}

inline static int dlaev2_(double *a, double *b, double *c__, 
	double *rt1, double *rt2, double *cs1, double *sn1)
{
/*  -- LAPACK auxiliary routine (version 3.1) --   
       Univ. of Tennessee, Univ. of California Berkeley and NAG Ltd..   
       November 2006   


    Purpose   
    =======   

    DLAEV2 computes the eigendecomposition of a 2-by-2 symmetric matrix   
       [  A   B  ]   
       [  B   C  ].   
    On return, RT1 is the eigenvalue of larger absolute value, RT2 is the   
    eigenvalue of smaller absolute value, and (CS1,SN1) is the unit right   
    eigenvector for RT1, giving the decomposition   

       [ CS1  SN1 ] [  A   B  ] [ CS1 -SN1 ]  =  [ RT1  0  ]   
       [-SN1  CS1 ] [  B   C  ] [ SN1  CS1 ]     [  0  RT2 ].   

    Arguments   
    =========   

    A       (input) DOUBLE PRECISION   
            The (1,1) element of the 2-by-2 matrix.   

    B       (input) DOUBLE PRECISION   
            The (1,2) element and the conjugate of the (2,1) element of   
            the 2-by-2 matrix.   

    C       (input) DOUBLE PRECISION   
            The (2,2) element of the 2-by-2 matrix.   

    RT1     (output) DOUBLE PRECISION   
            The eigenvalue of larger absolute value.   

    RT2     (output) DOUBLE PRECISION   
            The eigenvalue of smaller absolute value.   

    CS1     (output) DOUBLE PRECISION   
    SN1     (output) DOUBLE PRECISION   
            The vector (CS1, SN1) is a unit right eigenvector for RT1.   

    Further Details   
    ===============   

    RT1 is accurate to a few ulps barring over/underflow.   

    RT2 may be inaccurate if there is massive cancellation in the   
    determinant A*C-B*B; higher precision or correctly rounded or   
    correctly truncated arithmetic would be needed to compute RT2   
    accurately in all cases.   

    CS1 and SN1 are accurate to a few ulps barring over/underflow.   

    Overflow is possible only if RT1 is within a factor of 5 of overflow.   
    Underflow is harmless if the input data is 0 or exceeds   
       underflow_threshold / macheps.   

   =====================================================================   


       Compute the eigenvalues */
    /* System generated locals */
    double d__1;
    /* Builtin functions */
    /* Local variables */
    static double ab, df, cs, ct, tb, sm, tn, rt, adf, acs;
    static int sgn1, sgn2;
    static double acmn, acmx;


    sm = *a + *c__;
    df = *a - *c__;
    adf = abs(df);
    tb = *b + *b;
    ab = abs(tb);
    if (abs(*a) > abs(*c__)) {
	acmx = *a;
	acmn = *c__;
    } else {
	acmx = *c__;
	acmn = *a;
    }
    if (adf > ab) {
/* Computing 2nd power */
	d__1 = ab / adf;
	rt = adf * sqrt(d__1 * d__1 + 1.);
    } else if (adf < ab) {
/* Computing 2nd power */
	d__1 = adf / ab;
	rt = ab * sqrt(d__1 * d__1 + 1.);
    } else {

/*        Includes case AB=ADF=0 */

	rt = ab * sqrt(2.);
    }
    if (sm < 0.) {
	*rt1 = (sm - rt) * .5;
	sgn1 = -1;

/*        Order of execution important.   
          To get fully accurate smaller eigenvalue,   
          next line needs to be executed in higher precision. */

	*rt2 = acmx / *rt1 * acmn - *b / *rt1 * *b;
    } else if (sm > 0.) {
	*rt1 = (sm + rt) * .5;
	sgn1 = 1;

/*        Order of execution important.   
          To get fully accurate smaller eigenvalue,   
          next line needs to be executed in higher precision. */

	*rt2 = acmx / *rt1 * acmn - *b / *rt1 * *b;
    } else {

/*        Includes case RT1 = RT2 = 0 */

	*rt1 = rt * .5;
	*rt2 = rt * -.5;
	sgn1 = 1;
    }

/*     Compute the eigenvector */

    if (df >= 0.) {
	cs = df + rt;
	sgn2 = 1;
    } else {
	cs = df - rt;
	sgn2 = -1;
    }
    acs = abs(cs);
    if (acs > ab) {
	ct = -tb / cs;
	*sn1 = 1. / sqrt(ct * ct + 1.);
	*cs1 = ct * *sn1;
    } else {
	if (ab == 0.) {
	    *cs1 = 1.;
	    *sn1 = 0.;
	} else {
	    tn = -cs / tb;
	    *cs1 = 1. / sqrt(tn * tn + 1.);
	    *sn1 = tn * *cs1;
	}
    }
    if (sgn1 == sgn2) {
	tn = *cs1;
	*cs1 = -(*sn1);
	*sn1 = tn;
    }
    return 0;

/*     End of DLAEV2 */

} /* dlaev2_ */

