#ifndef STEVESCH_MATHVEC_INTERNAL_QUAT_H_
#define STEVESCH_MATHVEC_INTERNAL_QUAT_H_

// Quaternion library header
//
// Copyright © 2002, PulseCode Interactive LLC, All Rights Reserved
//
// History:
//	Created:	4/30/2002, Stephen Schlueter
//	Modified:

#include "stevesch-MathBase.h"
#include "vector4.h"

#define SQUATINLINE SVECINLINE

namespace stevesch
{
  const float cfQuatSlerpLinearEpsilon = 0.00015f; // 1 - cos(angle) of max. angle where direct linear interpolation
                                                   // is used in place of spherical linear interpolation
                                                   // (1 - cos(1 degree) ~= 0.00015)

  class stevesch::quat
  {
  protected:
    stevesch::vector4 m_vQuat;

  public:
    // overload new, delete, new[] and delete[] to provide aligned allocation
    // SOVERLOAD_NEW_ALIGNED(stevesch::quat, 16)

    SQUATINLINE stevesch::quat() {}
    SQUATINLINE stevesch::quat(const stevesch::quat &q);

    explicit SQUATINLINE stevesch::quat(const stevesch::vector4 &vAxis, float fAngle); // axis-angle initialization
    explicit SQUATINLINE stevesch::quat(const stevesch::matrix4 &m);                   // matrix-to-quaternion initialization
    explicit SQUATINLINE stevesch::quat(const stevesch::vector3 &vEuler);              // euler-to-quaternion initialization

    explicit SQUATINLINE stevesch::quat(float x, float y, float z, float a); // explicit member initialization
    explicit SQUATINLINE stevesch::quat(float x, float y, float z);          // euler-to-quaternion initialization
    explicit SQUATINLINE stevesch::quat(bool bDummy);                        // initialize as an identity quaternion

    SQUATINLINE bool operator==(const stevesch::quat &crOther) const;
    SQUATINLINE bool operator!=(const stevesch::quat &crOther) const;

    SQUATINLINE void identity(); // initialize as an identity quaternion

    SQUATINLINE bool isIdentity() const; // returns 'true' if quaternion represents zero rotation

    SQUATINLINE const stevesch::vector4 &V() const { return m_vQuat; }
    SQUATINLINE stevesch::vector4 &V() { return m_vQuat; }

    SQUATINLINE float &X() { return m_vQuat.x; } // v.x
    SQUATINLINE float &Y() { return m_vQuat.y; } // v.y
    SQUATINLINE float &Z() { return m_vQuat.z; } // v.z
    SQUATINLINE float &A() { return m_vQuat.w; } // "t"

    SQUATINLINE float X() const { return m_vQuat.x; }
    SQUATINLINE float Y() const { return m_vQuat.y; }
    SQUATINLINE float Z() const { return m_vQuat.z; }
    SQUATINLINE float A() const { return m_vQuat.w; }

    SQUATINLINE stevesch::quat &operator+=(const stevesch::quat &r);
    SQUATINLINE stevesch::quat &operator-=(const stevesch::quat &r);
    SQUATINLINE stevesch::quat &operator*=(const stevesch::quat &r);
    SQUATINLINE stevesch::quat &operator*=(float fScale);
    SQUATINLINE stevesch::quat &operator=(const stevesch::quat &q);

    SQUATINLINE stevesch::quat &postMul_Norm(const stevesch::quat &qRight); // (*this) = (*this) * qRight
    SQUATINLINE stevesch::quat &preMul_Norm(const stevesch::quat &qLeft);   // (*this) = qLeft * (*this)
    SQUATINLINE stevesch::quat &postMul_Conj(const stevesch::quat &qRight); // (*this) = (*this) * qRight~
    SQUATINLINE stevesch::quat &preMul_Conj(const stevesch::quat &qLeft);   // (*this) = qLeft~ * (*this)

    SQUATINLINE static void mul(stevesch::quat &dst, const stevesch::quat &q1, const stevesch::quat &q2);

    SQUATINLINE static void mul_Norm_Norm(stevesch::quat &dst, const stevesch::quat &q1, const stevesch::quat &q2); // dst = q1 * q2 (standard product)
    SQUATINLINE static void mul_Conj_Norm(stevesch::quat &dst, const stevesch::quat &q1, const stevesch::quat &q2); // dst = q1~ * q2 (euclidean product)
    SQUATINLINE static void mul_Norm_Conj(stevesch::quat &dst, const stevesch::quat &q1, const stevesch::quat &q2); // dst = q1 * q2~
    SQUATINLINE static void mul_Conj_Conj(stevesch::quat &dst, const stevesch::quat &q1, const stevesch::quat &q2); // dst = q1~ * q2~

    SQUATINLINE stevesch::quat &scale(float fScale); // same as *=(scalar)

    SQUATINLINE void conj();                          // Conjugate (self): q* = (t, -v)
    SQUATINLINE void conj(stevesch::quat &dst) const; // Conjugate

    SQUATINLINE float norm() const;    // (q)(q*) == t*t + v.v
    SQUATINLINE float invNorm() const; // 1.0 / (q)(q*) == 1.0 / (t*t + v.v)

    SQUATINLINE float abs() const;    // sqrt(t*t + v.v)
    SQUATINLINE float invAbs() const; // 1.0 / sqrt(t*t + v.v)

    SQUATINLINE float det() const;    // (t*t + v.v)^2
    SQUATINLINE float invDet() const; // 1.0 / [(t*t + v.v)^2]

    SQUATINLINE void adj();                          // Adjunct (self)-- (q*)[(q)(q*)]
    SQUATINLINE void adj(stevesch::quat &dst) const; // Adjunct

    SQUATINLINE void invert();
    SQUATINLINE void invert(stevesch::quat &dst) const;

    void ln();                          // natural log (self)
    void ln(stevesch::quat &dst) const; // natural log

    void exp();                          // exponential (self)-- e^(this)
    void exp(stevesch::quat &dst) const; // exponential-- dst = e^(this)

    SQUATINLINE void pow(const stevesch::quat &q);                                                        // power (self)-- this = this^q
    static SQUATINLINE void pow(stevesch::quat &dst, const stevesch::quat &q1, const stevesch::quat &q2); // power-- dst = q1^q2

    SQUATINLINE void pow(float fPower);              // power (self)-- this = this^fPower
    SQUATINLINE void normalize();                    // normalize (self)-- this = this/|this|
    SQUATINLINE void normalize(stevesch::quat &dst); // normalize-- dst = this/|this|

    bool safeNormalize();                          // normalize (self), setting value to identity if near 0.  returns 'true' if quaternion needed normalization
    bool safeNormalize(stevesch::quat &dst) const; // normalize, setting value to identity if near 0.  returns 'true' if quaternion needed normalization

    SQUATINLINE void rotate(stevesch::vector4 &dst, const stevesch::vector4 &src) const; // dst = q * src * ~q

    stevesch::matrix4 &toMatrix(stevesch::matrix4 &m) const;                                        // Convert from quaternion to matrix
    stevesch::matrix4 &toMatrix(stevesch::matrix4 &m, const stevesch::vector3 &vTranslation) const; // Convert from quaternion and translation to matrix

    void fromMatrix(const stevesch::matrix4 &m);
    void fromOrthonormalMatrix(const stevesch::matrix4 &m); // matrix must not contain scale (rotation only-- translation is ignored)

    void toEuler(stevesch::vector4 &vEuler) const;
    void fromEuler(const stevesch::vector3 &vEuler);
    void fromEuler(float fRadiansX, float fRadiansY, float fRadiansZ);

    float getAngle() const; // get rotation amount represented by quaternion (as in 'toAxisAngle')
    void toAxisAngle(stevesch::vector4 &vAxis, float &angle) const;
    void fromAxisAngle(const stevesch::vector4 &vAxis, float angle);

    SQUATINLINE void cross(const stevesch::quat &q);   // cross product (self)-- this = this x q (Grassman outer product)
    SQUATINLINE void dotQuat(const stevesch::quat &q); // quaternion-result dot product (self)-- this = this . q (Euclidean inner product)
                                                       // (results in <0 0 0 scalar-dot>

    SQUATINLINE float dot(const stevesch::quat &q) const; // standard euclidean inner (even) product
                                                          // [(q1*)(q2) + (q2)(q1*)] / 2
                                                          // == ( t*t' + v.v', 0 )
                                                          // *** Common dot product ***

    //		void ToBasisMatrix(stevesch::matrix4& dst) const;

  public:
    // public static functions
    static void grassmanProduct(stevesch::quat &dst, const stevesch::quat &q1, const stevesch::quat &q2); // Common multiplication
    static void grassmanEven(stevesch::quat &dst, const stevesch::quat &q1, const stevesch::quat &q2);
    static void grassmanOdd(stevesch::quat &dst, const stevesch::quat &q1, const stevesch::quat &q2); // Common outer or "cross" product

    static void euclideanProduct(stevesch::quat &dst, const stevesch::quat &q1, const stevesch::quat &q2);
    static void euclideanEven(stevesch::quat &dst, const stevesch::quat &q1, const stevesch::quat &q2); // Common inner or "dot" product
    static void euclideanOdd(stevesch::quat &dst, const stevesch::quat &q1, const stevesch::quat &q2);

    static void slerp(stevesch::quat &dst, const stevesch::quat &q1, const stevesch::quat &q2, float t); // spherical linear interpolation t=[0, 1] -> dst=[q1, q2]

    // compute angular velocity given angular positions and a time step
    // fDeltaSeconds must be non-zero
    static void instantaneousVelocity(stevesch::vector4 &vVAngular, const stevesch::quat &q0, const stevesch::quat &q1, float fDeltaSeconds);

    // compute linear and angular velocities given linear and angular positions and a time step
    // fDeltaSeconds must be non-zero
    static void instantaneousVelocities(stevesch::vector4 &vVLinear, stevesch::vector4 &vVAngular,
                                        const stevesch::vector4 &v0, const stevesch::quat &q0,
                                        const stevesch::vector4 &v1, const stevesch::quat &q1,
                                        float fDeltaSeconds);

    // Compute derivative of angular velocity in quaternion form and apply a time step
    // (used by quaternion integration steps, below)
    // *this: in/out, quaternion position
    // w: in, angular velocity
    // dt: time step
    void integrate(const stevesch::vector3 &w, float dt);

    // generates a local-to-world transform that would rotate an **object**
    // at world coordinate 'rAt' to look at world coordinate 'rEye'
    // (only the rotation can be represented by the quaternion)
    static void lookAtLHWorld(stevesch::quat &dst, const stevesch::vector4 &rEye, const stevesch::vector4 &rAt, const stevesch::vector4 &rUp);
    static void lookAtRHWorld(stevesch::quat &dst, const stevesch::vector4 &rEye, const stevesch::vector4 &rAt, const stevesch::vector4 &rUp);

    // generates a local-to-world transform that would rotate the **camera**
    // at world coordinate 'rAt' to look at world coordinate 'rEye'
    // (only the rotation can be represented by the quaternion)
    static void lookAtLHView(stevesch::quat &dst, const stevesch::vector4 &rEye, const stevesch::vector4 &rAt, const stevesch::vector4 &rUp);
    static void lookAtRHView(stevesch::quat &dst, const stevesch::vector4 &rEye, const stevesch::vector4 &rAt, const stevesch::vector4 &rUp);

    // construct q such that v0 transformed by q equals v1
    // returns angle between v0 and v1.
    // if v0 is "close enough" (within fTolerance) to v1, angle will be 0.0f
    // (or c_fpi if vectors face opposite directions)
    // v0 and v1 need not be normalized (but should be non-zero)
    static float fromTwoVectors(stevesch::quat &q, const stevesch::vector4 &v0, const stevesch::vector4 &v1, float fTolerance = 0.000001f);

    // same as fromTwoVectors, but v0 and v1 must be normalized
    static float fromTwoNormVectors(stevesch::quat &q, const stevesch::vector4 &v0, const stevesch::vector4 &v1, float fTolerance = 0.000001f);

    // construct q such that v0 transformed by q equals v1 or -v1
    // returns '0.0f' if v0 is "close enough", otherwise returns angle rotated by resulting q.
    // (aligns v0 to be colinear with v1, but possibly the opposite direction)
    static float alignTwoVectors(stevesch::quat &q, const stevesch::vector4 &v0, const stevesch::vector4 &v1, float fTolerance = 0.000001f);

  public:
    // public static data members
    /*		
		// quaternion R4 basis matrices: q = a*R4A + x*R4I + y*R4J + z*R4K, real scalars x, y, z, a
		static const stevesch::matrix4&	R4A;	// == identity matrix
		static const stevesch::matrix4	R4I;
		static const stevesch::matrix4	R4J;
		static const stevesch::matrix4	R4K;
*/
  };

  extern const stevesch::quat c_qIDENTITY; // <0 0 0 1>

} // namespace stevesch

#include "quat-inline.h"

#endif // STEVESCH_MATHVEC_INTERNAL_QUAT_H_