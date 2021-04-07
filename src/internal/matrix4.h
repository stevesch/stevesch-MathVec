#ifndef STEVESCH_MATHVEC_INTERNAL_MATRIX4_H_
#define STEVESCH_MATHVEC_INTERNAL_MATRIX4_H_
// Matrix4 library header
//
// Copyright © 2002, Stephen Schlueter, All Rights Reserved
//
// History:
//	Created:	4/30/2002, Stephen Schlueter
//	Modified:

#include "mathVec.h"
#include "stevesch-MathBase.h"
// #include "SError.h"
#include "vector3.h"
#include "vector4.h"

#define SMATRIX4INLINE SVECINLINE

namespace stevesch
{
  struct SMat4_t
  {
    stevesch::vector4 r[4];
  };

  // SALIGN_DECL(16)
  class matrix4
  {
  public:
    union
    {
      // matrix mxx - mapped from exx
      struct
      {
        float m00, m10, m20, m30;
        float m01, m11, m21, m31;
        float m02, m12, m22, m32;
        float m03, m13, m23, m33;
      };

      float xy[4][4];

      struct
      {
        stevesch::vector4 col[4];
      };
    };

  public:
    // overload new, delete, new[] and delete[] to provide aligned allocation
    // SOVERLOAD_NEW_ALIGNED(matrix4, 16)

    matrix4() {}
    matrix4(float fDiagonal3);                                                                                                   // uniform scale matrix
    matrix4(float x, float y, float z);                                                                                          // translation matrix
    matrix4(const stevesch::vector3 &vTranslation);                                                                              // translation matrix
    matrix4(const matrix4 &m);                                                                                                   // copy
    matrix4(const stevesch::vector4 &vx, const stevesch::vector4 &vy, const stevesch::vector4 &vz, const stevesch::vector4 &vw); // initialize by basis vector (column)

    matrix4(float _xx, float _xy, float _xz, float _xw,
            float _yx, float _yy, float _yz, float _yw,
            float _zx, float _zy, float _zz, float _zw,
            float _wx, float _wy, float _wz, float _ww);

    matrix4(const SMat4_t &m) { *(SMat4_t *)this = m; }
    const matrix4 &operator=(const SMat4_t &m)
    {
      *(SMat4_t *)this = m;
      return *this;
    }

    operator const SMat4_t &() const { return *reinterpret_cast<const SMat4_t *>(this); }
    operator SMat4_t &() { return *reinterpret_cast<SMat4_t *>(this); }

    operator const SMat4_t *() const { return reinterpret_cast<const SMat4_t *>(this); }
    operator SMat4_t *() { return reinterpret_cast<SMat4_t *>(this); }

    // >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    // PLATFORM-SPECIFIC IMPLEMENTATIONS ARE PROVIDED FOR THESE -->

    float det() const;    // compute determinant for entire 4x4 matrix
    float det3x3() const; // compute determinant for 3x3 part

    void normalizeCol3x3();                   // normalize columns of a 3x3 matrix
    void normalizeCol3x3(matrix4 &dst) const; // normalize columns of a 3x3 matrix

    static void transpose(matrix4 &dst, const matrix4 &src);
    static void transposeSub(matrix4 &dst, const matrix4 &src); // transpose 3x3 part (the rest is set to 4x4 identity)

    static void mul(matrix4 &dst, const matrix4 &m1, const matrix4 &m2);
    static void mulSubA(matrix4 &dst, const matrix4 &m1, const matrix4 &m2); // dst[R] = m1[R] * m2[R], dst[T] = m1[T] (R=col[0..2], T=col[3])
    static void mulSubB(matrix4 &dst, const matrix4 &m1, const matrix4 &m2); // dst[R] = m1[R] * m2[R], dst[T] = m2[T] (R=col[0..2], T=col[3])

    static void invert(matrix4 &dst, const matrix4 &src);    // inverts a 4x3 matrix, ignores 4th column (NOT VALID FOR 4x4)
    static void invert4x4(matrix4 &dst, const matrix4 &src); // full 4x4 matrix inverse

    // <-- PLATFORM-SPECIFIC IMPLEMENTATIONS ARE PROVIDED FOR THESE
    // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

    const matrix4 &diag3(float fDiag); // set diagonal 3x3 matrix	(last column and row = 0 0 0 1)
    const matrix4 &diag4(float fDiag); // set diagonal matrix		(e<nn> = fDiag)

    const matrix4 &copy(const matrix4 &m);
    const matrix4 &operator=(const matrix4 &m);
    const matrix4 &operator+=(const matrix4 &m);    // member-wise addition
    const matrix4 &operator-=(const matrix4 &m);    // member-wise subtraction
    const matrix4 &mulMembers4x4(const matrix4 &m); // member-wise multiplication
    const matrix4 &mulMembers4x4(float fScale);     // member-wise multiplication
    const matrix4 &mulMembers3x3(float fScale);     // member-wise multiplication of 3x3 part

    const matrix4 &identity();
    const matrix4 &zero();
    const matrix4 &diag3(const stevesch::vector3 &v); // create a diagonal matrix (0 matrix, m00=v.x, m11=v.y, m22v.z, m33=1.0)
    const matrix4 &diag4(const stevesch::vector4 &v); // create a diagonal matrix (0 matrix, m00=v.x, m11=v.y, m22v.z, m33=v.w)

    const matrix4 &forceSymmetric(); // m = 0.5*(m + m^T)

    // Tri-diagonalize a real symmetric 3x3 matrix (see "Householder's method")
    // *this(input) contains a real symmetric 3x3 matrix
    // *this(output) contains the orthogonal matrix (rotation) extracted from the symmetric matrix
    // vDiag3(output) contains the eigenvalues of the original matrix (scale components)
    bool eigen3(stevesch::vector3 &vDiag3);

    // generate a cross-product matrix V such that a*V = a x v for all row vectors a - TO-CHECK: a*V vs V*a, a x v vs v x a
    // TO-CHECK: crossOperator
    const matrix4 &crossOperator(const stevesch::vector3 &v);

    // generate a cross-product matrix within a large matrix
    // this function assumes the region of interest in the large matrix is already zeroed.
    // a 3x3 submatrix is generated such that a*V = a x v for all row vectors a
    // TO-CHECK: crossOperator
    static void crossOperator(stevesch::vector4 *pMatrix, const stevesch::vector3 &v, uint32_t nRowStride);

    // generate a cross-product matrix within a large matrix, reverse cross product if <bNegate> is true
    // this function assumes the region of interest in the large matrix is already zeroed.
    // a 3x3 submatrix is generated such that a*V = v x a for all row vectors a (or a x v if <bNegate> is false)
    static void crossOperator(stevesch::vector4 *pMatrix, const stevesch::vector3 &v, uint32_t nRowStride, bool bNegate);

    // TO-CHECK: use of []
    const stevesch::vector4 &operator[](int n) const; // access by column
    // TO-CHECK: use of []
    stevesch::vector4 &operator[](int n); // access by column

    stevesch::vector4 getRow(int n) const; // access by row (return by value only)

    const matrix4 &translate(const stevesch::vector4 &v);   // translate by v (col[3] += v)
    const matrix4 &translate(const stevesch::vector3 &v);   // translate by v (col[3].xyz += v)
    const matrix4 &translate(float dx, float dy, float dz); // translate by <dx, dy, dz> (col[3] += v)

    void setColumn(int n, const stevesch::vector4 &v); // place v in column n
    void getColumn(int n, stevesch::vector4 &v) const; // retrieve column n and place it in 'v'

    void getTranslation(stevesch::vector3 &vPos) const; // returns translation of matrix in vPos (tx, ty, tz)
    void getTranslation(stevesch::vector4 &vPos) const; // returns translation of matrix in vPos (tx, ty, tz, 1.0f)
    void setTranslation(const stevesch::vector3 &vPos); // sets x,y,z translation of matrix to vPos
    void setTranslation(float tx, float ty, float tz);  // sets x,y,z translation of matrix to tx,ty,tz

    const matrix4 &transpose();

    const matrix4 &operator*=(const matrix4 &m); // this = this * m (4x4)
    const matrix4 &mul(const matrix4 &r);        // this = this * r (4x4)
    const matrix4 &mulSub(const matrix4 &r);     // this = this * r (3x3 only)

    const matrix4 &lMul(const matrix4 &m);    // this = m * this (4x4)
    const matrix4 &lMulSub(const matrix4 &m); // this = m * this (3x3 only)

    // each of the following generate a matrix that performs rotation
    // about each respective axis.  NOTE-- the operation is independent
    // of handedness (the same matrix is used whether right or left handed
    // because swapping handedness also reverses rotation direction), but
    // the form assumes the use of row vectors, v = v0*A.  If column vectors
    // are used (e.g. v = A*v0), the transposes of the resulting matrices
    // should be used when multiplying.
    // TO-CHECK: positive rotation for X, Y and Z
    const matrix4 &XMatrix(float fRadians);
    const matrix4 &YMatrix(float fRadians);
    const matrix4 &ZMatrix(float fRadians);

    const matrix4 &fromEuler(const stevesch::vector3 &vEuler);

    const matrix4 &invert();    // inverts a 4x3 matrix, ignores 4th column (NOT VALID FOR 4x4)
    const matrix4 &invert4x4(); // full 4x4 matrix inverse

    const matrix4 &perspectiveLH(float fFOVhRadians, float fFOVvRadians, float fZNear, float fZFar);
    const matrix4 &perspectiveRH(float fFOVhRadians, float fFOVvRadians, float fZNear, float fZFar);

    static void mulMembers4x4(matrix4 &dst, const matrix4 &m1, const matrix4 &m2); // member-wise multiplication
    static void mulMembers4x4(matrix4 &dst, const matrix4 &m, float fScale);       // member-wise multiplication of entire matrix (4x4)
    static void mulMembers3x3(matrix4 &dst, const matrix4 &m, float fScale);       // member-wise multiplication of 3x3 part

    // generates a local-to-world transform that would orient an **object** to look at
    // the world coordinate 'rAt' from the world coordinate 'rEye'
    static void lookAtLHWorld(matrix4 &dst, const stevesch::vector4 &rEye, const stevesch::vector4 &rAt, const stevesch::vector4 &rUp);
    static void lookAtRHWorld(matrix4 &dst, const stevesch::vector4 &rEye, const stevesch::vector4 &rAt, const stevesch::vector4 &rUp);

    // generates a world-to-view transform that would orient the **camera** to look at
    // the world-coordinate 'rAt' from the world coordinate 'rEye'
    // (this is the inverse of lookAtLHWorld)
    static void lookAtLHView(matrix4 &dst, const stevesch::vector4 &rEye, const stevesch::vector4 &rAt, const stevesch::vector4 &rUp);
    static void lookAtRHView(matrix4 &dst, const stevesch::vector4 &rEye, const stevesch::vector4 &rAt, const stevesch::vector4 &rUp);

    // mimics of D3DX/XG functions:
    static void perspectiveFovLH(matrix4 &mtxProjDst, float fFOVVertical, float fAspectWoverH, float fZNear, float fZFar);
    static void perspectiveFovRH(matrix4 &mtxProjDst, float fFOVVertical, float fAspectWoverH, float fZNear, float fZFar);

    static void mulSub(matrix4 &dst, const matrix4 &m1, const matrix4 &m2) { mulSubA(dst, m1, m2); }

  public:
    static const matrix4 I; // identity matrix (4x4)

  } /* SALIGN(16) */;

} // namespace stevesch

#include "matrix4-inline.h"

#endif // STEVESCH_MATHVEC_INTERNAL_MATRIX4_H_
