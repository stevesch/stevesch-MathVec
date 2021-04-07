#include "matrix4.h" // precompiled header for SMath project
//
// Copyright © 2002, Stephen Schlueter, All Rights Reserved
//

#define SMATRIX4_USE_D3DX 0

namespace stevesch
{
  // normalize columns of a 3x3 matrix
  void matrix4::normalizeCol3x3()
  {
    vector4 vSum;
    vector4::mul(vSum, col[0], col[0]);
    vector4::mad(vSum, col[1], col[1], vSum);
    vector4::mad(vSum, col[2], col[2], vSum);
    vector4::recipSqrt(vSum, vSum);
    col[0].mul(vSum);
    col[1].mul(vSum);
    col[2].mul(vSum);
  }

  // normalize columns of a 3x3 matrix
  void matrix4::normalizeCol3x3(matrix4 &dst) const
  {
    vector4 vSum;
    vector4::mul(vSum, col[0], col[0]);
    vector4::mad(vSum, col[1], col[1], vSum);
    vector4::mad(vSum, col[2], col[2], vSum);
    vector4::recipSqrt(vSum, vSum);
    vector4::mul(dst.col[0], col[0], vSum);
    vector4::mul(dst.col[1], col[1], vSum);
    vector4::mul(dst.col[2], col[2], vSum);
    dst.col[3] = col[3];
  }

  void matrix4::transpose(matrix4 &dst, const matrix4 &src)
  {
#if SMATRIX4_USE_D3DX
    D3DXMatrixTranspose(dst, src);
#else
    matrix4 m;
    src.getColumn(0, m.col[0]);
    src.getColumn(1, m.col[1]);
    src.getColumn(2, m.col[2]);
    src.getColumn(3, m.col[3]);

    dst.copy(m);
#endif
  }

  // transpose 3x3 part
  void matrix4::transposeSub(matrix4 &dst, const matrix4 &src)
  {
    float _m00 = src.m00;
    float _m10 = src.m10;
    float _m20 = src.m20;
    float _m30 = src.m30;
    float _m01 = src.m01;
    float _m11 = src.m11;
    float _m21 = src.m21;
    float _m31 = src.m31;
    float _m02 = src.m02;
    float _m12 = src.m12;
    float _m22 = src.m22;
    float _m32 = src.m32;
    float _m03 = src.m03;
    float _m13 = src.m13;
    float _m23 = src.m23;
    float _m33 = src.m33;

    dst.col[0].set(_m00, _m01, _m02, (_m30)); // note last element not transposed
    dst.col[1].set(_m10, _m11, _m12, (_m31)); // note last element not transposed
    dst.col[2].set(_m20, _m21, _m22, (_m32)); // note last element not transposed

    dst.col[3].set(_m03, _m13, _m23, _m33); // copy last column
  }

  void matrix4::mul(matrix4 &dst, const matrix4 &m1, const matrix4 &m2)
  {
    matrix4 result;
    matrix4 a;
    transpose(a, m1);
    const vector4 &a0 = a.col[0];
    const vector4 &a1 = a.col[1];
    const vector4 &a2 = a.col[2];
    const vector4 &a3 = a.col[3];
    const vector4 &b0 = m2.col[0];
    const vector4 &b1 = m2.col[1];
    const vector4 &b2 = m2.col[2];
    const vector4 &b3 = m2.col[3];
    result.col[0].set(
        b0.dot4(a0),
        b0.dot4(a1),
        b0.dot4(a2),
        b0.dot4(a3));
    result.col[1].set(
        b1.dot4(a0),
        b1.dot4(a1),
        b1.dot4(a2),
        b1.dot4(a3));
    result.col[2].set(
        b2.dot4(a0),
        b2.dot4(a1),
        b2.dot4(a2),
        b2.dot4(a3));
    result.col[3].set(
        b3.dot4(a0),
        b3.dot4(a1),
        b3.dot4(a2),
        b3.dot4(a3));

    dst.copy(result);
  }

  void matrix4::mulSubA(matrix4 &dst, const matrix4 &m1, const matrix4 &m2)
  {
    matrix4 b;
    vector4 r0, r1, r2;

    transpose(b, m2);

    r0.set(
        m1.col[0].dot(b.col[0]),
        m1.col[0].dot(b.col[1]),
        m1.col[0].dot(b.col[2]),
        m1.col[0].w);
    r1.set(
        m1.col[1].dot(b.col[0]),
        m1.col[1].dot(b.col[1]),
        m1.col[1].dot(b.col[2]),
        m1.col[1].w);
    r2.set(
        m1.col[2].dot(b.col[0]),
        m1.col[2].dot(b.col[1]),
        m1.col[2].dot(b.col[2]),
        m1.col[2].w);

    dst.col[0] = r0;
    dst.col[1] = r1;
    dst.col[2] = r2;
    dst.col[3] = m1.col[3];
  }

  void matrix4::mulSubB(matrix4 &dst, const matrix4 &m1, const matrix4 &m2)
  {
    matrix4 b;
    vector4 r0, r1, r2;

    transpose(b, m2);

    r0.set(
        m1.col[0].dot(b.col[0]),
        m1.col[0].dot(b.col[1]),
        m1.col[0].dot(b.col[2]),
        m1.col[0].w);
    r1.set(
        m1.col[1].dot(b.col[0]),
        m1.col[1].dot(b.col[1]),
        m1.col[1].dot(b.col[2]),
        m1.col[1].w);
    r2.set(
        m1.col[2].dot(b.col[0]),
        m1.col[2].dot(b.col[1]),
        m1.col[2].dot(b.col[2]),
        m1.col[2].w);

    dst.col[0] = r0;
    dst.col[1] = r1;
    dst.col[2] = r2;
    dst.col[3] = m2.col[3];
  }

  // compute determinant
  float matrix4::det() const
  {
#if SMATRIX4_USE_D3DX
    //		return D3DXMatrixfDeterminant( *this );
    return D3DXMatrixDeterminant(*this);
#else
    // m00 m10 m20 m30
    // m01 m11 m21 m31
    // m02 m12 m22 m32
    // m03 m13 m23 m33

    float fDeterminant =
        m00 *
            (m11 * (m22 * m33 - m23 * m32) -
             m21 * (m12 * m33 - m13 * m32) +
             m31 * (m12 * m23 - m13 * m22)) -
        m10 *
            (m01 * (m22 * m33 - m23 * m32) -
             m21 * (m02 * m33 - m03 * m32) +
             m31 * (m02 * m23 - m03 * m22)) +
        m20 *
            (m01 * (m12 * m33 - m13 * m32) -
             m11 * (m02 * m33 - m03 * m32) +
             m31 * (m02 * m13 - m03 * m12)) -
        m30 *
            (m01 * (m12 * m23 - m13 * m22) -
             m11 * (m02 * m23 - m03 * m22) +
             m21 * (m02 * m13 - m03 * m12));

    return fDeterminant;
#endif
  }

  // inverts a 4x3 matrix (NOT VALID FOR 4x4)
  void matrix4::invert(matrix4 &dst, const matrix4 &src)
  {
#if SMATRIX4_USE_D3DX
    D3DXMatrixInverse(dst, NULL, src);
#else
    matrix4 c; // cofactor matrix;
    vector4 v; // temporary vector
    float det; // determinant

    vector4::cross(c.col[0], src.col[1], src.col[2]);
    c.col[0].w = 0.0f;
    vector4::cross(c.col[1], src.col[2], src.col[0]);
    c.col[0].w = 0.0f;
    vector4::cross(c.col[2], src.col[0], src.col[1]);
    c.col[0].w = 0.0f;
    c.col[3].set(0.0f, 0.0f, 0.0f, 1.0f);
    // note uninitialized w of cofactor columns

    // determinant of src is m00 of (cT * src)
    c.transpose();

    src.getColumn(0, v);
    det = c.col[0].dot(v);

    if (det == 0.0f)
    {
      // singular matrix can't be inverted
      dst.identity();
    }
    else
    {
      // compute the inverse of the matrix
      float invdet = 1.0f / det;

      src.col[3].negate(v); // v = -translation of src

      vector4::scale(dst.col[0], c.col[0], invdet);
      vector4::scale(dst.col[1], c.col[1], invdet);
      vector4::scale(dst.col[2], c.col[2], invdet);

      v.transformSub(dst);
      v.w = 1.0f;
      dst.col[3] = v;
    }
#endif
  }

  // lifted from MESA implementation of the GLU library.
  bool gluInvertMatrix(const float *m, float *invOut)
  {
    float inv[16], det;
    int i;

    inv[0] = m[5] * m[10] * m[15] -
             m[5] * m[11] * m[14] -
             m[9] * m[6] * m[15] +
             m[9] * m[7] * m[14] +
             m[13] * m[6] * m[11] -
             m[13] * m[7] * m[10];

    inv[4] = -m[4] * m[10] * m[15] +
             m[4] * m[11] * m[14] +
             m[8] * m[6] * m[15] -
             m[8] * m[7] * m[14] -
             m[12] * m[6] * m[11] +
             m[12] * m[7] * m[10];

    inv[8] = m[4] * m[9] * m[15] -
             m[4] * m[11] * m[13] -
             m[8] * m[5] * m[15] +
             m[8] * m[7] * m[13] +
             m[12] * m[5] * m[11] -
             m[12] * m[7] * m[9];

    inv[12] = -m[4] * m[9] * m[14] +
              m[4] * m[10] * m[13] +
              m[8] * m[5] * m[14] -
              m[8] * m[6] * m[13] -
              m[12] * m[5] * m[10] +
              m[12] * m[6] * m[9];

    inv[1] = -m[1] * m[10] * m[15] +
             m[1] * m[11] * m[14] +
             m[9] * m[2] * m[15] -
             m[9] * m[3] * m[14] -
             m[13] * m[2] * m[11] +
             m[13] * m[3] * m[10];

    inv[5] = m[0] * m[10] * m[15] -
             m[0] * m[11] * m[14] -
             m[8] * m[2] * m[15] +
             m[8] * m[3] * m[14] +
             m[12] * m[2] * m[11] -
             m[12] * m[3] * m[10];

    inv[9] = -m[0] * m[9] * m[15] +
             m[0] * m[11] * m[13] +
             m[8] * m[1] * m[15] -
             m[8] * m[3] * m[13] -
             m[12] * m[1] * m[11] +
             m[12] * m[3] * m[9];

    inv[13] = m[0] * m[9] * m[14] -
              m[0] * m[10] * m[13] -
              m[8] * m[1] * m[14] +
              m[8] * m[2] * m[13] +
              m[12] * m[1] * m[10] -
              m[12] * m[2] * m[9];

    inv[2] = m[1] * m[6] * m[15] -
             m[1] * m[7] * m[14] -
             m[5] * m[2] * m[15] +
             m[5] * m[3] * m[14] +
             m[13] * m[2] * m[7] -
             m[13] * m[3] * m[6];

    inv[6] = -m[0] * m[6] * m[15] +
             m[0] * m[7] * m[14] +
             m[4] * m[2] * m[15] -
             m[4] * m[3] * m[14] -
             m[12] * m[2] * m[7] +
             m[12] * m[3] * m[6];

    inv[10] = m[0] * m[5] * m[15] -
              m[0] * m[7] * m[13] -
              m[4] * m[1] * m[15] +
              m[4] * m[3] * m[13] +
              m[12] * m[1] * m[7] -
              m[12] * m[3] * m[5];

    inv[14] = -m[0] * m[5] * m[14] +
              m[0] * m[6] * m[13] +
              m[4] * m[1] * m[14] -
              m[4] * m[2] * m[13] -
              m[12] * m[1] * m[6] +
              m[12] * m[2] * m[5];

    inv[3] = -m[1] * m[6] * m[11] +
             m[1] * m[7] * m[10] +
             m[5] * m[2] * m[11] -
             m[5] * m[3] * m[10] -
             m[9] * m[2] * m[7] +
             m[9] * m[3] * m[6];

    inv[7] = m[0] * m[6] * m[11] -
             m[0] * m[7] * m[10] -
             m[4] * m[2] * m[11] +
             m[4] * m[3] * m[10] +
             m[8] * m[2] * m[7] -
             m[8] * m[3] * m[6];

    inv[11] = -m[0] * m[5] * m[11] +
              m[0] * m[7] * m[9] +
              m[4] * m[1] * m[11] -
              m[4] * m[3] * m[9] -
              m[8] * m[1] * m[7] +
              m[8] * m[3] * m[5];

    inv[15] = m[0] * m[5] * m[10] -
              m[0] * m[6] * m[9] -
              m[4] * m[1] * m[10] +
              m[4] * m[2] * m[9] +
              m[8] * m[1] * m[6] -
              m[8] * m[2] * m[5];

    det = m[0] * inv[0] + m[1] * inv[4] + m[2] * inv[8] + m[3] * inv[12];

    if (det == 0)
      return false;

    det = 1.0f / det;

    for (i = 0; i < 16; i++)
      invOut[i] = inv[i] * det;

    return true;
  }

  // full 4x4 invert
  void matrix4::invert4x4(matrix4 &dst, const matrix4 &src)
  {
#if SMATRIX4_USE_D3DX
    D3DXMatrixInverse(dst, NULL, src);
#else
    const float *fsrc = (const float *)&src.m00;
    float *fdst = (float *)&dst.m00;
    if (!gluInvertMatrix(fsrc, fdst))
    {
      dst.identity();
    }
#endif
  }

} // namespace stevesch
