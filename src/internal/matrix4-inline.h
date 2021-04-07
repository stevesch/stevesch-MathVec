#ifndef STEVESCH_MATHVEC_INTERNAL_MATRIX4_INLINE_H_
#define STEVESCH_MATHVEC_INTERNAL_MATRIX4_INLINE_H_
// Copyright © 2002, PulseCode Interactive LLC, All Rights Reserved
// inline header for matrix4

namespace stevesch
{

  SMATRIX4INLINE matrix4::matrix4(float s)
  {
    diag3(s);
  }

  // translation matrix
  SMATRIX4INLINE matrix4::matrix4(float x, float y, float z)
  {
    col[0].set(1.0f, 0.0f, 0.0f, 0.0f);
    col[1].set(0.0f, 1.0f, 0.0f, 0.0f);
    col[2].set(0.0f, 0.0f, 1.0f, 0.0f);
    col[3].set(x, y, z, 1.0f);
  }

  // translation matrix
  SMATRIX4INLINE matrix4::matrix4(const stevesch::vector3 &vTranslation)
  {
    col[0].set(1.0f, 0.0f, 0.0f, 0.0f);
    col[1].set(0.0f, 1.0f, 0.0f, 0.0f);
    col[2].set(0.0f, 0.0f, 1.0f, 0.0f);
    col[3].set(vTranslation.x, vTranslation.y, vTranslation.z, 1.0f);
  }

  // copy
  SMATRIX4INLINE matrix4::matrix4(const matrix4 &m)
  {
    col[0] = m.col[0];
    col[1] = m.col[1];
    col[2] = m.col[2];
    col[3] = m.col[3];
  }

  // initialize by row
  SMATRIX4INLINE matrix4::matrix4(const stevesch::vector4 &vx, const stevesch::vector4 &vy, const stevesch::vector4 &vz, const stevesch::vector4 &vw)
  {
    col[0] = vx;
    col[1] = vy;
    col[2] = vz;
    col[3] = vw;
  }

  // initialize by element
  SMATRIX4INLINE matrix4::matrix4(float _xx, float _xy, float _xz, float _xw,
                                                      float _yx, float _yy, float _yz, float _yw,
                                                      float _zx, float _zy, float _zz, float _zw,
                                                      float _wx, float _wy, float _wz, float _ww) : m00(_xx), m10(_xy), m20(_xz), m30(_xw),
                                                                                                    m01(_yx), m11(_yy), m21(_yz), m31(_yw),
                                                                                                    m02(_zx), m12(_zy), m22(_zz), m32(_zw),
                                                                                                    m03(_wx), m13(_wy), m23(_wz), m33(_ww)
  {
  }

  SMATRIX4INLINE const matrix4 &matrix4::copy(const matrix4 &m)
  {
    col[0] = m.col[0];
    col[1] = m.col[1];
    col[2] = m.col[2];
    col[3] = m.col[3];
    return *this;
  }

  SMATRIX4INLINE const matrix4 &matrix4::diag3(float s)
  {
    col[0].set(s, 0.0f, 0.0f, 0.0f);
    col[1].set(0.0f, s, 0.0f, 0.0f);
    col[2].set(0.0f, 0.0f, s, 0.0f);
    col[3].set(0.0f, 0.0f, 0.0f, 1.0f);
    return *this;
  }

  SMATRIX4INLINE const matrix4 &matrix4::diag4(float s)
  {
    col[0].set(s, 0.0f, 0.0f, 0.0f);
    col[1].set(0.0f, s, 0.0f, 0.0f);
    col[2].set(0.0f, 0.0f, s, 0.0f);
    col[3].set(0.0f, 0.0f, 0.0f, s);
    return *this;
  }

  // create a diagonal matrix (0 matrix, m00=v.x, m11=v.y, m22v.z, m33=1.0)
  SMATRIX4INLINE const matrix4 &matrix4::diag3(const stevesch::vector3 &v)
  {
    col[0].set(v.x, 0.0f, 0.0f, 0.0f);
    col[1].set(0.0f, v.y, 0.0f, 0.0f);
    col[2].set(0.0f, 0.0f, v.z, 0.0f);
    col[3].set(0.0f, 0.0f, 0.0f, 1.0f);
    return *this;
  }

  // create a diagonal matrix (0 matrix, m00=v.x, m11=v.y, m22v.z, m33=v.w)
  SMATRIX4INLINE const matrix4 &matrix4::diag4(const stevesch::vector4 &v)
  {
    col[0].set(v.x, 0.0f, 0.0f, 0.0f);
    col[1].set(0.0f, v.y, 0.0f, 0.0f);
    col[2].set(0.0f, 0.0f, v.z, 0.0f);
    col[3].set(0.0f, 0.0f, 0.0f, v.w);
    return *this;
  }

  SMATRIX4INLINE const matrix4 &matrix4::operator=(const matrix4 &m)
  {
    return copy(m);
  }

  // retrieve a column
  SMATRIX4INLINE void matrix4::getColumn(int n, stevesch::vector4 &v) const
  {
    v.x = col[0][n];
    v.y = col[1][n];
    v.z = col[2][n];
    v.w = col[3][n];
  }

  // set a column
  SMATRIX4INLINE void matrix4::setColumn(int n, const stevesch::vector4 &v)
  {
    col[0][n] = v.x;
    col[1][n] = v.y;
    col[2][n] = v.z;
    col[3][n] = v.w;
  }

  // returns translation of matrix in vPos (tx, ty, tz)
  SMATRIX4INLINE void matrix4::getTranslation(stevesch::vector3 &vPos) const
  {
    vPos.set(m03, m13, m23);
  }

  // returns translation of matrix in vPos (tx, ty, tz, 1.0f)
  SMATRIX4INLINE void matrix4::getTranslation(stevesch::vector4 &vPos) const
  {
    vPos.set(m03, m13, m23, 1.0f);
  }

  // sets x,y,z translation of matrix to vPos
  SMATRIX4INLINE void matrix4::setTranslation(const stevesch::vector3 &vPos)
  {
    m03 = vPos.x;
    m13 = vPos.y;
    m23 = vPos.z;
  }

  // sets x,y,z translation of matrix to tx,ty,tz
  SMATRIX4INLINE void matrix4::setTranslation(float tx, float ty, float tz)
  {
    m03 = tx;
    m13 = ty;
    m23 = tz;
  }

  SMATRIX4INLINE const matrix4 &matrix4::transpose()
  {
    transpose(*this, *this);
    return *this;
  }

  SMATRIX4INLINE const matrix4 &matrix4::mul(const matrix4 &mtxRight)
  {
    matrix4::mul(*this, *this, mtxRight);
    return *this;
  }

  // SMATRIX4INLINE const matrix4& matrix4::RMul(const matrix4& mtxRight)
  // {
  // 	matrix4::mul( *this, *this, mtxRight );
  // 	return *this;
  // }

  SMATRIX4INLINE const matrix4 &matrix4::mulSub(const matrix4 &r)
  {
    mulSub(*this, *this, r);
    return *this;
  }

  SMATRIX4INLINE const matrix4 &matrix4::lMulSub(const matrix4 &mtxLeft)
  {
    mulSubB(*this, mtxLeft, *this);
    return *this;
  }

  SMATRIX4INLINE const matrix4 &matrix4::operator*=(const matrix4 &r)
  {
    return mul(r);
  }

  // member-wise addition
  SMATRIX4INLINE const matrix4 &matrix4::operator+=(const matrix4 &m)
  {
    col[0].add4(m.col[0]);
    col[1].add4(m.col[1]);
    col[2].add4(m.col[2]);
    col[3].add4(m.col[3]);
    return *this;
  }

  // member-wise subtraction
  SMATRIX4INLINE const matrix4 &matrix4::operator-=(const matrix4 &m)
  {
    col[0].sub4(m.col[0]);
    col[1].sub4(m.col[1]);
    col[2].sub4(m.col[2]);
    col[3].sub4(m.col[3]);
    return *this;
  }

  // translate by v (col[3] += v)
  SMATRIX4INLINE const matrix4 &matrix4::translate(const stevesch::vector4 &v)
  {
    col[3].add(v);
    return *this;
  }

  // translate by v (col[3].xyz += v)
  SMATRIX4INLINE const matrix4 &matrix4::translate(const stevesch::vector3 &v)
  {
    stevesch::vector4 &vt = col[3];
    vt.x += v.x;
    vt.y += v.y;
    vt.z += v.z;
    // ((stevesch::vector3&)col[3]) += v;
    return *this;
  }

  // translate by <dx, dy, dz> (col[3] += v)
  SMATRIX4INLINE const matrix4 &matrix4::translate(float dx, float dy, float dz)
  {
    stevesch::vector4 &vt = col[3];
    vt.x += dx;
    vt.y += dy;
    vt.z += dz;
    return *this;
  }

  // member-wise multiplication
  SMATRIX4INLINE const matrix4 &matrix4::mulMembers4x4(const matrix4 &m)
  {
    col[0].mul4(m.col[0]);
    col[1].mul4(m.col[1]);
    col[2].mul4(m.col[2]);
    col[3].mul4(m.col[3]);
    return *this;
  }

  // member-wise multiplication by scalar fScale
  SMATRIX4INLINE const matrix4 &matrix4::mulMembers4x4(float fScale)
  {
    col[0].mul4(fScale);
    col[1].mul4(fScale);
    col[2].mul4(fScale);
    col[3].mul4(fScale);
    return *this;
  }

  // member-wise multiplication by scalar fScale
  SMATRIX4INLINE const matrix4 &matrix4::mulMembers3x3(float fScale)
  {
    col[0].mul(fScale);
    col[1].mul(fScale);
    col[2].mul(fScale);
    return *this;
  }

  // m = 0.5*(m + m^T)
  SMATRIX4INLINE const matrix4 &matrix4::forceSymmetric()
  {
    matrix4 mT;
    transpose(mT, *this);
    *this += mT;
    return mulMembers4x4(0.5f);
  }

  SMATRIX4INLINE const matrix4 &matrix4::invert()
  {
    invert(*this, *this);
    return *this;
  }

  SMATRIX4INLINE const matrix4 &matrix4::invert4x4()
  {
    invert4x4(*this, *this);
    return *this;
  }

  // member-wise multiplication
  SMATRIX4INLINE void matrix4::mulMembers4x4(matrix4 &dst, const matrix4 &m1, const matrix4 &m2)
  {
    stevesch::vector4::mul4(dst.col[0], m1.col[0], m2.col[0]);
    stevesch::vector4::mul4(dst.col[1], m1.col[1], m2.col[1]);
    stevesch::vector4::mul4(dst.col[2], m1.col[2], m2.col[2]);
    stevesch::vector4::mul4(dst.col[3], m1.col[3], m2.col[3]);
  }

  // member-wise multiplication by scalar fScale for entire matrix (4x4)
  SMATRIX4INLINE void matrix4::mulMembers4x4(matrix4 &dst, const matrix4 &m, float fScale)
  {
    stevesch::vector4::mul4(dst.col[0], m.col[0], fScale);
    stevesch::vector4::mul4(dst.col[1], m.col[1], fScale);
    stevesch::vector4::mul4(dst.col[2], m.col[2], fScale);
    stevesch::vector4::mul4(dst.col[3], m.col[3], fScale);
  }

  // member-wise multiplication by scalar fScale for 3x3 part
  SMATRIX4INLINE void matrix4::mulMembers3x3(matrix4 &dst, const matrix4 &m, float fScale)
  {
    stevesch::vector4::mul(dst.col[0], m.col[0], fScale);
    stevesch::vector4::mul(dst.col[1], m.col[1], fScale);
    stevesch::vector4::mul(dst.col[2], m.col[2], fScale);
    dst.col[3] = m.col[3];
  }

  SMATRIX4INLINE const matrix4 &matrix4::identity()
  {
    *this = I;
    //		col[0].set(1.0f, 0.0f, 0.0f, 0.0f);
    //		col[1].set(0.0f, 1.0f, 0.0f, 0.0f);
    //		col[2].set(0.0f, 0.0f, 1.0f, 0.0f);
    //		col[3].set(0.0f, 0.0f, 0.0f, 1.0f);
    return *this;
  }

  SMATRIX4INLINE const matrix4 &matrix4::zero()
  {
    col[0].set(0.0f, 0.0f, 0.0f, 0.0f);
    col[1].set(0.0f, 0.0f, 0.0f, 0.0f);
    col[2].set(0.0f, 0.0f, 0.0f, 0.0f);
    col[3].set(0.0f, 0.0f, 0.0f, 0.0f);
    return *this;
  }

  // cross-product operator matrix A~:
  //  0 -z +y  0
  // +z  0 -x  0
  // -y +x  0  0
  //  0  0  0  1	// NOTE [3,3] is 1.0
  SMATRIX4INLINE const matrix4 &matrix4::crossOperator(const stevesch::vector3 &v)
  {
    m00 = 0.0f; //  0 -z +y
    m10 = -v.z;
    m20 = v.y;
    m30 = 0.0f;

    m01 = v.z; // +z  0 -x
    m11 = 0.0f;
    m21 = -v.x;
    m31 = 0.0f;

    m02 = -v.y; // -y +x  0
    m12 = v.x;
    m22 = 0.0f;
    m32 = 0.0f;

    m03 = 0.0f;
    m13 = 0.0f;
    m23 = 0.0f;
    m33 = 1.0f;

    return *this;
  }

  SMATRIX4INLINE void matrix4::crossOperator(stevesch::vector4 *pMatrix, const stevesch::vector3 &v, uint32_t nRowStride)
  {
    pMatrix->y = -v.z;
    pMatrix->z = v.y;

    pMatrix += nRowStride;
    pMatrix->x = v.z;
    pMatrix->z = -v.x;

    pMatrix += nRowStride;
    pMatrix->x = -v.y;
    pMatrix->y = v.x;
  }

  SMATRIX4INLINE void matrix4::crossOperator(stevesch::vector4 *pMatrix, const stevesch::vector3 &v, uint32_t nRowStride, bool bNegate)
  {
    if (!bNegate)
    {
      // execute positive version
      crossOperator(pMatrix, v, nRowStride);
      return;
    }

    pMatrix->y = v.z;
    pMatrix->z = -v.y;

    pMatrix += nRowStride;
    pMatrix->x = -v.z;
    pMatrix->z = v.x;

    pMatrix += nRowStride;
    pMatrix->x = v.y;
    pMatrix->y = -v.x;
  }

  SMATRIX4INLINE const stevesch::vector4 &matrix4::operator[](int n) const
  {
    return col[n];
  }

  SMATRIX4INLINE stevesch::vector4 &matrix4::operator[](int n)
  {
    return col[n];
  }

  SMATRIX4INLINE stevesch::vector4 matrix4::getRow(int n) const
  {
    SASSERT((n >= 0) && (n < 4));
    stevesch::vector4 v(
        col[0][n],
        col[1][n],
        col[2][n],
        col[3][n]);
    return v;
  }

  SMATRIX4INLINE const matrix4 &matrix4::perspectiveLH(
      float fFOVhRadians, float fFOVvRadians, float fZNear, float fZFar)
  {
    float Q = fZFar / (fZFar - fZNear);
    zero();
    m00 = 1.0f / tanf(fFOVhRadians * 0.5f);
    m11 = 1.0f / tanf(fFOVvRadians * 0.5f);
    m22 = Q;
    m23 = -fZNear * Q;
    m32 = 1.0f;

    return *this;
  }

  SMATRIX4INLINE const matrix4 &matrix4::perspectiveRH(
      float fFOVhRadians, float fFOVvRadians, float fZNear, float fZFar)
  {
    float Q = fZFar / (fZFar - fZNear);
    zero();
    m00 = 1.0f / tanf(fFOVhRadians * 0.5f);
    m11 = 1.0f / tanf(fFOVvRadians * 0.5f);
    m22 = -Q;
    m23 = fZNear * Q;
    m32 = -1.0f;

    return *this;
  }

  SMATRIX4INLINE void matrix4::perspectiveFovLH(matrix4 &mtxProjDst,
                                                          float fFOVVertical, float fAspectWoverH, float fZNear, float fZFar)
  {
#if defined(SPLATFORM_XBOX)
    XGMatrixPerspectiveFovLH((XGMATRIX *)&mtxProjDst, fFOVVertical, fAspectWoverH, fZNear, fZFar);
#elif defined(SPLATFORM_PC)
#if (SDXVER > 0x9fff)
    mtxProjDst = DirectX::XMMatrixPerspectiveFovLH(fFOVVertical, fAspectWoverH, fZNear, fZFar);
#else
    D3DXMatrixPerspectiveFovLH(mtxProjDst, fFOVVertical, fAspectWoverH, fZNear, fZFar);
#endif

//#define D3DXMatrixPerspectiveFovRH	DirectX::XMMatrixPerspectiveFovRH	// TODO: fix arg order + return value
#else
    float fFOVHorizontal = fFOVVertical / fAspectWoverH;
    mtxProjDst.perspectiveLH(fFOVHorizontal, fFOVVertical, fZNear, fZFar);
#endif
  }

  SMATRIX4INLINE void matrix4::perspectiveFovRH(matrix4 &mtxProjDst,
                                                          float fFOVVertical, float fAspectWoverH, float fZNear, float fZFar)
  {
#if defined(SPLATFORM_XBOX)
    XGMatrixPerspectiveFovRH((XGMATRIX *)&mtxProjDst, fFOVVertical, fAspectWoverH, fZNear, fZFar);
#elif defined(SPLATFORM_PC)
#if (SDXVER > 0x9fff)
    mtxProjDst = DirectX::XMMatrixPerspectiveFovRH(fFOVVertical, fAspectWoverH, fZNear, fZFar);
#else
    D3DXMatrixPerspectiveFovRH(mtxProjDst, fFOVVertical, fAspectWoverH, fZNear, fZFar);
#endif
#else
    float fFOVHorizontal = fFOVVertical / fAspectWoverH;
    mtxProjDst.perspectiveRH(fFOVHorizontal, fFOVVertical, fZNear, fZFar);
#endif
  }

} // namespace stevesch

#endif // STEVESCH_MATHVEC_INTERNAL_MATRIX4_INLINE_H_
