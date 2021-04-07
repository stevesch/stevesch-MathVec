#ifndef STEVESCH_MATHVEC_INTERNAL_VECTOR2_INLINE_H_
#define STEVESCH_MATHVEC_INTERNAL_VECTOR2_INLINE_H_

#include "stevesch::vector2.h"

namespace stevesch
{
  ///////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////
  // use x and y from stevesch::vector4
  SFORCEINLINE void stevesch::vector2::set(const stevesch::vector4 &v)
  {
    x = v.x;
    y = v.y;
  }

  // access by index
  SFORCEINLINE float stevesch::vector2::operator[](int n) const
  {
    SASSERT((n == 0) || (n == 1));
    return m_v[n];
  }

  // access by index
  SFORCEINLINE float &stevesch::vector2::operator[](int n)
  {
    SASSERT((n == 0) || (n == 1));
    return m_v[n];
  }

  SFORCEINLINE stevesch::vector2 &stevesch::vector2::operator+=(const stevesch::vector4 &v1)
  {
    x += v1.x;
    y += v1.y;
    return *this;
  }

  SFORCEINLINE stevesch::vector2 &stevesch::vector2::operator-=(const stevesch::vector4 &v1)
  {
    x -= v1.x;
    y -= v1.y;
    return *this;
  }

  SFORCEINLINE stevesch::vector2 operator*(float fScale, const stevesch::vector2 &v)
  {
    return v * fScale;
  }

  // inline stevesch::vector2& stevesch::vector2::operator*=( const stevesch::matrix2& crRight )
  // {
  // 	float _x = x*crRight.m00 + y*crRight.m01;
  // 	float _y = x*crRight.m10 + y*crRight.m11;
  // 	x = _x;
  // 	y = _y;
  // 	return *this;
  // }

  inline stevesch::vector2 &stevesch::vector2::transform(const stevesch::matrix2 &m)
  {
    //stevesch::vector2::transform(*this, m, *this);
    float _x = m.m00 * x + m.m01 * y;
    float _y = m.m10 * x + m.m11 * y;
    x = _x;
    y = _y;
    return *this;
  }

  inline stevesch::vector2 &stevesch::vector2::transformTransposed(const stevesch::matrix2 &m)
  {
    float _x = m.m00 * x + m.m10 * y;
    float _y = m.m01 * x + m.m11 * y;
    this->set(_x, _y);
    return *this;
  }

  // inline stevesch::vector2 stevesch::vector2::operator*( const stevesch::matrix2& crRight ) const
  // {
  // 	float _x = x*crRight.m00 + y*crRight.m01;
  // 	float _y = x*crRight.m10 + y*crRight.m11;
  // 	return stevesch::vector2( _x, _y );
  // }

  inline void stevesch::vector2::transform(stevesch::vector2 &vOut, const stevesch::matrix2 &m, const stevesch::vector2 &v)
  {
    // vOut = m*v
    float _x = m.m00 * v.x + m.m01 * v.y;
    float _y = m.m10 * v.x + m.m11 * v.y;
    vOut.set(_x, _y);
  }

  // get difference of x and y components in v0 and v1
  SFORCEINLINE void stevesch::vector2::sub(stevesch::vector2 &vDst, const stevesch::vector4 &v0, const stevesch::vector4 &v1)
  {
    vDst.set(v0.x - v1.x, v0.y - v1.y);
  }

  // dst = v1 + v2*s2
  inline void stevesch::vector2::addScaled(stevesch::vector2 &dst, const stevesch::vector2 &v1, const stevesch::vector2 &v2, float s2)
  {
    dst.x = v1.x + v2.x * s2;
    dst.y = v1.y + v2.y * s2;
  }

  // dst = v1*s1 + v2*s2
  inline void stevesch::vector2::addScaled(stevesch::vector2 &dst, const stevesch::vector2 &v1, float s1, const stevesch::vector2 &v2, float s2)
  {
    dst.x = v1.x * s1 + v2.x * s2;
    dst.y = v1.y * s1 + v2.y * s2;
  }

  // linear interpolation t=[0, 1] -> dst=[v1, v2] (3-element)
  inline void stevesch::vector2::lerp(stevesch::vector2 &dst, const stevesch::vector2 &v1, const stevesch::vector2 &v2, float t)
  {
    // (1-t)*v1 + t*v2 == v1 - t*v1 + t*v2 == v1 + t*(v2 - v1)
    dst.x = Lerpf(v1.x, v2.x, t);
    dst.y = Lerpf(v1.y, v2.y, t);
  }

  ///////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////

  // access by row
  SFORCEINLINE const stevesch::vector2 &stevesch::matrix2::operator[](int n) const
  {
    SASSERT((n == 0) || (n == 1));
    return col[n];
  }

  // access by row
  SFORCEINLINE stevesch::vector2 &stevesch::matrix2::operator[](int n)
  {
    SASSERT((n == 0) || (n == 1));
    return col[n];
  }

  // ( m00 m10 )^-1 == ( m11 -m10)
  // ( m01 m11 )       (-m01  m00) / (det) where det = (m00*m11 - m10*m01)
  SFORCEINLINE stevesch::matrix2 *stevesch::matrix2::invert()
  {
    const float cfSingularTolerance = 1.0e-6f;

    stevesch::vector4 &v = *(stevesch::vector4 *)this;
    float fDet = v.x * v.w - v.y * v.z; // V4 is <m00, m10, m01, m11>
    if (fabsf(fDet) > cfSingularTolerance)
    {
      v.div4(stevesch::vector4(fDet, -fDet, -fDet, fDet));
      swapf(v.x, v.w);
      return this;
    }
    return NULL;
  }

  // returns &rDst if success
  SFORCEINLINE stevesch::matrix2 *stevesch::matrix2::getInverse(stevesch::matrix2 &rDst) const
  {
    const float cfSingularTolerance = 1.0e-6f;

    const stevesch::vector4 &v = *(stevesch::vector4 *)this;
    float fDet = v.x * v.w - v.y * v.z; // V4 is <m00, m10, m01, m11>
    if (fabsf(fDet) > cfSingularTolerance)
    {
      stevesch::vector4 &dst = *(stevesch::vector4 *)&rDst;
      stevesch::vector4::div4(dst, v, stevesch::vector4(fDet, -fDet, -fDet, fDet));
      swapf(dst.x, dst.w);
      return &rDst;
    }
    return NULL;
  }

  // transpose self
  SFORCEINLINE void stevesch::matrix2::transpose()
  {
    swapf(m10, m01);
  }

  // return transpose
  SFORCEINLINE stevesch::matrix2 stevesch::matrix2::getTranspose()
  {
    stevesch::matrix2 mtxOut(m00, m01, m10, m11);
    return mtxOut;
  }

} // namespace stevesch

#endif // STEVESCH_MATHVEC_INTERNAL_VECTOR2_INLINE_H_
