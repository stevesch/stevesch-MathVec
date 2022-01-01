#ifndef STEVESCH_MATHVEC_INTERNAL_VECTOR2_INLINE_H_
#define STEVESCH_MATHVEC_INTERNAL_VECTOR2_INLINE_H_

#include "vector2.h"

namespace stevesch
{
  ///////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////

  inline vector2::vector2(const vector2 &v)
  {
    copy(v);
  }

  // copy (all memebers)
  inline vector2 &vector2::copy(const vector2 &v)
  {
    x = v.x;
    y = v.y;
    return *this;
  }

  inline vector2& vector2::set(float _x, float _y)
  {
    x = _x;
    y = _y;
    return *this;
  }

  inline vector2& vector2::set(const vector2 &v)
  {
    x = v.x;
    y = v.y;
    return *this;
  }

  // use x and y from stevesch::vector4
  inline vector2& vector2::set(const stevesch::vector4 &v)
  {
    x = v.x;
    y = v.y;
    return *this;
  }

  ////////////////////////////////////////////////////

  // member-wise addition (3-element)
  inline vector2 &vector2::add(const vector2 &v)
  {
    x += v.x;
    y += v.y;
    return *this;
  }

  // member-wise subtraction (3-element)
  inline vector2 &vector2::sub(const vector2 &v)
  {
    x -= v.x;
    y -= v.y;
    return *this;
  }

  // member-wise multiplication (3-element)
  inline vector2 &vector2::mul(const vector2 &v)
  {
    x *= v.x;
    y *= v.y;
    return *this;
  }

  inline vector2 &vector2::mul(float scale)
  {
    x *= scale;
    y *= scale;
    return *this;
  }

  inline vector2 &vector2::scale(float scale)
  {
    return mul(scale);
  }

  inline vector2 &vector2::div(float scale)
  {
    return mul(recipf(scale));
  }

  inline float vector2::cross(const vector2 &v1) const
  {
    return (x * v1.y - y * v1.x);
  }

  // 3-element negation (x=-x, y=-y, z=-z)
  inline vector2 &vector2::negate()
  {
    x = -x;
    y = -y;
    return *this;
  }

  // 3-element negation (dst.x=-x, dst.y=-y, dst.z=-z)
  inline vector2 &vector2::negate(vector2 &dst) const
  {
    dst.x = -x;
    dst.y = -y;
    return dst;
  }

  inline vector2 vector2::operator-() const
  {
    return vector2(-x, -y);
  }

  inline float vector2::dot(const vector2 &v1) const
  {
    return x * v1.x + y * v1.y;
  }

  // 2-element squared-magnitude
  inline float vector2::squareMag() const
  {
    return (x * x + y * y);
  }

  // 2-element magnitude
  inline float vector2::abs() const
  {
    return sqrtf(squareMag());
  }

  ////////////////////////////////////////////////////

  inline vector2 &vector2::operator=(const vector2 &v)
  {
    return copy(v);
  }

  // add
  inline vector2 &vector2::operator+=(const vector2 &v)
  {
    return add(v);
  }

  // sub
  inline vector2 &vector2::operator-=(const vector2 &v)
  {
    return sub(v);
  }

  // scale (mul(scalar))
  inline vector2 &vector2::operator*=(float scale)
  {
    return mul(scale);
  }

  // mul(1/scalar)
  inline vector2 &vector2::operator/=(float scale)
  {
    return div(scale);
  }

  inline vector2 &vector2::operator+=(const stevesch::vector4 &v1)
  {
    x += v1.x;
    y += v1.y;
    return *this;
  }

  inline vector2 &vector2::operator-=(const stevesch::vector4 &v1)
  {
    x -= v1.x;
    y -= v1.y;
    return *this;
  }

  // access by index
  inline float vector2::operator[](int n) const
  {
    SASSERT((n == 0) || (n == 1));
    return m_v[n];
  }

  // access by index
  inline float &vector2::operator[](int n)
  {
    SASSERT((n == 0) || (n == 1));
    return m_v[n];
  }

  // inline vector2& vector2::operator*=( const matrix2& crRight )
  // {
  // 	float _x = x*crRight.m00 + y*crRight.m01;
  // 	float _y = x*crRight.m10 + y*crRight.m11;
  // 	x = _x;
  // 	y = _y;
  // 	return *this;
  // }

  inline vector2 &vector2::transform(const matrix2 &m)
  {
    //vector2::transform(*this, m, *this);
    float _x = m.m00 * x + m.m01 * y;
    float _y = m.m10 * x + m.m11 * y;
    x = _x;
    y = _y;
    return *this;
  }

  inline vector2 &vector2::transformTransposed(const matrix2 &m)
  {
    float _x = m.m00 * x + m.m10 * y;
    float _y = m.m01 * x + m.m11 * y;
    this->set(_x, _y);
    return *this;
  }

  // inline vector2 vector2::operator*( const matrix2& crRight ) const
  // {
  // 	float _x = x*crRight.m00 + y*crRight.m01;
  // 	float _y = x*crRight.m10 + y*crRight.m11;
  // 	return vector2( _x, _y );
  // }

  inline void vector2::transform(vector2 &vOut, const matrix2 &m, const vector2 &v)
  {
    // vOut = m*v
    float _x = m.m00 * v.x + m.m01 * v.y;
    float _y = m.m10 * v.x + m.m11 * v.y;
    vOut.set(_x, _y);
  }

  ///////////////////////////////////////
  // static methods

  // dst = v1 + v2
  inline void vector2::add(vector2 &dst, const vector2 &v1, const vector2 &v2)
  {
    vector2 temp(v1);
    temp.add(v2);
    dst = temp;
  }

  // dst = v1 - v2
  inline void vector2::sub(vector2 &dst, const vector2 &v1, const vector2 &v2)
  {
    vector2 temp(v1);
    temp.sub(v2);
    dst = temp;
  }

  // dst = v1 * v2
  inline void vector2::mul(vector2 &dst, const vector2 &v1, const vector2 &v2)
  {
    vector2 temp(v1);
    temp.mul(v2);
    dst = temp;
  }

  // dst = v1 / s
  inline void vector2::div(vector2 &dst, const vector2 &v1, float s)
  {
    dst = v1;
    dst.div(s);
  }

  // dst = v1 * s
  inline void vector2::scale(vector2 &dst, const vector2 &v1, float s)
  {
    dst = v1;
    dst.mul(s);
  }

  inline float vector2::dot(const vector2 &v1, const vector2 &v2)
  {
    return v1.dot(v2);
  }

  // dst = v1 x v2
  inline float vector2::cross(const vector2 &v1, const vector2 &v2)
  {
    return v1.cross(v2);
  }

  inline float vector2::squareDist(const vector2 &v0, const vector2 &v1)
  {
    return (v1 - v0).squareMag();
  }

  // dst = min(v1, v2) per element (3-element)
  inline void vector2::min(vector2 &dst, const vector2 &v1, const vector2 &v2)
  {
    dst.x = stevesch::minf(v1.x, v2.x);
    dst.y = stevesch::minf(v1.y, v2.y);
  }

  // dst = max(v1, v2) per element (3-element)
  inline void vector2::max(vector2 &dst, const vector2 &v1, const vector2 &v2)
  {
    dst.x = stevesch::maxf(v1.x, v2.x);
    dst.y = stevesch::maxf(v1.y, v2.y);
  }

  // linear interpolation t=[0, 1] -> dst=[v1, v2] (2-element)
  inline void vector2::lerp(vector2 &dst, const vector2 &v1, const vector2 &v2, float t)
  {
    // (1-t)*v1 + t*v2 == v1 - t*v1 + t*v2 == v1 + t*(v2 - v1)
    dst = v1 + (v2 - v1)*t;
  }

  inline void vector2::addScaled(vector2 &dst, const vector2 &v1, const vector2 &v2, float s2)
  {
    dst = v1 + v2*s2;
  }

  inline void vector2::addScaled(vector2 &dst, const vector2 &v1, float s1, const vector2 &v2, float s2)
  {
    dst = v1*s1 + v2*s2;
  }

  ////////////////////////////////////////////////////

  inline vector2 operator+(const vector2 &v1, const vector2 &v2)
  {
    vector2 v;
    vector2::add(v, v1, v2);
    return v;
  }

  inline vector2 operator-(const vector2 &v1, const vector2 &v2)
  {
    vector2 v;
    vector2::sub(v, v1, v2);
    return v;
  }

  inline vector2 operator*(const vector2 &v1, float s)
  {
    vector2 v;
    vector2::scale(v, v1, s);
    return v;
  }

  inline vector2 operator*(float s, const vector2 &v1)
  {
    vector2 v;
    vector2::scale(v, v1, s);
    return v;
  }

  inline vector2 operator/(const vector2 &v1, float d)
  {
    vector2 v;
    vector2::div(v, v1, d);
    return v;
  }

  ///////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////

  // access by row
  inline const vector2 &matrix2::operator[](int n) const
  {
    SASSERT((n == 0) || (n == 1));
    return col[n];
  }

  // access by row
  inline vector2 &matrix2::operator[](int n)
  {
    SASSERT((n == 0) || (n == 1));
    return col[n];
  }

  // ( m00 m10 )^-1 == ( m11 -m10)
  // ( m01 m11 )       (-m01  m00) / (det) where det = (m00*m11 - m10*m01)
  inline matrix2 *matrix2::invert()
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
  inline matrix2 *matrix2::getInverse(matrix2 &rDst) const
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
  inline void matrix2::transpose()
  {
    swapf(m10, m01);
  }

  // return transpose
  inline matrix2 matrix2::getTranspose()
  {
    matrix2 mtxOut(m00, m01, m10, m11);
    return mtxOut;
  }


  inline const matrix2 &matrix2::identity()
  {
    *this = matrix2::I;
    return *this;
  }

  inline const matrix2 &matrix2::zero()
  {
    col[0].set(0.0f, 0.0f);
    col[1].set(0.0f, 0.0f);
    return *this;
  }
} // namespace stevesch

#endif // STEVESCH_MATHVEC_INTERNAL_VECTOR2_INLINE_H_
