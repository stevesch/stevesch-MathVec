#ifndef STEVESCH_MATHVEC_INTERNAL_VECTOR3_INLINE_H_
#define STEVESCH_MATHVEC_INTERNAL_VECTOR3_INLINE_H_
// Copyright © 2002, PulseCode Interactive LLC, All Rights Reserved

//#include "vector4.h"
#include "matrix4.h"

namespace stevesch
{
  SVECINLINE stevesch::vector3::stevesch::vector3(float _x, float _y, float _z) : x(_x), y(_y), z(_z)
  {
  }

  SVECINLINE stevesch::vector3::stevesch::vector3(const stevesch::vector3 &v)
  {
    copy(v);
  }

  // copy (all memebers)
  SVECINLINE const stevesch::vector3 &stevesch::vector3::copy(const stevesch::vector3 &v)
  {
    x = v.x;
    y = v.y;
    z = v.z;
    return *this;
  }

  SVECINLINE const stevesch::vector3 &stevesch::vector3::set(float _x, float _y, float _z)
  {
    x = _x;
    y = _y;
    z = _z;
    return *this;
  }

  // member-wise addition (3-element)
  SVECINLINE const stevesch::vector3 &stevesch::vector3::add(const stevesch::vector3 &v)
  {
    x += v.x;
    y += v.y;
    z += v.z;
    return *this;
  }

  // member-wise subtraction (3-element)
  SVECINLINE const stevesch::vector3 &stevesch::vector3::sub(const stevesch::vector3 &v)
  {
    x -= v.x;
    y -= v.y;
    z -= v.z;
    return *this;
  }

  // member-wise multiplication (3-element)
  SVECINLINE const stevesch::vector3 &stevesch::vector3::mul(const stevesch::vector3 &v)
  {
    x *= v.x;
    y *= v.y;
    z *= v.z;
    return *this;
  }

  SVECINLINE const stevesch::vector3 &stevesch::vector3::mul(float scale)
  {
    x *= scale;
    y *= scale;
    z *= scale;
    return *this;
  }

  SVECINLINE const stevesch::vector3 &stevesch::vector3::scale(float scale)
  {
    return mul(scale);
  }

  SVECINLINE const stevesch::vector3 &stevesch::vector3::div(float scale)
  {
    return mul(recipf(scale));
  }

  // 3-element cross (outer) product
  SVECINLINE const stevesch::vector3 &stevesch::vector3::cross(const stevesch::vector3 &v)
  {
    float tx = y * v.z - z * v.y;
    float ty = z * v.x - x * v.z;
    z = x * v.y - y * v.x;
    x = tx;
    y = ty;
    return *this;
  }

  // 3-element negation (x=-x, y=-y, z=-z)
  SVECINLINE const stevesch::vector3 &stevesch::vector3::negate()
  {
    x = -x;
    y = -y;
    z = -z;
    return *this;
  }

  // 3-element negation (dst.x=-x, dst.y=-y, dst.z=-z)
  SVECINLINE const stevesch::vector3 &stevesch::vector3::negate(stevesch::vector3 &dst) const
  {
    dst.x = -x;
    dst.y = -y;
    dst.z = -z;
    return dst;
  }

  // 3-element dot (inner) product
  SVECINLINE float stevesch::vector3::dot(const stevesch::vector3 &v) const
  {
    return (x * v.x + y * v.y + z * v.z);
  }

  // 3-element squared-magnitude
  SVECINLINE float stevesch::vector3::squareMag() const
  {
    return (x * x + y * y + z * z);
  }

  // 3-element magnitude
  SVECINLINE float stevesch::vector3::abs() const
  {
    return S::Sqrtf(squareMag());
  }

  ////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////

  SVECINLINE const stevesch::vector3 &stevesch::vector3::operator=(const stevesch::vector3 &v)
  {
    return copy(v);
  }

  SVECINLINE const float &stevesch::vector3::operator[](int n) const
  {
    SASSERT_EXTRA((n >= 0) && (n <= 2));
    return ((float *)&x)[n];
  }

  SVECINLINE float &stevesch::vector3::operator[](int n)
  {
    SASSERT_EXTRA((n >= 0) && (n <= 2));
    return ((float *)&x)[n];
  }

  // add
  SVECINLINE const stevesch::vector3 &stevesch::vector3::operator+=(const stevesch::vector3 &v)
  {
    return add(v);
  }

  // sub
  SVECINLINE const stevesch::vector3 &stevesch::vector3::operator-=(const stevesch::vector3 &v)
  {
    return sub(v);
  }

  // scale (mul(scalar))
  SVECINLINE const stevesch::vector3 &stevesch::vector3::operator*=(float scale)
  {
    return mul(scale);
  }

  // mul(1/scalar)
  SVECINLINE const stevesch::vector3 &stevesch::vector3::operator/=(float scale)
  {
    return div(scale);
  }

  // 3-element randomize (0.0f, 1.0f)
  SVECINLINE void stevesch::vector3::rand()
  {
    x = Randf();
    y = Randf();
    z = Randf();
  }

  // 3-element randomize (a.*, b.*)
  SVECINLINE void stevesch::vector3::randAB(const stevesch::vector3 &a, const stevesch::vector3 &b)
  {
    stevesch::vector3 dif;
    this->rand();
    sub(dif, b, a);
    mul(dif);
    add(a);
  }

  ////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////

  // transform
  //SVECINLINE const stevesch::vector3& stevesch::vector3::operator *=(const stevesch::matrix4& mRight)
  //{
  //	return transform(mRight);
  //}

  ////////////////////////////////////////////////////
  ////////////////////////////////////////////////////
  // static methods

  // dst = v1 + v2
  SVECINLINE void stevesch::vector3::add(stevesch::vector3 &dst, const stevesch::vector3 &v1, const stevesch::vector3 &v2)
  {
    stevesch::vector3 temp(v1);
    temp.add(v2);
    dst = temp;
  }

  // dst = v1 - v2
  SVECINLINE void stevesch::vector3::sub(stevesch::vector3 &dst, const stevesch::vector3 &v1, const stevesch::vector3 &v2)
  {
    stevesch::vector3 temp(v1);
    temp.sub(v2);
    dst = temp;
  }

  // dst = v1 * v2
  SVECINLINE void stevesch::vector3::mul(stevesch::vector3 &dst, const stevesch::vector3 &v1, const stevesch::vector3 &v2)
  {
    stevesch::vector3 temp(v1);
    temp.mul(v2);
    dst = temp;
  }

  // dst = v1 / s
  SVECINLINE void stevesch::vector3::div(stevesch::vector3 &dst, const stevesch::vector3 &v1, float s)
  {
    dst = v1;
    dst.div(s);
  }

  // dst = v1 * s
  SVECINLINE void stevesch::vector3::scale(stevesch::vector3 &dst, const stevesch::vector3 &v1, float s)
  {
    dst = v1;
    dst.mul(s);
  }

  // dst = v1 x v2
  SVECINLINE void stevesch::vector3::cross(stevesch::vector3 &dst, const stevesch::vector3 &v1, const stevesch::vector3 &v2)
  {
    stevesch::vector3 temp(v1);
    temp.cross(v2);
    dst = temp;
  }

  SVECINLINE float stevesch::vector3::dot(const stevesch::vector3 &v1, const stevesch::vector3 &v2)
  {
    return v1.dot(v2);
  }

  // dst = v * m (4x4)
  SVECINLINE void stevesch::vector3::transform(stevesch::vector3 &dst, const stevesch::matrix4 &m, const stevesch::vector3 &v)
  {
    dst = v;
    dst.transform(m);
  }

  // dst = v * m (4x4, as if v.w = 0.0)
  SVECINLINE void stevesch::vector3::transformSub(stevesch::vector3 &dst, const stevesch::matrix4 &m, const stevesch::vector3 &v)
  {
    dst = v;
    dst.transformSub(m);
  }

  // 3-element squared-distance
  SVECINLINE float stevesch::vector3::squareDist(const stevesch::vector3 &v1, const stevesch::vector3 &v2)
  {
    stevesch::vector3 temp;
    stevesch::vector3::sub(temp, v1, v2);
    return temp.squareMag();
  }

  // dst = min(v1, v2) per element (3-element)
  SVECINLINE void stevesch::vector3::min(stevesch::vector3 &dst, const stevesch::vector3 &v1, const stevesch::vector3 &v2)
  {
    dst.x = S::Minf(v1.x, v2.x);
    dst.y = S::Minf(v1.y, v2.y);
    dst.z = S::Minf(v1.z, v2.z);
  }

  // dst = max(v1, v2) per element (3-element)
  SVECINLINE void stevesch::vector3::max(stevesch::vector3 &dst, const stevesch::vector3 &v1, const stevesch::vector3 &v2)
  {
    dst.x = S::Maxf(v1.x, v2.x);
    dst.y = S::Maxf(v1.y, v2.y);
    dst.z = S::Maxf(v1.z, v2.z);
  }

  // linear interpolation t=[0, 1] -> dst=[v1, v2] (3-element)
  SVECINLINE void stevesch::vector3::lerp(stevesch::vector3 &dst, const stevesch::vector3 &v1, const stevesch::vector3 &v2, float t)
  {
    // (1-t)*v1 + t*v2 == v1 - t*v1 + t*v2 == v1 + t*(v2 - v1)
    stevesch::vector3 tmp;
    stevesch::vector3::sub(tmp, v2, v1); // tmp in case dst is v1
    stevesch::vector3::addScaled(dst, v1, tmp, t);
  }

  // dst = v1 + s*v2
  SVECINLINE void stevesch::vector3::addScaled(stevesch::vector3 &dst, const stevesch::vector3 &v1, const stevesch::vector3 &v2, float s2)
  {
    stevesch::vector3 temp(v2);
    temp *= s2;
    temp += v1;
    dst = temp;
  }

  // dst = v1*s1 + v2*s2
  SVECINLINE void stevesch::vector3::addScaled(stevesch::vector3 &dst, const stevesch::vector3 &v1, float s1, const stevesch::vector3 &v2, float s2)
  {
    stevesch::vector3 temp1(v1);
    stevesch::vector3 temp2(v2);
    temp1 *= s1;
    temp2 *= s2;

    temp1 += temp2;
    dst = temp1;
  }

  ////////////////////////////////////////////////////
  ////////////////////////////////////////////////////

} // namespace SMath

#endif // STEVESCH_MATHVEC_INTERNAL_VECTOR3_INLINE_H_
