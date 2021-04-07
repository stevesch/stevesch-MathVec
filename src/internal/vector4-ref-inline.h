#ifndef STEVESCH_MATHVEC_INTERNAL_VECTOR4_REF_INLINE_H_
#define STEVESCH_MATHVEC_INTERNAL_VECTOR4_REF_INLINE_H_
// inline functions for stevesch::vector4
//
// Copyright © 2002, PulseCode Interactive LLC, All Rights Reserved
//

namespace stevesch
{
  // dst = min(v1, v2) per element (3-element)
  SVECINLINE void stevesch::vector4::min(stevesch::vector4 &dst, const stevesch::vector4 &v1, const stevesch::vector4 &v2)
  {
    dst.x = S::Minf(v1.x, v2.x);
    dst.y = S::Minf(v1.y, v2.y);
    dst.z = S::Minf(v1.z, v2.z);
    dst.w = v1.w;
  }

  // dst = max(v1, v2) per element (3-element)
  SVECINLINE void stevesch::vector4::max(stevesch::vector4 &dst, const stevesch::vector4 &v1, const stevesch::vector4 &v2)
  {
    dst.x = S::Maxf(v1.x, v2.x);
    dst.y = S::Maxf(v1.y, v2.y);
    dst.z = S::Maxf(v1.z, v2.z);
    dst.w = v1.w;
  }

  SVECINLINE void stevesch::vector4::clampMag(stevesch::vector4 &vDst, const stevesch::vector4 &vSrc, const stevesch::vector4 &vClamp)
  {
    vDst.x = Clampf(vSrc.x, -vClamp.x, vClamp.x);
    vDst.y = Clampf(vSrc.y, -vClamp.y, vClamp.y);
    vDst.z = Clampf(vSrc.z, -vClamp.z, vClamp.z);
    vDst.w = vSrc.w;
  }

  // dst = min(v1, v2) per element (4-element)
  SVECINLINE void stevesch::vector4::min4(stevesch::vector4 &dst, const stevesch::vector4 &v1, const stevesch::vector4 &v2)
  {
    dst.x = S::Minf(v1.x, v2.x);
    dst.y = S::Minf(v1.y, v2.y);
    dst.z = S::Minf(v1.z, v2.z);
    dst.w = S::Minf(v1.w, v2.w);
  }

  // dst = max(v1, v2) per element (4-element)
  SVECINLINE void stevesch::vector4::max4(stevesch::vector4 &dst, const stevesch::vector4 &v1, const stevesch::vector4 &v2)
  {
    dst.x = S::Maxf(v1.x, v2.x);
    dst.y = S::Maxf(v1.y, v2.y);
    dst.z = S::Maxf(v1.z, v2.z);
    dst.w = S::Maxf(v1.w, v2.w);
  }

  SVECINLINE void stevesch::vector4::clampMag4(stevesch::vector4 &vDst, const stevesch::vector4 &vSrc, const stevesch::vector4 &vClamp)
  {
    vDst.x = Clampf(vSrc.x, -vClamp.x, vClamp.x);
    vDst.y = Clampf(vSrc.y, -vClamp.y, vClamp.y);
    vDst.z = Clampf(vSrc.z, -vClamp.z, vClamp.z);
    vDst.w = Clampf(vSrc.w, -vClamp.w, vClamp.w);
  }

  // linear interpolation t=[0, 1] -> dst=[v1, v2] (4-element)
  SVECINLINE void stevesch::vector4::lerp4(stevesch::vector4 &dst, const stevesch::vector4 &v1, const stevesch::vector4 &v2, float t)
  {
    // (1-t)*v1 + t*v2 == v1 - t*v1 + t*v2 == v1 + t*(v2 - v1)
    stevesch::vector4 tmp;
    SVECTORSUB4(tmp, v2, v1); // tmp in case dst is v1
    stevesch::vector4::addScaled4(dst, v1, tmp, t);
  }

  SVECINLINE const stevesch::vector4 &stevesch::vector4::div(float scale)
  {
    return mul(recipf(scale));
  }

  // dst = v1 + v2 (4-element)
  SVECINLINE void stevesch::vector4::add4(stevesch::vector4 &dst, const stevesch::vector4 &v1, const stevesch::vector4 &v2)
  {
    SVECTORADD4(dst, v1, v2);
  }

  // dst = v1 - v2 (4-element)
  SVECINLINE void stevesch::vector4::sub4(stevesch::vector4 &dst, const stevesch::vector4 &v1, const stevesch::vector4 &v2)
  {
    SVECTORSUB4(dst, v1, v2);
  }

  // dst = v1 * v2 (4-element)
  SVECINLINE void stevesch::vector4::mul4(stevesch::vector4 &dst, const stevesch::vector4 &v1, const stevesch::vector4 &v2)
  {
    SVECTORMUL4(dst, v1, v2);
  }

  // dst = v1 * fScale (4-element)
  SVECINLINE void stevesch::vector4::mul4(stevesch::vector4 &dst, const stevesch::vector4 &v1, float fScale)
  {
    SVECTORMUL4S(dst, v1, fScale);
  }

  // dst = v1 / s
  SVECINLINE void stevesch::vector4::div(stevesch::vector4 &dst, const stevesch::vector4 &v1, float s)
  {
    dst = v1;
    dst.div(s);
  }

  // member-wise division (4-element)
  SVECINLINE const stevesch::vector4 &stevesch::vector4::div4(const stevesch::vector4 &v)
  {
    x /= v.x;
    y /= v.y;
    z /= v.z;
    w /= v.w;
    return *this;
  }

  SVECINLINE const stevesch::vector4 &stevesch::vector4::div4(float scale)
  {
    return mul4(recipf(scale));
  }

  // dst = v1 / v2 (4-element member-wise division)
  SVECINLINE void stevesch::vector4::div4(stevesch::vector4 &dst, const stevesch::vector4 &v1, const stevesch::vector4 &v2)
  {
    dst.x = v1.x / v2.x,
    dst.y = v1.y / v2.y,
    dst.z = v1.z / v2.z,
    dst.w = v1.w / v2.w;
  }

  // dst = v1 / s (4-element)
  SVECINLINE void stevesch::vector4::div4(stevesch::vector4 &dst, const stevesch::vector4 &v1, float s)
  {
    //		SVECTORDIV4S( dst, v1, s );
    dst = v1;
    dst.div4(s);
  }

  // dst = v1 * s (4-element)
  SVECINLINE void stevesch::vector4::scale4(stevesch::vector4 &dst, const stevesch::vector4 &v1, float s)
  {
    SVECTORMUL4S(dst, v1, s);
  }

  // dst = v1 * s
  SVECINLINE void stevesch::vector4::scale(stevesch::vector4 &dst, const stevesch::vector4 &v1, float s)
  {
    SVECTORMULS(dst, v1, s);
    //		dst = v1;
    //		dst.mul(s);
  }

  // 3-element dot (inner) product
  SVECINLINE float stevesch::vector4::dot(const stevesch::vector4 &v) const
  {
    return SVECTORDOT(*this, v);
  }

  SVECINLINE float stevesch::vector4::dot(const stevesch::vector4 &v1, const stevesch::vector4 &v2)
  {
    return SVECTORDOT(v1, v2);
  }

  // 4-element dot (inner) product
  SVECINLINE float stevesch::vector4::dot4(const stevesch::vector4 &v) const
  {
    return SVECTORDOT4(*this, v);
  }

  SVECINLINE float stevesch::vector4::dot4(const stevesch::vector4 &v1, const stevesch::vector4 &v2)
  {
    return SVECTORDOT4(v1, v2);
  }

  // member-wise addition (4-element)
  SVECINLINE const stevesch::vector4 &stevesch::vector4::add4(const stevesch::vector4 &v)
  {
    x += v.x;
    y += v.y;
    z += v.z;
    w += v.w;
    return *this;
  }

  // member-wise subtraction (4-element)
  SVECINLINE const stevesch::vector4 &stevesch::vector4::sub4(const stevesch::vector4 &v)
  {
    x -= v.x;
    y -= v.y;
    z -= v.z;
    w -= v.w;
    return *this;
  }

  // member-wise multiplication (4-element)
  SVECINLINE const stevesch::vector4 &stevesch::vector4::mul4(const stevesch::vector4 &v)
  {
    x *= v.x;
    y *= v.y;
    z *= v.z;
    w *= v.w;
    return *this;
  }

  SVECINLINE const stevesch::vector4 &stevesch::vector4::mul4(float scale)
  {
    x *= scale;
    y *= scale;
    z *= scale;
    w *= scale;
    return *this;
  }

  SVECINLINE float stevesch::vector4::squareMag4() const
  {
    return (x * x + y * y + z * z + w * w);
  }

  // dst = v1 + s*v2
  SVECINLINE void stevesch::vector4::addScaled(stevesch::vector4 &dst, const stevesch::vector4 &v1, const stevesch::vector4 &v2, float s2)
  {
    float x = v1.x + v2.x * s2;
    float y = v1.y + v2.y * s2;
    float z = v1.z + v2.z * s2;
    dst.x = x;
    dst.y = y;
    dst.z = z;
    dst.w = v1.w;
  }

  // dst = v1*s1 + v2*s2
  SVECINLINE void stevesch::vector4::addScaled(stevesch::vector4 &dst, const stevesch::vector4 &v1, float s1, const stevesch::vector4 &v2, float s2)
  {
    float x = v1.x * s1 + v2.x * s2;
    float y = v1.y * s1 + v2.y * s2;
    float z = v1.z * s1 + v2.z * s2;
    dst.x = x;
    dst.y = y;
    dst.z = z;
    dst.w = v1.w;
  }

  // dst = v1 + s*v2 (4-element)
  SVECINLINE void stevesch::vector4::addScaled4(stevesch::vector4 &dst, const stevesch::vector4 &v1, const stevesch::vector4 &v2, float s2)
  {
    stevesch::vector4 temp(v2);
    SVECTORMUL4S(temp, v2, s2);
    SVECTORADD4(dst, temp, v1);
  }

  // dst = v1*s1 + v2*s2 (4-element)
  SVECINLINE void stevesch::vector4::addScaled4(stevesch::vector4 &dst, const stevesch::vector4 &v1, float s1, const stevesch::vector4 &v2, float s2)
  {
    stevesch::vector4 temp1, temp2;
    SVECTORMUL4S(temp1, v1, s1);
    SVECTORMUL4S(temp2, v2, s2);
    SVECTORADD4(dst, temp1, temp2);
  }

  SVECINLINE stevesch::vector4::stevesch::vector4(const stevesch::vector4 &v)
  {
    SVECTORCOPY(*this, v);
  }

  // copy (all memebers)
  SVECINLINE const stevesch::vector4 &stevesch::vector4::copy(const stevesch::vector4 &v)
  {
    SVECTORCOPY(*this, v);
    return *this;
  }

  SVECINLINE const stevesch::vector4 &stevesch::vector4::operator=(const stevesch::vector4 &v)
  {
    SVECTORCOPY(*this, v);
    return *this;
  }

  // 3-element squared-magnitude
  SVECINLINE float stevesch::vector4::squareMag() const
  {
    return (x * x + y * y + z * z);
  }

  // 1.0 / squared magnitued (3-element)
  SVECINLINE float stevesch::vector4::recipSquareMag() const
  {
    return S::recipf(squareMag()); // TBD: optimize with intrinsics
  }

  // 3-element magnitude
  SVECINLINE float stevesch::vector4::abs() const
  {
    return S::Sqrtf(squareMag());
  }

  // 3-element 1/magnitude
  SVECINLINE float stevesch::vector4::recipAbs() const
  {
    return S::RSqrtf(squareMag());
  }

  // normalize self (3-element)
  SVECINLINE const stevesch::vector4 &stevesch::vector4::normalize()
  {
    return mul(RSqrtf(squareMag()));
  }

  SVECINLINE void stevesch::vector4::normalize(stevesch::vector4 &dst, const stevesch::vector4 &src)
  {
    scale(dst, src, RSqrtf(src.squareMag()));
  } // (3-element)

  // normalize self (4-element)
  SVECINLINE const stevesch::vector4 &stevesch::vector4::normalize4()
  {
    return mul4(RSqrtf(squareMag4()));
  }

  // squared distance between two points: (v2-v1).(v2-v1)
  SVECINLINE float stevesch::vector4::distanceSquared(const stevesch::vector4 &v1, const stevesch::vector4 &v2)
  {
    stevesch::vector4 vDif;
    stevesch::vector4::sub(vDif, v2, v1);
    return vDif.squareMag();
  }

  // squared distance between two points: (v2-v1).(v2-v1)
  SVECINLINE float stevesch::vector4::distanceSquared(stevesch::vector4 &dst, const stevesch::vector4 &v1, const stevesch::vector4 &v2)
  {
    stevesch::vector4 vDif;
    stevesch::vector4::sub(dst, v2, v1);
    return dst.squareMag();
  }

  // 3-element cross (outer) product
  SVECINLINE const stevesch::vector4 &stevesch::vector4::cross(const stevesch::vector4 &v)
  {
    SCROSS(*this, *this, v);
    return *this;
  }

  // dst = v1 x v2
  SVECINLINE void stevesch::vector4::cross(stevesch::vector4 &dst, const stevesch::vector4 &v1, const stevesch::vector4 &v2)
  {
    SCROSS(dst, v1, v2);
  }

  // dst = (1.0 / v) (3-element)
  SVECINLINE void stevesch::vector4::recip(stevesch::vector4 &dst, const stevesch::vector4 &v)
  {
    dst.x = recipf(v.x);
    dst.y = recipf(v.y);
    dst.z = recipf(v.z);
    dst.w = v.w;
  }

  // dst = sqrt(v) (3-element)
  SVECINLINE void stevesch::vector4::sqrt(stevesch::vector4 &dst, const stevesch::vector4 &v)
  {
    dst.x = Sqrtf(v.x);
    dst.y = Sqrtf(v.y);
    dst.z = Sqrtf(v.z);
    dst.w = v.w;
  }

  // dst = sqrt(v) (3-element)
  SVECINLINE void stevesch::vector4::recipSqrt(stevesch::vector4 &dst, const stevesch::vector4 &v)
  {
    dst.x = RSqrtf(v.x);
    dst.y = RSqrtf(v.y);
    dst.z = RSqrtf(v.z);
    dst.w = v.w;
  }

  // dst = (1.0 / v) (4-element)
  SVECINLINE void stevesch::vector4::recip4(stevesch::vector4 &dst, const stevesch::vector4 &v)
  {
    dst.x = recipf(v.x);
    dst.y = recipf(v.y);
    dst.z = recipf(v.z);
    dst.w = recipf(v.w);
  }

  // dst = sqrt(v) (4-element)
  SVECINLINE void stevesch::vector4::sqrt4(stevesch::vector4 &dst, const stevesch::vector4 &v)
  {
    dst.x = Sqrtf(v.x);
    dst.y = Sqrtf(v.y);
    dst.z = Sqrtf(v.z);
    dst.w = Sqrtf(v.w);
  }

  // dst = sqrt(v) (4-element)
  SVECINLINE void stevesch::vector4::recipSqrt4(stevesch::vector4 &dst, const stevesch::vector4 &v)
  {
    dst.x = RSqrtf(v.x);
    dst.y = RSqrtf(v.y);
    dst.z = RSqrtf(v.z);
    dst.w = RSqrtf(v.w);
  }

  // *this += vm1*vm2 (3-element)
  SVECINLINE void stevesch::vector4::mad(const stevesch::vector4 &vm1, const stevesch::vector4 &vm2)
  {
    x += vm1.x * vm2.x;
    y += vm1.y * vm2.y;
    z += vm1.z * vm2.z;
  }

  // *this += vm1*vm2 (4-element)
  SVECINLINE void stevesch::vector4::mad4(const stevesch::vector4 &vm1, const stevesch::vector4 &vm2)
  {
    x += vm1.x * vm2.x;
    y += vm1.y * vm2.y;
    z += vm1.z * vm2.z;
    w += vm1.w * vm2.w;
  }

  // dst = vm1*vm2 + va
  SVECINLINE void stevesch::vector4::mad(stevesch::vector4 &dst, const stevesch::vector4 &vm1, const stevesch::vector4 &vm2, const stevesch::vector4 &va)
  {
    dst.x = (vm1.x * vm2.x) + va.x;
    dst.y = (vm1.y * vm2.y) + va.y;
    dst.z = (vm1.z * vm2.z) + va.z;
    dst.w = va.w;
  }

  // dst = vm1*vm2 + va
  SVECINLINE void stevesch::vector4::mad4(stevesch::vector4 &dst, const stevesch::vector4 &vm1, const stevesch::vector4 &vm2, const stevesch::vector4 &va)
  {
    dst.x = (vm1.x * vm2.x) + va.x;
    dst.y = (vm1.y * vm2.y) + va.y;
    dst.z = (vm1.z * vm2.z) + va.z;
    dst.w = (vm1.w * vm2.w) + va.w;
  }

  // the above functions (triangleEdges and triangleVectors), combined
  SVECINLINE void stevesch::vector4::triangleEdgesAndVectors(stevesch::vector4 *pEdge, stevesch::vector4 *pVector, const stevesch::vector4 *pTriangleVertex, const stevesch::vector4 &vPoint)
  {
    SVECTORSUB(pEdge[0], pTriangleVertex[2], pTriangleVertex[1]);
    SVECTORSUB(pEdge[1], pTriangleVertex[0], pTriangleVertex[2]);
    SVECTORSUB(pEdge[2], pTriangleVertex[1], pTriangleVertex[0]);

    SVECTORSUB(pVector[0], vPoint, pTriangleVertex[0]);
    SVECTORSUB(pVector[1], vPoint, pTriangleVertex[1]);
    SVECTORSUB(pVector[2], vPoint, pTriangleVertex[2]);
  }

} // namespace stevesch

#endif // STEVESCH_MATHVEC_INTERNAL_VECTOR4_REF_INLINE_H_
