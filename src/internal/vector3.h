#ifndef STEVESCH_MATHVEC_INTERNAL_VECTOR3_H_
#define STEVESCH_MATHVEC_INTERNAL_VECTOR3_H_
// Copyright © 2002, PulseCode Interactive LLC, All Rights Reserved

#include "stevesch-MathBase.h"

namespace stevesch
{
  class stevesch::matrix4;
  class SRandGen;
  ///////////////////

  struct SVec3_t
  {
    float x;
    float y;
    float z;
  };

  class stevesch::vector3
  {
  public:
    float x, y, z;

  public:
    SVECINLINE stevesch::vector3() {}
    SVECINLINE stevesch::vector3(float _x, float _y, float _z);
    SVECINLINE stevesch::vector3(const stevesch::vector3 &_v);

    SVECINLINE operator const SVec3_t &() const { return *reinterpret_cast<const SVec3_t *>(this); }
    SVECINLINE operator SVec3_t &() { return *reinterpret_cast<SVec3_t *>(this); }

    SVECINLINE operator const SVec3_t *() const { return reinterpret_cast<const SVec3_t *>(this); }
    SVECINLINE operator SVec3_t *() { return reinterpret_cast<SVec3_t *>(this); }

    SVECINLINE const stevesch::vector3 &copy(const stevesch::vector3 &v);  // copy (all memebers)
    SVECINLINE const stevesch::vector3 &set(float _x, float _y, float _z); // w=1.0

    SVECINLINE const stevesch::vector3 &add(const stevesch::vector3 &v);   // member-wise addition
    SVECINLINE const stevesch::vector3 &sub(const stevesch::vector3 &v);   // member-wise subtraction
    SVECINLINE const stevesch::vector3 &mul(const stevesch::vector3 &v);   // member-wise multiplication
    SVECINLINE const stevesch::vector3 &cross(const stevesch::vector3 &v); // 3-element cross (outer) product

    SVECINLINE const stevesch::vector3 &negate();                             // 3-element negation (x=-x, y=-y, z=-z)
    SVECINLINE const stevesch::vector3 &negate(stevesch::vector3 &dst) const; // 3-element negation (dst.x=-x, dst.y=-y, dst.z=-z)

    SVECINLINE const stevesch::vector3 &scale(float scale); // multiply by scalar
    SVECINLINE const stevesch::vector3 &mul(float scale);   // multiply by scalar (same as scale(float))
    SVECINLINE const stevesch::vector3 &div(float scale);   // divide by scalar

    SVECINLINE float dot(const stevesch::vector3 &v) const; // dot (inner) product
    SVECINLINE float squareMag() const;                     // squared magnitude
    SVECINLINE float abs() const;                           // magnitude

    SVECINLINE const stevesch::vector3 &normalize() { return mul(RSqrtf(squareMag())); }

    SVECINLINE const stevesch::vector3 &operator=(const stevesch::vector3 &v);  // copy
    SVECINLINE const stevesch::vector3 &operator+=(const stevesch::vector3 &v); // add
    SVECINLINE const stevesch::vector3 &operator-=(const stevesch::vector3 &v); // sub
    SVECINLINE const stevesch::vector3 &operator*=(float scale);                // scale
    SVECINLINE const stevesch::vector3 &operator/=(float scale);                // scale(1/scale)
    SVECINLINE const float &operator[](int n) const;                            // 0->x, 1->y, 2->z
    SVECINLINE float &operator[](int n);                                        // 0->x, 1->y, 2->z

    ////////////////////////////////////////////////////

    const stevesch::vector3 &transform(const stevesch::matrix4 &mLeft);    // v = M*v -- * 4x4 matrix (w assumed=1.0f)
    const stevesch::vector3 &transformSub(const stevesch::matrix4 &mLeft); // v = M*v -- * 3x3 matrix (as if w=0)

    //SVECINLINE const stevesch::vector3& operator *=(const stevesch::matrix4& mLeft);	// transform (full 4x4)

    SVECINLINE void rand();                                                         // 3-element randomize (0.0f, 1.0f)
    SVECINLINE void randAB(const stevesch::vector3 &a, const stevesch::vector3 &b); // 3-element randomize (a.*, b.*)

    void randSpherical();            // produce random spherical distribution on unit sphere
    void randSpherical(SRandGen &r); // produce random spherical distribution on unit sphere

    ////////////////////////////////////////////////////

    // 3-element static methods
    static SVECINLINE void add(stevesch::vector3 &dst, const stevesch::vector3 &v1, const stevesch::vector3 &v2); // dst = v1 + v2
    static SVECINLINE void sub(stevesch::vector3 &dst, const stevesch::vector3 &v1, const stevesch::vector3 &v2); // dst = v1 - v2
    static SVECINLINE void mul(stevesch::vector3 &dst, const stevesch::vector3 &v1, const stevesch::vector3 &v2); // dst = v1 * v2
    static SVECINLINE void div(stevesch::vector3 &dst, const stevesch::vector3 &v1, float s);                     // dst = v1 / s
    static SVECINLINE void scale(stevesch::vector3 &dst, const stevesch::vector3 &v1, float s);                   // dst = v1 * s
    static SVECINLINE float dot(const stevesch::vector3 &v1, const stevesch::vector3 &v2);                        // dot (inner) product of v1 and v2

    static SVECINLINE void transform(stevesch::vector3 &dst, const stevesch::matrix4 &m, const stevesch::vector3 &v);    // dst = m * v (4x4)
    static SVECINLINE void transformSub(stevesch::vector3 &dst, const stevesch::matrix4 &m, const stevesch::vector3 &v); // dst = m * v (4x4, as if v.w = 0.0)
    static SVECINLINE void transformAff(stevesch::vector3 &dst, const stevesch::matrix4 &m, const stevesch::vector3 &v); // dst = m * v (4x4, as if v.w = 1.0)

    static SVECINLINE float squareDist(const stevesch::vector3 &v1, const stevesch::vector3 &v2);

    static SVECINLINE void min(stevesch::vector3 &dst, const stevesch::vector3 &v1, const stevesch::vector3 &v2); // dst = min(v1, v2) per element
    static SVECINLINE void max(stevesch::vector3 &dst, const stevesch::vector3 &v1, const stevesch::vector3 &v2); // dst = max(v1, v2) per element

    static SVECINLINE void lerp(stevesch::vector3 &dst, const stevesch::vector3 &v1, const stevesch::vector3 &v2, float t); // linear interpolation t=[0, 1] -> dst=[v1, v2]

    static SVECINLINE void cross(stevesch::vector3 &dst, const stevesch::vector3 &v1, const stevesch::vector3 &v2);                         // dst = v1 x v2
    static SVECINLINE void addScaled(stevesch::vector3 &dst, const stevesch::vector3 &v1, const stevesch::vector3 &v2, float s2);           // dst = v1 + v2*s2
    static SVECINLINE void addScaled(stevesch::vector3 &dst, const stevesch::vector3 &v1, float s1, const stevesch::vector3 &v2, float s2); // dst = v1*s1 + v2*s2
  };

} // namespace stevesch

#include "vector3-inline.h"

#endif
