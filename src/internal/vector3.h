#ifndef STEVESCH_MATHVEC_INTERNAL_VECTOR3_H_
#define STEVESCH_MATHVEC_INTERNAL_VECTOR3_H_
// Copyright © 2002, Stephen Schlueter, All Rights Reserved

#include <stevesch-MathBase.h>
#include "mathVec.h"

#define SVECINLINE inline

namespace stevesch
{
  class matrix4;
  class RandGen;
  ///////////////////

  struct SVec3_t
  {
    float x;
    float y;
    float z;
  };

  class vector3
  {
  public:
    float x, y, z;

  public:
    SVECINLINE vector3() {}
    SVECINLINE vector3(float _x, float _y, float _z);
    SVECINLINE vector3(const vector3 &_v);

    SVECINLINE operator const SVec3_t &() const { return *reinterpret_cast<const SVec3_t *>(this); }
    SVECINLINE operator SVec3_t &() { return *reinterpret_cast<SVec3_t *>(this); }

    SVECINLINE operator const SVec3_t *() const { return reinterpret_cast<const SVec3_t *>(this); }
    SVECINLINE operator SVec3_t *() { return reinterpret_cast<SVec3_t *>(this); }

    SVECINLINE vector3 &copy(const vector3 &v);            // copy (all memebers)
    SVECINLINE vector3 &set(float _x, float _y, float _z); // w=1.0

    SVECINLINE vector3 &add(const vector3 &v);   // member-wise addition
    SVECINLINE vector3 &sub(const vector3 &v);   // member-wise subtraction
    SVECINLINE vector3 &mul(const vector3 &v);   // member-wise multiplication
    SVECINLINE vector3 &cross(const vector3 &v); // 3-element cross (outer) product

    SVECINLINE vector3 &negate();                   // 3-element negation (x=-x, y=-y, z=-z)
    SVECINLINE vector3 &negate(vector3 &dst) const; // 3-element negation (dst.x=-x, dst.y=-y, dst.z=-z)

    SVECINLINE vector3 &scale(float scale); // multiply by scalar
    SVECINLINE vector3 &mul(float scale);   // multiply by scalar (same as scale(float))
    SVECINLINE vector3 &div(float scale);   // divide by scalar

    SVECINLINE float dot(const vector3 &v) const; // dot (inner) product
    SVECINLINE float squareMag() const;           // squared magnitude
    SVECINLINE float abs() const;                 // magnitude

    SVECINLINE vector3 &normalize() { return mul(rsqrtf(squareMag())); }

    SVECINLINE vector3 &operator=(const vector3 &v);  // copy
    SVECINLINE vector3 &operator+=(const vector3 &v); // add
    SVECINLINE vector3 &operator-=(const vector3 &v); // sub
    SVECINLINE vector3 &operator*=(float scale);      // scale
    SVECINLINE vector3 &operator/=(float scale);      // scale(1/scale)
    SVECINLINE const float &operator[](int n) const;        // 0->x, 1->y, 2->z
    SVECINLINE float &operator[](int n);                    // 0->x, 1->y, 2->z

    ////////////////////////////////////////////////////

    vector3 &transform(const stevesch::matrix4 &mLeft);    // v = M*v -- * 4x4 matrix (w assumed=1.0f)
    vector3 &transformSub(const stevesch::matrix4 &mLeft); // v = M*v -- * 3x3 matrix (as if w=0)

    SVECINLINE void rand();                                     // 3-element randomize (0.0f, 1.0f)
    SVECINLINE void randAB(const vector3 &a, const vector3 &b); // 3-element randomize (a.*, b.*)

    void randSpherical();                     // produce random spherical distribution on unit sphere
    void randSpherical(stevesch::RandGen &r); // produce random spherical distribution on unit sphere

    ////////////////////////////////////////////////////

    // 3-element static methods
    static SVECINLINE void add(vector3 &dst, const vector3 &v1, const vector3 &v2); // dst = v1 + v2
    static SVECINLINE void sub(vector3 &dst, const vector3 &v1, const vector3 &v2); // dst = v1 - v2
    static SVECINLINE void mul(vector3 &dst, const vector3 &v1, const vector3 &v2); // dst = v1 * v2
    static SVECINLINE void div(vector3 &dst, const vector3 &v1, float s);           // dst = v1 / s
    static SVECINLINE void scale(vector3 &dst, const vector3 &v1, float s);         // dst = v1 * s
    static SVECINLINE float dot(const vector3 &v1, const vector3 &v2);              // dot (inner) product of v1 and v2

    static SVECINLINE void transform(vector3 &dst, const stevesch::matrix4 &m, const vector3 &v);    // dst = m * v (4x4)
    static SVECINLINE void transformSub(vector3 &dst, const stevesch::matrix4 &m, const vector3 &v); // dst = m * v (4x4, as if v.w = 0.0)
    static SVECINLINE void transformAff(vector3 &dst, const stevesch::matrix4 &m, const vector3 &v); // dst = m * v (4x4, as if v.w = 1.0)

    static SVECINLINE float squareDist(const vector3 &v1, const vector3 &v2);

    static SVECINLINE void min(vector3 &dst, const vector3 &v1, const vector3 &v2); // dst = min(v1, v2) per element
    static SVECINLINE void max(vector3 &dst, const vector3 &v1, const vector3 &v2); // dst = max(v1, v2) per element

    static SVECINLINE void lerp(vector3 &dst, const vector3 &v1, const vector3 &v2, float t); // linear interpolation t=[0, 1] -> dst=[v1, v2]

    static SVECINLINE void cross(vector3 &dst, const vector3 &v1, const vector3 &v2);                         // dst = v1 x v2
    static SVECINLINE void addScaled(vector3 &dst, const vector3 &v1, const vector3 &v2, float s2);           // dst = v1 + v2*s2
    static SVECINLINE void addScaled(vector3 &dst, const vector3 &v1, float s1, const vector3 &v2, float s2); // dst = v1*s1 + v2*s2

    friend SVECINLINE vector3 operator+(const vector3 &v1, const vector3 &v2); // v1 + v2
    friend SVECINLINE vector3 operator-(const vector3 &v1, const vector3 &v2); // v1 - v2
    friend SVECINLINE vector3 operator*(const vector3 &v1, float s); // v1 * s
    friend SVECINLINE vector3 operator*(float s, const vector3 &v1); // s * v
    friend SVECINLINE vector3 operator/(const vector3 &v1, float d); // v1 / d

    friend SVECINLINE vector3 operator*(const stevesch::matrix4 &M, const vector3 &v); // M*v -- multiply by 4x4 matrix (v.w assumed=1.0f)
  };

} // namespace stevesch

#include "vector3-inline.h"

#endif
