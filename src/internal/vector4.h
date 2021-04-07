#ifndef STEVESCH_MATHVEC_INTERNAL_VECTOR4_H_
#define STEVESCH_MATHVEC_INTERNAL_VECTOR4_H_
// Vector4 library header
//
// Copyright © 2002, PulseCode Interactive LLC, All Rights Reserved
//
// History:
//	Created:	4/30/2002, Stephen Schlueter
//	Modified:

#include "stevesch-MathBase.h"

namespace stevesch
{
  class stevesch::vector3;
  class stevesch::matrix4;
  class SRandGen;

  extern SRandGen S_RandGen;

  struct SVec4_t
  {
    float x;
    float y;
    float z;
    float w;
  };

#pragma pack(push, 16)
  // forward class declarations
  //SALIGN_DECL(16) class stevesch::matrix4;	// extern

  SALIGN_DECL(16)
  class stevesch::vector4
  {
  public:
#if SVEC_USESIMD
    union
    {
      __m128 m_align;

      struct
      {
        float x, y, z, w;
      };
    };
#else
    float x, y, z, w;
#endif

  public:
    // overload new, delete, new[] and delete[] to provide aligned allocation
    // SOVERLOAD_NEW_ALIGNED(stevesch::vector4, 16)

    stevesch::vector4(){};
    stevesch::vector4(float _x, float _y, float _z);
    stevesch::vector4(float _x, float _y, float _z, float _w);
    stevesch::vector4(const stevesch::vector4 &_v);

    bool operator==(const stevesch::vector4 &crOther) const;
    bool operator!=(const stevesch::vector4 &crOther) const;

    SFORCEINLINE operator const SVec4_t &() const { return *reinterpret_cast<const SVec4_t *>(this); }
    SFORCEINLINE operator SVec4_t &() { return *reinterpret_cast<SVec4_t *>(this); }

    SFORCEINLINE operator const SVec4_t *() const { return reinterpret_cast<const SVec4_t *>(this); }
    SFORCEINLINE operator SVec4_t *() { return reinterpret_cast<SVec4_t *>(this); }

    SFORCEINLINE operator const stevesch::vector3 &() const { return *reinterpret_cast<const stevesch::vector3 *>(this); }
    //operator stevesch::vector3& ()					{ return *reinterpret_cast<stevesch::vector3*>(this); }

    const stevesch::vector4 &copy(const stevesch::vector4 &v);  // copy (all memebers)
    const stevesch::vector4 &set(float _x, float _y, float _z); // w=1.0
    const stevesch::vector4 &set(float _x, float _y, float _z, float _w);
    const stevesch::vector4 &set(const stevesch::vector3 &v);

    ////////////////////////////////////////////////////

    const stevesch::vector4 &add(const stevesch::vector4 &v);   // member-wise addition (3-element)
    const stevesch::vector4 &sub(const stevesch::vector4 &v);   // member-wise subtraction (3-element)
    const stevesch::vector4 &mul(const stevesch::vector4 &v);   // member-wise multiplication (3-element)
    const stevesch::vector4 &cross(const stevesch::vector4 &v); // 3-element cross (outer) product

    const stevesch::vector4 &scale(float scale); // multiply by scalar (3-element)
    const stevesch::vector4 &mul(float scale);   // multiply by scalar (3-element) (same as scale(float))
    const stevesch::vector4 &div(float scale);   // divide by scalar (3-element)

    const stevesch::vector4 &negate();                             // 3-element negation (x=-x, y=-y, z=-z)
    const stevesch::vector4 &negate(stevesch::vector4 &dst) const; // 3-element negation (dst.x=-x, dst.y=-y, dst.z=-z)

    float dot(const stevesch::vector4 &v) const; // dot (inner) product (3-element)
    float squareMag() const;                     // squared magnitude (3-element)
    float recipSquareMag() const;                // 1.0 / squared magnitued (3-element)
    float abs() const;                           // magnitude (3-element)
    float recipAbs() const;                      // 1.0 / magnitude (3-element)

    const stevesch::vector4 &normalize(); // normalize self (3-element)

    void mad(const stevesch::vector4 &vm1, const stevesch::vector4 &vm2); // *this += vm1*vm2 (3-element)

    ////////////////////////////////////////////////////

    const stevesch::vector4 &add4(const stevesch::vector4 &v); // member-wise addition (4-element)
    const stevesch::vector4 &sub4(const stevesch::vector4 &v); // member-wise subtraction (4-element)
    const stevesch::vector4 &mul4(const stevesch::vector4 &v); // member-wise multiplication (4-element)
    const stevesch::vector4 &div4(const stevesch::vector4 &v); // member-wise division (4-element)

    const stevesch::vector4 &scale4(float scale); // multiply by scalar (4-element)
    const stevesch::vector4 &mul4(float scale);   // multiply by scalar (4-element) (same as scale4(float))
    const stevesch::vector4 &div4(float scale);   // divide by scalar (4-element)

    const stevesch::vector4 &negate4();                             // 4-element negation (x=-x, y=-y, z=-z, w=-w)
    const stevesch::vector4 &negate4(stevesch::vector4 &dst) const; // 4-element negation (dst.x=-x, dst.y=-y, dst.z=-z, dst.w=-w)

    float dot4(const stevesch::vector4 &v) const; // dot (inner) product (4-element)
    float squareMag4() const;                     // squared magnitude (4-element)
    float recipSquareMag4() const;                // 1.0 / squared magnitued (4-element)
    float abs4() const;                           // magnitude (4-element)
    float recipAbs4() const;                      // 1.0 / magnitude (4-element)

    const stevesch::vector4 &normalize4(); // normalize self (4-element)

    void mad4(const stevesch::vector4 &vm1, const stevesch::vector4 &vm2); // *this += vm1*vm2 (4-element)

    ////////////////////////////////////////////////////

    const stevesch::vector4 &operator=(const stevesch::vector4 &v);  // copy
    const stevesch::vector4 &operator+=(const stevesch::vector4 &v); // add (3-element)
    const stevesch::vector4 &operator-=(const stevesch::vector4 &v); // sub (3-element)
    const stevesch::vector4 &operator*=(float scale);                // scale (3-element)
    const stevesch::vector4 &operator/=(float scale);                // scale(1/scale) (3-element)
    const float &operator[](int n) const;                            // 0->x, 1->y, 2->z, 3->w
    float &operator[](int n);                                        // 0->x, 1->y, 2->z, 3->w

    ////////////////////////////////////////////////////

    const stevesch::vector4 &transform(const stevesch::matrix4 &mLeft);              // v = M*v -- 4x4 matrix * vector
    const stevesch::vector4 &transformSub(const stevesch::matrix4 &mLeft);           // v = M*v --  3x3 matrix * vector (as 4x4*v as if v.w=0)
    const stevesch::vector4 &transformSubTransposed(const stevesch::matrix4 &mLeft); // v = (M^T)*v -- transposed matrix * vector (as 4x4*v as if v.w=0)
    const stevesch::vector4 &transformAff(const stevesch::matrix4 &mLeft);           // v = M*v -- 4x4 matrix * vector (as if w=1)

    //const stevesch::vector4& operator *=(const stevesch::matrix4& mRight);	// Mul (full 4x4)

    void rand(SRandGen &r = S_RandGen);                                                            // 3-element randomize (0.0f, 1.0f)
    void randAB(const stevesch::vector4 &a, const stevesch::vector4 &b, SRandGen &r = S_RandGen);  // 3-element randomize (a.*, b.*)
    void rand4(SRandGen &r = S_RandGen);                                                           // 4-element randomize (0.0f, 1.0f)
    void randAB4(const stevesch::vector4 &a, const stevesch::vector4 &b, SRandGen &r = S_RandGen); // 4-element randomize (a.*, b.*)

    void randSpherical(SRandGen &r = S_RandGen); // produce random spherical distribution on unit sphere (xyz, w=1.0f)
    ////////////////////////////////////////////////////

    // 3-element static methods
    static void add(stevesch::vector4 &dst, const stevesch::vector4 &v1, const stevesch::vector4 &v2); // dst = v1 + v2 (3-element)
    static void sub(stevesch::vector4 &dst, const stevesch::vector4 &v1, const stevesch::vector4 &v2); // dst = v1 - v2 (3-element)
    static void mul(stevesch::vector4 &dst, const stevesch::vector4 &v1, const stevesch::vector4 &v2); // dst = v1 * v2 (3-element)
    static void div(stevesch::vector4 &dst, const stevesch::vector4 &v1, float s);                     // dst = v1 / s
    static void scale(stevesch::vector4 &dst, const stevesch::vector4 &v1, float s);                   // dst = v1 * s
    SFORCEINLINE static void mul(stevesch::vector4 &dst, const stevesch::vector4 &v1, float s) { stevesch::vector4::scale(dst, v1, s); }
    static float dot(const stevesch::vector4 &v1, const stevesch::vector4 &v2); // 3-element dot (inner) product of v1 and v2

    static void normalize(stevesch::vector4 &dst, const stevesch::vector4 &src); // (3-element)

    static void mad(stevesch::vector4 &dst, const stevesch::vector4 &vm1, const stevesch::vector4 &vm2, const stevesch::vector4 &va); // dst = vm1*vm2 + va

    static void transform(stevesch::vector4 &dst, const stevesch::matrix4 &m, const stevesch::vector4 &v);              // dst = m * v (4x4)
    static void transformSub(stevesch::vector4 &dst, const stevesch::matrix4 &m, const stevesch::vector4 &v);           // dst = m * v (4x4, as if v.w = 0.0)
    static void transformSubTransposed(stevesch::vector4 &dst, const stevesch::matrix4 &m, const stevesch::vector4 &v); // dst = (m^T) * v, 3x3
    static void transformAff(stevesch::vector4 &dst, const stevesch::matrix4 &m, const stevesch::vector4 &v);           // dst = m * v (4x4, as if v.w = 1.0)

    SFORCEINLINE static float squareDist(const stevesch::vector4 &v1, const stevesch::vector4 &v2) { return distanceSquared(v1, v2); }
    SFORCEINLINE static float squareDist(stevesch::vector4 &dst, const stevesch::vector4 &v1, const stevesch::vector4 &v2) { return distanceSquared(dst, v1, v2); }

    static void min(stevesch::vector4 &dst, const stevesch::vector4 &v1, const stevesch::vector4 &v2);             // dst = min(v1, v2) per element (3-element)
    static void max(stevesch::vector4 &dst, const stevesch::vector4 &v1, const stevesch::vector4 &v2);             // dst = max(v1, v2) per element (3-element)
    static void clampMag(stevesch::vector4 &vDst, const stevesch::vector4 &vSrc, const stevesch::vector4 &vClamp); // clamp vSrc to +/- vClamp

    static void lerp(stevesch::vector4 &dst, const stevesch::vector4 &v1, const stevesch::vector4 &v2, float t);    // linear interpolation t=[0, 1] -> dst = [v1, v2] (3-element)
    static void addLerp(stevesch::vector4 &dst, const stevesch::vector4 &v1, const stevesch::vector4 &v2, float t); // add linear interpolation t=[0, 1] -> dst += [v1, v2] (3-element)

    // v = v + (a*fScalea)*dt; r = r + v*dt
    // this can be pure "Backward Euler" if a is a(t0+dt) or "semi-implicit euler" if a = a(t0)
    static void applyEulerImplicit(stevesch::vector4 &r, stevesch::vector4 &v, const stevesch::vector4 &a, float fScalea, float dt);

    static void cross(stevesch::vector4 &dst, const stevesch::vector4 &v1, const stevesch::vector4 &v2);                         // dst = v1 x v2
    static void addScaled(stevesch::vector4 &dst, const stevesch::vector4 &v1, const stevesch::vector4 &v2, float s2);           // dst = v1 + v2*s2
    static void addScaled(stevesch::vector4 &dst, const stevesch::vector4 &v1, float s1, const stevesch::vector4 &v2, float s2); // dst = v1*s1 + v2*s2

    static float distanceSquared(const stevesch::vector4 &v1, const stevesch::vector4 &v2); // squared distance between two points: (v2-v1).(v2-v1)

    // returns squared distance between two points: |v2-v1|^2 and sets dst=(v2-v1)
    static float distanceSquared(stevesch::vector4 &dst, const stevesch::vector4 &v1, const stevesch::vector4 &v2);

    // 4-element static methods
    static void add4(stevesch::vector4 &dst, const stevesch::vector4 &v1, const stevesch::vector4 &v2); // dst = v1 + v2 (4-element)
    static void sub4(stevesch::vector4 &dst, const stevesch::vector4 &v1, const stevesch::vector4 &v2); // dst = v1 - v2 (4-element)
    static void mul4(stevesch::vector4 &dst, const stevesch::vector4 &v1, const stevesch::vector4 &v2); // dst = v1 * v2 (4-element)
    static void div4(stevesch::vector4 &dst, const stevesch::vector4 &v1, const stevesch::vector4 &v2); // dst = v1 / v2 (4-element)
    static void mul4(stevesch::vector4 &dst, const stevesch::vector4 &v1, float fScale);                // dst = v1 * fScale (4-element)

    static void recip(stevesch::vector4 &dst, const stevesch::vector4 &v);     // dst = sqrt(v) (3-element)
    static void sqrt(stevesch::vector4 &dst, const stevesch::vector4 &v);      // dst = sqrt(v) (3-element)
    static void recipSqrt(stevesch::vector4 &dst, const stevesch::vector4 &v); // dst = 1.0/sqrt(v) (3-element)

    static void recip4(stevesch::vector4 &dst, const stevesch::vector4 &v);     // dst = sqrt(v) (4-element)
    static void sqrt4(stevesch::vector4 &dst, const stevesch::vector4 &v);      // dst = sqrt(v) (4-element)
    static void recipSqrt4(stevesch::vector4 &dst, const stevesch::vector4 &v); // dst = 1.0/sqrt(v) (4-element)

    static void normalize4(stevesch::vector4 &dst, const stevesch::vector4 &src) { scale4(dst, src, RSqrtf(src.squareMag4())); } // (4-element)

    static void mad4(stevesch::vector4 &dst, const stevesch::vector4 &vm1, const stevesch::vector4 &vm2, const stevesch::vector4 &va); // dst = vm1*vm2 + va

    // given a triangle, compute edge vectors
    // e0 = p2 - p1; e1 = p0 - p2, e2 = p1 - p0
    static void triangleEdges(stevesch::vector4 *pEdge, const stevesch::vector4 *pTriangleVertex);

    // given a point vPoint, compute vectors from each triangle vertex to that point
    // ri = vPoint - pi
    static void triangleVectors(stevesch::vector4 *pVector, const stevesch::vector4 *pTriangleVertex, const stevesch::vector4 &vPoint);

    // the above functions (triangleEdges and triangleVectors), combined
    static void triangleEdgesAndVectors(stevesch::vector4 *pEdge, stevesch::vector4 *pVector, const stevesch::vector4 *pTriangleVertex, const stevesch::vector4 &vPoint);

    // returns true if point lies within 3-D triangle (within volume perpendicular to triangle)
    bool inTriangle(const stevesch::vector4 *pTri, const stevesch::vector4 &vNormal) const; // vNormal is precomputed triangle normal
    bool inTriangle(const stevesch::vector4 *pTri) const;

    // point-in-triangle test, given triangle edges and vectors as computed in triangleEdges() and
    // triangleVectors().  Specified normal 'vNormal' does not need to be normalized.
    // (vNormal should be direction as defined be pEdges[2] x pEdges[0])
    bool inTriangleEx(const stevesch::vector4 *pEdges, const stevesch::vector4 *pVectors, const stevesch::vector4 &vNormal) const;

    // multiple square distance [vSquareDisti = (di . di)], di = (c0 - vi)
    static void squareDistQuad(stevesch::vector4 &vSquareDist,
                               stevesch::vector4 &d0, stevesch::vector4 &d1, stevesch::vector4 &d2, stevesch::vector4 &d3,
                               const stevesch::vector4 &c0,
                               const stevesch::vector4 &v0, const stevesch::vector4 &v1, const stevesch::vector4 &v2, const stevesch::vector4 &v3);

    // multiple square distance [vSquareDisti = (di . di)], di = (c0 - vi)
    static void squareDistQuad(stevesch::vector4 &vSquareDist,
                               const stevesch::vector4 &c0,
                               const stevesch::vector4 &v0, const stevesch::vector4 &v1, const stevesch::vector4 &v2, const stevesch::vector4 &v3);

    static void sumBOverAQuad(stevesch::vector4 &dst,
                              const stevesch::vector4 &a,
                              const stevesch::vector4 &b0,
                              const stevesch::vector4 &b1,
                              const stevesch::vector4 &b2,
                              const stevesch::vector4 &b3);

    // dst += [(c0 - bi)/|c0 - bi|] * [vScalei / (fZeroOffset + |c0 - bi|^2]
    static void sumInverseFalloff(stevesch::vector4 &dst,
                                  const stevesch::vector4 &c0,
                                  const stevesch::vector4 &b0,
                                  const stevesch::vector4 &b1,
                                  const stevesch::vector4 &b2,
                                  const stevesch::vector4 &b3,
                                  const stevesch::vector4 &vScale,
                                  float fRadius2Factor,
                                  float fZeroOffset);

    static void applyEulerImplicitQuad(
        stevesch::vector4 &r0, stevesch::vector4 &r1, stevesch::vector4 &r2, stevesch::vector4 &r3,
        stevesch::vector4 &v0, stevesch::vector4 &v1, stevesch::vector4 &v2, stevesch::vector4 &v3,
        const stevesch::vector4 &a0, const stevesch::vector4 &a1, const stevesch::vector4 &a2, const stevesch::vector4 &a3,
        const stevesch::vector4 &vAScale0, const stevesch::vector4 &vAScale1, float dt);

    static void div4(stevesch::vector4 &dst, const stevesch::vector4 &v1, float s);   // dst = v1 / s (4-element)
    static void scale4(stevesch::vector4 &dst, const stevesch::vector4 &v1, float s); // dst = v1 * s (4-element)
    static float dot4(const stevesch::vector4 &v1, const stevesch::vector4 &v2);      // 4-element dot (inner) product of v1 and v2

    static void addScaled4(stevesch::vector4 &dst, const stevesch::vector4 &v1, const stevesch::vector4 &v2, float s2);           // dst = v1 + s*v2
    static void addScaled4(stevesch::vector4 &dst, const stevesch::vector4 &v1, float s1, const stevesch::vector4 &v2, float s2); // dst = v1*s1 + v2*s2

    static void min4(stevesch::vector4 &dst, const stevesch::vector4 &v1, const stevesch::vector4 &v2);             // dst = min(v1, v2) per element (4-element)
    static void max4(stevesch::vector4 &dst, const stevesch::vector4 &v1, const stevesch::vector4 &v2);             // dst = max(v1, v2) per element (4-element)
    static void clampMag4(stevesch::vector4 &vDst, const stevesch::vector4 &vSrc, const stevesch::vector4 &vClamp); // clamp vSrc to +/- vClamp

    static void lerp4(stevesch::vector4 &dst, const stevesch::vector4 &v1, const stevesch::vector4 &v2, float t); // linear interpolation t=[0, 1] -> dst=[v1, v2] (4-element)

    // distance from triangle, returning triangle basis parameters in pfSParam and pfTParam
    // where basis S is (pTriangle[1] - pTriangle[0]) and T is (pTriangle[2] - pTriangle[0])
    float distanceSquaredST(const stevesch::vector4 *pTriangle, float *pfSParam, float *pfTParam) const;

    // distance from triangle
    // (*pnNearestEdge is filled with edge index if nearest point is on an edge, otherwise -1)
    float distanceSquared(stevesch::vector4 &vNearestPoint, const stevesch::vector4 *pTriangle, int *pnNearestEdge = 0) const;

    // compute linear velocity given linear positions and a time step
    // fDeltaSeconds must be non-zero
    static void instantaneousVelocity(stevesch::vector4 &vVLinear, const stevesch::vector4 &v0, const stevesch::vector4 &v1, float fDeltaSeconds);

  } SALIGN(16);

  extern const stevesch::vector4 c_vUP;
  extern const stevesch::vector4 c_vDOWN;
  extern const stevesch::vector4 c_vLEFT;
  extern const stevesch::vector4 c_vRIGHT;
  extern const stevesch::vector4 c_vFORWARD;
  extern const stevesch::vector4 c_vBACKWARD;
  extern const stevesch::vector4 c_vZERO; // <0 0 0 1>

  ////////////////////////////////////////////////////////////
  SFORCEINLINE void SASSERT_NORMALIZED(const stevesch::vector4 &v, float fSquareEpsilon = 0.02f)
  {
    SASSERT(fabsf(v.squareMag() - 1.0f) < fSquareEpsilon);
  }

  SFORCEINLINE void SASSERT_NONZEROLENGTH(const stevesch::vector4 &v, float fSquareEpsilon = 0.0000005f)
  {
    SASSERT(v.squareMag() > fSquareEpsilon);
  }

  ////////////////////////////////////////////////////////////

#pragma pack(pop)
} // namespace SMath

#include "vector4-inline.h"

#endif
