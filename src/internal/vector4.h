#ifndef STEVESCH_MATHVEC_INTERNAL_VECTOR4_H_
#define STEVESCH_MATHVEC_INTERNAL_VECTOR4_H_
// Vector4 library header
//
// Copyright © 2002, Stephen Schlueter, All Rights Reserved
//
// History:
//	Created:	4/30/2002, Stephen Schlueter
//	Modified:

#include "mathVec.h"
#include "stevesch-MathBase.h"

namespace stevesch
{
  class vector3;
  class matrix4;
  class RandGen;

  extern RandGen S_RandGen;

  struct SVec4_t
  {
    float x;
    float y;
    float z;
    float w;
  };

#pragma pack(push, 16)
  // forward class declarations
  //SALIGN_DECL(16) class matrix4;	// extern

  // SALIGN_DECL(16)
  class vector4
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
    // SOVERLOAD_NEW_ALIGNED(vector4, 16)

    vector4(){};
    vector4(float _x, float _y, float _z);
    vector4(float _x, float _y, float _z, float _w);
    vector4(const vector4 &_v);

    bool operator==(const vector4 &crOther) const;
    bool operator!=(const vector4 &crOther) const;

    inline operator const SVec4_t &() const { return *reinterpret_cast<const SVec4_t *>(this); }
    inline operator SVec4_t &() { return *reinterpret_cast<SVec4_t *>(this); }

    inline operator const SVec4_t *() const { return reinterpret_cast<const SVec4_t *>(this); }
    inline operator SVec4_t *() { return reinterpret_cast<SVec4_t *>(this); }

    inline operator const stevesch::vector3 &() const { return *reinterpret_cast<const stevesch::vector3 *>(this); }
    //operator stevesch::vector3& ()					{ return *reinterpret_cast<stevesch::vector3*>(this); }

    const vector4 &copy(const vector4 &v);            // copy (all memebers)
    const vector4 &set(float _x, float _y, float _z); // w=1.0
    const vector4 &set(float _x, float _y, float _z, float _w);
    const vector4 &set(const stevesch::vector3 &v);

    ////////////////////////////////////////////////////

    const vector4 &add(const vector4 &v);   // member-wise addition (3-element)
    const vector4 &sub(const vector4 &v);   // member-wise subtraction (3-element)
    const vector4 &mul(const vector4 &v);   // member-wise multiplication (3-element)
    const vector4 &cross(const vector4 &v); // 3-element cross (outer) product

    const vector4 &scale(float scale); // multiply by scalar (3-element)
    const vector4 &mul(float scale);   // multiply by scalar (3-element) (same as scale(float))
    const vector4 &div(float scale);   // divide by scalar (3-element)

    const vector4 &negate();                   // 3-element negation (x=-x, y=-y, z=-z)
    const vector4 &negate(vector4 &dst) const; // 3-element negation (dst.x=-x, dst.y=-y, dst.z=-z)

    float dot(const vector4 &v) const; // dot (inner) product (3-element)
    float squareMag() const;           // squared magnitude (3-element)
    float recipSquareMag() const;      // 1.0 / squared magnitued (3-element)
    float abs() const;                 // magnitude (3-element)
    float recipAbs() const;            // 1.0 / magnitude (3-element)

    const vector4 &normalize(); // normalize self (3-element)

    void mad(const vector4 &vm1, const vector4 &vm2); // *this += vm1*vm2 (3-element)

    ////////////////////////////////////////////////////

    const vector4 &add4(const vector4 &v); // member-wise addition (4-element)
    const vector4 &sub4(const vector4 &v); // member-wise subtraction (4-element)
    const vector4 &mul4(const vector4 &v); // member-wise multiplication (4-element)
    const vector4 &div4(const vector4 &v); // member-wise division (4-element)

    const vector4 &scale4(float scale); // multiply by scalar (4-element)
    const vector4 &mul4(float scale);   // multiply by scalar (4-element) (same as scale4(float))
    const vector4 &div4(float scale);   // divide by scalar (4-element)

    const vector4 &negate4();                   // 4-element negation (x=-x, y=-y, z=-z, w=-w)
    const vector4 &negate4(vector4 &dst) const; // 4-element negation (dst.x=-x, dst.y=-y, dst.z=-z, dst.w=-w)

    float dot4(const vector4 &v) const; // dot (inner) product (4-element)
    float squareMag4() const;           // squared magnitude (4-element)
    float recipSquareMag4() const;      // 1.0 / squared magnitued (4-element)
    float abs4() const;                 // magnitude (4-element)
    float recipAbs4() const;            // 1.0 / magnitude (4-element)

    const vector4 &normalize4(); // normalize self (4-element)

    void mad4(const vector4 &vm1, const vector4 &vm2); // *this += vm1*vm2 (4-element)

    ////////////////////////////////////////////////////

    const vector4 &operator=(const vector4 &v);  // copy
    const vector4 &operator+=(const vector4 &v); // add (3-element)
    const vector4 &operator-=(const vector4 &v); // sub (3-element)
    const vector4 &operator*=(float scale);      // scale (3-element)
    const vector4 &operator/=(float scale);      // scale(1/scale) (3-element)
    const float &operator[](int n) const;        // 0->x, 1->y, 2->z, 3->w
    float &operator[](int n);                    // 0->x, 1->y, 2->z, 3->w

    ////////////////////////////////////////////////////

    const vector4 &transform(const stevesch::matrix4 &mLeft);              // v = M*v -- 4x4 matrix * vector
    const vector4 &transformSub(const stevesch::matrix4 &mLeft);           // v = M*v --  3x3 matrix * vector (as 4x4*v as if v.w=0)
    const vector4 &transformSubTransposed(const stevesch::matrix4 &mLeft); // v = (M^T)*v -- transposed matrix * vector (as 4x4*v as if v.w=0)
    const vector4 &transformAff(const stevesch::matrix4 &mLeft);           // v = M*v -- 4x4 matrix * vector (as if w=1)

    //const vector4& operator *=(const stevesch::matrix4& mRight);	// Mul (full 4x4)

    void rand(stevesch::RandGen &r = S_RandGen);                                        // 3-element randomize (0.0f, 1.0f)
    void randAB(const vector4 &a, const vector4 &b, stevesch::RandGen &r = S_RandGen);  // 3-element randomize (a.*, b.*)
    void rand4(stevesch::RandGen &r = S_RandGen);                                       // 4-element randomize (0.0f, 1.0f)
    void randAB4(const vector4 &a, const vector4 &b, stevesch::RandGen &r = S_RandGen); // 4-element randomize (a.*, b.*)

    void randSpherical(stevesch::RandGen &r = S_RandGen); // produce random spherical distribution on unit sphere (xyz, w=1.0f)
    ////////////////////////////////////////////////////

    // 3-element static methods
    static void add(vector4 &dst, const vector4 &v1, const vector4 &v2); // dst = v1 + v2 (3-element)
    static void sub(vector4 &dst, const vector4 &v1, const vector4 &v2); // dst = v1 - v2 (3-element)
    static void mul(vector4 &dst, const vector4 &v1, const vector4 &v2); // dst = v1 * v2 (3-element)
    static void div(vector4 &dst, const vector4 &v1, float s);           // dst = v1 / s
    static void scale(vector4 &dst, const vector4 &v1, float s);         // dst = v1 * s
    inline static void mul(vector4 &dst, const vector4 &v1, float s) { vector4::scale(dst, v1, s); }
    static float dot(const vector4 &v1, const vector4 &v2); // 3-element dot (inner) product of v1 and v2

    static void normalize(vector4 &dst, const vector4 &src); // (3-element)

    static void mad(vector4 &dst, const vector4 &vm1, const vector4 &vm2, const vector4 &va); // dst = vm1*vm2 + va

    static void transform(vector4 &dst, const stevesch::matrix4 &m, const vector4 &v);              // dst = m * v (4x4)
    static void transformSub(vector4 &dst, const stevesch::matrix4 &m, const vector4 &v);           // dst = m * v (4x4, as if v.w = 0.0)
    static void transformSubTransposed(vector4 &dst, const stevesch::matrix4 &m, const vector4 &v); // dst = (m^T) * v, 3x3
    static void transformAff(vector4 &dst, const stevesch::matrix4 &m, const vector4 &v);           // dst = m * v (4x4, as if v.w = 1.0)

    inline static float squareDist(const vector4 &v1, const vector4 &v2) { return distanceSquared(v1, v2); }
    inline static float squareDist(vector4 &dst, const vector4 &v1, const vector4 &v2) { return distanceSquared(dst, v1, v2); }

    static void min(vector4 &dst, const vector4 &v1, const vector4 &v2);             // dst = min(v1, v2) per element (3-element)
    static void max(vector4 &dst, const vector4 &v1, const vector4 &v2);             // dst = max(v1, v2) per element (3-element)
    static void clampMag(vector4 &vDst, const vector4 &vSrc, const vector4 &vClamp); // clamp vSrc to +/- vClamp

    static void lerp(vector4 &dst, const vector4 &v1, const vector4 &v2, float t);    // linear interpolation t=[0, 1] -> dst = [v1, v2] (3-element)
    static void addLerp(vector4 &dst, const vector4 &v1, const vector4 &v2, float t); // add linear interpolation t=[0, 1] -> dst += [v1, v2] (3-element)

    // v = v + (a*fScalea)*dt; r = r + v*dt
    // this can be pure "Backward Euler" if a is a(t0+dt) or "semi-implicit euler" if a = a(t0)
    static void applyEulerImplicit(vector4 &r, vector4 &v, const vector4 &a, float fScalea, float dt);

    static void cross(vector4 &dst, const vector4 &v1, const vector4 &v2);                         // dst = v1 x v2
    static void addScaled(vector4 &dst, const vector4 &v1, const vector4 &v2, float s2);           // dst = v1 + v2*s2
    static void addScaled(vector4 &dst, const vector4 &v1, float s1, const vector4 &v2, float s2); // dst = v1*s1 + v2*s2

    static float distanceSquared(const vector4 &v1, const vector4 &v2); // squared distance between two points: (v2-v1).(v2-v1)

    // returns squared distance between two points: |v2-v1|^2 and sets dst=(v2-v1)
    static float distanceSquared(vector4 &dst, const vector4 &v1, const vector4 &v2);

    // 4-element static methods
    static void add4(vector4 &dst, const vector4 &v1, const vector4 &v2); // dst = v1 + v2 (4-element)
    static void sub4(vector4 &dst, const vector4 &v1, const vector4 &v2); // dst = v1 - v2 (4-element)
    static void mul4(vector4 &dst, const vector4 &v1, const vector4 &v2); // dst = v1 * v2 (4-element)
    static void div4(vector4 &dst, const vector4 &v1, const vector4 &v2); // dst = v1 / v2 (4-element)
    static void mul4(vector4 &dst, const vector4 &v1, float fScale);      // dst = v1 * fScale (4-element)

    static void recip(vector4 &dst, const vector4 &v);     // dst = sqrt(v) (3-element)
    static void sqrt(vector4 &dst, const vector4 &v);      // dst = sqrt(v) (3-element)
    static void recipSqrt(vector4 &dst, const vector4 &v); // dst = 1.0/sqrt(v) (3-element)

    static void recip4(vector4 &dst, const vector4 &v);     // dst = sqrt(v) (4-element)
    static void sqrt4(vector4 &dst, const vector4 &v);      // dst = sqrt(v) (4-element)
    static void recipSqrt4(vector4 &dst, const vector4 &v); // dst = 1.0/sqrt(v) (4-element)

    static void normalize4(vector4 &dst, const vector4 &src) { scale4(dst, src, rsqrtf(src.squareMag4())); } // (4-element)

    static void mad4(vector4 &dst, const vector4 &vm1, const vector4 &vm2, const vector4 &va); // dst = vm1*vm2 + va

    // given a triangle, compute edge vectors
    // e0 = p2 - p1; e1 = p0 - p2, e2 = p1 - p0
    static void triangleEdges(vector4 *pEdge, const vector4 *pTriangleVertex);

    // given a point vPoint, compute vectors from each triangle vertex to that point
    // ri = vPoint - pi
    static void triangleVectors(vector4 *pVector, const vector4 *pTriangleVertex, const vector4 &vPoint);

    // the above functions (triangleEdges and triangleVectors), combined
    static void triangleEdgesAndVectors(vector4 *pEdge, vector4 *pVector, const vector4 *pTriangleVertex, const vector4 &vPoint);

    // returns true if point lies within 3-D triangle (within volume perpendicular to triangle)
    bool inTriangle(const vector4 *pTri, const vector4 &vNormal) const; // vNormal is precomputed triangle normal
    bool inTriangle(const vector4 *pTri) const;

    // point-in-triangle test, given triangle edges and vectors as computed in triangleEdges() and
    // triangleVectors().  Specified normal 'vNormal' does not need to be normalized.
    // (vNormal should be direction as defined be pEdges[2] x pEdges[0])
    bool inTriangleEx(const vector4 *pEdges, const vector4 *pVectors, const vector4 &vNormal) const;

    // multiple square distance [vSquareDisti = (di . di)], di = (c0 - vi)
    static void squareDistQuad(vector4 &vSquareDist,
                               vector4 &d0, vector4 &d1, vector4 &d2, vector4 &d3,
                               const vector4 &c0,
                               const vector4 &v0, const vector4 &v1, const vector4 &v2, const vector4 &v3);

    // multiple square distance [vSquareDisti = (di . di)], di = (c0 - vi)
    static void squareDistQuad(vector4 &vSquareDist,
                               const vector4 &c0,
                               const vector4 &v0, const vector4 &v1, const vector4 &v2, const vector4 &v3);

    static void sumBOverAQuad(vector4 &dst,
                              const vector4 &a,
                              const vector4 &b0,
                              const vector4 &b1,
                              const vector4 &b2,
                              const vector4 &b3);

    // dst += [(c0 - bi)/|c0 - bi|] * [vScalei / (fZeroOffset + |c0 - bi|^2]
    static void sumInverseFalloff(vector4 &dst,
                                  const vector4 &c0,
                                  const vector4 &b0,
                                  const vector4 &b1,
                                  const vector4 &b2,
                                  const vector4 &b3,
                                  const vector4 &vScale,
                                  float fRadius2Factor,
                                  float fZeroOffset);

    static void applyEulerImplicitQuad(
        vector4 &r0, vector4 &r1, vector4 &r2, vector4 &r3,
        vector4 &v0, vector4 &v1, vector4 &v2, vector4 &v3,
        const vector4 &a0, const vector4 &a1, const vector4 &a2, const vector4 &a3,
        const vector4 &vAScale0, const vector4 &vAScale1, float dt);

    static void div4(vector4 &dst, const vector4 &v1, float s);   // dst = v1 / s (4-element)
    static void scale4(vector4 &dst, const vector4 &v1, float s); // dst = v1 * s (4-element)
    static float dot4(const vector4 &v1, const vector4 &v2);      // 4-element dot (inner) product of v1 and v2

    static void addScaled4(vector4 &dst, const vector4 &v1, const vector4 &v2, float s2);           // dst = v1 + s*v2
    static void addScaled4(vector4 &dst, const vector4 &v1, float s1, const vector4 &v2, float s2); // dst = v1*s1 + v2*s2

    static void min4(vector4 &dst, const vector4 &v1, const vector4 &v2);             // dst = min(v1, v2) per element (4-element)
    static void max4(vector4 &dst, const vector4 &v1, const vector4 &v2);             // dst = max(v1, v2) per element (4-element)
    static void clampMag4(vector4 &vDst, const vector4 &vSrc, const vector4 &vClamp); // clamp vSrc to +/- vClamp

    static void lerp4(vector4 &dst, const vector4 &v1, const vector4 &v2, float t); // linear interpolation t=[0, 1] -> dst=[v1, v2] (4-element)

    // distance from triangle, returning triangle basis parameters in pfSParam and pfTParam
    // where basis S is (pTriangle[1] - pTriangle[0]) and T is (pTriangle[2] - pTriangle[0])
    float distanceSquaredST(const vector4 *pTriangle, float *pfSParam, float *pfTParam) const;

    // distance from triangle
    // (*pnNearestEdge is filled with edge index if nearest point is on an edge, otherwise -1)
    float distanceSquared(vector4 &vNearestPoint, const vector4 *pTriangle, int *pnNearestEdge = 0) const;

    // compute linear velocity given linear positions and a time step
    // fDeltaSeconds must be non-zero
    static void instantaneousVelocity(vector4 &vVLinear, const vector4 &v0, const vector4 &v1, float fDeltaSeconds);

  } /* SALIGN(16) */;

  extern const vector4 c_vUP;
  extern const vector4 c_vDOWN;
  extern const vector4 c_vLEFT;
  extern const vector4 c_vRIGHT;
  extern const vector4 c_vFORWARD;
  extern const vector4 c_vBACKWARD;
  extern const vector4 c_vZERO; // <0 0 0 1>

  ////////////////////////////////////////////////////////////
  inline void SASSERT_NORMALIZED(const vector4 &v, float fSquareEpsilon = 0.02f)
  {
    SASSERT(fabsf(v.squareMag() - 1.0f) < fSquareEpsilon);
  }

  inline void SASSERT_NONZEROLENGTH(const vector4 &v, float fSquareEpsilon = 0.0000005f)
  {
    SASSERT(v.squareMag() > fSquareEpsilon);
  }

  ////////////////////////////////////////////////////////////

#pragma pack(pop)
} // namespace SMath

#include "vector4-inline.h"

#endif
