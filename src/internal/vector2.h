#ifndef STEVESCH_MATHVEC_INTERNAL_VECTOR2_H_
#define STEVESCH_MATHVEC_INTERNAL_VECTOR2_H_
// Copyright © 2002, Stephen Schlueter, All Rights Reserved

#include <stevesch-MathBase.h>
#include "mathVec.h"
#include "vector4.h"

namespace stevesch
{
  class vector4; // extern

  class vector2;
  class matrix2;

  class vector2
  {
  public:
    union
    {
      struct
      {
        float x, y;
      };

      float m_v[2];
    };

    vector2() {} // uninitialized
    vector2(float _x, float _y) : x(_x), y(_y) {}
    vector2(const vector2 &_v);
    vector2(eZero) : x(0.0f), y(0.0f) {}
    vector2(eOnes) : x(1.0f), y(1.0f) {}

    inline float X() const { return x; }
    inline float Y() const { return y; }
    inline float &X() { return x; }
    inline float &Y() { return y; }

    vector2 &copy(const vector2 &v);  // copy (all memebers)
    vector2 &set(float _x, float _y);
    vector2 &set(const vector2 &v);

    ////////////////////////////////////////////////////

    vector2 &add(const vector2 &v);   // member-wise addition
    vector2 &sub(const vector2 &v);   // member-wise subtraction
    vector2 &mul(const vector2 &v);   // member-wise multiplication
    float cross(const vector2 &v1) const;

    vector2 &negate();                   // 3-element negation (x=-x, y=-y, z=-z)
    vector2 &negate(vector2 &dst) const; // 3-element negation (dst.x=-x, dst.y=-y, dst.z=-z)
    vector2 operator-() const;  // negate

    vector2 &scale(float scale); // multiply by scalar
    vector2 &mul(float scale);   // multiply by scalar (same as scale(float))
    vector2 &div(float scale);   // divide by scalar

    float dot(const vector2 &v) const; // dot (inner) product
    float squareMag() const;           // squared magnitude
    float abs() const;                 // magnitude

    const vector2 &normalize() { return mul(rsqrtf(squareMag())); }

    vector2 &operator=(const vector2 &v);  // copy
    vector2 &operator+=(const vector2 &v); // add
    vector2 &operator-=(const vector2 &v); // sub
    vector2 &operator*=(float scale);      // scale
    vector2 &operator/=(float scale);      // scale(1/scale)

    float operator[](int n) const;
    float &operator[](int n);

    ////////////////////////////////////////////////////

    friend inline vector2 operator+(const vector2 &v1, const vector2 &v2); // v1 + v2
    friend inline vector2 operator-(const vector2 &v1, const vector2 &v2); // v1 - v2
    friend vector2 operator*(const vector2 &v1, float s); // v1 * s
    friend vector2 operator*(float s, const vector2 &v1); // s * v
    friend inline vector2 operator/(const vector2 &v1, float d); // v1 / d

    //vector2& operator*=( const matrix2& crRight );		// v = v*m
    //vector2 operator*( const matrix2& crRight ) const;	// v*m

    // v = m*v (returns *this),
    vector2 &transform(const matrix2 &m);
    vector2 &transformTransposed(const matrix2 &m);

    static void transform(vector2 &vOut, const matrix2 &m, const vector2 &v); // vOut = m*v

    // static methods
    static inline void add(vector2 &dst, const vector2 &v1, const vector2 &v2); // dst = v1 + v2
    static inline void sub(vector2 &dst, const vector2 &v1, const vector2 &v2); // dst = v1 - v2
    static inline void mul(vector2 &dst, const vector2 &v1, const vector2 &v2); // dst = v1 * v2
    static inline void div(vector2 &dst, const vector2 &v1, float s);           // dst = v1 / s
    static inline void scale(vector2 &dst, const vector2 &v1, float s);         // dst = v1 * s
    static inline float dot(const vector2 &v1, const vector2 &v2);              // dot (inner) product of v1 and v2
    static inline float cross(const vector2 &v1, const vector2 &v2);            // 2D cross

    static inline float squareDist(const vector2 &v1, const vector2 &v2);

    static inline void min(vector2 &dst, const vector2 &v1, const vector2 &v2); // dst = min(v1, v2) per element
    static inline void max(vector2 &dst, const vector2 &v1, const vector2 &v2); // dst = max(v1, v2) per element

    static inline void lerp(vector2 &dst, const vector2 &v1, const vector2 &v2, float t); // linear interpolation t=[0, 1] -> dst=[v1, v2]

    static inline void addScaled(vector2 &dst, const vector2 &v1, const vector2 &v2, float s2);           // dst = v1 + v2*s2
    static inline void addScaled(vector2 &dst, const vector2 &v1, float s1, const vector2 &v2, float s2); // dst = v1*s1 + v2*s2

    friend vector2 operator+(const vector2 &v1, const vector2 &v2); // v1 + v2
    friend vector2 operator-(const vector2 &v1, const vector2 &v2); // v1 - v2
    friend vector2 operator*(const vector2 &v1, float s); // v1 * s
    friend vector2 operator*(float s, const vector2 &v1); // s * v
    friend vector2 operator/(const vector2 &v1, float d); // v1 / d

    // vector4 helpers:
    vector2 &set(const stevesch::vector4 &v); // use x and y from stevesch::vector4
    vector2 &operator+=(const stevesch::vector4 &v1); // add x and y of stevesch::vector4
    vector2 &operator-=(const stevesch::vector4 &v1); // subtract x and y of stevesch::vector4
    static void sub(vector2 &vDst, const stevesch::vector4 &v0, const stevesch::vector4 &v1);
  };

  // SALIGN_DECL(16)
  class matrix2
  {
  public:
    union
    {
      // memory order: m00, m10, m01, m11
      struct
      {
        float m00, m10; // column 0
        float m01, m11; // column 1
      };

      float xy[2][2]; // [column][row]

      struct
      {
        vector2 col[2];
      };
    };

    const vector2 &operator[](int n) const; // access by column
    vector2 &operator[](int n);             // access by column

    inline matrix2() {} // uninitialized
    inline matrix2(float _m00, float _m10, float _m01, float _m11) : m00(_m00), m10(_m10), m01(_m01), m11(_m11) {}
    inline matrix2(float fDiag) : m00(fDiag), m10(0.0f), m01(0.0f), m11(fDiag) {}
    inline matrix2(const vector2 &vRow0, const vector2 &vRow1) : m00(vRow0.x), m10(vRow0.y), m01(vRow1.x), m11(vRow1.y) {}
    inline matrix2(const matrix2 &r)
    {
      col[0] = r.col[0];
      col[1] = r.col[1];
    } //{ V4 = r.V4; }
    inline matrix2 &operator=(const matrix2 &r)
    {
      col[0] = r.col[0];
      col[1] = r.col[1];
      return *this;
    } //{ V4 = r.V4; return *this; }

    matrix2(eIdentity) { identity(); }
    matrix2(eZero) { zero(); }


    // return 'NULL' failure occurs
    matrix2 *invert();                        // returns 'this' if success
    matrix2 *getInverse(matrix2 &rDst) const; // returns &rDst if success

    matrix2 &operator*=(const matrix2 &rRight);
    matrix2 operator*(const matrix2 &rRight);

    const matrix2 &identity();
    const matrix2 &zero();

    float det() const;

    void transpose();       // transpose self
    matrix2 getTranspose(); // return transpose

    static const matrix2 I; // identity matrix (2x2)
  } /* SALIGN(16) */;

  //----------------------------------------------------------------------------
  // Homogeneous mapping of quadrilateral <p00,p10,p11,p01> to square [0,1]^2.
  // The quadrilateral points are ordered counterclockwise and map onto the
  // corners (0,0), (1,0), (1,1), and (0,1), respectively.
  class hmQuadToSqr
  {
  public:
    hmQuadToSqr(const vector2 &rkP00, const vector2 &rkP10,
                const vector2 &rkP11, const vector2 &rkP01);

    vector2 transform(const vector2 &rkP);

  protected:
    vector2 m_kT, m_kG, m_kD;
    matrix2 m_kM;
  };

  //----------------------------------------------------------------------------
  // Homogeneous mapping of square [0,1]^2 to quadrilateral <p00,p10,p11,p01>.
  // The quadrilateral points are ordered counterclockwise and map onto the
  // corners (0,0), (1,0), (1,1), and (0,1), respectively.
  class HmSqrToQuad
  {
  public:
    HmSqrToQuad(const vector2 &rkP00, const vector2 &rkP10,
                const vector2 &rkP11, const vector2 &rkP01);

    vector2 transform(const vector2 &rkP);

  protected:
    vector2 m_kT, m_kG, m_kD;
    matrix2 m_kM;
  };

  // linear interpolation t=[0, 1] -> dst=[v1, v2]
  inline void lerp(vector2 &dst, const vector2 &v1, const vector2 &v2, float t)
  {
    vector2::lerp(dst, v1, v2, t);
  }
} // namespace stevesch

#include "vector2-inline.h"

#endif // STEVESCH_MATHVEC_INTERNAL_VECTOR2_H_
