#ifndef STEVESCH_MATHVEC_INTERNAL_VECTOR2_H_
#define STEVESCH_MATHVEC_INTERNAL_VECTOR2_H_
// Copyright © 2002, Stephen Schlueter, All Rights Reserved

#include "mathVec.h"
#include "stevesch-MathBase.h"
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

    inline float X() const { return x; }
    inline float Y() const { return y; }
    inline float &X() { return x; }
    inline float &Y() { return y; }

    // element access by index
    float operator[](int n) const;
    float &operator[](int n);

    void set(float _x, float _y)
    {
      x = _x;
      y = _y;
    }
    void set(const stevesch::vector4 &v); // use x and y from stevesch::vector4

    float cross(const vector2 &v1) const
    {
      return (x * v1.y - y * v1.x);
    }

    float dot(const vector2 &v1) const
    {
      return x * v1.x + y * v1.y;
    }

    // 2-element squared-magnitude
    float squareMag() const
    {
      return (x * x + y * y);
    }

    // 2-element magnitude
    float abs() const
    {
      return sqrtf(squareMag());
    }

    vector2 operator+(const vector2 &v1) const
    {
      return vector2(x + v1.x, y + v1.y);
    }

    vector2 operator-(const vector2 &v1) const
    {
      return vector2(x - v1.x, y - v1.y);
    }

    vector2 operator*(float fScale) const
    {
      return vector2(x * fScale, y * fScale);
    }

    vector2 operator/(float fScale) const
    {
      float fRecip = recipf(fScale);
      return vector2(x * fRecip, y * fRecip);
    }

    vector2 operator-() const
    {
      return vector2(-x, -y);
    }

    vector2 &operator+=(const vector2 &v1)
    {
      x += v1.x;
      y += v1.y;
      return *this;
    }

    vector2 &operator-=(const vector2 &v1)
    {
      x -= v1.x;
      y -= v1.y;
      return *this;
    }

    vector2 &operator+=(const stevesch::vector4 &v1); // add x and y of stevesch::vector4
    vector2 &operator-=(const stevesch::vector4 &v1); // subtract x and y of stevesch::vector4

    vector2 &operator*=(float fScale)
    {
      x *= fScale;
      y *= fScale;
      return *this;
    }

    //vector2& operator*=( const matrix2& crRight );		// v = v*m
    //vector2 operator*( const matrix2& crRight ) const;	// v*m

    // v = m*v (returns *this),
    vector2 &transform(const matrix2 &m);
    vector2 &transformTransposed(const matrix2 &m);

    static void transform(vector2 &vOut, const matrix2 &m, const vector2 &v); // vOut = m*v

    vector2 &operator/=(float fScale)
    {
      float fRecip = recipf(fScale);
      x *= fRecip;
      y *= fRecip;
      return *this;
    }

    static float squareDist(const vector2 &v0, const vector2 &v1)
    {
      float dx = v0.x - v1.x;
      float dy = v0.y - v1.y;
      return dx * dx + dy * dy;
    }

    // get sum of x and y components in v0 and v1
    static void add(vector2 &vDst, const vector2 &v0, const vector2 &v1)
    {
      vDst.set(v0.x + v1.x, v0.y + v1.y);
    }

    // get difference of x and y components in v0 and v1
    static void sub(vector2 &vDst, const vector2 &v0, const vector2 &v1)
    {
      vDst.set(v0.x - v1.x, v0.y - v1.y);
    }

    // get difference of stevesch::vector4 x and y components in v0 and v1
    static void sub(vector2 &vDst, const stevesch::vector4 &v0, const stevesch::vector4 &v1);

    static void addScaled(vector2 &dst, const vector2 &v1, const vector2 &v2, float s2);           // dst = v1 + v2*s2
    static void addScaled(vector2 &dst, const vector2 &v1, float s1, const vector2 &v2, float s2); // dst = v1*s1 + v2*s2

    static void lerp(vector2 &dst, const vector2 &v1, const vector2 &v2, float t); // linear interpolation t=[0, 1] -> dst=[v1, v2]
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

    // return 'NULL' failure occurs
    matrix2 *invert();                        // returns 'this' if success
    matrix2 *getInverse(matrix2 &rDst) const; // returns &rDst if success

    matrix2 &operator*=(const matrix2 &rRight);
    matrix2 operator*(const matrix2 &rRight);

    float det() const;

    void transpose();       // transpose self
    matrix2 getTranspose(); // return transpose

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

} // namespace stevesch

#include "vector2-inline.h"

#endif // STEVESCH_MATHVEC_INTERNAL_VECTOR2_H_
