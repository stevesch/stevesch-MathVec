#ifndef STEVESCH_MATHVEC_INTERNAL_VECTOR2_H_
#define STEVESCH_MATHVEC_INTERNAL_VECTOR2_H_
// Copyright © 2002, PulseCode Interactive LLC, All Rights Reserved

#include "stevesch-MathBase.h"
#include "vector4.h"

namespace stevesch
{
  class stevesch::vector4; // extern

  class stevesch::vector2;
  class stevesch::matrix2;

  class stevesch::vector2
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

    stevesch::vector2() {} // uninitialized
    stevesch::vector2(float _x, float _y) : x(_x), y(_y) {}

    SFORCEINLINE float X() const { return x; }
    SFORCEINLINE float Y() const { return y; }
    SFORCEINLINE float &X() { return x; }
    SFORCEINLINE float &Y() { return y; }

    // element access by index
    float operator[](int n) const;
    float &operator[](int n);

    void set(float _x, float _y)
    {
      x = _x;
      y = _y;
    }
    void set(const stevesch::vector4 &v); // use x and y from stevesch::vector4

    float cross(const stevesch::vector2 &v1) const
    {
      return (x * v1.y - y * v1.x);
    }

    float dot(const stevesch::vector2 &v1) const
    {
      return x * v1.x + y * v1.y;
    }

    stevesch::vector2 operator+(const stevesch::vector2 &v1) const
    {
      return stevesch::vector2(x + v1.x, y + v1.y);
    }

    stevesch::vector2 operator-(const stevesch::vector2 &v1) const
    {
      return stevesch::vector2(x - v1.x, y - v1.y);
    }

    stevesch::vector2 operator*(float fScale) const
    {
      return stevesch::vector2(x * fScale, y * fScale);
    }

    stevesch::vector2 operator/(float fScale) const
    {
      float fRecip = recipf(fScale);
      return stevesch::vector2(x * fRecip, y * fRecip);
    }

    stevesch::vector2 operator-() const
    {
      return stevesch::vector2(-x, -y);
    }

    stevesch::vector2 &operator+=(const stevesch::vector2 &v1)
    {
      x += v1.x;
      y += v1.y;
      return *this;
    }

    stevesch::vector2 &operator-=(const stevesch::vector2 &v1)
    {
      x -= v1.x;
      y -= v1.y;
      return *this;
    }

    stevesch::vector2 &operator+=(const stevesch::vector4 &v1); // add x and y of stevesch::vector4
    stevesch::vector2 &operator-=(const stevesch::vector4 &v1); // subtract x and y of stevesch::vector4

    stevesch::vector2 &operator*=(float fScale)
    {
      x *= fScale;
      y *= fScale;
      return *this;
    }

    //stevesch::vector2& operator*=( const stevesch::matrix2& crRight );		// v = v*m
    //stevesch::vector2 operator*( const stevesch::matrix2& crRight ) const;	// v*m

    // v = m*v (returns *this),
    stevesch::vector2 &transform(const stevesch::matrix2 &m);
    stevesch::vector2 &transformTransposed(const stevesch::matrix2 &m);

    static void transform(stevesch::vector2 &vOut, const stevesch::matrix2 &m, const stevesch::vector2 &v); // vOut = m*v

    stevesch::vector2 &operator/=(float fScale)
    {
      float fRecip = recipf(fScale);
      x *= fRecip;
      y *= fRecip;
      return *this;
    }

    static float squareDist(const stevesch::vector2 &v0, const stevesch::vector2 &v1)
    {
      float dx = v0.x - v1.x;
      float dy = v0.y - v1.y;
      return dx * dx + dy * dy;
    }

    // get sum of x and y components in v0 and v1
    static void add(stevesch::vector2 &vDst, const stevesch::vector2 &v0, const stevesch::vector2 &v1)
    {
      vDst.set(v0.x + v1.x, v0.y + v1.y);
    }

    // get difference of x and y components in v0 and v1
    static void sub(stevesch::vector2 &vDst, const stevesch::vector2 &v0, const stevesch::vector2 &v1)
    {
      vDst.set(v0.x - v1.x, v0.y - v1.y);
    }

    // get difference of stevesch::vector4 x and y components in v0 and v1
    static void sub(stevesch::vector2 &vDst, const stevesch::vector4 &v0, const stevesch::vector4 &v1);

    static void addScaled(stevesch::vector2 &dst, const stevesch::vector2 &v1, const stevesch::vector2 &v2, float s2);           // dst = v1 + v2*s2
    static void addScaled(stevesch::vector2 &dst, const stevesch::vector2 &v1, float s1, const stevesch::vector2 &v2, float s2); // dst = v1*s1 + v2*s2

    static void lerp(stevesch::vector2 &dst, const stevesch::vector2 &v1, const stevesch::vector2 &v2, float t); // linear interpolation t=[0, 1] -> dst=[v1, v2]
  };

  SALIGN_DECL(16)
  class stevesch::matrix2
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
        stevesch::vector2 col[2];
      };
    };

    const stevesch::vector2 &operator[](int n) const; // access by column
    stevesch::vector2 &operator[](int n);             // access by column

    inline stevesch::matrix2() {} // uninitialized

    inline stevesch::matrix2(float _m00, float _m10, float _m01, float _m11) : m00(_m00), m10(_m10), m01(_m01), m11(_m11) {}

    inline stevesch::matrix2(float fDiag) : m00(fDiag), m10(0.0f), m01(0.0f), m11(fDiag) {}

    inline stevesch::matrix2(const stevesch::vector2 &vRow0, const stevesch::vector2 &vRow1) : m00(vRow0.x), m10(vRow0.y), m01(vRow1.x), m11(vRow1.y) {}

    inline stevesch::matrix2(const stevesch::matrix2 &r)
    {
      col[0] = r.col[0];
      col[1] = r.col[1];
    } //{ V4 = r.V4; }
    inline stevesch::matrix2 &operator=(const stevesch::matrix2 &r)
    {
      col[0] = r.col[0];
      col[1] = r.col[1];
      return *this;
    } //{ V4 = r.V4; return *this; }

    // return 'NULL' failure occurs
    stevesch::matrix2 *invert();                         // returns 'this' if success
    stevesch::matrix2 *getInverse(stevesch::matrix2 &rDst) const; // returns &rDst if success

    stevesch::matrix2 &operator*=(const stevesch::matrix2 &rRight);
    stevesch::matrix2 operator*(const stevesch::matrix2 &rRight);

    float det() const;

    void transpose();        // transpose self
    stevesch::matrix2 getTranspose(); // return transpose

  } SALIGN(16);

  //----------------------------------------------------------------------------
  // Homogeneous mapping of quadrilateral <p00,p10,p11,p01> to square [0,1]^2.
  // The quadrilateral points are ordered counterclockwise and map onto the
  // corners (0,0), (1,0), (1,1), and (0,1), respectively.
  class hmQuadToSqr
  {
  public:
    hmQuadToSqr(const stevesch::vector2 &rkP00, const stevesch::vector2 &rkP10,
                const stevesch::vector2 &rkP11, const stevesch::vector2 &rkP01);

    stevesch::vector2 transform(const stevesch::vector2 &rkP);

  protected:
    stevesch::vector2 m_kT, m_kG, m_kD;
    stevesch::matrix2 m_kM;
  };

  //----------------------------------------------------------------------------
  // Homogeneous mapping of square [0,1]^2 to quadrilateral <p00,p10,p11,p01>.
  // The quadrilateral points are ordered counterclockwise and map onto the
  // corners (0,0), (1,0), (1,1), and (0,1), respectively.
  class HmSqrToQuad
  {
  public:
    HmSqrToQuad(const stevesch::vector2 &rkP00, const stevesch::vector2 &rkP10,
                const stevesch::vector2 &rkP11, const stevesch::vector2 &rkP01);

    stevesch::vector2 transform(const stevesch::vector2 &rkP);

  protected:
    stevesch::vector2 m_kT, m_kG, m_kD;
    stevesch::matrix2 m_kM;
  };

} // namespace stevesch

#include "vector2-inline.h"

#endif // STEVESCH_MATHVEC_INTERNAL_VECTOR2_H_
