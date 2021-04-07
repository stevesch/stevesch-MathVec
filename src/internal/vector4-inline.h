#ifndef STEVESCH_MATHVEC_INTERNAL_VECTOR4_INLINE_H_
#define STEVESCH_MATHVEC_INTERNAL_VECTOR4_INLINE_H_
// Copyright © 2002, Stephen Schlueter, All Rights Reserved
// SVECINLINE functions for vector4

#include "vector3.h"

#define SINLINE_X86_ASM 0

namespace stevesch
{
// using macros to avoid some of the non-inlining in Debug builds
#define SVECTORCOPY(dst, src) \
  (dst).x = (src).x,          \
  (dst).y = (src).y,          \
  (dst).z = (src).z,          \
  (dst).w = (src).w

#define SVECTORADD(dst, v1, v2) \
  (dst).x = (v1).x + (v2).x,    \
  (dst).y = (v1).y + (v2).y,    \
  (dst).z = (v1).z + (v2).z,    \
  (dst).w = (v1).w

#define SVECTORSUB(dst, v1, v2) \
  (dst).x = (v1).x - (v2).x,    \
  (dst).y = (v1).y - (v2).y,    \
  (dst).z = (v1).z - (v2).z,    \
  (dst).w = (v1).w

#define SVECTORMUL(dst, v1, v2) \
  (dst).x = (v1).x * (v2).x,    \
  (dst).y = (v1).y * (v2).y,    \
  (dst).z = (v1).z * (v2).z,    \
  (dst).w = (v1).w

#define SVECTORMULS(dst, v1, s) \
  (dst).x = (v1).x * s,         \
  (dst).y = (v1).y * s,         \
  (dst).z = (v1).z * s,         \
  (dst).w = (v1).w

#define SVECTORDOT(v1, v2) ((v1).x * (v2).x + (v1).y * (v2).y + (v1).z * (v2).z)

#define SVECTORADD4(dst, v1, v2) \
  (dst).x = (v1).x + (v2).x,     \
  (dst).y = (v1).y + (v2).y,     \
  (dst).z = (v1).z + (v2).z,     \
  (dst).w = (v1).w + (v2).w

#define SVECTORSUB4(dst, v1, v2) \
  (dst).x = (v1).x - (v2).x,     \
  (dst).y = (v1).y - (v2).y,     \
  (dst).z = (v1).z - (v2).z,     \
  (dst).w = (v1).w - (v2).w

#define SVECTORMUL4(dst, v1, v2) \
  (dst).x = (v1).x * (v2).x,     \
  (dst).y = (v1).y * (v2).y,     \
  (dst).z = (v1).z * (v2).z,     \
  (dst).w = (v1).w * (v2).w

#define SVECTORMUL4S(dst, v1, s) \
  (dst).x = (v1).x * s,          \
  (dst).y = (v1).y * s,          \
  (dst).z = (v1).z * s,          \
  (dst).w = (v1).w * s

#define SVECTORDOT4(v1, v2) ((v1).x * (v2).x + (v1).y * (v2).y + (v1).z * (v2).z + (v1).w * (v2).w)

#define SCROSS(dst, v1, v2)                       \
  do                                              \
  {                                               \
    float tx = (v1).y * (v2).z - (v1).z * (v2).y; \
    float ty = (v1).z * (v2).x - (v1).x * (v2).z; \
    (dst).z = (v1).x * (v2).y - (v1).y * (v2).x;  \
    (dst).x = tx;                                 \
    (dst).y = ty;                                 \
    (dst).w = (v1).w;                             \
  } while (0)

  ////////////////////////////////////////////////////////////

#if SINLINE_X86_ASM
  __declspec(naked) SVECINLINE vector4::vector4(float _x, float _y, float _z)
  {
    __asm
    {
			mov		eax,ecx
			mov		ecx,dword ptr [esp+4]
			mov		dword ptr [eax],ecx
			mov		ecx,dword ptr [esp+8]
			mov		dword ptr [eax+4],ecx
			mov		ecx,dword ptr [esp+0Ch]
			mov		dword ptr [eax+8],ecx
			mov		dword ptr [eax+0Ch],0x3f800000
			ret		0Ch
    }
    //x = _x; y = _y; z = _z; w = 1.0f;
  }

  __declspec(naked) SVECINLINE vector4::vector4(float _x, float _y, float _z, float _w)
  {
    __asm
    {
			mov		eax,ecx
			mov		ecx,dword ptr [esp+4]
			mov		dword ptr [eax],ecx
			mov		ecx,dword ptr [esp+8]
			mov		dword ptr [eax+4],ecx
			mov		ecx,dword ptr [esp+0Ch]
			mov		dword ptr [eax+8],ecx
			mov		ecx,dword ptr [esp+10h]
			mov		dword ptr [eax+0Ch],ecx
			ret		10h
    }
    //x = _x; y = _y; z = _z; w = _w;
  }

  /*
	__declspec( naked ) SVECINLINE vector4::vector4(const vector4& v)
	{
		__asm
		{
			mov		eax,ecx
			mov		ecx,dword ptr [esp+4]
			mov		edx,dword ptr [ecx]
			mov		dword ptr [eax],edx
			mov		edx,dword ptr [ecx+4]
			mov		dword ptr [eax+4],edx
			mov		edx,dword ptr [ecx+8]
			mov		dword ptr [eax+8],edx
			mov		edx,dword ptr [ecx+0Ch]
			mov		dword ptr [eax+0Ch],edx
			ret		4
		}
	}

	// copy (all memebers)
	__declspec( naked ) SVECINLINE const vector4& vector4::copy(const vector4& v)
	{
		__asm
		{
			mov		eax,ecx
			mov		ecx,dword ptr [esp+4]
			mov		edx,dword ptr [ecx]
			mov		dword ptr [eax],edx
			mov		edx,dword ptr [ecx+4]
			mov		dword ptr [eax+4],edx
			mov		edx,dword ptr [ecx+8]
			mov		dword ptr [eax+8],edx
			mov		edx,dword ptr [ecx+0Ch]
			mov		dword ptr [eax+0Ch],edx
			ret		4
		}
	}

	__declspec( naked ) SVECINLINE const vector4& vector4::operator =(const vector4& v)
	{
		// assumes this ptr in ecx
		__asm
		{
			mov		eax,ecx
			mov		ecx,dword ptr [esp+4]
			mov		edx,dword ptr [ecx]
			mov		dword ptr [eax],edx
			mov		edx,dword ptr [ecx+4]
			mov		dword ptr [eax+4],edx
			mov		edx,dword ptr [ecx+8]
			mov		dword ptr [eax+8],edx
			mov		edx,dword ptr [ecx+0Ch]
			mov		dword ptr [eax+0Ch],edx
			ret		4
		}
		// *this returned in eax
	}
*/

#else
  SVECINLINE vector4::vector4(float _x, float _y, float _z) : x(_x), y(_y), z(_z), w(1.0f)
  {
  }

  SVECINLINE vector4::vector4(float _x, float _y, float _z, float _w) : x(_x), y(_y), z(_z), w(_w)
  {
  }

  /*
	SVECINLINE vector4::vector4(const vector4& v)
	{
		SVECTORCOPY(*this, v);
	}

	// copy (all memebers)
	SVECINLINE const vector4& vector4::copy(const vector4& v)
	{
		SVECTORCOPY(*this, v);
		return *this;
	}
	
	SVECINLINE const vector4& vector4::operator =(const vector4& v)
	{
		SVECTORCOPY(*this, v);
		return *this;
	}
*/

#endif

  SVECINLINE bool vector4::operator==(const vector4 &crOther) const
  {
    return ((x == crOther.x) &&
            (y == crOther.y) &&
            (z == crOther.z) &&
            (w == crOther.w));
  }

  SVECINLINE bool vector4::operator!=(const vector4 &crOther) const
  {
    return ((x != crOther.x) ||
            (y != crOther.y) ||
            (z != crOther.z) ||
            (w != crOther.w));
  }

  SVECINLINE const vector4 &vector4::set(float _x, float _y, float _z)
  {
    x = _x;
    y = _y;
    z = _z;
    w = 1.0f;
    return *this;
  }

  SVECINLINE const vector4 &vector4::set(float _x, float _y, float _z, float _w)
  {
    x = _x;
    y = _y;
    z = _z;
    w = _w;
    return *this;
  }

  SVECINLINE const vector4 &vector4::set(const stevesch::vector3 &v)
  {
    x = v.x;
    y = v.y;
    z = v.z;
    w = 1.0f;
    return *this;
  }

  // member-wise addition (3-element)
  SVECINLINE const vector4 &vector4::add(const vector4 &v)
  {
    x += v.x;
    y += v.y;
    z += v.z;
    return *this;
  }

  // member-wise subtraction (3-element)
  SVECINLINE const vector4 &vector4::sub(const vector4 &v)
  {
    x -= v.x;
    y -= v.y;
    z -= v.z;
    return *this;
  }

  // member-wise multiplication (3-element)
  SVECINLINE const vector4 &vector4::mul(const vector4 &v)
  {
    x *= v.x;
    y *= v.y;
    z *= v.z;
    return *this;
  }

  SVECINLINE const vector4 &vector4::mul(float scale)
  {
    x *= scale;
    y *= scale;
    z *= scale;
    return *this;
  }

  SVECINLINE const vector4 &vector4::scale(float scale)
  {
    return mul(scale);
  }

  /*	
	SVECINLINE const vector4& vector4::div(float scale)
	{
		return mul( recipf(scale) );
	}

	// 3-element cross (outer) product
	SVECINLINE const vector4& vector4::cross(const vector4& v)
	{
		SCROSS( *this, *this, v );
		return *this;
	}
*/

  // 3-element negation (x=-x, y=-y, z=-z)
  SVECINLINE const vector4 &vector4::negate()
  {
    x = -x;
    y = -y;
    z = -z;
    return *this;
  }

  // 3-element negation (dst.x=-x, dst.y=-y, dst.z=-z)
  SVECINLINE const vector4 &vector4::negate(vector4 &dst) const
  {
    dst.x = -x;
    dst.y = -y;
    dst.z = -z;
    dst.w = w;
    return dst;
  }

  /*
	// 3-element squared-magnitude
	SVECINLINE float vector4::squareMag() const
	{
		return (x*x + y*y + z*z);
	}

	
	// 1.0 / squared magnitued (3-element)
	SVECINLINE float vector4::recipSquareMag() const
	{
		return recipf( squareMag() );	// TBD: optimize with intrinsics
	}

	// 3-element magnitude
	SVECINLINE float vector4::abs() const
	{
		return sqrtf(squareMag());
	}

	
	// 3-element 1/magnitude
	SVECINLINE float vector4::recipAbs() const
	{
		return rsqrtf(squareMag());
	}
	

	// normalize self (3-element)
	SVECINLINE const vector4& vector4::normalize()
	{
		return mul( rsqrtf(squareMag()) );
	}

	SVECINLINE void vector4::normalize(vector4& dst, const vector4& src)
	{
		scale( dst, src, rsqrtf(src.squareMag()) );
	}	// (3-element)
*/
  ////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////

  SVECINLINE const float &vector4::operator[](int n) const
  {
    SASSERT_EXTRA((n >= 0) && (n <= 3));
    return ((float *)&x)[n];
  }

  SVECINLINE float &vector4::operator[](int n)
  {
    SASSERT_EXTRA((n >= 0) && (n <= 3));
    return ((float *)&x)[n];
  }

  // add
  SVECINLINE const vector4 &vector4::operator+=(const vector4 &v)
  {
    return add(v);
  }

  // sub
  SVECINLINE const vector4 &vector4::operator-=(const vector4 &v)
  {
    return sub(v);
  }

  // scale (mul(scalar))
  SVECINLINE const vector4 &vector4::operator*=(float scale)
  {
    return mul(scale);
  }

  // mul(1/scalar)
  SVECINLINE const vector4 &vector4::operator/=(float scale)
  {
    return div(scale);
  }

  // 3-element randomize (0.0f, 1.0f)
  SVECINLINE void vector4::rand(stevesch::RandGen &r)
  {
    x = r.getFloat();
    y = r.getFloat();
    z = r.getFloat();
    w = 1.0f;
  }

  // 3-element randomize (a.*, b.*)
  SVECINLINE void vector4::randAB(const vector4 &a, const vector4 &b, stevesch::RandGen &r)
  {
    vector4 dif;
    this->rand(r);
    SVECTORSUB(dif, b, a);
    mul(dif);
    add(a);
  }

  SVECINLINE void vector4::randSpherical(stevesch::RandGen &r)
  {
    ((stevesch::vector3 *)this)->randSpherical(r);
    w = 1.0f;
  }

  ////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////

  SVECINLINE const vector4 &vector4::scale4(float scale)
  {
    return mul4(scale);
  }

  // 4-element negation (x=-x, y=-y, z=-z, w=-w)
  SVECINLINE const vector4 &vector4::negate4()
  {
    x = -x;
    y = -y;
    z = -z;
    w = -w;
    return *this;
  }

  // 4-element negation (dst.x=-x, dst.y=-y, dst.z=-z, dst.w=-w)
  SVECINLINE const vector4 &vector4::negate4(vector4 &dst) const
  {
    dst.x = -x;
    dst.y = -y;
    dst.z = -z;
    dst.w = -w;
    return dst;
  }

  // 1.0 / squared magnitued (4-element)
  SVECINLINE float vector4::recipSquareMag4() const
  {
    return recipf(squareMag4()); // TBD: optimize with intrinsics
  }

  // 4-element magnitude
  SVECINLINE float vector4::abs4() const
  {
    return sqrtf(squareMag4());
  }

  // 4-element 1/magnitude
  SVECINLINE float vector4::recipAbs4() const
  {
    return rsqrtf(squareMag4());
  }

  /*
	// normalize self (4-element)
	SVECINLINE const vector4& vector4::normalize4()
	{
		return mul4( rsqrtf(squareMag4()) );
	}
*/

  // 4-element randomize (0.0f, 1.0f)
  SVECINLINE void vector4::rand4(stevesch::RandGen &r)
  {
    x = r.getFloat();
    y = r.getFloat();
    z = r.getFloat();
    w = r.getFloat();
  }

  // 4-element randomize (a.*, b.*)
  SVECINLINE void vector4::randAB4(const vector4 &a, const vector4 &b, stevesch::RandGen &r)
  {
    vector4 dif;
    rand4(r);
    SVECTORSUB4(dif, b, a);
    mul4(dif);
    add4(a);
  }

  ////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////

  // v = M*v -- multiply (on left of column vector) by 4x4 matrix
  SVECINLINE const vector4 &vector4::transform(const stevesch::matrix4 &left)
  {
    transform(*this, left, *this);
    return *this;
  }

  // v = M*v -- (as if w=0) * 3x3 matrix
  SVECINLINE const vector4 &vector4::transformSub(const stevesch::matrix4 &left)
  {
    transformSub(*this, left, *this);
    return *this;
  }

  // v = (M^T)*v -- (as if w=0) * transposed 3x3 matrix
  SVECINLINE const vector4 &vector4::transformSubTransposed(const stevesch::matrix4 &left)
  {
    transformSubTransposed(*this, left, *this);
    return *this;
  }

  // v = M*v -- multiply (on left of column vector) by 3x3 matrix (as if w=1)
  SVECINLINE const vector4 &vector4::transformAff(const stevesch::matrix4 &left)
  {
    transformAff(*this, left, *this);
    return *this;
  }

  // // transform
  // SVECINLINE const vector4& vector4::operator *=(const stevesch::matrix4& left)
  // {
  // 	return transform(left);
  // }

  ////////////////////////////////////////////////////
  ////////////////////////////////////////////////////
  // static methods

  // dst = v1 + v2
  SVECINLINE void vector4::add(vector4 &dst, const vector4 &v1, const vector4 &v2)
  {
    SVECTORADD(dst, v1, v2);
  }

  // dst = v1 - v2
  SVECINLINE void vector4::sub(vector4 &dst, const vector4 &v1, const vector4 &v2)
  {
    SVECTORSUB(dst, v1, v2);
  }

  // dst = v1 * v2 (4-element)
  SVECINLINE void vector4::mul(vector4 &dst, const vector4 &v1, const vector4 &v2)
  {
    SVECTORMUL(dst, v1, v2);
  }

  /*	
	// dst = v1 / s
	SVECINLINE void vector4::div(vector4& dst, const vector4& v1, float s)
	{
		dst = v1;
		dst.div(s);
	}
	
	
	// dst = v1 * s
	SVECINLINE void vector4::scale(vector4& dst, const vector4& v1, float s)
	{
		SVECTORMULS( dst, v1, s );
//		dst = v1;
//		dst.mul(s);
	}

	// dst = v1 x v2
	SVECINLINE void vector4::cross(vector4& dst, const vector4& v1, const vector4& v2)
	{
		SCROSS( dst, v1, v2 );
	}
*/

  /*
	// 3-element squared-distance
	SVECINLINE float vector4::squareDist(const vector4& v1, const vector4& v2)
	{
		float dx = v1.x - v2.x;
		float dy = v1.y - v2.y;
		float dz = v1.z - v2.z;
		return (dx*dx + dy*dy + dz*dz);
//		vector4 temp;
//		SVECTORSUB(temp, v1, v2);
//		return temp.squareMag();
	}
*/

  // linear interpolation t=[0, 1] -> dst=[v1, v2] (3-element)
  SVECINLINE void vector4::lerp(vector4 &dst, const vector4 &v1, const vector4 &v2, float t)
  {
    // (1-t)*v1 + t*v2 == v1 - t*v1 + t*v2 == v1 + t*(v2 - v1)
    vector4 tmp;
    SVECTORSUB(tmp, v2, v1); // tmp in case dst is v1
    vector4::addScaled(dst, v1, tmp, t);
  }

  // add linear interpolation t=[0, 1] -> dst += [v1, v2] (3-element)
  SVECINLINE void vector4::addLerp(vector4 &dst, const vector4 &v1, const vector4 &v2, float t)
  {
    // (1-t)*v1 + t*v2 == v1 - t*v1 + t*v2 == v1 + t*(v2 - v1)
    vector4 tmp;
    SVECTORSUB(tmp, v2, v1); // tmp in case dst is v1
    vector4::addScaled(tmp, v1, tmp, t);
    dst += tmp;
  }

  /*
	// linear interpolation t=[0, 1] -> dst=[v1, v2] (4-element)
	SVECINLINE void vector4::lerp4(vector4& dst, const vector4& v1, const vector4& v2, float t)
	{
		// (1-t)*v1 + t*v2 == v1 - t*v1 + t*v2 == v1 + t*(v2 - v1)
		vector4 tmp;
		SVECTORSUB4(tmp, v2, v1);	// tmp in case dst is v1
		vector4::addScaled4(dst, v1, tmp, t);
	}

  
	// squared distance between two points: (v2-v1).(v2-v1)
	SVECINLINE float vector4::distanceSquared( const vector4& v1, const vector4& v2 )
	{
		vector4 vDif;
		vector4::sub( vDif, v2, v1 );
		return vDif.squareMag();
	}
*/

  ////////////////////////////////////////////////////
  ////////////////////////////////////////////////////

  /*
	// dst = v1 + v2 (4-element)
	SVECINLINE void vector4::add4(vector4& dst, const vector4& v1, const vector4& v2)
	{
		SVECTORADD4(dst, v1, v2);
	}
	
	// dst = v1 - v2 (4-element)
	SVECINLINE void vector4::sub4(vector4& dst, const vector4& v1, const vector4& v2)
	{
		SVECTORSUB4(dst, v1, v2);
	}
	
	// dst = v1 * v2 (4-element)
	SVECINLINE void vector4::mul4(vector4& dst, const vector4& v1, const vector4& v2)
	{
		SVECTORMUL4(dst, v1, v2);
	}
	
	// dst = v1 * fScale (4-element)
	SVECINLINE void vector4::mul4(vector4& dst, const vector4& v1, float fScale)
	{
		SVECTORMUL4S(dst, v1, fScale);
	}
*/

  /*	
	// dst = v1 / v2 (4-element member-wise division)
	SVECINLINE void vector4::div4(vector4& dst, const vector4& v1, const vector4& v2)
	{
		dst.x = v1.x / v2.x,
		dst.y = v1.y / v2.y,
		dst.z = v1.z / v2.z,
		dst.w = v1.w / v2.w;
	}

	// dst = v1 / s (4-element)
	SVECINLINE void vector4::div4(vector4& dst, const vector4& v1, float s)
	{
//		SVECTORDIV4S( dst, v1, s );
		dst = v1;
		dst.div4(s);
	}


	// dst = v1 * s (4-element)
	SVECINLINE void vector4::scale4(vector4& dst, const vector4& v1, float s)
	{
		SVECTORMUL4S( dst, v1, s );
	}
*/

  ////////////////////////////////////////////////////
  ////////////////////////////////////////////////////

  SVECINLINE void vector4::applyEulerImplicit(vector4 &r, vector4 &v, const vector4 &a, float fScalea, float dt)
  {
    vector4::addScaled(v, v, a, (dt * fScalea));
    vector4::addScaled(r, r, v, dt);
  }

  ////////////////////////////////////////////////////
  ////////////////////////////////////////////////////

  SVECINLINE void vector4::triangleEdges(vector4 *pEdge, const vector4 *pTriangleVertex)
  {
    SVECTORSUB(pEdge[0], pTriangleVertex[2], pTriangleVertex[1]);
    SVECTORSUB(pEdge[1], pTriangleVertex[0], pTriangleVertex[2]);
    SVECTORSUB(pEdge[2], pTriangleVertex[1], pTriangleVertex[0]);
  }

  // given a point vPoint, compute vectors from each triangle vertex to that point
  // ri = vPoint - pi
  SVECINLINE void vector4::triangleVectors(vector4 *pVector, const vector4 *pTriangleVertex, const vector4 &vPoint)
  {
    SVECTORSUB(pVector[0], vPoint, pTriangleVertex[0]);
    SVECTORSUB(pVector[1], vPoint, pTriangleVertex[1]);
    SVECTORSUB(pVector[2], vPoint, pTriangleVertex[2]);
  }

  /*
	// the above functions (triangleEdges and triangleVectors), combined
	SVECINLINE void vector4::triangleEdgesAndVectors( vector4* pEdge, vector4* pVector, const vector4* pTriangleVertex, const vector4& vPoint )
	{
		SVECTORSUB(pEdge[0], pTriangleVertex[2], pTriangleVertex[1]);
		SVECTORSUB(pEdge[1], pTriangleVertex[0], pTriangleVertex[2]);
		SVECTORSUB(pEdge[2], pTriangleVertex[1], pTriangleVertex[0]);

		SVECTORSUB(pVector[0], vPoint, pTriangleVertex[0]);
		SVECTORSUB(pVector[1], vPoint, pTriangleVertex[1]);
		SVECTORSUB(pVector[2], vPoint, pTriangleVertex[2]);
	}
*/

  // compute linear velocity given linear positions and a time step
  // fDeltaSeconds must be non-zero
  SVECINLINE void vector4::instantaneousVelocity(vector4 &vVLinear, const vector4 &v0, const vector4 &v1, float fDeltaSeconds)
  {
    SASSERT(fabsf(fDeltaSeconds) > 1e-8f);

    vector4 vDelta;
    float fInvDeltaSeconds = recipf(fDeltaSeconds);

    vector4::sub(vVLinear, v1, v0);
    vVLinear *= fInvDeltaSeconds;
  }

} // namespace SMath

#include "vector4-ref-inline.h"

#endif // STEVESCH_MATHVEC_INTERNAL_VECTOR4_INLINE_H_
