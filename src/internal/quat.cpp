#include "quat.h"
//
// Copyright © 2002, Stephen Schlueter, All Rights Reserved
//

namespace stevesch
{
  static const float scfQuatSquareMagTolerance = 0.005f;

  const quat c_qIDENTITY(true); // <0 0 0 1>

#if 0
	const matrix4& quat::R4A = matrix4::I;

	const matrix4 quat::R4I(
		 0.0f,	 1.0f,	 0.0f,	 0.0f,
		-1.0f,	 0.0f,	 0.0f,	 0.0f,
		 0.0f,	 0.0f,	 0.0f,	 1.0f,
		 0.0f,	 0.0f,	-1.0f,	 0.0f
		);
	
	const matrix4 quat::R4J(
		 0.0f,	 1.0f,	 0.0f,	 0.0f,
		-1.0f,	 0.0f,	 0.0f,	 0.0f,
		 0.0f,	 0.0f,	 0.0f,	 1.0f,
		 0.0f,	 0.0f,	-1.0f,	 0.0f
		);
	
	const matrix4 quat::R4K(
		 0.0f,	 1.0f,	 0.0f,	 0.0f,
		-1.0f,	 0.0f,	 0.0f,	 0.0f,
		 0.0f,	 0.0f,	 0.0f,	 1.0f,
		 0.0f,	 0.0f,	-1.0f,	 0.0f
		);

	void quat::ToBasisMatrix(matrix4& dst) const
	{
		matrix4 m;
		dst.diag4(A());

		matrix4::mulMembers4x4(m, R4I, X());
		dst *= m;
		
		matrix4::mulMembers4x4(m, R4J, Y());
		dst *= m;

		matrix4::mulMembers4x4(m, R4K, Z());
		dst *= m;
	}
#endif

  // normalize, setting value to identity if near 0
  bool quat::safeNormalize(quat &dst) const
  {
    float fMag2 = norm(); // == V().squareMag4()
    if (fabsf(fMag2 - 1.0f) > scfQuatSquareMagTolerance)
    {
      // not normalized
      if (fabsf(fMag2) < scfQuatSquareMagTolerance)
      {
        // near zero
        dst.identity();
      }
      else
      {
        vector4::mul4(dst.V(), V(), rsqrtf(fMag2));
      }
      return true;
    }

    dst.V() = V();
    return false;
  }

  // normalize, setting value to identity if near 0
  bool quat::safeNormalize()
  {
    float fMag2 = norm(); // == V().squareMag4()
    if (fabsf(fMag2 - 1.0f) > 0.0001f)
    {
      // not normalized
      if (fabsf(fMag2) < 0.0001f)
      {
        // near zero
        identity();
      }
      else
      {
        V().mul4(rsqrtf(fMag2));
      }

      return true;
    }

    return false;
  }

  // angle tolerance considered zero
  static const float cfCosZeroRotationComponent = 0.999999904807f; // cos( 0.05 degrees / 2 )

  float quat::getAngle() const
  {
    float fA = clampf(A(), -1.0f, 1.0f);
    //		float a = acosf (fA);	// a = angle/2
    //		float s = sinf (a);
    //		if (fabsf(s) < 0.0001f)
    if (fabsf(fA) > cfCosZeroRotationComponent)
    {
      return 0.0f;
    }

    float a = acosf(fA); // a = angle/2
    return closeMod2pi(2.0f * a);
  }

  void quat::toAxisAngle(vector4 &vAxis, float &angle) const
  {
    float fA = clampf(A(), -1.0f, 1.0f);
    //		float a = acosf (fA);	// a = angle/2
    //		float s = sinf (a);
    //		if (fabsf(s) < 0.0001f)
    if (fabsf(fA) > cfCosZeroRotationComponent)
    {
      // no rotation
      vAxis.set(0.0f, 0.0f, 1.0f, 1.0f);
      angle = 0.0f;
    }
    else
    {
      float a = acosf(fA); // a = angle/2
      float s = sinf(a);

      vector4::div3(vAxis, V(), s); // axis = v / sin(angle/2)
      angle = closeMod2pi(2.0f * a);
      vAxis.w = 1.0f;
    }
  }

  void quat::fromAxisAngle(const vector4 &vAxis, float angle)
  {
    float a = mod2pi(angle * 0.5f);
    float s = sinf(a);

    vector4::scale3(V(), vAxis, s); // v = sin(angle/2) * axis
    A() = cosf(a);                 // t = cos(angle/2)
  }

  void quat::toEuler(vector4 &vEuler) const
  {
    vector4 vsq;
    vector4::mul4(vsq, V(), V());

#if SCOORDINATE_SYSTEM_IS_NASA_STANDARD_AIRPLANE
    // NASA standard airplane
    // heading (yaw)
    vEuler.z = atan2f(2.0f * (X() * Y() + Z() * A()), (vsq.x + vsq.w - vsq.y - vsq.z));
    // bank (roll)
    vEuler.x = atan2f(2.0f * (Y() * Z() + X() * A()), (vsq.z + vsq.w - vsq.x - vsq.y));
    // attitude (pitch)
    {
      float asin_arg = -2.0f * (X() * Z() - Y() * A());
      SASSERT((asin_arg >= -1.05f) && (asin_arg < 1.05f));

      // sanity check-- we'll allow the magnitude of the value to
      // slightly exceed 1.0 (and we'll clamp to +/- 1.0), but
      // assert when the value is unreasonable.
      asin_arg = clampf(asin_arg, -1.0f, 1.0f);
      vEuler.y = asinf(asin_arg);
    }
#else // y=yaw, x=pitch, z=roll
    // heading
    vEuler.y = atan2f(2.0f * (Z() * X() + Y() * A()), (vsq.z + vsq.w - vsq.x - vsq.y));

    // bank
    vEuler.z = atan2f(2.0f * (X() * Y() + Z() * A()), (vsq.y + vsq.w - vsq.x - vsq.z));

    // attitude
    {
      float asin_arg = -2.0f * (Z() * Y() - X() * A());

      // sanity check-- we'll allow the magnitude of the value to
      // slightly exceed 1.0 (and we'll clamp to +/- 1.0), but
      // assert when the value is unreasonable.
      SASSERT((asin_arg >= -1.05f) && (asin_arg < 1.05f));
      asin_arg = clampf(asin_arg, -1.0f, 1.0f);
      vEuler.x = asinf(asin_arg);
    }
#endif
  }

  void quat::fromEuler(float fRadiansX, float fRadiansY, float fRadiansZ)
  {
    // NOT OPTIMIZED, but for axis clarity:
    quat qx, qy, qz;
    vector4 vx(1.0f, 0.0f, 0.0f);
    vector4 vy(0.0f, 1.0f, 0.0f);
    vector4 vz(0.0f, 0.0f, 1.0f);
    qx.fromAxisAngle(vx, fRadiansX);
    qy.fromAxisAngle(vy, fRadiansY);
    qz.fromAxisAngle(vz, fRadiansZ);

    // here's where computation becomes order-dependent:
    // axis order should be roll, pitch, yaw
    // (when using column vectors on left side of matrix)

#if SCOORDINATE_SYSTEM_IS_NASA_STANDARD_AIRPLANE
    // NASA standard airplane (xyz)
    // final form is q = qz*qy*qx, where v' = q * <v, 0> * q~
    qz *= qy;
    qz *= qx;
    V() = qz.V();
#else
    // zxy (q = qy*qx*qz)
    qy *= qx;
    qy *= qz;
    V() = qy.V();
#endif
  }

  // Convert from quaternion to matrix
  matrix4 &quat::toMatrix(matrix4 &m) const
  {
    if (isIdentity())
    {
      m.identity();
    }
    else
    {
      vector4 rs;
      vector4 rx, ry, rz, rw;
      //			float nq = V().squareMag4();
      //			float s = (nq > 0.0f) ? (2.0f / nq) : 0.0f;
      const float s = 2.0f;
#ifdef SDEBUG_EXTRA
      float fMag2 = V().squareMag4();
      SASSERT(fabsf(fMag2 - 1.0f) < scfQuatSquareMagTolerance);
#endif

      vector4::scale3(rs, V(), s);
      vector4::scale3(rx, rs, X());
      vector4::scale3(ry, rs, Y());
      ry.x = 0.0f;

      rz.z = rs.z * Z();
      rz.y = rz.x = 0.0f;

      vector4::scale3(rw, rs, A());

      // The following is the equivalent of (t==theta)
      //
      // I + (sin(t))*S + (1 - cos(t))*S*S
      // (Rodrigues' rotation formula)
      //
      // where
      //     [ 0 -z  y  0 ]
      // S = [ z  0 -x  0 ]
      //     [-y  x  0  0 ]
      //     [ 0  0  0  1 ]
      //
      // and
      //  identities 2sin^2(t/2) = 1-cos(t), sin(t) = 2sin(t/2)cos(t/2) yield
      //	2wx = sin(t)w0, 2wy = sin(t)w1, 2wz = sin(t)w2
      //	2xx = (1-cos(t))w0w0, 2xy = (1-cos(t))w0w1, 2xz = (1-cos(t))w0w2
      //	2yy = (1-cos(t))w1w1, 2yz = (1-cos(t))w1w2, 2zz = (1-cos(t))w2w2
      //

      m.m00 = 1.0f - (ry.y + rz.z);
      m.m10 = rx.y + rw.z;
      m.m20 = rx.z - rw.y;
      m.m30 = 0.0f;

      m.m01 = rx.y - rw.z;
      m.m11 = 1.0f - (rx.x + rz.z);
      m.m21 = ry.z + rw.x;
      m.m31 = 0.0f;

      m.m02 = rx.z + rw.y;
      m.m12 = ry.z - rw.x;
      m.m22 = 1.0f - (rx.x + ry.y);
      m.m32 = 0.0f;

      m.m03 = 0.0f;
      m.m13 = 0.0f;
      m.m23 = 0.0f;
      m.m33 = 1.0f;
    }

    return m;
  }

  // Convert from quaternion and translation to matrix
  matrix4 &quat::toMatrix(matrix4 &m, const vector3 &vTranslation) const
  {
    if (isIdentity())
    {
      m = matrix4(vTranslation);
    }
    else
    {
      vector4 rs;
      vector4 rx, ry, rz, rw;
      //		float nq = V().squareMag4();
      //		float s = (nq > 0.0f) ? (2.0f / nq) : 0.0f;
      const float s = 2.0f;
#ifdef SDEBUG_EXTRA
      float fMag2 = V().squareMag4();
      SASSERT(fabsf(fMag2 - 1.0f) < scfQuatSquareMagTolerance);
#endif

      vector4::scale3(rs, V(), s);
      vector4::scale3(rx, rs, X());
      vector4::scale3(ry, rs, Y());
      ry.x = 0.0f;

      rz.z = rs.z * Z();
      rz.y = rz.x = 0.0f;

      vector4::scale3(rw, rs, A());

      // see toMatrix(m) for an explaination of the following values
      m.m00 = 1.0f - (ry.y + rz.z);
      m.m10 = rx.y + rw.z;
      m.m20 = rx.z - rw.y;
      m.m30 = 0.0f;

      m.m01 = rx.y - rw.z;
      m.m11 = 1.0f - (rx.x + rz.z);
      m.m21 = ry.z + rw.x;
      m.m31 = 0.0f;

      m.m02 = rx.z + rw.y;
      m.m12 = ry.z - rw.x;
      m.m22 = 1.0f - (rx.x + ry.y);
      m.m32 = 0.0f;

      m.m03 = vTranslation.x;
      m.m13 = vTranslation.y;
      m.m23 = vTranslation.z;
      m.m33 = 1.0f;
    }

    return m;
  }

  /*	
	void CMatrix3x3::RotationFromQuaternion(const CQuaternion& Q)
	{
		real two_x = Q.x * REAL(2);
		real two_y = Q.y * REAL(2);
		real two_z = Q.z * REAL(2);
		// fill in the non-diagonal entries first
		_12 = _21 = two_x * Q.y;
		_23 = _32 = two_y * Q.z;
		_31 = _13 = two_z * Q.x;
		real t = Q.w * two_x;
		_23 -= t; _32 += t;
		t = Q.w * two_y;
		_31 -= t; _13 += t;
		t = Q.w * two_z;
		_12 -= t; _21 += t;
		// fill in the diagonals
		_11 = _22 = _33 = REAL(1);
		t = two_x * Q.x;
		_22 -= t; _33 -= t;
		t = two_y * Q.y;
		_11 -= t; _33 -= t;
		t = two_z * Q.z;
		_11 -= t; _22 -= t;
	}
*/

  void quat::fromOrthonormalMatrix(const matrix4 &m)
  {
    float fTrace;
    float s;

    fTrace = m[0][0] + m[1][1] + m[2][2];

    if (fTrace > 0.0f)
    {
      s = sqrtf(fTrace + 1.0f);

      A() = (s * 0.5f);
      s = 0.5f / s;

      X() = (m[1][2] - m[2][1]);
      Y() = (m[2][0] - m[0][2]);
      Z() = (m[0][1] - m[1][0]);
      V().mul3(s); // 3-element
    }
    else
    {
      int i, j, k;

      i = 0;
      if (m[1][1] > m[0][0])
      {
        i = 1;
      }
      if (m[2][2] > m[i][i])
      {
        i = 2;
      }

      j = (i + 1) % 3;
      k = (j + 1) % 3;

      s = sqrtf(m[i][i] - m[j][j] - m[k][k] + 1.0f);

      V()
      [i] = (s * 0.5f);
      s = 0.5f / s;
      V()
      [3] = (m[j][k] - m[k][j]) * s;
      V()
      [j] = (m[i][j] + m[j][i]) * s;
      V()
      [k] = (m[i][k] + m[k][i]) * s;
    }

#ifdef SDEBUG
    quat qn;
    if (safeNormalize(qn))
    {
      static int snWarn = 5;
      if (snWarn > 0)
      {
        snWarn--;
        STrace(SWARNSTR "quat::fromOrthonormalMatrix passed non-othonormal matrix (will warn %d more times)\n",
               snWarn);
      }
    }
#endif
  }

  void quat::fromMatrix(const matrix4 &src)
  {
    matrix4 m;
    src.normalizeCol3x3(m); // copy src to m and normalize the 3x3 columns of m
    fromOrthonormalMatrix(m);

    /*
		float fTrace;
		float s;

		matrix4 m;
		matrix4::transpose(m, src);
		m.col[0].normalize();
		m.col[1].normalize();
		m.col[2].normalize();

		fTrace = m[0][0] + m[1][1] + m[2][2];
		
		if (fTrace > 0.0f)
		{
			s = sqrtf(fTrace + 1.0f);
			
			A() = (s * 0.5f);
			s = 0.5f / s;
			
			X() = (m[2][1] - m[1][2]);
			Y() = (m[0][2] - m[2][0]);
			Z() = (m[1][0] - m[0][1]);
			V().mul(s);		// 3-element
		}
		else
		{
			int i, j, k;
			
			i = 0;
			if (m[1][1] > m[0][0])
			{
				i = 1;
			}
			if (m[2][2] > m[i][i])
			{
				i = 2;
			}
			
			j = (i+1) % 3;
			k = (j+1) % 3;
			
			s = sqrtf (m[i][i] - m[j][j] - m[k][k] + 1.0f);
			
			V()[i] = (s * 0.5f);
			s = 0.5f / s;
			V()[3] = (m[k][j] - m[j][k]) * s;
			V()[j] = (m[j][i] + m[i][j]) * s;
			V()[k] = (m[k][i] + m[i][k]) * s;
		}

#ifdef SDEBUG
		quat qn;
		SASSERT( !safeNormalize( qn ) );
#endif
		//return *this;
*/
  }

  void quat::slerp(quat &dst, const quat &q1, const quat &q2, float t)
  {
    // slerp(p, q, t) = (q1*sin((1-t)*W) + q2*sin(t*W)) / sin(W), where W = acos(q1.q2)
    vector4 quat2(q2.V());
    float fCosOmega = q1.dot(q2); // full quaternion dot-product (4-element)

    if (fCosOmega < 0.0f)
    {
      fCosOmega = -fCosOmega;
      quat2.negate4();
    }

    if (fCosOmega < (1.0f - cfQuatSlerpLinearEpsilon))
    {
      // use spherical linear interpolation for larger angles
      float fOmega = acosf(fCosOmega);
      float fSinOmegaRecip = recipf(sinf(fOmega));
      float fOmegat = fOmega * t;
      float fScale1 = sinf(fOmega - fOmegat) * fSinOmegaRecip; // sine((1-t)*W) / sine(W)
      float fScale2 = sinf(fOmegat) * fSinOmegaRecip;          // sine(W*t) / sine(W)

      vector4::addScaled4(dst.V(), q1.V(), fScale1, quat2, fScale2);

#ifdef SDEBUG_EXTRA
      float fMag2 = dst.V().squareMag4();
      SASSERT(fabsf(fMag2 - 1.0f) < scfQuatSquareMagTolerance);
#endif
    }
    else
    {
      // use direct linear interpolation for small angles
      vector4::lerp4(dst.V(), q1.V(), quat2, t); // dst = (1-t)*q1 + t*q2
#ifdef SDEBUG_EXTRA
      float fMag2 = dst.V().squareMag4();
      SASSERT(fabsf(fMag2 - 1.0f) < scfQuatSquareMagTolerance);
#endif
    }
  }

  // compute angular velocity given angular positions and a time step
  // fDeltaSeconds must be non-zero
  void quat::instantaneousVelocity(vector4 &vVAngular, const quat &q0, const quat &q1, float fDeltaSeconds)
  {
    SASSERT(fabsf(fDeltaSeconds) > 1e-8f);

    quat qDelta;
    float fInvDeltaSeconds = recipf(fDeltaSeconds);
    float fAngle;

    //		q0.conj(qDelta);	// inverse, but we'll assume already normalized
    //		quat::mul( qDelta, q1, qDelta );
    quat::mul_norm_conj(qDelta, q1, q0); // qDelta = q1 * q0~

    qDelta.toAxisAngle(vVAngular, fAngle);
    vVAngular.mul3(fAngle * fInvDeltaSeconds);
  }

  // compute linear and angular velocities given linear and angular positions and a time step
  // fDeltaSeconds must be non-zero
  void quat::instantaneousVelocities(
      vector4 &vVLinear, vector4 &vVAngular,
      const vector4 &v0, const quat &q0,
      const vector4 &v1, const quat &q1,
      float fDeltaSeconds)
  {
    SASSERT(fabsf(fDeltaSeconds) > 1e-8f);

    quat qDelta;
    float fInvDeltaSeconds = recipf(fDeltaSeconds);
    float fAngle;

    //		q0.conj(qDelta);	// inverse, but we'll assume already normalized
    //		quat::mul( qDelta, q1, qDelta );
    quat::mul_norm_conj(qDelta, q1, q0); // qDelta = q1 * q0~

    qDelta.toAxisAngle(vVAngular, fAngle);
    vVAngular.mul3(fAngle * fInvDeltaSeconds);

    vector4::sub3(vVLinear, v1, v0);
    vVLinear.mul3(fInvDeltaSeconds);
  }

  /*
	// non-inline version of quaternion integrator
	void quat::integrate(const vector4& w, float dt)
	{
		quat qDeltaA, qFinalA;
		quat qDeltaB, qFinalB;

		vector4 vOmega;
		vector4 vAxis;
		float fAngle;

		vector4::scale( vOmega, w, dt );
		fAngle = vOmega.abs();

		if (fabsf(fAngle) > 1.0e-6f)
		{
			vector4::scale( vAxis, vOmega, 1.0f / fAngle );

			fAngle = mod2pi( fAngle );

			qDeltaA.fromAxisAngle( vAxis, fAngle );

			quat::mul( qFinalA, qDeltaA, *this );
//			quat::mul( qFinalA, *this, qDeltaA );
			//(*this) *= qDelta;
			qFinalA.normalize();
		}
		else
		{
			qFinalA = *this;
		}

		qDeltaB = quat(w.x, w.y, w.z, 0.0f);	// element-by-element initialization (NOT axis-angle)
		qDeltaB *= (0.5f*dt);
		qDeltaB *= (*this);
		
		(*this) += qDeltaB;
		normalize();		// renormalize :(
		qFinalB = *this;

		*this = qFinalA;
	}
*/

  // natural log (self)
  // (0.5 ln(t*t + v.v), atan(|v|/t)(v/|v|))
  void quat::ln()
  {
    float fVectorSquareMag = V().squareMag3();

    if (fabsf(fVectorSquareMag - 1.0f) < c_fSQuatUnityTolerance2)
    {
      // quaternion is normalized
      V().mul3(atan2f(1.0f, A())); // v = v * atan(1/t)
      A() = 0.0f;               // ln(1) == 0
    }
    else
    {
      // quaternion is not normalized
      float fVectorMag = sqrtf(fVectorSquareMag);
      float a = 0.5f * logf(A() * A() + fVectorSquareMag); // a = (1/2) ln(t*t + v.v)

      V().mul3(atan2f(fVectorMag, A()) / fVectorMag); // v *= atan(1/t) / |v|
      A() = a;
    }
  }

  // natural log
  // dst = (0.5 ln(t*t + v.v), atan(|v|/t)(v/|v|))
  void quat::ln(quat &dst) const
  {
    float fVectorSquareMag = V().squareMag3(); // |v|*|v|

    if (fabsf(fVectorSquareMag - 1.0f) < c_fSQuatUnityTolerance2)
    {
      // quaternion is normalized
      vector4::scale3(dst.V(), V(), atan2f(1.0f, A())); // v = v * atan(1/t)
      dst.A() = 0.0f;                                  // ln(1) == 0
    }
    else
    {
      // quaternion is not normalized
      float fVectorMag = sqrtf(fVectorSquareMag);
      float a = 0.5f * logf(A() * A() + fVectorSquareMag); // a = (1/2) ln(t*t + v.v)

      vector4::scale3(dst.V(), V(), atan2f(fVectorMag, A()) / fVectorMag); // v *= atan(1/t) / |v|
      dst.A() = a;
    }
  }

  // exponential (self)-- e^(this)
  void quat::exp()
  {
    float fVectorMag = V().abs3();
    float fExpT = expf(A());
    float fCosMag;
    float fSinMag;

    // fCosMag = cosine(|v|); fSinMag = sine(|v|)
    cosSinf(fVectorMag, &fCosMag, &fSinMag);

    // assume vector is not normalized-- usually the case, since
    // a normalized quaternion has a normalized xyz component only
    // if the rotation represented is +/- 180 degrees [|sine(theta/2)| == 1]
    SASSERT(fabsf(fVectorMag) > c_fSQuatUnityTolerance);
    V().mul3(fExpT * fSinMag / fVectorMag);

    A() = fExpT * fCosMag;
  }

  // exponential-- dst = e^(this)
  void quat::exp(quat &dst) const
  {
    float fVectorMag = V().abs3();
    float fExpT = expf(A());
    float fCosMag;
    float fSinMag;

    // fCosMag = cosine(|v|); fSinMag = sine(|v|)
    cosSinf(fVectorMag, &fCosMag, &fSinMag);

    // assume vector is not normalized-- usually the case, since
    // a normalized quaternion has a normalized xyz component only
    // if the rotation represented is +/- 90 degrees [|sine(theta)| == 1]
    SASSERT(fabsf(fVectorMag) > c_fSQuatUnityTolerance);
    vector4::scale3(dst.V(), V(), fExpT * fSinMag / fVectorMag);

    dst.A() = fExpT * fCosMag;
  }

  // construct q such that v0 transformed by q equals v1
  // returns 'false' if v0 is "close enough" (q is unchanged)
  float quat::fromTwoVectors(quat &q, const vector4 &v0, const vector4 &v1, float fTolerance)
  {
    vector4 v0norm;
    vector4 v1norm;
    vector4::normalize3(v0norm, v0);
    vector4::normalize3(v1norm, v1);

    return fromTwoNormVectors(q, v0norm, v1norm, fTolerance);
  }

  float quat::fromTwoNormVectors(quat &q, const vector4 &v0norm, const vector4 &v1norm, float fTolerance)
  {
    vector4 vAxis;
    float fDotProduct = v0norm.dot3(v1norm); // check dot product to see if vectors have negative dot product

    vector4::cross3(vAxis, v0norm, v1norm);

    float fSquareMag = vAxis.squareMag3();
    if (fSquareMag < fTolerance)
    {
      if (fDotProduct < 0.0f)
      {
        // vectors face opposite directions -- choose pi rotation about y axis
        q = quat(0.0f, 1.0f, 0.0f, 0.0f);
        return c_fpi;
      }

      q.identity();
      return 0.0f;
    }

    float fAxisMag = clampf(sqrtf(fSquareMag), 0.0f, 1.0f);
    float fAngle = asinf(fAxisMag);

    // vectors face opposite directions
    if (fDotProduct < 0.0f)
      fAngle = closeMod2pi(c_fpi - fAngle);

    vAxis.div3(fAxisMag);

    q.fromAxisAngle(vAxis, fAngle);

    return fAngle;
  }

  // construct q such that v0 transformed by q equals v1 or -v1
  // returns '0.0f' if v0 is "close enough", otherwise returns angle rotated by resulting q.
  // (aligns v0 to be colinear with v1, but possibly the opposite direction)
  float quat::alignTwoVectors(quat &q, const vector4 &v0, const vector4 &v1, float fTolerance)
  {
    vector4 vAxis;
    vector4 v0norm;
    vector4 v1norm;
    vector4::normalize3(v0norm, v0);
    vector4::normalize3(v1norm, v1);

    float fDotProduct = v0norm.dot3(v1norm); // check dot product to see if vectors have negative dot product

    vector4::cross3(vAxis, v0norm, v1norm);

    float fSquareMag = vAxis.squareMag3();
    if (fSquareMag < fTolerance)
    {
      // may face opposite directions, but that's OK
      q.identity();
      return 0.0f;
    }

    float fAxisMag = clampf(sqrtf(fSquareMag), 0.0f, 1.0f);
    float fAngle = asinf(fAxisMag);

    // choose smaller of two angles
    if (fDotProduct < 0.0f)
      fAngle = -fAngle; // choose q such that v0 is aligned with -v1

    vAxis.div3(fAxisMag);

    q.fromAxisAngle(vAxis, fAngle);

    return fAngle;
  }

}
