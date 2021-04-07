#ifndef STEVESCH_MATHVEC_INTERNAL_QUAT_H_
#define STEVESCH_MATHVEC_INTERNAL_QUAT_H_
// Copyright © 2002, PulseCode Interactive LLC, All Rights Reserved
// inline functions for stevesch::quat

namespace stevesch
{

  // used with magnitudes
  const float c_fSQuatUnityTolerance = 0.0001f;

  // used with squared magnitudes
  const float c_fSQuatUnityTolerance2 = c_fSQuatUnityTolerance * c_fSQuatUnityTolerance;

  // initialize as an identity quaternion
  SQUATINLINE void stevesch::quat::identity()
  {
    // (cos(0), v sin(0)) == (1, <0, 0, 0>)
    Z() = Y() = X() = 0.0f;
    A() = 1.0f;
  }

  SQUATINLINE bool stevesch::quat::operator==(const stevesch::quat &crOther) const
  {
    return (m_vQuat == crOther.m_vQuat);
  }

  SQUATINLINE bool stevesch::quat::operator!=(const stevesch::quat &crOther) const
  {
    return (m_vQuat != crOther.m_vQuat);
  }

  SQUATINLINE bool stevesch::quat::isIdentity() const
  {
    return (m_vQuat == c_vZERO);
  }

  // initialize as an identity quaternion
  SQUATINLINE stevesch::quat::stevesch::quat(bool bDummy)
  {
    identity();
  }

  SQUATINLINE stevesch::quat::stevesch::quat(const stevesch::vector4 &vAxis, float fAngle)
  {
    fromAxisAngle(vAxis, fAngle);
  }

  SQUATINLINE stevesch::quat::stevesch::quat(float x, float y, float z, float a) : m_vQuat(x, y, z, a)
  {
  }

  SQUATINLINE stevesch::quat::stevesch::quat(const stevesch::quat &q)
  {
    V() = q.V(); // use vector cast
  }

  SQUATINLINE stevesch::quat::stevesch::quat(const stevesch::matrix4 &m)
  {
    fromMatrix(m);
  }

  SQUATINLINE stevesch::quat::stevesch::quat(const stevesch::vector3 &vEuler)
  {
    fromEuler(vEuler.x, vEuler.y, vEuler.z);
  }

  SQUATINLINE stevesch::quat::stevesch::quat(float x, float y, float z)
  {
    fromEuler(x, y, z);
  }

  SQUATINLINE void stevesch::quat::fromEuler(const stevesch::vector3 &vEuler)
  {
    fromEuler(vEuler.x, vEuler.y, vEuler.z);
  }

  // (t, v) = (t, -v)
  SQUATINLINE void stevesch::quat::conj()
  {
    V().negate(); // negates (*= -1) x, y, and z
  }

  // dst(t, v) = (t, -v)
  SQUATINLINE void stevesch::quat::conj(stevesch::quat &dst) const
  {
    V().negate(dst.V()); // negates (*= -1) x, y, and z
  }

  // Adjunct-- (q*)[(q)(q*)]
  SQUATINLINE void stevesch::quat::adj()
  {
    float a = norm();
    conj();
    *this *= a;
  }

  SQUATINLINE void stevesch::quat::adj(stevesch::quat &dst) const
  {
    float a = norm();
    conj(dst);
    dst *= a;
  }

  // (q)(q*) == t*t + v.v
  SQUATINLINE float stevesch::quat::norm() const
  {
    float a = V().squareMag4(); // t*t + v.v
    return a;
  }

  // 1.0 / (q)(q*) == 1.0 / (t*t + v.v)
  SQUATINLINE float stevesch::quat::invNorm() const
  {
    return V().recipSquareMag4();
    //		return recipf( norm() );
  }

  // sqrt(t*t + v.v)
  SQUATINLINE float stevesch::quat::abs() const
  {
    return V().abs4();
    //		return S::Sqrtf(norm());
  }

  // 1.0 / sqrt(t*t + v.v)
  SQUATINLINE float stevesch::quat::invAbs() const
  {
    return V().recipAbs4();
    //		return S::RSqrtf(norm());	// 1.0 / sqrt(norm())
  }

  // (t*t + v.v)^2
  SQUATINLINE float stevesch::quat::det() const
  {
    float a = norm();
    a *= a;
    return a;
  }

  // 1.0 / [(t*t + v.v)^2]
  SQUATINLINE float stevesch::quat::invDet() const
  {
    float a = norm();
    a = recipf(a * a);
    return a;
  }

  SQUATINLINE stevesch::quat &stevesch::quat::operator+=(const stevesch::quat &r)
  {
    V().add4(r.V());
    return *this;
  }

  SQUATINLINE stevesch::quat &stevesch::quat::operator-=(const stevesch::quat &r)
  {
    V().sub4(r.V());
    return *this;
  }

  SQUATINLINE stevesch::quat &stevesch::quat::operator*=(const stevesch::quat &r)
  {
    return postMul_Norm(r);
  }

  SQUATINLINE void stevesch::quat::mul(stevesch::quat &dst, const stevesch::quat &q1, const stevesch::quat &q2)
  {
    mul_Norm_Norm(dst, q1, q2);
  }

  // multiply all components by a scalar
  SQUATINLINE stevesch::quat &stevesch::quat::operator*=(float fScale)
  {
    V().mul4(fScale);
    return *this;
  }

  SQUATINLINE stevesch::quat &stevesch::quat::operator=(const stevesch::quat &q)
  {
    V() = q.V(); // use vector cast
    return *this;
  }

  // multiply all components by a scalar
  // [same as *=(scalar)]
  SQUATINLINE stevesch::quat &stevesch::quat::scale(float fScale)
  {
    V().mul4(fScale);
    return *this;
  }

  // q* / [(q)(q*)] == (t, -v) / (t*t + v.v)
  SQUATINLINE void stevesch::quat::invert()
  {
    float fInvMag = invAbs(); // 1.0 / t*t + v.v
    conj();

    // NOT NECESSARY FOR NORMALIZED QUATERNIONS-- TO BE OPTIMIZED:
    *this *= fInvMag;
  }

  SQUATINLINE void stevesch::quat::invert(stevesch::quat &dst) const
  {
    float fInvMag = invAbs(); // 1.0 / t*t + v.v
    conj(dst);

    // NOT NECESSARY FOR NORMALIZED QUATERNIONS-- TO BE OPTIMIZED:
    dst *= fInvMag;
  }

  // power (self)-- this = this^q
  // q1^q2 = exp[ ln(q1) x q2 ]
  SQUATINLINE void stevesch::quat::pow(const stevesch::quat &q)
  {
    pow(*this, *this, q); // must use temporary variable anyway in case q is 'this'
  }

  // power-- dst = q1^q2
  // q1^q2 = exp[ ln(q1) x q2 ]
  SQUATINLINE void stevesch::quat::pow(stevesch::quat &dst, const stevesch::quat &q1, const stevesch::quat &q2)
  {
    stevesch::quat qTemp;
    q1.ln(qTemp);
    qTemp.cross(q2);
    qTemp.exp(dst);
  }

  // power (self)-- this = this^fPower
  // q1^y = exp[ y ln(q1) ]
  SQUATINLINE void stevesch::quat::pow(float fPower)
  {
    ln();
    scale(fPower);
    exp();
  }

  // normalize (self)-- this = this/|this|
  SQUATINLINE void stevesch::quat::normalize()
  {
    V().normalize4();
    //		V().mul4( invAbs() );
  }

  // normalize-- dst = this/|this|
  SQUATINLINE void stevesch::quat::normalize(stevesch::quat &dst)
  {
    stevesch::vector4::scale4(dst.V(), V(), invAbs());
  }

  // cross product (self)-- this = this x q (Grassman outer product)
  SQUATINLINE void stevesch::quat::cross(const stevesch::quat &q)
  {
    grassmanOdd(*this, *this, q);
  }

  // dot product (self)-- this = this . q (Euclidean inner product)
  SQUATINLINE void stevesch::quat::dotQuat(const stevesch::quat &q)
  {
    euclideanEven(*this, *this, q);
  }

  // Euclidean inner (even) product, returning real scalar value
  SQUATINLINE float stevesch::quat::dot(const stevesch::quat &q) const
  {
    return stevesch::vector4::dot4(V(), q.V());
  }

  // dst = q * src * ~q
  SQUATINLINE void stevesch::quat::rotate(stevesch::vector4 &dst, const stevesch::vector4 &src) const
  {
    stevesch::quat q1(src.x, src.y, src.z, 0.0f);
    //		stevesch::quat q2;
    mul(q1, *this, q1);           // q1 = q*src
    mul_Norm_Conj(q1, q1, *this); // q1 = q * src * q^-1
                                  //		conj(q2);
                                  //		mul(q1, q1, q2);	// q1 = q1*q' == q*src*q'
    dst.set(q1.X(), q1.Y(), q1.Z(), src.w);
  }

  ////////////////////////////////////////////////////////////////

  // dst = q1 * q2 (standard product)
  // (q1)(q2) (standard product)
  // == ( t1*t2 - v1.v2, t1*v2 + v1*t2 + [v1 x v2] )
  SQUATINLINE void stevesch::quat::mul_Norm_Norm(stevesch::quat &dst, const stevesch::quat &q1, const stevesch::quat &q2)
  {
    stevesch::vector4 v;
    float a = q1.A() * q2.A() - stevesch::vector4::dot(q1.V(), q2.V());
    stevesch::vector4::addScaled(v, q2.V(), q1.A(), q1.V(), q2.A()); // t1*v2 + v1*t2
    stevesch::vector4::cross(dst.V(), q1.V(), q2.V());               // may alter q1 or q2 if dst is q1 or q2
                                                                     // but OK because we don't use q1 or q2 again
    stevesch::vector4::add(dst.V(), dst.V(), v);                     // dst = (t1*v2 + v1*t2) + (v1 x v2)
    dst.A() = a;
  }

  // dst = q1~ * q2 (euclidean product)
  // == ( t1*t2 + v1.v2, t1*v2 - v1*t2 - [v1 x v2] )
  SQUATINLINE void stevesch::quat::mul_Conj_Norm(stevesch::quat &dst, const stevesch::quat &q1, const stevesch::quat &q2)
  {
    stevesch::vector4 v;
    float a = q1.dot(q2);                                             // t1*t2 + v1.v2
    stevesch::vector4::addScaled(v, q2.V(), q1.A(), q1.V(), -q2.A()); // t1*v2 - v1*t2
    stevesch::vector4::cross(dst.V(), q1.V(), q2.V());                // may alter q1 or q2 if dst is q1 or q2
                                                                      // but OK because we don't use q1 or q2 again
    stevesch::vector4::sub(dst.V(), v, dst.V());                      // dst = (t1*v2 - v1*t2) - (v1 x v2)
    dst.A() = a;
  }

  // dst = q1 * q2~
  // == ( t1*t2 + v1.v2, -t1*v2 + v1*t2 - [v1 x v2] )
  SQUATINLINE void stevesch::quat::mul_Norm_Conj(stevesch::quat &dst, const stevesch::quat &q1, const stevesch::quat &q2)
  {
    stevesch::vector4 v;
    float a = q1.dot(q2);                                             // t1*t2 + v1.v2
    stevesch::vector4::addScaled(v, q2.V(), -q1.A(), q1.V(), q2.A()); // -t1*v2 + v1*t2
    stevesch::vector4::cross(dst.V(), q1.V(), q2.V());                // may alter q1 or q2 if dst is q1 or q2
                                                                      // but OK because we don't use q1 or q2 again
    stevesch::vector4::sub(dst.V(), v, dst.V());                      // dst = (v1*t2 - t1*v2) - (v1 x v2)
    dst.A() = a;
  }

  // dst = q1~ * q2~
  // == ( t1*t2 - v1.v2, -t1*v2 - v1*t2 + [v1 x v2] )
  SQUATINLINE void stevesch::quat::mul_Conj_Conj(stevesch::quat &dst, const stevesch::quat &q1, const stevesch::quat &q2)
  {
    stevesch::vector4 v;
    float a = q1.A() * q2.A() - stevesch::vector4::dot(q1.V(), q2.V()); // t1*t2 - v1.v2
    stevesch::vector4::addScaled(v, q2.V(), q1.A(), q1.V(), q2.A());    // t1*v2 + v1*t2
    stevesch::vector4::cross(dst.V(), q1.V(), q2.V());                  // may alter q1 or q2 if dst is q1 or q2
                                                                        // but OK because we don't use q1 or q2 again
    stevesch::vector4::sub(dst.V(), dst.V(), v);                        // dst = (v1 x v2) - (t1*v2 + v1*t2)
    dst.A() = a;
  }

  // (*this) = (*this) * qRight
  SQUATINLINE stevesch::quat &stevesch::quat::postMul_Norm(const stevesch::quat &qRight)
  {
    stevesch::vector4 v;
    float a = A() * qRight.A() - stevesch::vector4::dot(V(), qRight.V());
    stevesch::vector4::addScaled(v, qRight.V(), A(), V(), qRight.A()); // t1*v2 + v1*t2
    V().cross(qRight.V());
    V().add(v); // dst = (t1*v2 + v1*t2) + (v1 x v2)
    A() = a;
    return *this;
  }

  // (*this) = qLeft * (*this)
  SQUATINLINE stevesch::quat &stevesch::quat::preMul_Norm(const stevesch::quat &qLeft)
  {
    stevesch::vector4 v;
    float a = qLeft.A() * A() - stevesch::vector4::dot(qLeft.V(), V());
    stevesch::vector4::addScaled(v, V(), qLeft.A(), qLeft.V(), A()); // t1*v2 + v1*t2
    stevesch::vector4::cross(V(), qLeft.V(), V());
    V().add(v); // dst = (t1*v2 + v1*t2) + (v1 x v2)
    A() = a;
    return *this;
  }

  // (*this) = *this * qRight~
  SQUATINLINE stevesch::quat &stevesch::quat::postMul_Conj(const stevesch::quat &qRight)
  {
    stevesch::vector4 v;
    float a = dot(qRight);                                              // t1*t2 + v1.v2
    stevesch::vector4::addScaled(v, qRight.V(), -A(), V(), qRight.A()); // -t1*v2 + v1*t2
    V().cross(qRight.V());
    stevesch::vector4::sub(V(), v, V()); // dst = (v1*t2 - t1*v2) - (v1 x v2)
    A() = a;
    return *this;
  }

  // (*this) = qLeft~ * (*this)
  SQUATINLINE stevesch::quat &stevesch::quat::preMul_Conj(const stevesch::quat &qLeft)
  {
    stevesch::vector4 v;
    float a = dot(qLeft);                                             // t1*t2 + v1.v2
    stevesch::vector4::addScaled(v, V(), qLeft.A(), qLeft.V(), -A()); // t1*v2 - v1*t2
    V().cross(qLeft.V());
    V().add(v); // dst = (t1*v2 - v1*t2) + (v2 x v1) (== -v1 x v2)
    A() = a;
    return *this;
  }

  ////////////////////////////////////////////////////////////////

  // (q1)(q2) (standard product)
  // == ( t*t' - v.v', t*v' + v*t' + [v x v'] )
  SQUATINLINE void stevesch::quat::grassmanProduct(stevesch::quat &dst, const stevesch::quat &q1, const stevesch::quat &q2)
  {
    mul_Norm_Norm(dst, q1, q2);
  }

  // [(q1)(q2) + (q2)(q1)] / 2
  // == ( t*t' - v.v', t*v' + v*t' )
  SQUATINLINE void stevesch::quat::grassmanEven(stevesch::quat &dst, const stevesch::quat &q1, const stevesch::quat &q2)
  {
    stevesch::vector4 v;
    stevesch::vector4::scale(v, q1.V(), q2.A());
    stevesch::vector4::addScaled(v, v, q2.V(), q1.A());
    v.w = q1.A() * q2.A() - stevesch::vector4::dot(q1.V(), q2.V());
    dst.V() = v;
  }

  // [(q1)(q2) - (q2)(q1)] / 2
  // == ( 0, [v x v'] )
  // *** Common cross product ***
  SQUATINLINE void stevesch::quat::grassmanOdd(stevesch::quat &dst, const stevesch::quat &q1, const stevesch::quat &q2)
  {
    stevesch::vector4::cross(dst.V(), q1.V(), q2.V());
    dst.A() = 0.0f;
  }

  // (q1*)(q2)
  // == ( t*t' + v.v', t*v' - v*t' - [v x v'] )
  SQUATINLINE void stevesch::quat::euclideanProduct(stevesch::quat &dst, const stevesch::quat &q1, const stevesch::quat &q2)
  {
    mul_Conj_Norm(dst, q1, q2);
  }

  // [(q1*)(q2) + (q2)(q1*)] / 2
  // == ( t*t' + v.v', 0 )
  // *** Common dot product ***
  SQUATINLINE void stevesch::quat::euclideanEven(stevesch::quat &dst, const stevesch::quat &q1, const stevesch::quat &q2)
  {
    dst.A() = stevesch::vector4::dot4(q1.V(), q2.V()); // t*t' + v.v'
    dst.Z() = dst.Y() = dst.X() = 0.0f;
  }

  // [(q1*)(q2) - (q2)(q1*)] / 2
  // == ( 0, t*v' - v*t' - [v x v'] )
  SQUATINLINE void stevesch::quat::euclideanOdd(stevesch::quat &dst, const stevesch::quat &q1, const stevesch::quat &q2)
  {
    stevesch::vector4 v;
    stevesch::vector4::addScaled(v, q2.V(), q1.A(), q1.V(), -q2.A()); // t*v' - v*t'
    stevesch::vector4::cross(dst.V(), q1.V(), q2.V());                // may alter q1 or q2 if dst is q1 or q2
    stevesch::vector4::sub(dst.V(), v, dst.V());                      // dst = (v*t' + t*v') - (v x v')
    dst.A() = 0.0f;
  }

  SQUATINLINE void stevesch::quat::lookAtLHWorld(stevesch::quat &dst, const stevesch::vector4 &rEye, const stevesch::vector4 &rAt,
                                                 const stevesch::vector4 &rUp)
  {
    stevesch::matrix4 mat;
    stevesch::matrix4::lookAtLHWorld(mat, rEye, rAt, rUp);
    dst.fromOrthonormalMatrix(mat);
  }

  SQUATINLINE void stevesch::quat::lookAtRHWorld(stevesch::quat &dst, const stevesch::vector4 &rEye, const stevesch::vector4 &rAt,
                                                 const stevesch::vector4 &rUp)
  {
    stevesch::matrix4 mat;
    stevesch::matrix4::lookAtRHWorld(mat, rEye, rAt, rUp);
    dst.fromOrthonormalMatrix(mat);
  }

  SQUATINLINE void stevesch::quat::lookAtLHView(stevesch::quat &dst, const stevesch::vector4 &rEye, const stevesch::vector4 &rAt,
                                                const stevesch::vector4 &rUp)
  {
    stevesch::matrix4 mat;
    stevesch::matrix4::lookAtLHView(mat, rEye, rAt, rUp);
    dst.fromOrthonormalMatrix(mat);
  }

  SQUATINLINE void stevesch::quat::lookAtRHView(stevesch::quat &dst, const stevesch::vector4 &rEye, const stevesch::vector4 &rAt,
                                                const stevesch::vector4 &rUp)
  {
    stevesch::matrix4 mat;
    stevesch::matrix4::lookAtRHView(mat, rEye, rAt, rUp);
    dst.fromOrthonormalMatrix(mat);
  }

  ////////////////////////////////////////////////////////////////

  // Compute derivative of angular velocity in quaternion form and apply a time step
  // (used by quaternion integration steps, below)
  // *this: in/out, quaternion position
  // w: in, angular velocity
  // dt: time step
  SQUATINLINE void stevesch::quat::integrate(const stevesch::vector3 &w, float dt)
  {
    {
      stevesch::quat qDeltaAngular(w.x, w.y, w.z, 0.0f); // element-by-element initialization (NOT axis-angle)
      qDeltaAngular *= (0.5f * dt);
      qDeltaAngular *= (*this);

      (*this) += qDeltaAngular;
      normalize(); // renormalize :(
    }
  }

} // namespace SMath

#endif // _STEVESCH_MATHVEC_INTERNAL_QUAT_H__
