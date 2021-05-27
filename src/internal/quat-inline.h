#ifndef STEVESCH_MATHVEC_INTERNAL_QUAT_INLINE_H_
#define STEVESCH_MATHVEC_INTERNAL_QUAT_INLINE_H_
// Copyright © 2002, Stephen Schlueter, All Rights Reserved
// inline functions for quat

namespace stevesch
{

  // used with magnitudes
  const float c_fSQuatUnityTolerance = 0.0001f;

  // used with squared magnitudes
  const float c_fSQuatUnityTolerance2 = c_fSQuatUnityTolerance * c_fSQuatUnityTolerance;

  // initialize as an identity quaternion
  SQUATINLINE void quat::identity()
  {
    // (cos(0), v sin(0)) == (1, <0, 0, 0>)
    Z() = Y() = X() = 0.0f;
    A() = 1.0f;
  }

  SQUATINLINE bool quat::operator==(const quat &crOther) const
  {
    return (m_vQuat == crOther.m_vQuat);
  }

  SQUATINLINE bool quat::operator!=(const quat &crOther) const
  {
    return (m_vQuat != crOther.m_vQuat);
  }

  SQUATINLINE bool quat::isIdentity() const
  {
    return (m_vQuat == c_vZERO);
  }

  // initialize as an identity quaternion
  SQUATINLINE quat::quat(bool bDummy)
  {
    identity();
  }

  SQUATINLINE quat::quat(const stevesch::vector3 &vAxis, float fAngle)
  {
    fromAxisAngle(vAxis, fAngle);
  }

  SQUATINLINE quat::quat(const stevesch::vector4 &vAxis, float fAngle)
  {
    fromAxisAngle(vAxis, fAngle);
  }

  SQUATINLINE quat::quat(float x, float y, float z, float a) : m_vQuat(x, y, z, a)
  {
  }

  SQUATINLINE quat::quat(const quat &q)
  {
    V() = q.V(); // use vector cast
  }

  SQUATINLINE quat::quat(const stevesch::matrix4 &m)
  {
    fromMatrix(m);
  }

  SQUATINLINE quat::quat(const stevesch::vector3 &vEuler)
  {
    fromEuler(vEuler.x, vEuler.y, vEuler.z);
  }

  SQUATINLINE quat::quat(float x, float y, float z)
  {
    fromEuler(x, y, z);
  }

  SQUATINLINE void quat::fromEuler(const stevesch::vector3 &vEuler)
  {
    fromEuler(vEuler.x, vEuler.y, vEuler.z);
  }

  // (t, v) = (t, -v)
  SQUATINLINE quat &quat::conj()
  {
    V().negate3(); // negates (*= -1) x, y, and z
    return *this;
  }

  // dst(t, v) = (t, -v)
  SQUATINLINE void quat::conj(quat &dst) const
  {
    V().negate3(dst.V()); // negates (*= -1) x, y, and z
  }

  // Adjunct-- (q*)[(q)(q*)]
  SQUATINLINE quat &quat::adj()
  {
    float a = norm();
    conj();
    *this *= a;
    return *this;
  }

  SQUATINLINE void quat::adj(quat &dst) const
  {
    float a = norm();
    conj(dst);
    dst *= a;
  }

  // (q)(q*) == t*t + v.v
  SQUATINLINE float quat::norm() const
  {
    float a = V().squareMag4(); // t*t + v.v
    return a;
  }

  // 1.0 / (q)(q*) == 1.0 / (t*t + v.v)
  SQUATINLINE float quat::invNorm() const
  {
    return V().recipSquareMag4();
    //		return recipf( norm() );
  }

  // sqrt(t*t + v.v)
  SQUATINLINE float quat::abs() const
  {
    return V().abs4();
    //		return sqrtf(norm());
  }

  // 1.0 / sqrt(t*t + v.v)
  SQUATINLINE float quat::invAbs() const
  {
    return V().recipAbs4();
    //		return rsqrtf(norm());	// 1.0 / sqrt(norm())
  }

  // (t*t + v.v)^2
  SQUATINLINE float quat::det() const
  {
    float a = norm();
    a *= a;
    return a;
  }

  // 1.0 / [(t*t + v.v)^2]
  SQUATINLINE float quat::invDet() const
  {
    float a = norm();
    a = recipf(a * a);
    return a;
  }

  SQUATINLINE quat &quat::operator+=(const quat &r)
  {
    V().add4(r.V());
    return *this;
  }

  SQUATINLINE quat &quat::operator-=(const quat &r)
  {
    V().sub4(r.V());
    return *this;
  }

  SQUATINLINE quat &quat::operator*=(const quat &r)
  {
    return postMul_norm(r);
  }

  SQUATINLINE void quat::mul(quat &dst, const quat &q1, const quat &q2)
  {
    mul_norm_norm(dst, q1, q2);
  }

  SQUATINLINE quat operator*(const quat& q1, const quat& q2)
  {
    quat q;
    quat::mul(q, q1, q2);
    return q;
  }

  // multiply all components by a scalar
  SQUATINLINE quat &quat::operator*=(float fScale)
  {
    V().mul4(fScale);
    return *this;
  }

  SQUATINLINE quat &quat::operator=(const quat &q)
  {
    V() = q.V(); // use vector cast
    return *this;
  }

  // multiply all components by a scalar
  // [same as *=(scalar)]
  SQUATINLINE quat &quat::scale(float fScale)
  {
    V().mul4(fScale);
    return *this;
  }

  // q* / [(q)(q*)] == (t, -v) / (t*t + v.v)
  SQUATINLINE quat &quat::invert()
  {
    float fInvMag = invAbs(); // 1.0 / t*t + v.v
    conj();

    // NOT NECESSARY FOR NORMALIZED QUATERNIONS-- TO BE OPTIMIZED:
    *this *= fInvMag;
    return *this;
  }

  SQUATINLINE void quat::invert(quat &dst) const
  {
    float fInvMag = invAbs(); // 1.0 / t*t + v.v
    conj(dst);

    // NOT NECESSARY FOR NORMALIZED QUATERNIONS-- TO BE OPTIMIZED:
    dst *= fInvMag;
  }

  // power (self)-- this = this^q
  // q1^q2 = exp[ ln(q1) x q2 ]
  SQUATINLINE quat &quat::pow(const quat &q)
  {
    pow(*this, *this, q); // must use temporary variable anyway in case q is 'this'
    return *this;
  }

  // power-- dst = q1^q2
  // q1^q2 = exp[ ln(q1) x q2 ]
  SQUATINLINE void quat::pow(quat &dst, const quat &q1, const quat &q2)
  {
    quat qTemp;
    q1.ln(qTemp);
    qTemp.cross(q2);
    qTemp.exp(dst);
  }

  // power (self)-- this = this^fPower
  // q1^y = exp[ y ln(q1) ]
  SQUATINLINE quat &quat::pow(float fPower)
  {
    ln();
    scale(fPower);
    return exp();
  }

  // normalize (self)-- this = this/|this|
  SQUATINLINE quat &quat::normalize()
  {
    V().normalize4();
    //		V().mul4( invAbs() );
    return *this;
  }

  // normalize-- dst = this/|this|
  SQUATINLINE void quat::normalize(quat &dst)
  {
    stevesch::vector4::scale4(dst.V(), V(), invAbs());
  }

  // cross product (self)-- this = this x q (Grassman outer product)
  SQUATINLINE quat &quat::cross(const quat &q)
  {
    grassmanOdd(*this, *this, q);
    return *this;
  }

  // dot product (self)-- this = this . q (Euclidean inner product)
  SQUATINLINE quat &quat::dotQuat(const quat &q)
  {
    euclideanEven(*this, *this, q);
    return *this;
  }

  // Euclidean inner (even) product, returning real scalar value
  SQUATINLINE float quat::dot(const quat &q) const
  {
    return stevesch::vector4::dot4(V(), q.V());
  }

  // dst = q * src * ~q
  SQUATINLINE void quat::rotate(stevesch::vector3 &dst, const stevesch::vector3 &src) const
  {
    quat q1(src.x, src.y, src.z, 0.0f);
    //		quat q2;
    mul(q1, *this, q1);           // q1 = q*src
    mul_norm_conj(q1, q1, *this); // q1 = q * src * q^-1
                                  //		conj(q2);
                                  //		mul(q1, q1, q2);	// q1 = q1*q' == q*src*q'
    dst.set(q1.X(), q1.Y(), q1.Z());
  }

  // dst = q * src * ~q
  SQUATINLINE void quat::rotate(stevesch::vector4 &dst, const stevesch::vector4 &src) const
  {
    quat q1(src.x, src.y, src.z, 0.0f);
    //		quat q2;
    mul(q1, *this, q1);           // q1 = q*src
    mul_norm_conj(q1, q1, *this); // q1 = q * src * q^-1
                                  //		conj(q2);
                                  //		mul(q1, q1, q2);	// q1 = q1*q' == q*src*q'
    dst.set(q1.X(), q1.Y(), q1.Z(), src.w);
  }

  ////////////////////////////////////////////////////////////////

  // dst = q1 * q2 (standard product)
  // (q1)(q2) (standard product)
  // == ( t1*t2 - v1.v2, t1*v2 + v1*t2 + [v1 x v2] )
  SQUATINLINE void quat::mul_norm_norm(quat &dst, const quat &q1, const quat &q2)
  {
    stevesch::vector4 v;
    float a = q1.A() * q2.A() - stevesch::vector4::dot3(q1.V(), q2.V());
    stevesch::vector4::addScaled3(v, q2.V(), q1.A(), q1.V(), q2.A()); // t1*v2 + v1*t2
    stevesch::vector4::cross3(dst.V(), q1.V(), q2.V());               // may alter q1 or q2 if dst is q1 or q2
                                                                      // but OK because we don't use q1 or q2 again
    stevesch::vector4::add3(dst.V(), dst.V(), v);                     // dst = (t1*v2 + v1*t2) + (v1 x v2)
    dst.A() = a;
  }

  // dst = q1~ * q2 (euclidean product)
  // == ( t1*t2 + v1.v2, t1*v2 - v1*t2 - [v1 x v2] )
  SQUATINLINE void quat::mul_conj_norm(quat &dst, const quat &q1, const quat &q2)
  {
    stevesch::vector4 v;
    float a = q1.dot(q2);                                              // t1*t2 + v1.v2
    stevesch::vector4::addScaled3(v, q2.V(), q1.A(), q1.V(), -q2.A()); // t1*v2 - v1*t2
    stevesch::vector4::cross3(dst.V(), q1.V(), q2.V());                // may alter q1 or q2 if dst is q1 or q2
                                                                       // but OK because we don't use q1 or q2 again
    stevesch::vector4::sub3(dst.V(), v, dst.V());                      // dst = (t1*v2 - v1*t2) - (v1 x v2)
    dst.A() = a;
  }

  // dst = q1 * q2~
  // == ( t1*t2 + v1.v2, -t1*v2 + v1*t2 - [v1 x v2] )
  SQUATINLINE void quat::mul_norm_conj(quat &dst, const quat &q1, const quat &q2)
  {
    stevesch::vector4 v;
    float a = q1.dot(q2);                                              // t1*t2 + v1.v2
    stevesch::vector4::addScaled3(v, q2.V(), -q1.A(), q1.V(), q2.A()); // -t1*v2 + v1*t2
    stevesch::vector4::cross3(dst.V(), q1.V(), q2.V());                // may alter q1 or q2 if dst is q1 or q2
                                                                       // but OK because we don't use q1 or q2 again
    stevesch::vector4::sub3(dst.V(), v, dst.V());                      // dst = (v1*t2 - t1*v2) - (v1 x v2)
    dst.A() = a;
  }

  // dst = q1~ * q2~
  // == ( t1*t2 - v1.v2, -t1*v2 - v1*t2 + [v1 x v2] )
  SQUATINLINE void quat::mul_conj_conj(quat &dst, const quat &q1, const quat &q2)
  {
    stevesch::vector4 v;
    float a = q1.A() * q2.A() - stevesch::vector4::dot3(q1.V(), q2.V()); // t1*t2 - v1.v2
    stevesch::vector4::addScaled3(v, q2.V(), q1.A(), q1.V(), q2.A());    // t1*v2 + v1*t2
    stevesch::vector4::cross3(dst.V(), q1.V(), q2.V());                  // may alter q1 or q2 if dst is q1 or q2
                                                                         // but OK because we don't use q1 or q2 again
    stevesch::vector4::sub3(dst.V(), dst.V(), v);                        // dst = (v1 x v2) - (t1*v2 + v1*t2)
    dst.A() = a;
  }

  // (*this) = (*this) * qRight
  SQUATINLINE quat &quat::postMul_norm(const quat &qRight)
  {
    stevesch::vector4 v;
    float a = A() * qRight.A() - stevesch::vector4::dot3(V(), qRight.V());
    stevesch::vector4::addScaled3(v, qRight.V(), A(), V(), qRight.A()); // t1*v2 + v1*t2
    V().cross3(qRight.V());
    V().add3(v); // dst = (t1*v2 + v1*t2) + (v1 x v2)
    A() = a;
    return *this;
  }

  // (*this) = qLeft * (*this)
  SQUATINLINE quat &quat::preMul_norm(const quat &qLeft)
  {
    stevesch::vector4 v;
    float a = qLeft.A() * A() - stevesch::vector4::dot3(qLeft.V(), V());
    stevesch::vector4::addScaled3(v, V(), qLeft.A(), qLeft.V(), A()); // t1*v2 + v1*t2
    stevesch::vector4::cross3(V(), qLeft.V(), V());
    V().add3(v); // dst = (t1*v2 + v1*t2) + (v1 x v2)
    A() = a;
    return *this;
  }

  // (*this) = *this * qRight~
  SQUATINLINE quat &quat::postMul_conj(const quat &qRight)
  {
    stevesch::vector4 v;
    float a = dot(qRight);                                               // t1*t2 + v1.v2
    stevesch::vector4::addScaled3(v, qRight.V(), -A(), V(), qRight.A()); // -t1*v2 + v1*t2
    V().cross3(qRight.V());
    stevesch::vector4::sub3(V(), v, V()); // dst = (v1*t2 - t1*v2) - (v1 x v2)
    A() = a;
    return *this;
  }

  // (*this) = qLeft~ * (*this)
  SQUATINLINE quat &quat::preMul_conj(const quat &qLeft)
  {
    stevesch::vector4 v;
    float a = dot(qLeft);                                              // t1*t2 + v1.v2
    stevesch::vector4::addScaled3(v, V(), qLeft.A(), qLeft.V(), -A()); // t1*v2 - v1*t2
    V().cross3(qLeft.V());
    V().add3(v); // dst = (t1*v2 - v1*t2) + (v2 x v1) (== -v1 x v2)
    A() = a;
    return *this;
  }

  ////////////////////////////////////////////////////////////////

  // (q1)(q2) (standard product)
  // == ( t*t' - v.v', t*v' + v*t' + [v x v'] )
  SQUATINLINE void quat::grassmanProduct(quat &dst, const quat &q1, const quat &q2)
  {
    mul_norm_norm(dst, q1, q2);
  }

  // [(q1)(q2) + (q2)(q1)] / 2
  // == ( t*t' - v.v', t*v' + v*t' )
  SQUATINLINE void quat::grassmanEven(quat &dst, const quat &q1, const quat &q2)
  {
    stevesch::vector4 v;
    stevesch::vector4::scale3(v, q1.V(), q2.A());
    stevesch::vector4::addScaled3(v, v, q2.V(), q1.A());
    v.w = q1.A() * q2.A() - stevesch::vector4::dot3(q1.V(), q2.V());
    dst.V() = v;
  }

  // [(q1)(q2) - (q2)(q1)] / 2
  // == ( 0, [v x v'] )
  // *** Common cross product ***
  SQUATINLINE void quat::grassmanOdd(quat &dst, const quat &q1, const quat &q2)
  {
    stevesch::vector4::cross3(dst.V(), q1.V(), q2.V());
    dst.A() = 0.0f;
  }

  // (q1*)(q2)
  // == ( t*t' + v.v', t*v' - v*t' - [v x v'] )
  SQUATINLINE void quat::euclideanProduct(quat &dst, const quat &q1, const quat &q2)
  {
    mul_conj_norm(dst, q1, q2);
  }

  // [(q1*)(q2) + (q2)(q1*)] / 2
  // == ( t*t' + v.v', 0 )
  // *** Common dot product ***
  SQUATINLINE void quat::euclideanEven(quat &dst, const quat &q1, const quat &q2)
  {
    dst.A() = stevesch::vector4::dot4(q1.V(), q2.V()); // t*t' + v.v'
    dst.Z() = dst.Y() = dst.X() = 0.0f;
  }

  // [(q1*)(q2) - (q2)(q1*)] / 2
  // == ( 0, t*v' - v*t' - [v x v'] )
  SQUATINLINE void quat::euclideanOdd(quat &dst, const quat &q1, const quat &q2)
  {
    stevesch::vector4 v;
    stevesch::vector4::addScaled3(v, q2.V(), q1.A(), q1.V(), -q2.A()); // t*v' - v*t'
    stevesch::vector4::cross3(dst.V(), q1.V(), q2.V());                // may alter q1 or q2 if dst is q1 or q2
    stevesch::vector4::sub3(dst.V(), v, dst.V());                      // dst = (v*t' + t*v') - (v x v')
    dst.A() = 0.0f;
  }

  SQUATINLINE void quat::lookAtLHWorld(quat &dst, const stevesch::vector4 &rEye, const stevesch::vector4 &rAt,
                                       const stevesch::vector4 &rUp)
  {
    stevesch::matrix4 mat;
    stevesch::matrix4::lookAtLHWorld(mat, rEye, rAt, rUp);
    dst.fromOrthonormalMatrix(mat);
  }

  SQUATINLINE void quat::lookAtRHWorld(quat &dst, const stevesch::vector4 &rEye, const stevesch::vector4 &rAt,
                                       const stevesch::vector4 &rUp)
  {
    stevesch::matrix4 mat;
    stevesch::matrix4::lookAtRHWorld(mat, rEye, rAt, rUp);
    dst.fromOrthonormalMatrix(mat);
  }

  SQUATINLINE void quat::lookAtLHView(quat &dst, const stevesch::vector4 &rEye, const stevesch::vector4 &rAt,
                                      const stevesch::vector4 &rUp)
  {
    stevesch::matrix4 mat;
    stevesch::matrix4::lookAtLHView(mat, rEye, rAt, rUp);
    dst.fromOrthonormalMatrix(mat);
  }

  SQUATINLINE void quat::lookAtRHView(quat &dst, const stevesch::vector4 &rEye, const stevesch::vector4 &rAt,
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
  SQUATINLINE void quat::integrate(const stevesch::vector3 &w, float dt)
  {
    {
      quat qDeltaAngular(w.x, w.y, w.z, 0.0f); // element-by-element initialization (NOT axis-angle)
      qDeltaAngular *= (0.5f * dt);
      qDeltaAngular *= (*this);

      (*this) += qDeltaAngular;
      normalize(); // renormalize :(
    }
  }

} // namespace SMath

#endif // STEVESCH_MATHVEC_INTERNAL_QUAT_INLINE_H_
