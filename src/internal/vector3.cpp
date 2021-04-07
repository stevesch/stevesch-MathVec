#include "vector3.h"
//
// Copyright © 2002, Stephen Schlueter, All Rights Reserved
//
namespace stevesch
{
  // utility function for random spherical distribution
  void vector3::randSpherical(stevesch::RandGen &r)
  {
    // produce uniform distribution over the sphere
    float fYaw = r.getFloatAB(-c_fpi, c_fpi);

    float fPitch = -c_fpi_2 + acosf(r.getFloatAB(-1.0f, 1.0f));
    float fRoll = 0.0f;

    matrix4 m;
    //		m.fromEuler( vector3( fYaw, fPitch, fRoll ) );
    m.fromEuler(vector3(fPitch, fYaw, fRoll));

    //	D3DXQUATERNION q;
    //	D3DXQuaternionRotationYawPitchRoll(&q, fYaw, fPitch, 0.0f);
    //	D3DXMatrixRotationQuaternion( &m, &q);

    // Yaw around the Y axis, a pitch around the X axis,
    // and a roll around the Z axis.
    //		SMat4_t m;
    //		D3DXMatrixRotationYawPitchRoll( &m, fYaw, fPitch, fRoll );
    //		r.x = m._31;
    //		r.y = m._32;
    //		r.z = m._33;
    x = m.m02;
    y = m.m12;
    z = m.m22;
  }

  void vector3::randSpherical()
  {
    randSpherical(S_RandGen);
  }

  ////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////

  // v = M*v -- * 4x4 matrix
  const vector3 &vector3::transform(const matrix4 &left)
  {
    vector4 v(x, y, z, 1.0f);
    matrix4 m;
    matrix4::transpose(m, left);

    x = v.dot4(m.col[0]);
    y = v.dot4(m.col[1]);
    z = v.dot4(m.col[2]);

    return *this;
  }

  // v = M*v -- * 3x3 matrix (as if w=0)
  const vector3 &vector3::transformSub(const matrix4 &left)
  {
    vector4 v(x, y, z, 0.0f);
    matrix4 m;
    matrix4::transpose(m, left);

    x = v.dot(m.col[0]);
    y = v.dot(m.col[1]);
    z = v.dot(m.col[2]);

    return *this;
  }

  ////////////////////////////////////////////////////////////////////

}
