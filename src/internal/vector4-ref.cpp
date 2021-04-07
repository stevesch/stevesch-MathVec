#include "vector4.h"
//
// Copyright © 2002, PulseCode Interactive LLC, All Rights Reserved
//
#include "matrix4.h"

namespace stevesch
{
  // dst = v * m (4x4)
  void vector4::transform(vector4 &dst, const matrix4 &mLeft, const vector4 &v)
  {
    //#if (SDXVER > 0x9fff)
    //		dst = *reinterpret_cast<const vector4*>(&DirectX::XMVector4Transform(v, mRight));
    //#else
    //		D3DXVec4Transform(dst, v, mRight);
    //#endif
    matrix4 mT;
    matrix4::transpose(mT, mLeft);

    float x = SVECTORDOT4(v, mT.col[0]);
    float y = SVECTORDOT4(v, mT.col[1]);
    float z = SVECTORDOT4(v, mT.col[2]);
    float w = SVECTORDOT4(v, mT.col[3]);

    dst.x = x;
    dst.y = y;
    dst.z = z;
    dst.w = w;
  }

  // dst = v * m (4x4, as if v.w = 0.0)
  void vector4::transformSub(vector4 &dst, const matrix4 &mLeft, const vector4 &v)
  {
    //#if (SDXVER > 0x9fff)
    //		float w0 = v.w;
    //		*(SVec4_t*)&dst = DirectX::XMVector3TransformNormal(v, mRight);
    //		dst.w = w0;
    //#else
    //		dst.w = v.w;
    //		D3DXVec3TransformNormal((SVec3_t*)&dst, (const SVec3_t*)&v, mRight);
    //#endif
    matrix4 mT;
    matrix4::transpose(mT, mLeft);

    float x = SVECTORDOT(v, mT.col[0]);
    float y = SVECTORDOT(v, mT.col[1]);
    float z = SVECTORDOT(v, mT.col[2]);

    dst.x = x;
    dst.y = y;
    dst.z = z;
    dst.w = v.w;
  }

  // v = (M^T)*v -- (as if w=0) * transposed 3x3 matrix
  void vector4::transformSubTransposed(vector4 &dst, const matrix4 &mLeft, const vector4 &v)
  {
    float x = SVECTORDOT(v, mLeft.col[0]);
    float y = SVECTORDOT(v, mLeft.col[1]);
    float z = SVECTORDOT(v, mLeft.col[2]);

    dst.x = x;
    dst.y = y;
    dst.z = z;
    dst.w = v.w;
  }

  // dst = v * m (4x4, as if v.w = 1.0)
  void vector4::transformAff(vector4 &dst, const matrix4 &mLeft, const vector4 &v)
  {
    //#if (SDXVER > 0x9fff)
    //		float w0 = v.w;
    //		*(SVec4_t*)&dst = DirectX::XMVector3Transform(v, mRight);
    //		dst.w = w0;
    //#else
    //		dst.w = v.w;
    //		D3DXVec3Transform((SVec3_t*)&dst, (const SVec3_t*)&v, mRight);
    //#endif
    vector4 vAff(v.x, v.y, v.z, 1.0f);
    matrix4 m;
    matrix4::transpose(m, mLeft);

    dst.x = SVECTORDOT4(vAff, m.col[0]);
    dst.y = SVECTORDOT4(vAff, m.col[1]);
    dst.z = SVECTORDOT4(vAff, m.col[2]);
    dst.w = v.w;
  }
}
