#include "vector2.h"

namespace stevesch
{
  hmQuadToSqr::hmQuadToSqr(const vector2 &rkP00,
                           const vector2 &rkP10, const vector2 &rkP11,
                           const vector2 &rkP01)
  {
    // translate to origin
    m_kT = rkP00;
    vector2 kQ10 = rkP10 - rkP00;
    vector2 kQ11 = rkP11 - rkP00;
    vector2 kQ01 = rkP01 - rkP00;

    matrix2 kInvM(kQ10.X(), kQ01.X(), kQ10.Y(), kQ01.Y());
    matrix2 *pSuccess = kInvM.getInverse(m_kM);
    SASSERT(NULL != pSuccess);
    m_kM.transpose();

    // compute where p11 is mapped to
    vector2 kCorner;
    vector2::transform(kCorner, m_kM, kQ11); // = (a,b)	//XXX

    // Compute homogeneous transform of quadrilateral
    // {(0,0),(1,0),(a,b),(0,1)} to square {(0,0),(1,0),(1,1),(0,1)}
    m_kG.X() = (kCorner.Y() - (float)1.0) / kCorner.X();
    m_kG.Y() = (kCorner.X() - (float)1.0) / kCorner.Y();
    m_kD.X() = (float)1.0 + m_kG.X();
    m_kD.Y() = (float)1.0 + m_kG.Y();
  }

  //----------------------------------------------------------------------------
  vector2 hmQuadToSqr::transform(const vector2 &rkP)
  {
    vector2 kProd = (rkP - m_kT);
    kProd.transform(m_kM);

    float fInvDenom = ((float)1.0) / ((float)1.0 + m_kG.dot(kProd));
    vector2 kResult = kProd * fInvDenom;
    kResult.X() *= m_kD.X();
    kResult.Y() *= m_kD.Y();
    return kResult;
  }
  //----------------------------------------------------------------------------
  HmSqrToQuad::HmSqrToQuad(const vector2 &rkP00,
                           const vector2 &rkP10, const vector2 &rkP11,
                           const vector2 &rkP01)
  {
    // translate to origin
    m_kT = rkP00;
    m_kM[0][0] = rkP10.X() - rkP00.X();
    m_kM[0][1] = rkP01.X() - rkP00.X();
    m_kM[1][0] = rkP10.Y() - rkP00.Y();
    m_kM[1][1] = rkP01.Y() - rkP00.Y();

    matrix2 kInvM;
    matrix2 *pSuccess = m_kM.getInverse(kInvM);
    SASSERT(NULL != pSuccess);

    m_kM.transpose();
    kInvM.transpose();

    // find point which is mapped to p11
    vector2 kCorner = (rkP11 - rkP00);
    kCorner.transform(kInvM); // = (a,b)	//XXX

    // compute homogeneous transform of square {(0,0),(1,0),(1,1),(0,1)} to
    // quadrilateral {(0,0),(1,0),(a,b),(0,1)}
    float fInvDenom = ((float)1.0) / (kCorner.X() + kCorner.Y() - (float)1.0);
    m_kG.X() = fInvDenom * ((float)1.0 - kCorner.Y());
    m_kG.Y() = fInvDenom * ((float)1.0 - kCorner.X());
    m_kD.X() = fInvDenom * kCorner.X();
    m_kD.Y() = fInvDenom * kCorner.Y();
  }
  //----------------------------------------------------------------------------
  vector2 HmSqrToQuad::transform(const vector2 &rkP)
  {
    float fInvDenom = ((float)1.0) / ((float)1.0 + m_kG.dot(rkP));
    vector2 kResult(m_kD.X() * rkP.X(), m_kD.Y() * rkP.Y());
    vector2 kProd = kResult;
    kProd.transform(m_kM); //XXX
    kResult.X() = fInvDenom * kProd.X() + m_kT.X();
    kResult.Y() = fInvDenom * kProd.Y() + m_kT.Y();
    return kResult;
  }

} // namespace stevesch
