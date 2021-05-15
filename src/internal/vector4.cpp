#include "vector4.h"
//
// Copyright © 2002, Stephen Schlueter, All Rights Reserved
//

/*
#if !defined(SPLATFORM_PC) && !defined(SPLATFORM_XBOX)
	#define LINK_VECTOREX_FUNCTIONS
#else
#endif
*/
//#define LINK_VECTOREX_FUNCTIONS

namespace stevesch
{
  // constants defined according to the coordinate system
  const vector4 c_vUP(0.0f, 1.0f, 0.0f, 1.0f);
  const vector4 c_vDOWN(0.0f, -1.0f, 0.0f, 1.0f);
  const vector4 c_vLEFT(-1.0f, 0.0f, 0.0f, 1.0f);
  const vector4 c_vRIGHT(1.0f, 0.0f, 0.0f, 1.0f);
  const vector4 c_vFORWARD(0.0f, 0.0f, 1.0f, 1.0f);
  const vector4 c_vBACKWARD(0.0f, 0.0f, -1.0f, 1.0f);
  const vector4 c_vZERO(0.0f, 0.0f, 0.0f, 1.0f);

  static inline void ASSERT_RANGEF(float x, float a, float b)
  {
    SASSERT(((x >= a) && (x <= b)));
  }

#define LOCAL_UNINIT(type, ptr, count) type *ptr = ((type *)SALLOCA(count * sizeof(type)))

  float vector4::distanceSquared3(vector4 &vNearestPoint, const vector4 *pTriangle, int *pnNearestEdge) const
  {
    //		LOCAL_UNINIT(vector4, e, 3);
    //		LOCAL_UNINIT(vector4, r, 3);
    vector4 e[3];
    vector4 r[3];
    vector4 vNormal;
    vector4 c;

    float f0, f1, f2;
    int nNearestEdge = -1; // -1 means nearest point is in interior of triangle

    vector4::triangleEdgesAndVectors(e, r, pTriangle, *this);
    //		vector4::sub3(e[0], pTriangle[2], pTriangle[1]);
    //		vector4::sub3(e[1], pTriangle[0], pTriangle[2]);
    //		vector4::sub3(e[2], pTriangle[1], pTriangle[0]);

    //		vector4::sub3(r[0], *this, pTriangle[0]);
    //		vector4::sub3(r[1], *this, pTriangle[1]);
    //		vector4::sub3(r[2], *this, pTriangle[2]);
    vector4::cross3(vNormal, e[1], e[2]);

    do
    {
      float t, tDenom;

      vector4::cross3(c, e[2], vNormal);
      f0 = r[0].dot3(c);
      if (f0 > 0.0f)
      {
        // nearest point is on an edge-- on edge 2 if angles are acute
        t = r[0].dot3(e[2]);

        if (t < 0.0f)
        {
          // nearest point is on edge 1
          t = r[2].dot3(e[1]);
          tDenom = e[1].squareMag3();
          t = clampf(t, 0.0f, tDenom);
          vector4::addScaled3(vNearestPoint, pTriangle[2], e[1], t / tDenom);
          nNearestEdge = 1;
        }
        else
        {
          tDenom = e[2].squareMag3();
          if (t > tDenom)
          {
            // nearest point is on edge 0
            t = r[1].dot3(e[0]);
            tDenom = e[0].squareMag3();
            t = clampf(t, 0.0f, tDenom);
            vector4::addScaled3(vNearestPoint, pTriangle[1], e[0], t / tDenom);
            nNearestEdge = 0;
          }
          else
          {
            // nearest point is on edge 2
            ASSERT_RANGEF(t, 0.0f, tDenom);
            t = clampf(t, 0.0f, tDenom);
            vector4::addScaled3(vNearestPoint, pTriangle[0], e[2], t / tDenom);
            nNearestEdge = 2;
          }
        }

        break;
      }

      vector4::cross3(c, e[0], vNormal);
      f1 = r[1].dot3(c);
      if (f1 > 0.0f)
      {
        // nearest point is on an edge-- on edge 0 if angles are acute
        t = r[1].dot3(e[0]);

        if (t < 0.0f)
        {
          // nearest point is on edge 2
          t = r[0].dot3(e[2]);
          tDenom = e[2].squareMag3();
          t = clampf(t, 0.0f, tDenom);
          vector4::addScaled3(vNearestPoint, pTriangle[0], e[2], t / tDenom);
          nNearestEdge = 2;
        }
        else
        {
          tDenom = e[0].squareMag3();
          if (t > tDenom)
          {
            // nearest point is on edge 1
            t = r[2].dot3(e[1]);
            tDenom = e[1].squareMag3();
            t = clampf(t, 0.0f, tDenom);
            vector4::addScaled3(vNearestPoint, pTriangle[2], e[1], t / tDenom);
            nNearestEdge = 1;
          }
          else
          {
            // nearest point is on edge 0
            ASSERT_RANGEF(t, 0.0f, tDenom);
            t = clampf(t, 0.0f, tDenom);
            vector4::addScaled3(vNearestPoint, pTriangle[1], e[0], t / tDenom);
            nNearestEdge = 0;
          }
        }
        break;
      }

      vector4::cross3(c, e[1], vNormal);
      f2 = r[2].dot3(c);
      if (f2 > 0.0f)
      {
        // nearest point is on an edge-- on edge 1 if angles are acute
        t = r[2].dot3(e[1]);
        if (t < 0.0f)
        {
          // nearest point is on edge 0
          t = r[1].dot3(e[0]);
          tDenom = e[0].squareMag3();
          t = clampf(t, 0.0f, tDenom);
          vector4::addScaled3(vNearestPoint, pTriangle[1], e[0], t / tDenom);
          nNearestEdge = 0;
        }
        else
        {
          tDenom = e[1].squareMag3();
          if (t > tDenom)
          {
            // nearest point is on edge 2
            t = r[0].dot3(e[2]);
            tDenom = e[2].squareMag3();
            t = clampf(t, 0.0f, tDenom);
            vector4::addScaled3(vNearestPoint, pTriangle[0], e[2], t / tDenom);
            nNearestEdge = 2;
          }
          else
          {
            // nearest point is on edge 1
            ASSERT_RANGEF(t, 0.0f, tDenom);
            t = clampf(t, 0.0f, tDenom);
            vector4::addScaled3(vNearestPoint, pTriangle[2], e[1], t / tDenom);
            nNearestEdge = 1;
          }
        }
        break;
      }

      // nearest point is on interior of triangle (project point to plane)
      // p' = p - n*[ (p - v[0]) . n] / (n . n)
      t = -r[0].dot3(vNormal);
      tDenom = vNormal.dot3(vNormal);
      vector4::addScaled3(vNearestPoint, *this, vNormal, t / tDenom);

    } while (false);

    if (pnNearestEdge)
    {
      *pnNearestEdge = nNearestEdge;
    }

    vNearestPoint.w = 1.0f; // triangleEdgesAndVectors() does not preserve w

    return squareDist3(vNearestPoint, *this);
  }

  float vector4::distanceSquaredST(const vector4 *pTriangle,
                                   float *pfSParam, float *pfTParam) const
  {
    // pTriangle[0] == origin
    // pTriangle[1] - pTriangle[0] = edge 0
    // pTriangle[2] - pTriangle[0] = edge 1
    vector4 e0 = pTriangle[1];
    vector4 e1 = pTriangle[2];
    e0.sub3(pTriangle[0]);
    e1.sub3(pTriangle[0]);

    vector4 kDiff = pTriangle[0];
    kDiff.sub3(*this);

    float fA00 = e0.squareMag3();
    float fA01 = e0.dot3(e1);
    float fA11 = e1.squareMag3();
    float fB0 = kDiff.dot3(e0);
    float fB1 = kDiff.dot3(e1);
    float fC = kDiff.squareMag3();
    float fDet = fabsf(fA00 * fA11 - fA01 * fA01);
    float fS = fA01 * fB1 - fA11 * fB0;
    float fT = fA01 * fB0 - fA00 * fB1;
    float fSqrDist;

    if (fS + fT <= fDet)
    {
      if (fS < 0.0f)
      {
        if (fT < 0.0f) // region 4
        {
          if (fB0 < 0.0f)
          {
            fT = 0.0f;
            if (-fB0 >= fA00)
            {
              fS = 1.0f;
              fSqrDist = fA00 + 2.0f * fB0 + fC;
            }
            else
            {
              fS = -fB0 / fA00;
              fSqrDist = fB0 * fS + fC;
            }
          }
          else
          {
            fS = 0.0f;
            if (fB1 >= 0.0f)
            {
              fT = 0.0f;
              fSqrDist = fC;
            }
            else if (-fB1 >= fA11)
            {
              fT = 1.0f;
              fSqrDist = fA11 + 2.0f * fB1 + fC;
            }
            else
            {
              fT = -fB1 / fA11;
              fSqrDist = fB1 * fT + fC;
            }
          }
        }
        else // region 3
        {
          fS = 0.0f;
          if (fB1 >= 0.0f)
          {
            fT = 0.0f;
            fSqrDist = fC;
          }
          else if (-fB1 >= fA11)
          {
            fT = 1.0f;
            fSqrDist = fA11 + 2.0f * fB1 + fC;
          }
          else
          {
            fT = -fB1 / fA11;
            fSqrDist = fB1 * fT + fC;
          }
        }
      }
      else if (fT < 0.0f) // region 5
      {
        fT = 0.0f;
        if (fB0 >= 0.0f)
        {
          fS = 0.0f;
          fSqrDist = fC;
        }
        else if (-fB0 >= fA00)
        {
          fS = 1.0f;
          fSqrDist = fA00 + 2.0f * fB0 + fC;
        }
        else
        {
          fS = -fB0 / fA00;
          fSqrDist = fB0 * fS + fC;
        }
      }
      else // region 0
      {
        // minimum at interior point
        float fInvDet = 1.0f / fDet;
        fS *= fInvDet;
        fT *= fInvDet;
        fSqrDist = fS * (fA00 * fS + fA01 * fT + 2.0f * fB0) +
                   fT * (fA01 * fS + fA11 * fT + 2.0f * fB1) + fC;
      }
    }
    else
    {
      float fTmp0, fTmp1, fNumer, fDenom;

      if (fS < 0.0f) // region 2
      {
        fTmp0 = fA01 + fB0;
        fTmp1 = fA11 + fB1;
        if (fTmp1 > fTmp0)
        {
          fNumer = fTmp1 - fTmp0;
          fDenom = fA00 - 2.0f * fA01 + fA11;
          if (fNumer >= fDenom)
          {
            fS = 1.0f;
            fT = 0.0f;
            fSqrDist = fA00 + 2.0f * fB0 + fC;
          }
          else
          {
            fS = fNumer / fDenom;
            fT = 1.0f - fS;
            fSqrDist = fS * (fA00 * fS + fA01 * fT + 2.0f * fB0) +
                       fT * (fA01 * fS + fA11 * fT + 2.0f * fB1) + fC;
          }
        }
        else
        {
          fS = 0.0f;
          if (fTmp1 <= 0.0f)
          {
            fT = 1.0f;
            fSqrDist = fA11 + 2.0f * fB1 + fC;
          }
          else if (fB1 >= 0.0f)
          {
            fT = 0.0f;
            fSqrDist = fC;
          }
          else
          {
            fT = -fB1 / fA11;
            fSqrDist = fB1 * fT + fC;
          }
        }
      }
      else if (fT < 0.0f) // region 6
      {
        fTmp0 = fA01 + fB1;
        fTmp1 = fA00 + fB0;
        if (fTmp1 > fTmp0)
        {
          fNumer = fTmp1 - fTmp0;
          fDenom = fA00 - 2.0f * fA01 + fA11;
          if (fNumer >= fDenom)
          {
            fT = 1.0f;
            fS = 0.0f;
            fSqrDist = fA11 + 2.0f * fB1 + fC;
          }
          else
          {
            fT = fNumer / fDenom;
            fS = 1.0f - fT;
            fSqrDist = fS * (fA00 * fS + fA01 * fT + 2.0f * fB0) +
                       fT * (fA01 * fS + fA11 * fT + 2.0f * fB1) + fC;
          }
        }
        else
        {
          fT = 0.0f;
          if (fTmp1 <= 0.0f)
          {
            fS = 1.0f;
            fSqrDist = fA00 + 2.0f * fB0 + fC;
          }
          else if (fB0 >= 0.0f)
          {
            fS = 0.0f;
            fSqrDist = fC;
          }
          else
          {
            fS = -fB0 / fA00;
            fSqrDist = fB0 * fS + fC;
          }
        }
      }
      else // region 1
      {
        fNumer = fA11 + fB1 - fA01 - fB0;
        if (fNumer <= 0.0f)
        {
          fS = 0.0f;
          fT = 1.0f;
          fSqrDist = fA11 + 2.0f * fB1 + fC;
        }
        else
        {
          fDenom = fA00 - 2.0f * fA01 + fA11;
          if (fNumer >= fDenom)
          {
            fS = 1.0f;
            fT = 0.0f;
            fSqrDist = fA00 + 2.0f * fB0 + fC;
          }
          else
          {
            fS = fNumer / fDenom;
            fT = 1.0f - fS;
            fSqrDist = fS * (fA00 * fS + fA01 * fT + 2.0f * fB0) +
                       fT * (fA01 * fS + fA11 * fT + 2.0f * fB1) + fC;
          }
        }
      }
    }

    if (pfSParam)
      *pfSParam = fS;

    if (pfTParam)
      *pfTParam = fT;

    return fabsf(fSqrDist);
  }

  bool vector4::inTriangle(const vector4 *pTri, const vector4 &vNormal) const
  {
    // 3-D point-in-triangle test:
    // for each edge of the triangle, computes a non-normalized
    // vector that is perpendicular to the edge and the normal.  If the
    // dot product between this vector and "the vector from the edge's
    // endpoint to the 'pPoint'" is negative(**), 'pPoint' lies outside
    // of the triangle.
    // If the test passes for all edges, the point is within the triangle
    // (**): assumes left-handed system w/clockwise facing vertices
    vector4 vEdgeNormal;
    vector4 rq;

    vector4::sub3(rq, *this, pTri[0]);
    vector4::sub3(vEdgeNormal, pTri[0], pTri[1]);
    vEdgeNormal.cross3(vNormal);
    if (rq.dot3(vEdgeNormal) < 0.0f)
      return false;

    vector4::sub3(rq, *this, pTri[1]);
    vector4::sub3(vEdgeNormal, pTri[1], pTri[2]);
    vEdgeNormal.cross3(vNormal);
    if (rq.dot3(vEdgeNormal) < 0.0f)
      return false;

    vector4::sub3(rq, *this, pTri[2]);
    vector4::sub3(vEdgeNormal, pTri[2], pTri[0]);
    vEdgeNormal.cross3(vNormal);
    if (rq.dot3(vEdgeNormal) < 0.0f)
      return false;

    return true;
  }

  bool vector4::inTriangle(const vector4 *pTri) const
  {
#if 0
		// 2-D point-in-triangle test
		vector4 vEdge;
		vector4 aq;
		vector4 rq;	// bq or cq

		vector4::sub3( vEdge, pTri[1], pTri[0] );
		vector4::sub3( aq, *this, pTri[0] );
		aq.cross3( vEdge );	// aq x ab

		vector4::sub3( vEdge, pTri[2], pTri[1] );
		vector4::sub3( rq, *this, pTri[1] );
		rq.cross3( vEdge );	// bq x bc
		if (aq.dot3(rq) < 0.0f)	// (aq x ab).(bq x bc) < 0
			return false;
		
		vector4::sub3( vEdge, pTri[0], pTri[2] );
		vector4::sub3( rq, *this, pTri[2] );
		rq.cross3( vEdge );	// cq x ca
		if (aq.dot3(rq) < 0.0f)	// (aq x ab).(cq x ca) < 0
			return false;

		return true;
#elif 1
    // 3-D point-in-triangle test:
    // for each edge of the triangle, computes a non-normalized
    // vector that is perpendicular to the edge and the normal.  If the
    // dot product between this vector and "the vector from the edge's
    // endpoint to the 'pPoint'" is negative(**), 'pPoint' lies outside
    // of the triangle.
    // If the test passes for all edges, the point is within the triangle
    // (**): assumes left-handed system w/clockwise facing vertices
    vector4 vNormal;
    vector4 vEdge[3];
    vector4 vEdgeNormal;
    vector4 rq;

    vector4::sub3(vEdge[0], pTri[1], pTri[0]);    // edge 0
    vector4::sub3(vEdge[1], pTri[2], pTri[1]);    // edge 1
    vector4::sub3(vEdge[2], pTri[0], pTri[2]);    // edge 2
    vector4::cross3(vNormal, vEdge[2], vEdge[0]); // normal = (edge2 x edge0)

    vector4::cross3(vEdgeNormal, vNormal, vEdge[0]);
    vector4::sub3(rq, *this, pTri[0]);
    if (rq.dot3(vEdgeNormal) < 0.0f)
      return false;

    vector4::cross3(vEdgeNormal, vNormal, vEdge[1]);
    vector4::sub3(rq, *this, pTri[1]);
    if (rq.dot3(vEdgeNormal) < 0.0f)
      return false;

    vector4::cross3(vEdgeNormal, vNormal, vEdge[2]);
    vector4::sub3(rq, *this, pTri[2]);
    if (rq.dot3(vEdgeNormal) < 0.0f)
      return false;

    return true;

#else
    // for each edge of the triangle, computes a non-normalized
    // vector that is perpendicular to the edge and the normal.  If the
    // dot product between this vector and "the vector from the edge's
    // endpoint to the 'pPoint'" is negative, 'pPoint' lies outside
    // of the triangle.
    // If the test passes for all edges, the point is within the triangle
    vector4 v1;
    vector4 v2;
    vector4 r;
    int i;

    vector4::sub3(v1, pTri[0], pTri[2]);

    for (i = 0; i < 3; i++)
    {
      float scale;
      int idx = ((i == 2) ? 0 : i + 1);
      vector4::sub3(v2, pTri[idx], pTri[i]);
      vector4::sub3(r, *this, pTri[i]);

      scale = v1.dot3(v2) / v1.dot3(v1);
      v1 *= scale;
      vector4::sub3(v1, v2, v1);

      if (r.dot3(v1) < 0.0f)
        break; // outside of triangle

      v1 = v2;
    }

    return (i == 3);
#endif
  }

  bool vector4::inTriangleEx(const vector4 *pEdges, const vector4 *pVectors, const vector4 &vNormal) const
  {
    // 3-D point-in-triangle test:
    // for each edge of the triangle, computes a non-normalized
    // vector that is perpendicular to the edge and the normal.  If the
    // dot product between this vector and "the vector from the edge's
    // endpoint to the 'point'" is negative(**), 'point' lies outside
    // of the triangle.
    // If the test passes for all edges, the point is within the triangle
    // (**): assumes left-handed system w/clockwise facing vertices
    //		vector4 vNormal;
    vector4 vEdgeNormal;

    //		vector4::cross3( vNormal, pEdges[2], pEdges[0] );		// normal = (edge2 x edge0)

    vector4::cross3(vEdgeNormal, vNormal, pEdges[0]);
    if (pVectors[0].dot3(vEdgeNormal) < 0.0f)
      return false;

    vector4::cross3(vEdgeNormal, vNormal, pEdges[1]);
    if (pVectors[1].dot3(vEdgeNormal) < 0.0f)
      return false;

    vector4::cross3(vEdgeNormal, vNormal, pEdges[2]);
    if (pVectors[2].dot3(vEdgeNormal) < 0.0f)
      return false;

    return true;
  }

#ifdef LINK_VECTOREX_FUNCTIONS

  // dst += [(c0 - bi)/|c0 - bi|] * [vScalei / (fZeroOffset + |c0 - bi|^2]
  void vector4::sumInverseFalloff(vector4 &dst,
                                  const vector4 &c0,
                                  const vector4 &b0,
                                  const vector4 &b1,
                                  const vector4 &b2,
                                  const vector4 &b3,
                                  const vector4 &vScale,
                                  float fRadius2Factor,
                                  float fZeroOffset)
  {
    vector4 d0, d1, d2, d3;
    vector4 r0;
    vector4 vZeroOffset(fZeroOffset, fZeroOffset, fZeroOffset, fZeroOffset);
    vector4 vSquareDist;
    vector4 vRecipDist;

    SVECTORSUB(d0, c0, b0);
    SVECTORSUB(d1, c0, b1);
    SVECTORSUB(d2, c0, b2);
    SVECTORSUB(d3, c0, b3);

    vSquareDist.x = d0.squareMag3();
    vSquareDist.y = d1.squareMag3();
    vSquareDist.z = d2.squareMag3();
    vSquareDist.w = d3.squareMag3();

    vSquareDist.mul4(fRadius2Factor);
    vSquareDist.add4(vZeroOffset);

    recipSqrt4(vRecipDist, vSquareDist);

    div4(r0, vScale, vSquareDist);
    mul4(vRecipDist, r0, vRecipDist);

    // normalize d vectors and scale [ vScalei * di / (1 + sqrt(di^2 + fZeroOffset)) ]
    d0 *= vRecipDist.x;
    d1 *= vRecipDist.y;
    d2 *= vRecipDist.z;
    d3 *= vRecipDist.w;

    dst += d0;
    dst += d1;
    dst += d2;
    dst += d3;
  }

  ////////////////////////////////////////////////////

  void vector4::squareDistQuad(vector4 &vSquareDist,
                               const vector4 &c0,
                               const vector4 &v0,
                               const vector4 &v1,
                               const vector4 &v2,
                               const vector4 &v3)
  {
    vector4 r0, r1, r2, r3;

    SVECTORSUB(r0, c0, v0);
    SVECTORSUB(r1, c0, v1);
    SVECTORSUB(r2, c0, v2);
    SVECTORSUB(r3, c0, v3);

    vSquareDist.x = r0.squareMag3();
    vSquareDist.y = r1.squareMag3();
    vSquareDist.z = r2.squareMag3();
    vSquareDist.w = r3.squareMag3();
  }

  // multiple square distance [vSquareDisti = (di . di)], di = (vi - c0)
  void vector4::squareDistQuad(vector4 &vSquareDist,
                               vector4 &d0, vector4 &d1, vector4 &d2, vector4 &d3,
                               const vector4 &c0,
                               const vector4 &v0, const vector4 &v1, const vector4 &v2, const vector4 &v3)
  {
    vector4 r0, r1, r2, r3;

    SVECTORSUB(d0, c0, v0);
    SVECTORSUB(d1, c0, v1);
    SVECTORSUB(d2, c0, v2);
    SVECTORSUB(d3, c0, v3);

    vSquareDist.x = d0.squareMag3();
    vSquareDist.y = d1.squareMag3();
    vSquareDist.z = d2.squareMag3();
    vSquareDist.w = d3.squareMag3();
  }

  void vector4::sumBOverAQuad(vector4 &dst,
                              const vector4 &a,
                              const vector4 &b0,
                              const vector4 &b1,
                              const vector4 &b2,
                              const vector4 &b3)
  {
    vector4::addScaled4(dst, dst, b0, recipf(a.x));
    vector4::addScaled4(dst, dst, b1, recipf(a.y));
    vector4::addScaled4(dst, dst, b2, recipf(a.z));
    vector4::addScaled4(dst, dst, b3, recipf(a.w));
  }
#endif // LINK_VECTOREX_FUNCTIONS

  ////////////////////////////////////////////////////

  void vector4::applyEulerImplicitQuad(
      vector4 &r0, vector4 &r1, vector4 &r2, vector4 &r3,
      vector4 &v0, vector4 &v1, vector4 &v2, vector4 &v3,
      const vector4 &a0, const vector4 &a1, const vector4 &a2, const vector4 &a3,
      const vector4 &vAScale0, const vector4 &vAScale1, float dt)
  {
    //vector4::applyEulerImplicit(r0, v0, a0, vAScale0.x * vAScale1.x, dt);
    //vector4::applyEulerImplicit(r1, v1, a1, vAScale0.y * vAScale1.y, dt);
    //vector4::applyEulerImplicit(r2, v2, a2, vAScale0.z * vAScale1.z, dt);
    //vector4::applyEulerImplicit(r3, v3, a3, vAScale0.w * vAScale1.w, dt);

    vector4 vAScale;

    mul4(vAScale, vAScale0, vAScale1);
    vAScale.mul4(dt);

    vector4::addScaled3(v0, v0, a0, vAScale.x);
    vector4::addScaled3(v1, v1, a1, vAScale.y);
    vector4::addScaled3(v2, v2, a2, vAScale.z);
    vector4::addScaled3(v3, v3, a3, vAScale.w);

    vector4::addScaled3(r0, r0, v0, dt);
    vector4::addScaled3(r1, r1, v1, dt);
    vector4::addScaled3(r2, r2, v2, dt);
    vector4::addScaled3(r3, r3, v3, dt);
  }

  ////////////////////////////////////////////////////

  ////////////////////////////////////////////////////
  /* now in platform-specific files
	
	// dst = v * m (4x4)
	void vector4::transform(vector4& dst, const matrix4& mLeft, const vector4& v)
	{
		matrix4 mT;
		matrix4::transpose(mT, mLeft);
		
		float x = SVECTORDOT4( v, mT.col[0] );
		float y = SVECTORDOT4( v, mT.col[1] );
		float z = SVECTORDOT4( v, mT.col[2] );
		float w = SVECTORDOT4( v, mT.col[3] );

		dst.x = x;
		dst.y = y;
		dst.z = z;
		dst.w = w;
	}

	// dst = v * m (4x4, as if v.w = 0.0)
	void vector4::transformSub(vector4& dst, const matrix4& mLeft, const vector4& v)
	{
		matrix4 mT;
		matrix4::transpose(mT, mLeft);
		
		float x = SVECTORDOT( v, mT.col[0] );
		float y = SVECTORDOT( v, mT.col[1] );
		float z = SVECTORDOT( v, mT.col[2] );
		
		dst.x = x;
		dst.y = y;
		dst.z = z;
		dst.w = v.w;
	}


	// v = v*(M^T) -- row vector (as if w=0) * transposed 3x3 matrix
	void vector4::transformSubTransposed(vector4& dst, const matrix4& mLeft, const vector4& v)
	{
//		dst = v;
//		dst.MulSubTransposed( m );

		float x = SVECTORDOT( v, mLeft.col[0] );
		float y = SVECTORDOT( v, mLeft.col[1] );
		float z = SVECTORDOT( v, mLeft.col[2] );

		dst.x = x;
		dst.y = y;
		dst.z = z;
		dst.w = v.w;
	}


	// dst = v * m (4x4, as if v.w = 1.0)
	void vector4::transformAff(vector4& dst, const matrix4& mLeft, const vector4& v)
	{
		vector4 vAff(v.x, v.y, v.z, 1.0f);
		matrix4 m;
		matrix4::transpose(m, mLeft);
		
		dst.x = SVECTORDOT4( vAff, m.col[0] );
		dst.y = SVECTORDOT4( vAff, m.col[1] );
		dst.z = SVECTORDOT4( vAff, m.col[2] );
		dst.w = v.w;
	}
*/

  ////////////////////////////////////////////////////
  /*
  void SOperateObjects
#ifdef SEXPLICIT_TEMPLATES
      <vector4>
#endif
      (vector4 &v0, const vector4 &v1, SARITH_OP op) // returns v0 = (v0 op v1)
  {
    switch (op & SAOP_MASK)
    {
    case SAOP_SET:
      v0 = v1;
      break;

    case SAOP_ADD:
      v0 += v1;
      break;

    case SAOP_SUB:
      v0 -= v1;
      break;

    case SAOP_MUL:
      v0.mul(v1);
      break;

    case SAOP_DIV:
    {
      vector4 v;
      vector4::recip(v, v1);
      v0.mul(v);
    }
    break;

    default:
      SASSERT(0); // unsupported
      break;
    }
  }
*/
} // namespace stevesch
