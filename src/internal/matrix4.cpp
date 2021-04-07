#include "matrix4.h"
//
// Copyright © 2002, PulseCode Interactive LLC, All Rights Reserved
//

namespace stevesch
{

  const matrix4 matrix4::I(
      1.0f, 0.0f, 0.0f, 0.0f,
      0.0f, 1.0f, 0.0f, 0.0f,
      0.0f, 0.0f, 1.0f, 0.0f,
      0.0f, 0.0f, 0.0f, 1.0f);

  // matrix that performs rotation about x-axis
  const matrix4 &matrix4::XMatrix(float fRadians)
  {
    float c, s;
    cosSinf(fRadians, &c, &s);

    identity();

    // matrix mxx
    m11 = c;
    m22 = c;
    m21 = s;
    m12 = -s;
    return *this;
  }

  // matrix that performs rotation about y-axis
  const matrix4 &matrix4::YMatrix(float fRadians)
  {
    float c, s;
    cosSinf(fRadians, &c, &s);

    identity();

    // matrix mxx
    m00 = c;
    m22 = c;
    m20 = -s;
    m02 = s;
    return *this;
  }

  // matrix that performs rotation about z-axis
  const matrix4 &matrix4::ZMatrix(float fRadians)
  {
    float c, s;
    cosSinf(fRadians, &c, &s);

    identity();

    // matrix mxx
    m00 = c;
    m11 = c;
    m10 = s;
    m01 = -s;
    return *this;
  }

  const matrix4 &matrix4::fromEuler(const vector3 &vEuler)
  {
    // NOT OPTIMIZED, but for axis clarity:

    // here's where computation becomes order-dependent:
#if SCOORDINATE_SYSTEM_IS_NASA_STANDARD_AIRPLANE
    // NASA standard airplane (xyz)
    // final form is m = mx*my*mz, where v' = v*m
    matrix4 mx, my, mz;
    mx.XMatrix(vEuler.x);
    my.YMatrix(vEuler.y);
    mz.ZMatrix(vEuler.z);
    matrix4::mul(*this, my, mx);
    *this *= mz;
#else
    // zxy (roll, pitch, yaw)

    matrix4 mtxTmp;
    matrix4 *pTmp = this;
    if (0.0f != vEuler.z)
    {
      ZMatrix(vEuler.z);
      pTmp = &mtxTmp;
    }

    if (0.0f != vEuler.x)
    {
      pTmp->XMatrix(vEuler.x);
      if (pTmp == &mtxTmp)
      {
        (*this) *= (*pTmp);
      }
      pTmp = &mtxTmp;
    }

    if (0.0f != vEuler.y)
    {
      pTmp->YMatrix(vEuler.y);
      if (pTmp == &mtxTmp)
      {
        (*this) *= (*pTmp);
      }
      pTmp = &mtxTmp;
    }

    if (this == pTmp)
    {
      identity();
    }
/*
		matrix4 mx, my, mz;
		mx.XMatrix( vEuler.x );
		my.YMatrix( vEuler.y );
		mz.ZMatrix( vEuler.z );
		matrix4::mul( *this, mx, mz );
		*this *= my;
*/
#endif

    return *this;
  }

  void matrix4::lookAtLHWorld(matrix4 &dst, const vector4 &rEye, const vector4 &rAt,
                              const vector4 &rUp)
  {
    vector4 vForward;
    vector4 vRight;

    vector4::sub(vForward, rAt, rEye);

    vForward.normalize();                  // assumes target != position
    vector4::cross(vRight, rUp, vForward); // left-handed coordinate system
    vRight.normalize();

    vector4 vCameraUp;
    vector4::cross(vCameraUp, vForward, vRight); // left-handed coordinate system

    dst.col[0].set(vRight.x, vRight.y, vRight.z, 0.0f);
    dst.col[1].set(vCameraUp.x, vCameraUp.y, vCameraUp.z, 0.0f);
    dst.col[2].set(vForward.x, vForward.y, vForward.z, 0.0f);
    dst.col[3].set(rEye.x, rEye.y, rEye.z, 1.0f);
  }

  void matrix4::lookAtRHWorld(matrix4 &dst, const vector4 &rEye, const vector4 &rAt,
                              const vector4 &rUp)
  {
    vector4 vBackward;
    vector4 vRight;

    vector4::sub(vBackward, rEye, rAt);

    vBackward.normalize();                  // assumes target != position
    vector4::cross(vRight, rUp, vBackward); // left-handed coordinate system
    vRight.normalize();

    vector4 vCameraUp;
    vector4::cross(vCameraUp, vBackward, vRight); // left-handed coordinate system

    dst.col[0].set(vRight.x, vRight.y, vRight.z, 0.0f);
    dst.col[1].set(vCameraUp.x, vCameraUp.y, vCameraUp.z, 0.0f);
    dst.col[2].set(vBackward.x, vBackward.y, vBackward.z, 0.0f);
    dst.col[3].set(rEye.x, rEye.y, rEye.z, 1.0f);
  }

  void matrix4::lookAtLHView(matrix4 &dst, const vector4 &rEye, const vector4 &rAt,
                             const vector4 &rUp)
  {
    // D3DXMatrixLookAtLH( mtxView, vFromPt, vLookatPt, vUpVec );

    vector4 vForward;
    vector4 vRight;

    vector4::sub(vForward, rAt, rEye);

    vForward.normalize();                  // assumes target != position
    vector4::cross(vRight, rUp, vForward); // left-handed coordinate system
    vRight.normalize();

    vector4 vCameraUp;
    vector4::cross(vCameraUp, vForward, vRight); // left-handed coordinate system

    // world-to-view look-at matrix is inverse of a local-to-world look-at (see lookAtLHWorld, above)
    // A == T * M
    // -> A^-1 = M^-1 * T^-1

    // M^-1 == M^T for orthonormal 3x3:
    dst.col[0].set(vRight.x, vCameraUp.x, vForward.x, 0.0f);
    dst.col[1].set(vRight.y, vCameraUp.y, vForward.y, 0.0f);
    dst.col[2].set(vRight.z, vCameraUp.z, vForward.z, 0.0f);
    dst.col[3].set(0.0f, 0.0f, 0.0f, 1.0f);

    // T^-1 is negation of eye point
    matrix4 mtxTranslation(-rEye.x, -rEye.y, -rEye.z);

    // A^-1 = M^-1 * T^-1
    matrix4::mul(dst, dst, mtxTranslation);
  }

  void matrix4::lookAtRHView(matrix4 &dst, const vector4 &rEye, const vector4 &rAt,
                             const vector4 &rUp)
  {
    // D3DXMatrixLookAtLH( mtxView, vFromPt, vLookatPt, vUpVec );

    vector4 vBackward;
    vector4 vRight;

    vector4::sub(vBackward, rEye, rAt);

    vBackward.normalize();                  // assumes target != position
    vector4::cross(vRight, rUp, vBackward); // right-handed coordinate system
    vRight.normalize();

    vector4 vCameraUp;
    vector4::cross(vCameraUp, vBackward, vRight); // left-handed coordinate system

    // world-to-view look-at matrix is inverse of a local-to-world look-at (see lookAtRHWorld, above)
    // A == T * M
    // -> A^-1 = M^-1 * T^-1

    // M^-1 == M^T for orthonormal 3x3:
    dst.col[0].set(vRight.x, vCameraUp.x, vBackward.x, 0.0f);
    dst.col[1].set(vRight.y, vCameraUp.y, vBackward.y, 0.0f);
    dst.col[2].set(vRight.z, vCameraUp.z, vBackward.z, 0.0f);
    dst.col[3].set(0.0f, 0.0f, 0.0f, 1.0f);

    // T^-1 is negation of eye point
    matrix4 mtxTranslation(-rEye.x, -rEye.y, -rEye.z);

    // A^-1 = M^-1 * T^-1
    matrix4::mul(dst, dst, mtxTranslation);
  }

  float matrix4::det3x3() const
  {
    vector4 v;
    vector4::cross(v, col[1], col[2]);
    return col[0].dot(v);
  }

#if 0
	// ----------------------------------------------------------------------------
	//  Name:	GPMatrix::Determinant
	//  Desc:	Return the matrix determinant. A = det[M].
	// ----------------------------------------------------------------------------
	float GPMatrix::Determinant() {
		__m128 Va,Vb,Vc;
		__m128 r1,r2,r3,t1,t2,sum;
		F32vec4 det;
		
		// First, Let's calculate the first four minterms of the first line
		t1 = _L4; t2 = _mm_ror_ps(_L3,1); 
		Vc = _mm_mul_ps(t2,_mm_ror_ps(t1,0));					// V3'�V4
		Va = _mm_mul_ps(t2,_mm_ror_ps(t1,2));					// V3'�V4"
		Vb = _mm_mul_ps(t2,_mm_ror_ps(t1,3));					// V3'�V4^
		
		r1 = _mm_sub_ps(_mm_ror_ps(Va,1),_mm_ror_ps(Vc,2));		// V3"�V4^ - V3^�V4"
		r2 = _mm_sub_ps(_mm_ror_ps(Vb,2),_mm_ror_ps(Vb,0));		// V3^�V4' - V3'�V4^
		r3 = _mm_sub_ps(_mm_ror_ps(Va,0),_mm_ror_ps(Vc,1));		// V3'�V4" - V3"�V4'
		
		Va = _mm_ror_ps(_L2,1);		sum = _mm_mul_ps(Va,r1);
		Vb = _mm_ror_ps(Va,1);		sum = _mm_add_ps(sum,_mm_mul_ps(Vb,r2));
		Vc = _mm_ror_ps(Vb,1);		sum = _mm_add_ps(sum,_mm_mul_ps(Vc,r3));
		
		// Now we can calculate the determinant:
		det = _mm_mul_ps(sum,_L1);
		det = _mm_add_ps(det,_mm_movehl_ps(det,det));
		det = _mm_sub_ss(det,_mm_shuffle_ps(det,det,1));
		return det[0];
	}

	
	const real CMatrix3x3::Determinant() const
	{
		return (_11 * _22 * _33 - _11 * _23 * _32) 
			+ (_12 * _23 * _31 - _12 * _21 * _33)
			+ (_13 * _21 * _32 - _13 * _22 * _31);
	}
	
	const CMatrix3x3 CMatrix3x3::Adjoint() const
	{
		return CMatrix3x3(
			_22 * _33 - _23 * _32, _23 * _31 - _21 * _33, _21 * _32 - _22 * _31, 
			_13 * _32 - _12 * _33, _11 * _33 - _13 * _31, _12 * _31 - _11 * _32, 
			_12 * _23 - _13 * _22, _13 * _21 - _11 * _23, _11 * _22 - _12 * _21
			);
	}
	
	const CMatrix3x3 CMatrix3x3::Inverse() const
	{
		return Adjoint() / Determinant();
	}
#endif

  void Tridiagonal3(matrix4 &mat, float *diag, float *subd)
  // input: mat = 3x3 real, symmetric A
  // output: mat = orthogonal matrix Q
  // diag = diagonal entries of T, diag[0,1,2]
  // subd = subdiagonal entry of T, subd[0,1]
  {
    float a = mat[0][0], b = mat[0][1], c = mat[0][2],
          d = mat[1][1], e = mat[1][2],
          f = mat[2][2];
    diag[0] = a;
    subd[2] = 0;
    if (c != 0)
    {
      float ell = sqrtf(b * b + c * c);
      b /= ell;
      c /= ell;
      float q = 2 * b * e + c * (f - d);
      diag[1] = d + c * q;
      diag[2] = f - c * q;
      subd[0] = ell;
      subd[1] = e - b * q;
      mat[0][0] = 1;
      mat[0][1] = 0;
      mat[0][2] = 0;
      mat[1][0] = 0;
      mat[1][1] = b;
      mat[1][2] = c;
      mat[2][0] = 0;
      mat[2][1] = c;
      mat[2][2] = -b;
    }
    else
    {
      diag[1] = d;
      diag[2] = f;
      subd[0] = b;
      subd[1] = e;
      mat[0][0] = 1;
      mat[0][1] = 0;
      mat[0][2] = 0;
      mat[1][0] = 0;
      mat[1][1] = 1;
      mat[1][2] = 0;
      mat[2][0] = 0;
      mat[2][1] = 0;
      mat[2][2] = 1;
    }
  }

  //---------------------------------------------------------------------------
  bool QLAlgorithm(int iSize, float *m_afDiag, float *m_afSubd,
                      matrix4 &m_aafMat)
  {
    const int iMaxIter = 32;

    for (int i0 = 0; i0 < iSize; i0++)
    {
      int i1;
      for (i1 = 0; i1 < iMaxIter; i1++)
      {
        int i2;
        for (i2 = i0; i2 <= iSize - 2; i2++)
        {
          float fTmp =
              fabsf(m_afDiag[i2]) + fabsf(m_afDiag[i2 + 1]);
          if (fabsf(m_afSubd[i2]) + fTmp == fTmp)
            break;
        }
        if (i2 == i0)
          break;

        float fG = (m_afDiag[i0 + 1] - m_afDiag[i0]) / (2.0f * m_afSubd[i0]);
        float fR = sqrtf(fG * fG + 1.0f);
        if (fG < 0.0f)
          fG = m_afDiag[i2] - m_afDiag[i0] + m_afSubd[i0] / (fG - fR);
        else
          fG = m_afDiag[i2] - m_afDiag[i0] + m_afSubd[i0] / (fG + fR);
        float fSin = 1.0f, fCos = 1.0f, fP = 0.0f;
        for (int i3 = i2 - 1; i3 >= i0; i3--)
        {
          float fF = fSin * m_afSubd[i3];
          float fB = fCos * m_afSubd[i3];
          if (fabsf(fF) >= fabsf(fG))
          {
            fCos = fG / fF;
            fR = sqrtf(fCos * fCos + 1.0f);
            m_afSubd[i3 + 1] = fF * fR;
            fSin = 1.0f / fR;
            fCos *= fSin;
          }
          else
          {
            fSin = fF / fG;
            fR = sqrtf(fSin * fSin + 1.0f);
            m_afSubd[i3 + 1] = fG * fR;
            fCos = 1.0f / fR;
            fSin *= fCos;
          }
          fG = m_afDiag[i3 + 1] - fP;
          fR = (m_afDiag[i3] - fG) * fSin + 2.0f * fB * fCos;
          fP = fSin * fR;
          m_afDiag[i3 + 1] = fG + fP;
          fG = fCos * fR - fB;

          for (int i4 = 0; i4 < iSize; i4++)
          {
            fF = m_aafMat[i4][i3 + 1];
            m_aafMat[i4][i3 + 1] = fSin * m_aafMat[i4][i3] + fCos * fF;
            m_aafMat[i4][i3] = fCos * m_aafMat[i4][i3] - fSin * fF;
          }
        }
        m_afDiag[i0] -= fP;
        m_afSubd[i0] = fG;
        m_afSubd[i2] = 0.0f;
      }
      if (i1 == iMaxIter)
        return false;
    }

    return true;
  }

  bool matrix4::eigen3(vector3 &vDiag3)
  {
    vector3 subd;
    Tridiagonal3(*this, (float *)&vDiag3, (float *)&subd);
    return QLAlgorithm(3, (float *)&vDiag3, (float *)&subd, *this);
  }

  /*
	calling sequence:

		...
		tol = 1.e-6;
		tri_diag(Cxd,d,e,A,L,tol);

		macheps = 1.e-16;
		calc_eigenstructure(d,e,A,L,macheps);
		...

	arguments:

	tri_diag( Cxd, d, e, A, L, tol) 
	Cxd - an LxL sized matrix containing the symmetric matrix to be analyzed, 
		  such as a covariance matrix (in C, allocate one longer, i.e., alloc length LxL+1 ) 
	d - a length L vector that passes results to the next routine 
		 (in C, allocate one longer, i.e., alloc length L+1 ); needs no initialization 
	e - a length L vector that passes results to the next routine 
		 (in C, allocate one longer, i.e., alloc length L+1 ); needs no initialization 
	A - an LxL matrix that holds the tri-diagonalized version of Cxd upon return; 
		needed to pass to the next routine (in C, allocate one longer, i.e., alloc length LxL+1 ); 
		needs no initialization 
	tol - tolerance for checking nearness to zero; 
		 I found 1.0e-6 to be sufficient for my applications but you may need a smaller value. 

	calc_eigenstructure( d, e, A, L, macheps ) 
	d - vector as above; on return it holds the eigenvalues in sorted order (smallest to largest) 
	e - vector as above; used to pass info into this routine from the previous one 
	A - matrix as above; 
	   on return it holds the array of eigenvectors as columns in order 
	   corresponding to the eigenvalues in d 
	macheps - an iteration error tolerance parameter; 
			  I found 1.0e-16 to work well in my applications; you may have to adjust 
			  this if you have convergence problems.
*/

  //
  //  routine to tri-diagonalize a real symmetric matrix
  //				uses Householder's method
  //
  void tri_diagonalize(float *Cxd, float *d, float *e, float *A, int L, float tol)
  {
    int i, j, k, l;
    float f, g, h, hh;
    for (i = 0; i < L; i++)
    {
      for (j = 0; j <= i; j++)
      {
        A[i * L + j] = Cxd[i * L + j];
      }
    }
    for (i = L - 1; i > 0; i--)
    {
      l = i - 2;
      f = A[i * L + i - 1];
      g = 0.0f;
      for (k = 0; k <= l; k++)
      {
        g += A[i * L + k] * A[i * L + k];
      }
      h = g + f * f;
      if (g <= tol)
      {
        e[i] = f;
        h = 0.0f;
        d[i] = h;
        continue;
      }
      l++;
      g = sqrtf(h);
      if (f >= 0.0f)
        g = -g;
      e[i] = g;
      h = h - f * g;
      A[i * L + i - 1] = f - g;
      f = 0.0f;
      for (j = 0; j <= l; j++)
      {
        A[j * L + i] = A[i * L + j] / h;
        g = 0.0f;
        for (k = 0; k <= j; k++)
        {
          g += A[j * L + k] * A[i * L + k];
        }
        for (k = j + 1; k <= l; k++)
        {
          g += A[k * L + j] * A[i * L + k];
        }
        e[j] = g / h;
        f += g * A[j * L + i];
      }
      hh = f / (h + h);
      for (j = 0; j <= l; j++)
      {
        f = A[i * L + j];
        g = e[j] - hh * f;
        e[j] = g;
        for (k = 0; k <= j; k++)
        {
          A[j * L + k] = A[j * L + k] - f * e[k] - g * A[i * L + k];
        }
      }
      d[i] = h;
    }
    d[0] = e[0] = 0.0f;
    for (i = 0; i < L; i++)
    {
      l = i - 1;
      if (d[i] != 0.0f)
      {
        for (j = 0; j <= l; j++)
        {
          g = 0.0f;
          for (k = 0; k <= l; k++)
          {
            g += A[i * L + k] * A[k * L + j];
          }
          for (k = 0; k <= l; k++)
          {
            A[k * L + j] = A[k * L + j] - g * A[k * L + i];
          }
        }
      }
      d[i] = A[i * L + i];
      A[i * L + i] = 1.0f;
      for (j = 0; j <= l; j++)
      {
        A[i * L + j] = A[j * L + i] = 0.0f;
      }
    }
  }

  //	routine to find eigenstructure of real tri-diagonal matrix
  //			 uses QL algorithm
  //		  returns  true: success      false: failure to converge
  bool calc_eigenstructure(float *d, float *e, float *A, int L, float macheps)
  {
    int i, j, k, l, m;
    float b, c, f, g, h, p, r, s;

    for (i = 1; i < L; i++)
      e[i - 1] = e[i];
    e[L - 1] = b = f = 0.0f;
    for (l = 0; l < L; l++)
    {
      h = macheps * (fabsf(d[l]) + fabsf(e[l]));
      if (b < h)
        b = h;
      for (m = l; m < L; m++)
      {
        if (fabsf(e[m]) <= b)
          break;
      }
      j = 0;
      if (m != l)
        do
        {
          if (j++ == 30)
            return false;
          p = (d[l + 1] - d[l]) / (2.0f * e[l]);
          r = sqrtf(p * p + 1);
          h = d[l] - e[l] / (p + (p < 0.0f ? -r : r));
          for (i = l; i < L; i++)
            d[i] = d[i] - h;
          f += h;
          p = d[m];
          c = 1.0f;
          s = 0.0f;
          for (i = m - 1; i >= l; i--)
          {
            g = c * e[i];
            h = c * p;
            if (fabsf(p) >= fabsf(e[i]))
            {
              c = e[i] / p;
              r = sqrtf(c * c + 1);
              e[i + 1] = s * p * r;
              s = c / r;
              c = 1.0f / r;
            }
            else
            {
              c = p / e[i];
              r = sqrtf(c * c + 1);
              e[i + 1] = s * e[i] * r;
              s = 1.0f / r;
              c = c / r;
            }
            p = c * d[i] - s * g;
            d[i + 1] = h + s * (c * g + s * d[i]);
            for (k = 0; k < L; k++)
            {
              h = A[k * L + i + 1];
              A[k * L + i + 1] = s * A[k * L + i] + c * h;
              A[k * L + i] = c * A[k * L + i] - s * h;
            }
          }
          e[l] = s * p;
          d[l] = c * p;
        } while (fabsf(e[l]) > b);
      d[l] = d[l] + f;
    }

    // order the eigenvectors
    for (i = 0; i < L; i++)
    {
      k = i;
      p = d[i];
      for (j = i + 1; j < L; j++)
      {
        if (d[j] < p)
        {
          k = j;
          p = d[j];
        }
      }
      if (k != i)
      {
        d[k] = d[i];
        d[i] = p;
        for (j = 0; j < L; j++)
        {
          p = A[j * L + i];
          A[j * L + i] = A[j * L + k];
          A[j * L + k] = p;
        }
      }
    }
    return true;
  }

}
