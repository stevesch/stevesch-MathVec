#ifndef STEVESCH_MATHVEC_INTERNAL_MATHVEC_H_
#define STEVESCH_MATHVEC_INTERNAL_MATHVEC_H_
#include <Arduino.h>

#define SUNUSED(x) ((void)(x))
#define SASSERT(x)  SUNUSED(x)
#define SASSERT_EXTRA(x) SUNUSED(x)

namespace stevesch
{
  enum class eZero { value }; // <0[, 0[, ...]]>
  enum class eOnes { value }; // <1[, 1[, ...]]>
  enum class eIdentity { value };

  namespace MathVec
  {
  }
}

#endif
