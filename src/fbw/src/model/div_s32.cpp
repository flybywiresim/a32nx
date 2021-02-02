#include "rtwtypes.h"
#include "div_s32.h"

int32_T div_s32(int32_T numerator, int32_T denominator)
{
  int32_T quotient;
  uint32_T tempAbsQuotient;
  if (denominator == 0) {
    quotient = numerator >= 0 ? MAX_int32_T : MIN_int32_T;
  } else {
    tempAbsQuotient = (numerator < 0 ? ~static_cast<uint32_T>(numerator) + 1U : static_cast<uint32_T>(numerator)) /
      (denominator < 0 ? ~static_cast<uint32_T>(denominator) + 1U : static_cast<uint32_T>(denominator));
    quotient = (numerator < 0) != (denominator < 0) ? -static_cast<int32_T>(tempAbsQuotient) : static_cast<int32_T>
      (tempAbsQuotient);
  }

  return quotient;
}
