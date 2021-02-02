#include "rtwtypes.h"
#include "div_s32.h"
#include <cmath>
#include <math.h>
#include "Double2MultiWord.h"

void Double2MultiWord(real_T u1, uint32_T y[], int32_T n)
{
  int32_T i;
  int32_T currExp;
  int32_T prevExp;
  int32_T msl;
  real_T yn;
  real_T yd;
  boolean_T isNegative;
  uint32_T u1i;
  uint32_T cb;
  isNegative = (u1 < 0.0);
  yn = frexp(u1, &currExp);
  msl = currExp <= 0 ? -1 : div_s32(currExp - 1, 32);
  cb = 1U;
  for (i = msl + 1; i < n; i++) {
    y[i] = 0U;
  }

  yn = isNegative ? -yn : yn;
  prevExp = 32 * msl;
  for (i = msl; i >= 0; i--) {
    yn = std::ldexp(yn, currExp - prevExp);
    yd = std::floor(yn);
    yn -= yd;
    if (i < n) {
      y[i] = static_cast<uint32_T>(yd);
    }

    currExp = prevExp;
    prevExp -= 32;
  }

  if (isNegative) {
    for (i = 0; i < n; i++) {
      u1i = ~y[i];
      cb += u1i;
      y[i] = cb;
      cb = (cb < u1i);
    }
  }
}
