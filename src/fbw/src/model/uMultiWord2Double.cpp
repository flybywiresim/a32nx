#include "rtwtypes.h"
#include <cmath>
#include "uMultiWord2Double.h"

real_T uMultiWord2Double(const uint32_T u1[], int32_T n1, int32_T e1)
{
  real_T y;
  int32_T i;
  int32_T exp_0;
  y = 0.0;
  exp_0 = e1;
  for (i = 0; i < n1; i++) {
    y += std::ldexp(static_cast<real_T>(u1[i]), exp_0);
    exp_0 += 32;
  }

  return y;
}
