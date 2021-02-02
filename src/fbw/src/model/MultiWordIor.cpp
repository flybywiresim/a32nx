#include "rtwtypes.h"
#include "MultiWordIor.h"

void MultiWordIor(const uint32_T u1[], const uint32_T u2[], uint32_T y[], int32_T n)
{
  int32_T i;
  for (i = 0; i < n; i++) {
    y[i] = u1[i] | u2[i];
  }
}
