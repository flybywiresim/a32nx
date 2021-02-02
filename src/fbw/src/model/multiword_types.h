#ifndef MULTIWORD_TYPES_H
#define MULTIWORD_TYPES_H
#include "rtwtypes.h"

typedef long int long_T;
typedef struct {
  uint32_T chunks[2];
} int64m_T;

typedef struct {
  uint32_T chunks[2];
} uint64m_T;

typedef struct {
  uint32_T chunks[3];
} int96m_T;

typedef struct {
  uint32_T chunks[3];
} uint96m_T;

typedef struct {
  uint32_T chunks[4];
} int128m_T;

typedef struct {
  uint32_T chunks[4];
} uint128m_T;

typedef struct {
  uint32_T chunks[5];
} int160m_T;

typedef struct {
  uint32_T chunks[5];
} uint160m_T;

typedef struct {
  uint32_T chunks[6];
} int192m_T;

typedef struct {
  uint32_T chunks[6];
} uint192m_T;

typedef struct {
  uint32_T chunks[7];
} int224m_T;

typedef struct {
  uint32_T chunks[7];
} uint224m_T;

typedef struct {
  uint32_T chunks[8];
} int256m_T;

typedef struct {
  uint32_T chunks[8];
} uint256m_T;

#endif

