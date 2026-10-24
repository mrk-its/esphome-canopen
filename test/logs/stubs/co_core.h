#pragma once
// the parts of canopen-stack's co_core.h that logs.h / logs.cpp use
#include <cstdint>
typedef uintptr_t CO_DATA;
typedef int16_t CO_ERR;
enum { CO_ERR_NONE = 0, CO_ERR_BAD_ARG = -1, CO_ERR_TYPE_WR = -2 };
struct CO_NODE_T {};
typedef struct CO_NODE_T CO_NODE;
struct CO_OBJ_T {
  uint32_t Key;
  const void *Type;
  CO_DATA Data;
};
typedef struct CO_OBJ_T CO_OBJ;
typedef struct CO_OBJ_TYPE_T {
  uint32_t (*Size)(CO_OBJ *, CO_NODE *, uint32_t);
  CO_ERR (*Init)(CO_OBJ *, CO_NODE *);
  CO_ERR (*Read)(CO_OBJ *, CO_NODE *, void *, uint32_t);
  CO_ERR (*Write)(CO_OBJ *, CO_NODE *, void *, uint32_t);
  CO_ERR (*Reset)(CO_OBJ *, CO_NODE *, uint32_t);
} CO_OBJ_TYPE;
