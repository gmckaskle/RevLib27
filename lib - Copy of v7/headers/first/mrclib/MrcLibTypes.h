// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <stdint.h>
#include <stddef.h>

#define MRC_STATUS_SUCCESS 0
#define MRC_STATUS_DO_NOT_SEND 1
#define MRC_STATUS_NO_TOKEN 2
#define MRC_STATUS_NO_VALUE 3
#define MRC_STATUS_INDEX_OUT_OF_RANGE -1
#define MRC_STATUS_RESOURCE_ALREADY_ALLOCATED -2
#define MRC_STATUS_INCOMPATIBLE_STATE -3
#define MRC_STATUS_RESOURCE_NOT_ALLOCATED -4
#define MRC_STATUS_PARAMETER_OUT_OF_RANGE -5
#define MRC_STATUS_INVALID_PARAMETER -6
#define MRC_STATUS_MULTIPLE_USER_PROGRAMS -7
#define MRC_STATUS_OUT_OF_MEMORY -8
#define MRC_STATUS_IO_ERROR -9

#ifndef MRC_CALLCONV
#ifdef _WIN32
#define MRC_CALLCONV __cdecl
#else
#define MRC_CALLCONV
#endif
#endif

typedef int32_t MRC_Status;
typedef uint8_t MRC_Bool;

#ifdef __cplusplus
#define MRC_ENUM_WITH_UNDERLYING_TYPE(name, type) enum name : type
#elif defined(__clang__)
#define MRC_ENUM_WITH_UNDERLYING_TYPE(name, type) \
    enum name : type;                             \
    typedef enum name name;                       \
    enum name : type
#else
#define MRC_ENUM_WITH_UNDERLYING_TYPE(name, type) \
    typedef type name;                            \
    enum name
#endif

#define MRC_ENUM(name) MRC_ENUM_WITH_UNDERLYING_TYPE(name, int32_t)
