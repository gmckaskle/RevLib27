// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "mrclib/MrcLibTypes.h"

#define MRC_API_VERSION 12

extern "C" {
MRC_Bool MRC_CheckApiVersion(uint32_t version);
}  // extern "C"

#define MRC_CHECK_API_VERSION() MRC_CheckApiVersion(MRC_API_VERSION)
