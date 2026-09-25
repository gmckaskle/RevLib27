// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "mrclib/MrcString.h"
#include "mrclib/MrcLibTypes.h"

extern "C" {
/**
 * Writes a line to the console output.
 *
 * @param line the line to write
 * @return status of the operation
 */
MRC_Status MRC_CALLCONV MRC_Console_WriteLine(const struct MRC_String* line);

/**
 * Writes an error message to the console output.
 *
 * @param error the error to write
 * @return status of the operation
 */
MRC_Status MRC_CALLCONV MRC_Console_WriteError(
    MRC_Bool isError, int32_t errorCode, const struct MRC_String* message,
    const struct MRC_String* location, const struct MRC_String* stackTrace);

/**
 * Writes a crash message to the console output.
 *
 * @param crash the crash to write
 * @return status of the operation
 */
MRC_Status MRC_CALLCONV MRC_Console_WriteProgramCrash(
    const struct MRC_String* message, const struct MRC_String* location,
    const struct MRC_String* stackTrace);
}  // extern "C"
