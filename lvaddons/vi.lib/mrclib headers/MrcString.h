// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "mrclib/MrcLibTypes.h"
/**
 * A const UTF8 string.
 */
struct MRC_String {
    /** Contents. */
    const char* str;
    /** Length */
    size_t len;
};

#ifdef __cplusplus
extern "C" {
#endif  // __cplusplus

/**
 * Initializes a MRC_String from a null terminated UTF-8 string.
 * If input string is null, initializes output to 0 length.
 * The output length does not include the null terminator.
 *
 * The lifetime of the output string is the lifetime of the input string.
 * Do not call MRC_FreeString() with the output of this call.
 *
 * @param mrcString output string
 * @param utf8String input string (null terminated)
 */
void MRC_CALLCONV MRC_InitString(struct MRC_String* mrcString,
                                 const char* utf8String);

/**
 * Initializes a MRC_String from a UTF-8 string and length.
 * If input string is null or 0 length, initializes output to 0 length.
 * The input string does not need to be null terminated.
 *
 * The lifetime of the output string is the lifetime of the input string.
 * Do not call MRC_FreeString() with the output of this call.
 *
 * @param mrcString output string
 * @param utf8String input string
 * @param length input string length in chars
 */
void MRC_CALLCONV MRC_InitStringWithLength(struct MRC_String* mrcString,
                                           const char* utf8String,
                                           size_t length);

/**
 * Allocates a MRC_String for the specified length.
 * The resultant string must be freed with MRC_FreeString().
 *
 * @param mrcString output string
 * @param length string length in chars to allocate
 * @return mutable pointer to allocated buffer
 *
 */
char* MRC_CALLCONV MRC_AllocateString(struct MRC_String* mrcString,
                                      size_t length);

/**
 * Frees a MRC_String that was allocated with MRC_AllocateString()
 *
 * @param mrcString string to free
 */
void MRC_CALLCONV MRC_FreeString(const struct MRC_String* mrcString);

/**
 * Allocates an array of MRC_Strings.
 *
 * @param length array length
 * @return string array
 */
struct MRC_String* MRC_CALLCONV MRC_AllocateStringArray(size_t length);

/**
 * Frees a MRC_String array returned by MRC_AllocateStringArray().
 *
 * @param mrcStringArray string array to free
 * @param length length of array
 */
void MRC_CALLCONV MRC_FreeStringArray(const struct MRC_String* mrcStringArray,
                                      size_t length);

#ifdef __cplusplus
}  // extern "C"
#endif  // __cplusplus
