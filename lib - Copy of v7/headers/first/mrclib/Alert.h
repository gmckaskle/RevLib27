// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "mrclib/MrcString.h"
#include "mrclib/MrcLibTypes.h"

typedef void* MRC_AlertHandle;

#define MRC_ALERT_HIGH 0
#define MRC_ALERT_MEDIUM 1
#define MRC_ALERT_LOW 2

struct MRC_AlertInfo {
    struct MRC_String group;
    struct MRC_String uniqueId;
    struct MRC_String text;
    int64_t lastActiveTime;
    int32_t level;
};

extern "C" {
/**
 * Creates an alert with the given group, text, and level. The handle must be
 * destroyed with MRC_Alert_DestroyAlert when no longer needed.
 *
 * @param group the group of the alert
 * @param uniqueId the unique identifier of the alert
 * @param text the text of the alert
 * @param level the level of the alert
 * @param handle the handle to the created alert
 * @return status of the operation
 */
MRC_Status MRC_CALLCONV MRC_Alert_CreateAlert(const struct MRC_String* group,
                                              const struct MRC_String* uniqueId,
                                              const struct MRC_String* text,
                                              int32_t level,
                                              MRC_AlertHandle* handle);

/**
 * Destroys an alert handle created with MRC_Alert_CreateAlert.
 *
 * @param handle the handle to destroy
 * @return status of the operation
 */
MRC_Status MRC_CALLCONV MRC_Alert_DestroyAlert(MRC_AlertHandle handle);

/**
 * Sets the active state of the alert.
 *
 * @param handle the alert handle
 * @param active whether the alert is active
 * @return status of the operation
 */
MRC_Status MRC_CALLCONV MRC_Alert_SetAlertActive(MRC_AlertHandle handle,
                                                 MRC_Bool active);

/**
 * Gets the active state of the alert.
 *
 * @param handle the alert handle
 * @param isActive pointer to store whether the alert is active
 * @return status of the operation
 */
MRC_Status MRC_CALLCONV MRC_Alert_IsAlertActive(MRC_AlertHandle handle,
                                                MRC_Bool* isActive);

/**
 * Sets the text of the alert.
 *
 * @param handle the alert handle
 * @param text the new text for the alert
 * @return status of the operation
 */
MRC_Status MRC_CALLCONV MRC_Alert_SetAlertText(MRC_AlertHandle handle,
                                               const struct MRC_String* text);

/**
 * Gets the text of the alert. The return value must be freed by the caller
 * using MRC_String_Free.
 *
 * @param handle the alert handle
 * @param text pointer to store the text of the alert. The caller is responsible
 * for freeing the string.
 * @return status of the operation
 */
MRC_Status MRC_CALLCONV MRC_Alert_GetAlertText(MRC_AlertHandle handle,
                                               struct MRC_String* text);

/**
 * Gets the text of the alert using an external allocation function. The return
 * value must be freed by the caller in a method compatible with allocFunc.
 *
 * @param handle the alert handle
 * @param text pointer to store the text of the alert. The caller is responsible
 * for freeing the string using the provided allocation function.
 * @param allocFunc the allocation function to use for allocating the string
 * @return status of the operation
 */
MRC_Status MRC_CALLCONV MRC_Alert_GetAlertTextExternalAlloc(
    MRC_AlertHandle handle, struct MRC_String* text,
    void*(MRC_CALLCONV* allocFunc)(size_t));

/**
 * Gets the level of the alert.
 *
 * @param handle the alert handle
 * @param level pointer to store the level of the alert
 * @return status of the operation
 */
MRC_Status MRC_CALLCONV MRC_Alert_GetAlertLevel(MRC_AlertHandle handle,
                                                int32_t* level);

/**
 * Gets the information of the alert.
 *
 * @param handle the alert handle
 * @param info pointer to store the information of the alert. The caller is
 * responsible for freeing the strings in the info struct using MRC_String_Free.
 * @return status of the operation
 */
MRC_Status MRC_CALLCONV MRC_Alert_GetAlertInfo(MRC_AlertHandle handle,
                                               struct MRC_AlertInfo* info);

/**
 * Gets the information of the alert using an external allocation function.
 *
 * @param handle the alert handle
 * @param info pointer to store the information of the alert. The caller is
 * responsible for freeing the strings in the info struct in a method compatible
 * with the allocation function.
 * @param allocFunc the allocation function to use for allocating the strings in
 * the info struct
 * @return status of the operation
 */
MRC_Status MRC_CALLCONV MRC_Alert_GetAlertInfoExternalAlloc(
    MRC_AlertHandle handle, struct MRC_AlertInfo* info,
    void*(MRC_CALLCONV* allocFunc)(size_t));
}  // extern "C"
