// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "mrclib/MrcLibTypes.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Callback used to receive a PCM status packet.
 *
 * @param userData user data supplied to MRC_PCM_Tokenize
 * @param receivedBytes output buffer for the 8-byte status packet
 * @return status of the receive operation; a non-success status causes the
 * tokenizer to emit a zero token and retry the receive on its next call
 */
typedef MRC_Status(MRC_CALLCONV* MRC_PCM_ReceiveMessageCallback)(
    void* userData, uint8_t* receivedBytes);

/**
 * @param busId bus identifier from 0 to 31
 * @param deviceId device identifier from 0 to 63
 * @param toSendBytes packet to tokenize
 * @param receiveMessage callback used to receive a PCM status packet when the
 * token is missing or stale
 * @param userData non-null user data passed to receiveMessage
 * @return status of the operation
 */
MRC_Status MRC_CALLCONV
MRC_PCM_Tokenize(uint8_t busId, uint8_t deviceId, uint8_t* toSendBytes,
                 MRC_PCM_ReceiveMessageCallback receiveMessage, void* userData);

#ifdef __cplusplus
}  // extern "C"
#endif
