/*
 * Copyright (c) 2025-2026 REV Robotics
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of REV Robotics nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include <rev/REVLibDevices.h>
#include <rev/REVLibErrors.h>
#include <rev/REVLibValue.h>
#include <stdint.h>

#define REVLIB_INVALID_HANDLE 0

#define SEND_PERIOD_NO_REPEAT 0
#define SEND_PERIOD_STOP -1

#ifdef __cplusplus
extern "C" {
#endif

struct CanMessage {
    uint32_t messageID;
    uint8_t dataSize;
    uint32_t timestamp;
    bool isRtr;
    uint8_t data[64];  // Be ready for FD size
};

#ifdef __cplusplus
}  // extern "C"
#endif

class REVLibDriver {
public:
    virtual void initializeDriver() = 0;
    virtual void writeCanFrame(int32_t busId, uint32_t messageId,
                               const uint8_t* data, uint8_t dataSize,
                               int32_t periodMs, int32_t* status) = 0;
    virtual void receiveCanMessageLatest(int32_t busId, uint32_t* messageId,
                                         uint32_t messageIdMask, uint8_t* data,
                                         uint8_t* dataSize, uint64_t* timeStamp,
                                         int32_t* status) = 0;
    virtual void receiveCanMessageTimeout(int32_t busId, uint32_t* messageId,
                                          uint32_t messageIdMask, uint8_t* data,
                                          uint8_t* dataSize,
                                          uint64_t* timeStamp, uint32_t timeout,
                                          int32_t* status) = 0;
    virtual void receiveCanMessageNew(int32_t busId, uint32_t id, uint8_t* data,
                                      uint8_t* dataSize, uint64_t* timestamp,
                                      int32_t* status) = 0;
    virtual void writeCanRtrFrame(int32_t busId, uint32_t messageId,
                                  uint8_t dataSize, int32_t periodMs,
                                  int32_t* status) = 0;
    virtual int32_t createCanStream(int32_t busId, uint32_t messageId,
                                    uint32_t messageMask, uint32_t maxMessages,
                                    int32_t* status) = 0;
    virtual void receiveCanMessages(int32_t streamHandle, CanMessage* messages,
                                    uint32_t length, uint32_t* numRead,
                                    int32_t* status) = 0;
    virtual void closeStream(int32_t streamHandle) = 0;

    virtual int32_t createSimDevice(const char* name) = 0;
    virtual int32_t createSimValue(int32_t device, const char* name,
                                   int direction,
                                   const REVLibValue defaultValue) = 0;
    virtual void setSimValue(int32_t handle, REVLibValue* value) = 0;
    virtual void getSimValue(int32_t handle, REVLibValue* value) = 0;
    virtual void freeSimDevice(int32_t handle) = 0;
    virtual void freeSimValue(int32_t handle) = 0;

    virtual bool areOutputsEnabled() = 0;

    virtual uint32_t allocatePreciseTimer() = 0;
    virtual void preciseDelayMicroseconds(uint32_t handle,
                                          uint64_t timeMicroseconds) = 0;
    virtual uint64_t getPreciseTime() = 0;
    virtual uint64_t getCANPacketBaseTime() = 0;
    virtual void closePreciseTimer(uint32_t handle) = 0;

    virtual const char* getErrorMessage(int status) = 0;
    // TODO(Landry): Should LabView be an extra param?
    virtual void sendError(int code, const char* message, bool printAlso) = 0;
    virtual void sendWarning(int code, const char* message, bool printAlso) = 0;
    virtual void sendConsoleLine(const char* message) = 0;

    virtual c_REVLib_ErrorCode driverErrorToRevlibError(int status) = 0;

    virtual void reportDeviceUsage(REVDevice device, int32_t busId,
                                   int deviceId) = 0;
};

#ifdef __cplusplus
extern "C" {
#endif

REVLibDriver* getREVLibDriver(void);
void setREVLibDriver(REVLibDriver* newDriver);

#ifdef __cplusplus
}  // extern "C"
#endif
