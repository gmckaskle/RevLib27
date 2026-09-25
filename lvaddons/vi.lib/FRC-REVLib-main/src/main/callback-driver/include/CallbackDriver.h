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

#ifndef FRC_REVLIB_SRC_MAIN_CALLBACK_DRIVER_INCLUDE_CALLBACKDRIVER_H_
#define FRC_REVLIB_SRC_MAIN_CALLBACK_DRIVER_INCLUDE_CALLBACKDRIVER_H_

#include "rev/driver/REVLibDriver.h"

extern "C" {

typedef enum { SUCCESS = 0, NO_MESSAGE } ResultCodes;

struct Callbacks {
    void (*writeFrame)(int32_t busId, uint32_t messageId, const uint8_t* data,
                       uint8_t dataSize, int32_t periodMs, uint8_t isRtr,
                       int32_t* status);
    void (*receiveCanMessageLatest)(int32_t busId, uint32_t* messageID,
                                    uint32_t messageIDMask, uint8_t* data,
                                    uint8_t* dataSize, uint64_t* timeStamp,
                                    int32_t* status);
    void (*receiveCanMessageTimeout)(int32_t busId, uint32_t* messageID,
                                     uint32_t messageIDMask, uint8_t* data,
                                     uint8_t* dataSize, uint64_t* timeStamp,
                                     uint32_t timeout, int32_t* status);
    void (*receiveCanMessageNew)(int32_t busId, uint32_t id, uint8_t* data,
                                 uint8_t* dataSize, uint64_t* timestamp,
                                 int32_t* status);
    // No support for streams currently
    // No support for Sim currently
    uint64_t (*getTime)();
    uint64_t (*getCanBaseTime)();
};

}  // extern "C"

class CallbackDriver : public REVLibDriver {
    Callbacks* callbacks = nullptr;

public:
    explicit CallbackDriver(Callbacks* callbacks) : callbacks(callbacks) {}
    ~CallbackDriver() = default;
    void initializeDriver() override;
    void writeCanFrame(int32_t busId, uint32_t messageId, const uint8_t* data,
                       uint8_t dataSize, int32_t periodMs,
                       int32_t* status) override;
    void receiveCanMessageLatest(int32_t busId, uint32_t* messageID,
                                 uint32_t messageIDMask, uint8_t* data,
                                 uint8_t* dataSize, uint64_t* timeStamp,
                                 int32_t* status) override;
    void receiveCanMessageTimeout(int32_t busId, uint32_t* messageID,
                                  uint32_t messageIDMask, uint8_t* data,
                                  uint8_t* dataSize, uint64_t* timeStamp,
                                  uint32_t timeout, int32_t* status) override;
    void receiveCanMessageNew(int32_t busId, uint32_t id, uint8_t* data,
                              uint8_t* dataSize, uint64_t* timestamp,
                              int32_t* status) override;
    void writeCanRtrFrame(int32_t busId, uint32_t messageId, uint8_t dataSize,
                          int32_t periodMs, int32_t* status) override;
    int32_t createCanStream(int32_t busId, uint32_t messageId,
                            uint32_t messageMask, uint32_t maxMessages,
                            int32_t* status) override;
    void receiveCanMessages(int32_t streamHandle, CanMessage* messages,
                            uint32_t length, uint32_t* numRead,
                            int32_t* status) override;
    void closeStream(int32_t streamHandle) override;
    int32_t createSimDevice(const char* name) override;
    int32_t createSimValue(int32_t device, const char* name, int direction,
                           const REVLibValue defaultValue) override;
    void setSimValue(int32_t handle, REVLibValue* value) override;
    void getSimValue(int32_t handle, REVLibValue* value) override;
    void freeSimDevice(int32_t handle) override;
    void freeSimValue(int32_t handle) override;
    bool areOutputsEnabled() override;
    uint32_t allocatePreciseTimer() override;
    void preciseDelayMicroseconds(uint32_t handle,
                                  uint64_t timeMicroseconds) override;
    uint64_t getPreciseTime() override;
    uint64_t getCANPacketBaseTime() override;
    void closePreciseTimer(uint32_t handle) override;
    const char* getErrorMessage(int status) override;
    void sendError(int code, const char* message, bool printAlso) override;
    void sendWarning(int code, const char* message, bool printAlso) override;
    void sendConsoleLine(const char* message) override;
    c_REVLib_ErrorCode driverErrorToRevlibError(int status) override;
    void reportDeviceUsage(REVDevice device, int busId, int deviceId) override;
};

extern "C" {
REVLibDriver* createCallbackDriver(Callbacks* callbacks);
}  // extern "C"

#endif  // FRC_REVLIB_SRC_MAIN_CALLBACK_DRIVER_INCLUDE_CALLBACKDRIVER_H_
