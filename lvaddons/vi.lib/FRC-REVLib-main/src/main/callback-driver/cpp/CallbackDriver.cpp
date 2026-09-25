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

#include "CallbackDriver.h"

#include <thread>

extern "C" {
REVLibDriver* createCallbackDriver(Callbacks* callbacks) {
    return new CallbackDriver(callbacks);
}
}  // extern "C"

void CallbackDriver::initializeDriver() {
    // No-Op.
}

void CallbackDriver::receiveCanMessageLatest(int32_t busId, uint32_t* messageID,
                                             uint32_t messageIDMask,
                                             uint8_t* data, uint8_t* dataSize,
                                             uint64_t* timeStamp,
                                             int32_t* status) {
    callbacks->receiveCanMessageLatest(busId, messageID, messageIDMask, data,
                                       dataSize, timeStamp, status);
}

void CallbackDriver::receiveCanMessageTimeout(
    int32_t busId, uint32_t* messageID, uint32_t messageIDMask, uint8_t* data,
    uint8_t* dataSize, uint64_t* timeStamp, uint32_t timeout, int32_t* status) {
    callbacks->receiveCanMessageTimeout(busId, messageID, messageIDMask, data,
                                        dataSize, timeStamp, timeout, status);
}

void CallbackDriver::receiveCanMessageNew(int32_t busId, uint32_t id,
                                          uint8_t* data, uint8_t* dataSize,
                                          uint64_t* timestamp,
                                          int32_t* status) {
    callbacks->receiveCanMessageNew(busId, id, data, dataSize, timestamp,
                                    status);
}

void CallbackDriver::writeCanFrame(int32_t busId, uint32_t messageId,
                                   const uint8_t* data, uint8_t dataSize,
                                   int32_t periodMs, int32_t* status) {
    callbacks->writeFrame(busId, messageId, data, dataSize, periodMs, false,
                          status);
}

void CallbackDriver::writeCanRtrFrame(int32_t busId, uint32_t messageId,
                                      uint8_t dataSize, int32_t periodMs,
                                      int32_t* status) {
    uint8_t data[8];
    callbacks->writeFrame(busId, messageId, data, dataSize, periodMs, true,
                          status);
}

int32_t CallbackDriver::createCanStream(int32_t busId, uint32_t messageId,
                                        uint32_t messageMask,
                                        uint32_t maxMessages, int32_t* status) {
    return 1;
}

void CallbackDriver::receiveCanMessages(int32_t streamHandle,
                                        CanMessage* messages, uint32_t length,
                                        uint32_t* numRead, int32_t* status) {}

void CallbackDriver::closeStream(int32_t streamHandle) {}
// Sim is not implemented
int32_t CallbackDriver::createSimDevice(const char* name) { return 0; }
int32_t CallbackDriver::createSimValue(int32_t device, const char* name,
                                       int direction,
                                       const REVLibValue defaultValue) {
    return 0;
}
void CallbackDriver::setSimValue(int32_t handle, REVLibValue* value) {}
void CallbackDriver::getSimValue(int32_t handle, REVLibValue* value) {}
void CallbackDriver::freeSimDevice(int32_t handle) {}
void CallbackDriver::freeSimValue(int32_t handle) {}

// indicate that we can handle outputs
bool CallbackDriver::areOutputsEnabled() { return true; }

// Return non-zero to indicate that we can handle timers.
uint32_t CallbackDriver::allocatePreciseTimer() { return 1; }
void CallbackDriver::preciseDelayMicroseconds(uint32_t handle,
                                              uint64_t timeMicroseconds) {
    std::this_thread::sleep_for(std::chrono::microseconds(timeMicroseconds));
}
uint64_t CallbackDriver::getPreciseTime() { return callbacks->getTime(); }
uint64_t CallbackDriver::getCANPacketBaseTime() {
    return callbacks->getCanBaseTime();
}
void CallbackDriver::closePreciseTimer(uint32_t handle) {}
const char* CallbackDriver::getErrorMessage(int status) {
    switch (status) {
        case SUCCESS:
            return "success";
        case NO_MESSAGE:
            return "No message available";
        default:
            return "Unknown Error";
    }
}
void CallbackDriver::sendError(int code, const char* message, bool printAlso) {}
void CallbackDriver::sendWarning(int code, const char* message,
                                 bool printAlso) {}
void CallbackDriver::sendConsoleLine(const char* message) {}
c_REVLib_ErrorCode CallbackDriver::driverErrorToRevlibError(const int status) {
    switch (status) {
        case SUCCESS:
            return c_REVLibError_None;
        case NO_MESSAGE:
            return c_REVLibError_CANTimeout;
        default:
            return c_REVLibError_HAL;
    }
}
void CallbackDriver::reportDeviceUsage(REVDevice device, int busId,
                                       int deviceId) {}
