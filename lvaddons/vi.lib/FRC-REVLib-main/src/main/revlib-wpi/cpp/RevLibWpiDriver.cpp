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

#include <rev/driver/REVLibDriver.h>

#include <chrono>
#include <cstring>
#include <map>
#include <string>

#include <fmt/format.h>
#include <fmt/ranges.h>
#include <wpi/hal/CAN.h>
#include <wpi/hal/DriverStation.h>
#include <wpi/hal/Errors.h>
#include <wpi/hal/HAL.h>
#include <wpi/hal/Notifier.h>
#include <wpi/hal/SimDevice.hpp>
#include <wpi/hal/UsageReporting.hpp>
#include <wpi/util/Synchronization.hpp>
#include <wpi/util/timestamp.hpp>

namespace {

struct ReceivedCanMessage {
    uint64_t lastTimeStamp;
    uint8_t length;
    uint8_t data[64];  // Be ready for FD size
};

}  // namespace

class RevLibWpiDriver : REVLibDriver {
    std::map<std::pair<int32_t, int32_t>, ReceivedCanMessage> messages;
    std::map<int32_t, HAL_CANStreamMessage*> streamMessageCaches;

    void initializeDriver() override { HAL_Initialize(500, 0); }

    void writeCanFrame(int32_t busId, uint32_t messageId, const uint8_t* data,
                       uint8_t dataSize, int32_t periodMs,
                       int32_t* status) override {
        // TODO: flags??
        HAL_CANMessage message{
            .flags = HAL_CAN_NO_FLAGS, .dataSize = dataSize, .data{}};
        std::memcpy(message.data, data, dataSize);
        HAL_CAN_SendMessage(busId, messageId, &message, periodMs, status);

#if 0  // For debugging
        fmt::println("S: {}:{:#x} -> {}", busId, messageId, *status);
#endif
    }

    void receiveCanMessageCore(int32_t busId, uint32_t* messageId,
                               uint32_t messageIdMask, uint8_t* data,
                               uint8_t* dataSize, uint64_t* timeStamp,
                               int32_t* status) {
        // TODO
        (void)messageIdMask;  // No longer in HAL API
        HAL_CANReceiveMessage halMessage;
        HAL_CAN_ReceiveMessage(busId, *messageId, &halMessage, status);

#if 0  // For debugging
        fmt::println("now = {}", wpi::util::Now());
        fmt::println("G: {}:{:#x} -> {}", busId, *messageId, *status);
#endif

        *dataSize = halMessage.message.dataSize;
        std::memcpy(data, halMessage.message.data, *dataSize);

#if 0  // For debugging
         std::span<const uint8_t> dataSpan(data, *dataSize);
         fmt::println("   {} @ {}({}): {:02X}", *dataSize,
                      halMessage.timeStamp, static_cast<uint32_t>(halMessage.timeStamp), fmt::join(dataSpan, ":"));
#endif

        if (timeStamp != nullptr) {
            *timeStamp = halMessage.timeStamp;
        }

        if (*status == 0) {
            ReceivedCanMessage& message = messages[{busId, *messageId}];
            message.length = *dataSize;
            message.lastTimeStamp = *timeStamp;
            std::memcpy(message.data, data, *dataSize);
        }
    }

    void receiveCanMessageTimeout(int32_t busId, uint32_t* messageID,
                                  uint32_t messageIDMask, uint8_t* data,
                                  uint8_t* dataSize, uint64_t* timeStamp,
                                  uint32_t timeout, int32_t* status) override {
        this->receiveCanMessageLatest(busId, messageID, messageIDMask, data,
                                      dataSize, timeStamp, status);

        const uint64_t now = wpi::util::Now();

#if 0  // For debugging
        fmt::println("timeout: now {} - ts {} ({}) > t*1000 = {}", now,
                     *timeStamp, now - *timeStamp,
                     (static_cast<uint64_t>(timeout) * 1000));
#endif

        if (now - *timeStamp > (static_cast<uint64_t>(timeout) * 1000)) {
            // we timed out
            *status = HAL_CAN_TIMEOUT;
        }
    }

    void receiveCanMessageNew(int32_t busId, uint32_t id, uint8_t* data,
                              uint8_t* dataSize, uint64_t* timestamp,
                              int32_t* status) override {
        this->receiveCanMessageCore(busId, &id, 0x1fffffff, data, dataSize,
                                    timestamp, status);
    }

    void receiveCanMessageLatest(int32_t busId, uint32_t* messageId,
                                 uint32_t messageIdMask, uint8_t* data,
                                 uint8_t* dataSize, uint64_t* timeStamp,
                                 int32_t* status) override {
        this->receiveCanMessageCore(busId, messageId, messageIdMask, data,
                                    dataSize, timeStamp, status);

        if (*status != 0) {
            auto i = messages.find({busId, *messageId});
            if (i != messages.end()) {
                std::memcpy(data, i->second.data, i->second.length);
                *dataSize = i->second.length;
                *timeStamp = i->second.lastTimeStamp;
                *status = 0;
            }
        }
    }

    void writeCanRtrFrame(int32_t busId, uint32_t messageId, uint8_t dataSize,
                          int32_t periodMs, int32_t* status) override {
        uint8_t data[64] = {};
        this->writeCanFrame(busId, messageId | HAL_CAN_IS_FRAME_REMOTE, data,
                            dataSize, periodMs, status);
    }

    int32_t createCanStream(int32_t busId, uint32_t messageId,
                            uint32_t messageMask, uint32_t maxMessages,
                            int32_t* status) override {
        HAL_CANStreamHandle handle = HAL_CAN_OpenStreamSession(
            busId, messageId, messageMask, maxMessages, status);
        if (*status == 0) {
            streamMessageCaches[handle] = static_cast<HAL_CANStreamMessage*>(
                std::malloc(sizeof(HAL_CANStreamMessage) * maxMessages));
        }
        return handle;
    }

    void receiveCanMessages(int32_t streamHandle, CanMessage* messages,
                            uint32_t length, uint32_t* numRead,
                            int32_t* status) override {
        HAL_CANStreamMessage* messageCache = streamMessageCaches[streamHandle];
        HAL_CAN_ReadStreamSession(streamHandle, messageCache, length, numRead,
                                  status);

        for (uint32_t i = 0u; i < *numRead; i++) {
            HAL_CANStreamMessage* received = &messageCache[i];
            messages[i].messageID = received->messageId;
            messages[i].dataSize = received->message.message.dataSize;
            messages[i].timestamp = received->message.timeStamp;
            std::memcpy(messages[i].data, received->message.message.data,
                        received->message.message.dataSize);
        }
    }

    void closeStream(int32_t streamHandle) override {
        if (streamMessageCaches.find(streamHandle) !=
            streamMessageCaches.end()) {
            std::free(streamMessageCaches[streamHandle]);
            streamMessageCaches.erase(streamHandle);
        }
        HAL_CAN_CloseStreamSession(streamHandle);
    }

    int32_t createSimDevice(const char* name) override {
        return HAL_CreateSimDevice(name);
    }

    int32_t createSimValue(int32_t device, const char* name, int direction,
                           const REVLibValue defaultValue) override {
        return HAL_CreateSimValue(device, name, direction,
                                  toHalValue(&defaultValue));
    }

    void setSimValue(int32_t handle, REVLibValue* value) override {
        HAL_SetSimValue(handle, toHalValue(value));
    }

    void getSimValue(int32_t handle, REVLibValue* value) override {
        HAL_Value halValue;
        std::memset(&halValue, 0, sizeof(HAL_Value));
        HAL_GetSimValue(handle, &halValue);

        value->type = toREVLibType(halValue.type);
        value->data.d = halValue.data.v_double;
    }

    void freeSimDevice(int32_t handle) override { HAL_FreeSimDevice(handle); }

    void freeSimValue(int32_t handle) override {
        // TODO(Landry): I thought this existed???
    }

    bool areOutputsEnabled() override { return HAL_GetOutputsEnabled(); }

    uint32_t allocatePreciseTimer() override {
        int32_t status;
        return HAL_CreateNotifier(&status);
    }

    void preciseDelayMicroseconds(uint32_t handle,
                                  uint64_t timeMicroseconds) override {
        int32_t status;
        HAL_SetNotifierAlarm(handle, timeMicroseconds, 0ULL, false, true,
                             &status);
        wpi::util::WaitForObject(handle);
    }

    uint64_t getPreciseTime() override { return HAL_GetMonotonicTime(); }

    uint64_t getCANPacketBaseTime() override { return wpi::util::Now(); }

    void closePreciseTimer(uint32_t handle) override {
        int32_t status;
        HAL_CancelNotifierAlarm(handle, false, &status);
        HAL_DestroyNotifier(handle);
    }

    const char* getErrorMessage(int status) override {
        return HAL_GetErrorMessage(status);
    }

    void sendError(int code, const char* message, bool printAlso) override {
        HAL_SendError(1, code, 0, message, "", "", printAlso);
    }

    void sendWarning(int code, const char* message, bool printAlso) override {
        HAL_SendError(0, code, 0, message, "", "", printAlso);
    }

    void sendConsoleLine(const char* message) override {
        HAL_SendConsoleLine(message);
    }

    c_REVLib_ErrorCode driverErrorToRevlibError(int status) override {
        switch (status) {
            case 0:
                return c_REVLibError_None;
            case HAL_CAN_TIMEOUT:
                return c_REVLibError_CANTimeout;
            case HAL_CAN_BUFFER_OVERRUN:
                return c_REVLibError_CANDisconnected;
            default:
                return c_REVLibError_HAL;
        }
    }

    void reportDeviceUsage(REVDevice device, int busId, int deviceId) override {
        std::string resource = [device, busId, deviceId]() {
            switch (device) {
                case ServoHub:
                    return fmt::format("REV_Servo_Hub[{},{}]", busId, deviceId);
                case SparkFlex:
                    return fmt::format("REV_SPARK_Flex[{},{}]", busId,
                                       deviceId);
                case SparkMax:
                    return fmt::format("REV_SPARK_MAX[{},{}]", busId, deviceId);
                case SplineEncoder:
                    return fmt::format("REV_MAXSpline_Encoder[{},{}]", busId,
                                       deviceId);
                case A301:
                    return fmt::format("FIRST_A301[{},{}]", busId, deviceId);
                default:
                    return fmt::format("REV_Unknown[{},{}]", busId, deviceId);
            }
        }();
        HAL_ReportUsage(resource, "");
    }

private:
    HAL_Type toHalType(REVLibType type) {
        switch (type) {
            case REVLibType::TYPE_BOOLEAN:
                return HAL_BOOLEAN;
            case REVLibType::TYPE_ENUM:
                return HAL_ENUM;
            case REVLibType::TYPE_INT:
                return HAL_INT;
            case REVLibType::TYPE_LONG:
                return HAL_LONG;
            case REVLibType::TYPE_DOUBLE:
                return HAL_DOUBLE;
            case REVLibType::TYPE_UNASSIGNED:
            default:
                return HAL_UNASSIGNED;
        }
    }

    HAL_Value toHalValue(const REVLibValue* value) {
        HAL_Value result;
        result.type = toHalType(value->type);
        result.data.v_double = value->data.d;

        return result;
    }

    REVLibType toREVLibType(HAL_Type type) {
        switch (type) {
            case HAL_BOOLEAN:
                return REVLibType::TYPE_BOOLEAN;
            case HAL_INT:
                return REVLibType::TYPE_INT;
            case HAL_LONG:
                return REVLibType::TYPE_LONG;
            case HAL_ENUM:
                return REVLibType::TYPE_ENUM;
            case HAL_DOUBLE:
                return REVLibType::TYPE_DOUBLE;
            case HAL_UNASSIGNED:
            default:
                return REVLibType::TYPE_UNASSIGNED;
        }
    }
};

RevLibWpiDriver driver;

extern "C" {
RevLibWpiDriver* getRevLibWpiDriver(void) { return &driver; }
}  // extern "C"
