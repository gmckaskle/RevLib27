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

#include "rev/CANDriverPrivate.h"

#include <rev/driver/REVLibDriver.h>

#include <algorithm>
#include <array>
#include <cassert>
#include <string>
#include <thread>

#include <fmt/format.h>

#include "rev/REVUtils.h"

// TODO: For the kludge below
#include "rev/CANSparkFrames.h"

c_BaseCAN_Obj::c_BaseCAN_Obj(REVDevice deviceType, int busId, int deviceId)
    : m_deviceType{deviceType}, m_busId{busId}, m_deviceId{deviceId} {}

namespace {

constexpr int32_t kBaseDefaultCANTimeout_ms{20};
constexpr int32_t kBaseDefaultCANMaxRetryCount{5};

}  // namespace

int32_t c_BaseCAN_Obj::m_canTimeout_ms = kBaseDefaultCANTimeout_ms;
int32_t c_BaseCAN_Obj::m_canMaxRetryCount = kBaseDefaultCANMaxRetryCount;

void c_BaseCAN_Obj::SetBaseDefaultCanTimeoutMs(int32_t timeout_ms) {
    m_canTimeout_ms = timeout_ms;
}

void c_BaseCAN_Obj::SetBaseDefaultCanRetries(int32_t retries) {
    m_canMaxRetryCount = retries;
}

///////////////////////////////////
// Free functions:

void REVLib_SendError(c_BaseCAN_handle handle, c_REVLib_ErrorCode error) {
    REVLib_SendErrorText(handle, error, "");
}

void REVLib_SendErrorText(c_BaseCAN_handle handle, c_REVLib_ErrorCode error,
                          const std::string_view context) {
    c_REVLib_SendErrorText(error,
                           handle ? handle->m_deviceType : UnknownREVDevice,
                           handle ? handle->m_busId : 0,
                           handle ? handle->m_deviceId : 0, context.data());
}

void REVLib_SetLastError(c_BaseCAN_handle handle, c_REVLib_ErrorCode error) {
    if (handle == nullptr) {
        return;
    }
    handle->m_lastError = error;
}

c_REVLib_ErrorCode REVLib_GetLastError(c_BaseCAN_handle handle) {
    if (handle == NULL) {
        return c_REVLibError_Invalid;
    }
    return handle->m_lastError;
}

c_REVLib_ErrorCode REVLib_HALErrorCheck(c_BaseCAN_handle handle, int32_t status,
                                        const std::string_view context) {
    const c_REVLib_ErrorCode code =
        getREVLibDriver()->driverErrorToRevlibError(status);

    if (status != c_REVLibError_None) {
        const std::string errorMsg{getREVLibDriver()->getErrorMessage(status)};
        const std::string errorText = fmt::format("{}: {}", context, errorMsg);

        c_REVLib_SendErrorText(code,
                               handle ? handle->m_deviceType : UnknownREVDevice,
                               handle ? handle->m_busId : 0,
                               handle ? handle->m_deviceId : 0, errorText);
    }

    REVLib_SetLastError(handle, code);

    return code;
}

c_REVLib_ErrorCode REVLib_ReadCANPacketTimeout(
    c_BaseCAN_handle handle, std::span<uint8_t> packet, uint64_t& timestamp,
    int32_t frameId, int32_t timeoutPeriod_ms, int32_t expectedLength,
    const std::string_view context) {
    const int32_t defaultTimeout_ms =
        timeoutPeriod_ms * kPeriodicTimeoutMultiplier;
    const int32_t timeout_ms{
        std::max(defaultTimeout_ms, handle->m_periodicFrameTimeout_ms)};

    int32_t status{0};
    uint32_t id{static_cast<uint32_t>(frameId) | handle->m_deviceId};
    uint8_t length{};

    // We always fill out the packet with the latest data, even if it's outside
    // the timeout
    getREVLibDriver()->receiveCanMessageLatest(handle->m_busId, &id, 0x1FFFFFFF,
                                               packet.data(), &length,
                                               &timestamp, &status);

    const uint64_t now = getREVLibDriver()->getCANPacketBaseTime();

    c_REVLib_ErrorCode revlibError =
        REVLib_HALErrorCheck(handle, status, context);

    if (revlibError == c_REVLibError_None) {
        if (length != expectedLength) {
            fmt::println("c:{}, bus: {}, id:{:#x}, l:{}, exp:{}", context,
                         handle->m_busId, id, length, expectedLength);
            getREVLibDriver()->sendError(c_REVLibError_Invalid,
                                         "Length Mismatch", true);
            return c_REVLibError_Invalid;
        }

        // Check and see if we've timed out
        if (now - timestamp > static_cast<uint64_t>(timeout_ms * 1000)) {
            revlibError = c_REVLibError_CANTimeout;
        }
    } else {
        // Zero the packet data if we couldn't read anything.
        std::memset(packet.data(), 0, packet.size_bytes());
    }

    return revlibError;
}

namespace {
enum CANWriteMode {
    kWrite_OneShot = 0,
    kWrite_Repeating = 1,
    kWrite_StopRepeating = 2
};

c_REVLib_ErrorCode REVLib_WriteCANPacketCore(c_BaseCAN_handle handle,
                                             std::span<const uint8_t> packet,
                                             int32_t frameId, CANWriteMode mode,
                                             const std::string_view context) {
    int32_t status{};
    switch (mode) {
        case kWrite_OneShot:
            getREVLibDriver()->writeCanFrame(
                handle->m_busId, frameId | handle->m_deviceId, packet.data(),
                packet.size(), SEND_PERIOD_NO_REPEAT, &status);
            break;
        case kWrite_Repeating:
            getREVLibDriver()->writeCanFrame(
                handle->m_busId, frameId | handle->m_deviceId, packet.data(),
                packet.size(), handle->m_controlFramePeriod_ms, &status);
            break;
        case kWrite_StopRepeating:
            getREVLibDriver()->writeCanFrame(
                handle->m_busId, frameId | handle->m_deviceId, packet.data(),
                packet.size(), SEND_PERIOD_STOP, &status);
            break;
        default:
            getREVLibDriver()->sendError(c_REVLibError_Invalid,
                                         "Invalid WriteMode", true);
            status = c_REVLibError_Invalid;
            break;
    }

    if (status != 0) {
        return REVLib_HALErrorCheck(handle, status, context);
    }

    return c_REVLibError_None;
}
}  // namespace

c_REVLib_ErrorCode REVLib_WriteCANPacket(c_BaseCAN_handle handle,
                                         std::span<const uint8_t> packet,
                                         int32_t frameId,
                                         const std::string_view context) {
    return REVLib_WriteCANPacketCore(handle, packet, frameId, kWrite_OneShot,
                                     context);
}

c_REVLib_ErrorCode REVLib_WriteCANPacketRepeating(
    c_BaseCAN_handle handle, std::span<const uint8_t> packet, int32_t frameId,
    const std::string_view context) {
    return REVLib_WriteCANPacketCore(handle, packet, frameId, kWrite_Repeating,
                                     context);
}

c_REVLib_ErrorCode REVLib_StopCANPacketRepeating(
    c_BaseCAN_handle handle, int32_t frameId, const std::string_view context) {
    std::array<uint8_t, 0> zeroLengthDataPacket{};
    return REVLib_WriteCANPacketCore(handle, zeroLengthDataPacket, frameId,
                                     kWrite_StopRepeating, context);
}

c_REVLib_ErrorCode REVLib_WriteBroadcastCANPacket(
    std::span<const uint8_t> packet, int32_t busId, int32_t frameId,
    const std::string_view context) {
    int32_t status{};
    getREVLibDriver()->writeCanFrame(busId, frameId, packet.data(),
                                     packet.size(), SEND_PERIOD_NO_REPEAT,
                                     &status);

    if (status != 0) {
        return REVLib_HALErrorCheck(nullptr, status, context);
    }

    return c_REVLibError_None;
}

namespace {
c_REVLib_ErrorCode REVLib_WriteAndReadCANPacketCore(
    c_BaseCAN_handle handle, std::span<const uint8_t> writePacket,
    int32_t writeFrameId, std::span<uint8_t> readPacket, int32_t readFrameId,
    int32_t readExpectedLength, int32_t readTimeout_ms,
    const std::string_view context, bool isRtr) {
    int32_t status{};
    uint8_t length{};

    uint64_t lastTimestamp{};
    int32_t oldMessageStatus{};
    // Check for existing message before triggering a new one
    getREVLibDriver()->receiveCanMessageNew(
        handle->m_busId, readFrameId | handle->m_deviceId, readPacket.data(),
        &length, &lastTimestamp, &oldMessageStatus);

    const bool hadMessage = oldMessageStatus == 0;

    for (int retry = 0; retry < handle->m_canMaxRetryCount + 1; ++retry) {
        // Send
        if (isRtr) {
            getREVLibDriver()->writeCanRtrFrame(
                handle->m_busId, writeFrameId | handle->m_deviceId,
                readExpectedLength, SEND_PERIOD_NO_REPEAT, &status);
        } else {
            getREVLibDriver()->writeCanFrame(
                handle->m_busId, writeFrameId | handle->m_deviceId,
                writePacket.data(), writePacket.size(), SEND_PERIOD_NO_REPEAT,
                &status);
        }

        // Retry if a send failed
        if (status != 0) {
            continue;
        }

        uint64_t timestamp{};

        for (int ms = 0; ms <= readTimeout_ms; ++ms) {
            getREVLibDriver()->receiveCanMessageNew(
                handle->m_busId, readFrameId | handle->m_deviceId,
                readPacket.data(), &length, &timestamp, &status);
            // If we had no previous message, we can assume this message is new.
            // If we did have a previous message, check that it's not the same
            // timestamp (it's definitionally either the same or newer).
            if (status == 0 && (!hadMessage || timestamp != lastTimestamp)) {
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        // Don't try again if we received a response
        if (status == 0) {
            // TODO: There is a bug in the Max firmware where the data and,
            // hence, the length of the message is different than in the Flex.
            // So, ignore the difference here. Total kludge.
            if (readFrameId != SPARK_GET_MOTOR_INTERFACE_FRAME_ID) {
                if (length != readExpectedLength) {
                    fmt::println(
                        "c:{}, bus: {}, w:{:#x}, ws:{}, r:{:#x}, rs:{}, l:{}, "
                        "exp:{}",
                        context, handle->m_busId, writeFrameId,
                        writePacket.size(), readFrameId, readPacket.size(),
                        length, readExpectedLength);
                    getREVLibDriver()->sendError(c_REVLibError_Invalid,
                                                 "Length Mismatch", true);
                    return c_REVLibError_Invalid;
                }
            }
            return c_REVLibError_None;
        }
    }

    // We should only get here if all retries failed
    {
        static const int32_t HAL_CAN_TIMEOUT{-1154};
        REVLib_HALErrorCheck(handle, HAL_CAN_TIMEOUT, context);
    }

    return c_REVLibError_CANTimeout;
}
}  // namespace

c_REVLib_ErrorCode REVLib_WriteAndReadCANPacket(
    c_BaseCAN_handle handle, std::span<const uint8_t> writePacket,
    int32_t writeFrameId, std::span<uint8_t> readPacket, int32_t readFrameId,
    int32_t readExpectedLength, std::string_view context) {
    return REVLib_WriteAndReadCANPacketCore(
        handle, writePacket, writeFrameId, readPacket, readFrameId,
        readExpectedLength, handle->m_canTimeout_ms, context, false);
}

c_REVLib_ErrorCode REVLib_WriteAndReadCANPacketWithReadTimeout(
    c_BaseCAN_handle handle, std::span<const uint8_t> writePacket,
    int32_t writeFrameId, std::span<uint8_t> readPacket, int32_t readFrameId,
    int32_t readExpectedLength, int32_t customReadTimeout_ms,
    const std::string_view context) {
    return REVLib_WriteAndReadCANPacketCore(
        handle, writePacket, writeFrameId, readPacket, readFrameId,
        readExpectedLength, customReadTimeout_ms, context, false);
}

c_REVLib_ErrorCode REVLib_WriteAndReadRtrCANPacket(
    c_BaseCAN_handle handle, std::span<uint8_t> readPacket, int32_t frameId,
    int32_t expectedLength, std::string_view context) {
    return REVLib_WriteAndReadCANPacketCore(
        handle, std::span<uint8_t>{}, frameId, readPacket, frameId,
        expectedLength, handle->m_canTimeout_ms, context, true);
}

void REVLib_SendLowLevelError(int32_t errorCode, const std::string_view details,
                              bool alsoStdout) {
    getREVLibDriver()->sendError(errorCode, details.data(), alsoStdout);
}

void REVLib_SendLowLevelWarning(int32_t errorCode,
                                const std::string_view details,
                                bool alsoStdout) {
    getREVLibDriver()->sendWarning(errorCode, details.data(), alsoStdout);
}

extern "C" {
void REVLib_SetBaseDefaultCanTimeoutMs(int32_t timeout_ms) {
    c_BaseCAN_Obj::SetBaseDefaultCanTimeoutMs(timeout_ms);
}

void REVLIB_SetBaseDefaultCanRetries(int32_t retries) {
    c_BaseCAN_Obj::SetBaseDefaultCanRetries(retries);
}
}  // extern "C"
