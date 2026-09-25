/*
 * Copyright (c) 2018-2026 REV Robotics
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

#include "rev/REVLibErrors.h"

#include <rev/driver/REVLibDriver.h>

#include <algorithm>
#include <array>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <iterator>
#include <mutex>
#include <set>
#include <string>
#include <thread>

#include <fmt/format.h>

namespace {

static const std::array<const std::string, c_REVLibError_NumCodes>
    c_REVLib_ErrorTextTable = {
        // c_REVLibError_None
        "No Error",

        // c_REVLibError_General
        "General Error",

        // c_REVLibError_CANTimeout
        "timed out while waiting for",

        // c_REVLibError_NotImplemented,
        "Function or feature not implemented",

        // c_REVLibError_HAL
        "WPILib or External HAL Error:",

        // c_REVLibError_CantFindFirmware
        "Unable to retrieve SPARK firmware "
        "version. Please verify the deviceID field matches the "
        "configured CAN ID of the controller, and that "
        "the controller is connected to the CAN Bus.",

        // c_REVLibError_FirmwareTooOld
        "The SPARK firmware is too old and needs "
        "to be updated. Please use REV Hardware Client 2 to update the device "
        "to 2026 firmware or later. "
        "https://docs.revrobotics.com/rev-hardware-client-2",

        // c_REVLibError_FirmwareTooNew
        "The SPARK firmware version is too new for this version of REVLib. "
        "Please use REV Hardware Client 2 to update the device "
        "to 2026 firmware or later. "
        "https://docs.revrobotics.com/rev-hardware-client-2",

        // c_REVLibError_ParamInvalidID
        "Invalid parameter id",

        // c_REVLibError_ParamMismatchType
        "Parameter type mismatch for parameter id",

        // c_REVLibError_ParamAccessMode
        "Invalid parameter access mode parameter id",

        // c_REVLibError_ParamInvalid
        "Invalid parameter value, parameter id",

        // c_REVLibError_ParamNotImplementedDeprecated
        "Parameter is either not implemented or has been deprecated id ",

        // c_REVLibError_FollowConfigMismatch
        "Follower config setup check failed, check follower mode "
        "is set properly on device!",

        // c_REVLibError_Invalid
        "Error Invalid",

        // c_REVLibError_SetpointOutOfBounds
        "Setpoint is out of the defined range",

        // c_REVLib_UnknownError
        "Unknown Error",

        // c_REVLibError_CANDisconnected
        "CAN Output Buffer Full. Ensure a device is attached to the CAN bus.",

        // c_REVLibError_DuplicateCANId
        "A CANREVLib object with this ID was already created.",

        // c_REVLib_InvalidCANId
        "A CANREVLib object was given an invalid CAN ID.",

        // c_REVLibError_SparkMaxDataPortAlreadyConfiguredDifferently
        "An alternate encoder cannot be used with an absolute encoder or limit "
        "switches on SPARK MAX. Please ensure both are not configured "
        "simultaneously.",

        // c_REVLibError_SparkFlexBrushedWithoutDock
        "Cannot set motor type to kBrushed for SPARK Flex without a dock "
        "connected.",

        // c_REVLibError_InvalidBrushlessEncoderConfiguration
        "The inverted and counts per revolution parameters cannot explicitly "
        "be set for the primary encoder while in brushless mode.",

        // c_REVLibError_FeedbackSensorIncompatibleWithDataPortConfig
        "The selected feedback sensor is incompatible with the current data "
        "port configuration.",

        // c_REVLibError_ParamInvalidChannel
        "The specified channel ID is not within in the valid range.",

        // c_REVLibError_ParamInvalidValue
        "The specified value is not within the valid range",

        // c_REVLibError_CannotPersistParametersWhileEnabled
        "Cannot persist parameters while the device is enabled.",

        // c_REVLibError_NotReadyToReceiveCommand
        "Command was sent too early, please ensure all requirements "
        "are met before sending this command"};

static const std::array REVLib_DeviceNameTable{
    "Spark Flex", "Spark Max",      "Servo Hub", "PH",
    "PDH",        "Spline Encoder", "A301",      "Unknown REV Device"};

static_assert(REVLib_DeviceNameTable.size() == (UnknownREVDevice + 1));

}  // namespace

/*
The intent here is to buffer the errors to prevent too many from
being sent to the DS at once, while still sending the relevent
information. For example, if 6 SPARK MAX devices don't get their
firmware version sent back correctly, only send one message with
a list of all SPARK MAX devices that were not found instead of
spamming 6 messages. This is more important for any inline message
that may occur in a loop.

This could also be used to allow suppression of text errors and
also allow for a getLastError() type API.

TODO:

Also need a helpful message for specific HAL errors (e.g. the error
that is returned when the CAN bus is disconnected)
*/

namespace {

class REVLib_ErrorContext {
private:
    std::string
        m_errorTextBuffer[c_REVLibError_NumCodes];  // NOLINT(runtime/arrays)

    struct DeviceInfo {
        REVDevice type;
        int bus;
        int id;

        friend inline bool operator<(const DeviceInfo& a, const DeviceInfo& b) {
            return (a.type < b.type) && (a.bus < b.bus) && (a.id < b.id);
        }
    };

    std::set<DeviceInfo>
        m_errorCodeIds[c_REVLibError_NumCodes];  // NOLINT(runtime/arrays)

    int m_numErrors{0};
    bool m_doSuppressErrors{false};
    std::mutex m_mutex;

    REVLib_ErrorContext() = default;

public:
    REVLib_ErrorContext(const REVLib_ErrorContext&) = delete;
    REVLib_ErrorContext& operator=(const REVLib_ErrorContext&) = delete;
    REVLib_ErrorContext(REVLib_ErrorContext&&) = delete;
    REVLib_ErrorContext& operator=(REVLib_ErrorContext&&) = delete;

    static auto& Instance() {
        static REVLib_ErrorContext ec;
        return ec;
    }

    void FlushErrors() {
        std::lock_guard<std::mutex> lock(m_mutex);
        for (int errorCode = 0; errorCode < c_REVLibError_NumCodes;
             errorCode++) {
            if (m_errorCodeIds[errorCode].empty()) {
                continue;
            }

            auto result = fmt::memory_buffer();

            std::for_each(
                m_errorCodeIds[errorCode].begin(),
                m_errorCodeIds[errorCode].end(), [&](const DeviceInfo& device) {
                    fmt::format_to(std::back_inserter(result),
                                   "Bus {}: [{}] IDs: ", device.bus,
                                   REVLib_DeviceNameTable[device.type]);
                    if (device.id == 0) {
                        fmt::format_to(std::back_inserter(result),
                                       "(0 or broadcast), ");
                    } else {
                        fmt::format_to(std::back_inserter(result), "{}, ",
                                       device.id);
                    }
                });

            fmt::format_to(std::back_inserter(result), "{} {}",
                           c_REVLib_ErrorTextTable[errorCode],
                           m_errorTextBuffer[errorCode]);

            m_errorCodeIds[errorCode].clear();
            m_errorTextBuffer[errorCode].clear();

            std::string resultStr = fmt::to_string(result);
            getREVLibDriver()->sendError(errorCode, resultStr.c_str(), false);
        }
        m_numErrors = 0;
    }

    void SuppressErrors(bool suppress) {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_doSuppressErrors = suppress;

        if (suppress) {
            for (int errorCode = 0; errorCode < c_REVLibError_NumCodes;
                 errorCode++) {
                m_errorCodeIds[errorCode].clear();
                m_errorTextBuffer[errorCode].clear();
            }
        }
    }

    int ErrorCount() {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (m_doSuppressErrors) {
            return 0;
        }
        return m_numErrors;
    }

    void BufferError(c_REVLib_ErrorCode code, REVDevice deviceType, int busId,
                     int deviceId, std::string context) {
        // TODO: Report error for this
        if (deviceType < SparkFlex || deviceType > UnknownREVDevice) {
            deviceType = SparkFlex;
        }
        if (busId < 0 || busId > 4) {
            busId = 0;
        }
        if (deviceId < 0 || deviceId > 63) {
            deviceId = 0;
        }

        {
            std::lock_guard<std::mutex> lock(m_mutex);
            if (m_doSuppressErrors) {
                return;
            }

            DeviceInfo device{deviceType, busId, deviceId};
            if (code >= c_REVLibError_NumCodes) {
                m_errorTextBuffer[c_REVLibError_Unknown] =
                    std::string("code: ") + std::to_string(code);
                m_errorCodeIds[c_REVLibError_Unknown].insert(device);
            } else {
                if (!context.empty()) {
                    m_errorTextBuffer[code] = std::string(context);
                }
                m_errorCodeIds[code].insert(device);
            }
            m_numErrors++;
        }

#ifdef MAX_ERRORS_BEFORE_SEND
        if (m_numErrors > MAX_ERRORS_BEFORE_SEND) {
            std::thread([]() { FlushErrors(); }).detach();
        }
#endif
    }
};

void REVLib_BufferError(c_REVLib_ErrorCode code, REVDevice deviceType,
                        int busId, int deviceId, std::string context) {
    REVLib_ErrorContext::Instance().BufferError(code, deviceType, busId,
                                                deviceId, context);
}

}  // namespace

void c_REVLib_FlushErrors() { REVLib_ErrorContext::Instance().FlushErrors(); }

void c_REVLib_SuppressErrors(bool suppress) {
    REVLib_ErrorContext::Instance().SuppressErrors(suppress);
}

int c_REVLib_ErrorSize() {
    return REVLib_ErrorContext::Instance().ErrorCount();
}

void c_REVLib_SendError(c_REVLib_ErrorCode code, REVDevice deviceType,
                        int busId, int deviceId) {
    REVLib_BufferError(code, deviceType, busId, deviceId, std::string());
}

void c_REVLib_SendErrorText(c_REVLib_ErrorCode code, REVDevice deviceType,
                            int busId, int deviceId, std::string context) {
    REVLib_BufferError(code, deviceType, busId, deviceId, context);
}

const char* c_REVLib_ErrorFromCode(c_REVLib_ErrorCode code) {
    if (code >= c_REVLibError_NumCodes) {
        return "\0";
    } else {
        return c_REVLib_ErrorTextTable[code].c_str();
    }
}
