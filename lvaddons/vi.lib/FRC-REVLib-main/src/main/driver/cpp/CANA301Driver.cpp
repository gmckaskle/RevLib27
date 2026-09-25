/*
 * Copyright (c) 2026 REV Robotics
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

#include "first/CANA301Driver.h"

#include <stdint.h>

#include <algorithm>
#include <array>
// #include <cassert>
#include <chrono>  // Required for std::chrono units
// #include <cmath>
#include <concepts>
// #include <cstdio>
// #include <cstdlib>
// #include <cstring>
// #include <mutex>
// #include <set>
// #include <sstream>
#include <string>
#include <thread>  // Required for std::this_thread::sleep_for
// #include <unordered_map>
// #include <vector>

#include <fmt/format.h>

// TODO: (dave) get rid of:
#include <rev/driver/REVLibDriver.h>

#include "rev/CANDriverPrivate.h"
// #include "rev/REVLibDaemon.h"
// #include "rev/REVLibErrors.h"
// #include "rev/REVUtils.h"
// #include "rev/A301FrameManager.h"

#include "first/SimA301.h"
#include "first/CANA301Frames.h"

namespace {

// TODO: remove when eeprom writes are optimized
constexpr int32_t kTempReadTimeout{500};

constexpr int kMaxPacketLength{8};

bool isVersionTooNew(const c_A301_FirmwareVersion& fwVersion) {
    return fwVersion.major > 28;
}

enum c_A301_ParameterStatus {
    c_A301_kParamOK = 0,
    c_A301_kInvalidID = 1,
    c_A301_kMismatchType = 2,
    c_A301_kAccessMode = 3,
    c_A301_kInvalid = 4,
    c_A301_kNotImplementedDeprecated = 5
};

static_assert(sizeof(float) == sizeof(uint32_t), "float isn't 32 bits wide");

CAN_ExistingDeviceIds s_A301_ExistingDeviceIds;

constexpr std::array<uint8_t, 0> zeroLengthDataPacket{};

// RAII Helper class that wraps a CANStream to ease error cleanup
class CanStreamWrapper {
    inline static REVLibDriver* const drv{getREVLibDriver()};

    const int32_t busId;
    const uint32_t msgId;
    const uint32_t mask;
    const uint32_t maxMsgs;

    int32_t status{};

    const int32_t handle;

public:
    CanStreamWrapper(int32_t busId, uint32_t messageId, uint32_t messageIDMask,
                     uint32_t maxMessages)
        : busId{busId},
          msgId{messageId},
          mask{messageIDMask},
          maxMsgs{maxMessages},
          handle{drv->createCanStream(busId, msgId, mask, maxMsgs, &status)} {
        if (status) {
            fmt::println("CanStreamWrapper::constructor - error: {}", status);
        }
    }

    ~CanStreamWrapper() { drv->closeStream(handle); }

    int32_t GetStatus() const { return status; }

    uint32_t ReadStream(CanMessage* messages, uint32_t messagesToRead) {
        uint32_t numRead{};
        drv->receiveCanMessages(handle, messages, messagesToRead, &numRead,
                                &status);
        if (status < 0) {
            fmt::println("CanStreamWrapper::ReadStream - error: {}", status);
        }

        if (numRead == 0) {
            fmt::println("No messages read. Please try again (later).");
        }

        return numRead;
    }
};

// TODO: experimental motion core device ID auto-detect
int A301_AutoDetectDeviceId(int busId) {
    constexpr uint32_t kMessageDeviceNumberMask{
        0b00000'00000000'00000'0000'111111};
    constexpr uint32_t kMessageAllExceptDeviceNumberMask{
        ~kMessageDeviceNumberMask};

    CanStreamWrapper stream{busId, A301_STATUS_0_FRAME_ID,
                            kMessageAllExceptDeviceNumberMask, 1u};
    if (auto status = stream.GetStatus(); status < 0) {
        fmt::println("CanStreamWrapper construction failed, status: {}",
                     status);
        return -1;
    }

    CanMessage msg{};

    // The Systemcore appears to require around 10ms after CANStream creation
    // before a read succeeds.
    using namespace std::chrono_literals;
    std::this_thread::sleep_for(10ms);

    constexpr auto kMaxAutoDetectRetries{4};

    int i = 0;
    for (; i < kMaxAutoDetectRetries; ++i) {
        const uint32_t numRead = stream.ReadStream(&msg, 1u);
        if (auto status = stream.GetStatus(); (status < 0) || (numRead != 1u)) {
            fmt::println("ReadStream failed, status: {}, numRead: {}", status,
                         numRead);
        } else {
            break;
        }

        std::this_thread::sleep_for(100ms);
    }
    if (i == kMaxAutoDetectRetries) {
        fmt::println("ReadStream retries failed");
        return -1;
    }

    const int deviceId = msg.messageID & kMessageDeviceNumberMask;
    fmt::println("Auto-detected device ID: {} on bus: {}", deviceId, busId);
    return deviceId;
}

}  // namespace

// NOTE: This struct can't be in the unnamed namespace

struct c_A301_Obj : public c_BaseCAN_Obj {
    c_A301_Obj(int busId, int deviceId)
        // TODO: merge base into A301
        : c_BaseCAN_Obj(A301, busId, deviceId),
          m_simDevice{c_SIM_A301_Create(busId, deviceId)} {}

    c_A301_FirmwareVersion m_firmwareVersion{};

    bool m_inverted{false};

    int m_activeSetpointApi{-1};

    c_SIM_A301_handle m_simDevice{nullptr};
};

namespace {

constexpr std::array kControlTypeFrames{
    A301_DUTY_CYCLE_SETPOINT_FRAME_ID,         // 0
    A301_VELOCITY_SETPOINT_FRAME_ID,           // 1
    A301_RELATIVE_POSITION_SETPOINT_FRAME_ID,  // 2
    A301_CURRENT_SETPOINT_FRAME_ID,            // 3
    A301_ABSOLUTE_POSITION_SETPOINT_FRAME_ID,  // 4
};

// c_A301_RegisterId() must be called first
c_A301_handle A301_Create_Inplace(int busId, int deviceId,
                                  c_REVLib_ErrorCode* status) {
    if (!CAN_IsValidDeviceId(deviceId)) {
        c_REVLib_SendError(c_REVLibError_InvalidCANId, A301, busId, deviceId);

        // Don't allow a nullptr to be returned, invalid deviceId will just
        // fail, error is already sent
        // return nullptr;
    }

    c_A301_handle handle = new c_A301_Obj(busId, deviceId);

    if (!s_A301_ExistingDeviceIds.ContainsDevice(busId, deviceId)) {
        // I used SendError directly instead of c_REVLib_SendError because
        // we don't want to expose a REVLib error code for this condition.
        REVLib_SendLowLevelError(
            1, "c_A301_RegisterId() was not called before c_A301_Create()");
    }

    REVLib_SetLastError(handle, c_REVLibError_None);

    *status = c_REVLibError_None;
    c_A301_FirmwareVersion fwVersion;
    if (c_SIM_A301_IsSim(handle->m_simDevice)) {
        *status = c_REVLibError_None;
    } else if (c_A301_GetFirmwareVersion(handle, &fwVersion) !=
               c_REVLibError_None) {
        *status = c_REVLibError_CantFindFirmware;
    } else {
        handle->m_firmwareVersion = fwVersion;

        if (fwVersion.versionRaw < kMinA301FirmwareVersion ||
            (fwVersion.versionRaw == kMinA301FirmwareVersion &&
             fwVersion.prerelease != 0 &&
             fwVersion.prerelease < kMinA301DebugBuild)) {
            *status = c_REVLibError_FirmwareTooOld;
        } else if (isVersionTooNew(fwVersion)) {
            *status = c_REVLibError_FirmwareTooNew;
        } else if (!kAllowDebugFirmwareBuilds && fwVersion.prerelease != 0) {
            *status = c_REVLibError_FirmwareTooOld;
        }
    }

    if (*status != c_REVLibError_None) {
        REVLib_SendError(handle, *status);
    }

    // Make sure no frames are repeating before registering
    int halStatus{};
    for (int controlType : kControlTypeFrames) {
        halStatus = REVLib_StopCANPacketRepeating(
            handle, controlType, "Create in place: Stop Repeating");
        if (halStatus != c_REVLibError_None) {
            break;
        }
    }

    handle->m_activeSetpointApi = 0;

    return handle;
}

struct status_period_info_t {
    c_A301_PeriodicFrame frameId;
    int* handle_statusPeriod_ms;
};

void GetA301StatusPeriodsFromDevice(c_A301_handle handle) {
    const std::array<status_period_info_t, 4> statusPeriods{
        {{c_A301_kStatus0, &handle->m_status0Period_ms},
         {c_A301_kStatus1, &handle->m_status1Period_ms},
         {c_A301_kStatus2, &handle->m_status2Period_ms},
         {c_A301_kStatus3, &handle->m_status3Period_ms}}};

    for (const auto& sp : statusPeriods) {
        uint32_t period{100};
        auto status{c_A301_GetStatusFramePeriod(handle, sp.frameId, &period)};

        if (status == c_REVLibError_None) {
            *sp.handle_statusPeriod_ms = static_cast<int32_t>(period);
        }
    }
}

constexpr int kFirstMotioncoreBus{5u};

}  // namespace

// High-level libraries should throw an exception if this
// returns c_REVLibError_DuplicateCANId
c_REVLib_ErrorCode c_A301_RegisterId(int busId, int deviceId,
                                     int* actualDeviceId) {
    // If on a Motioncore, ignore deviceId and try to detect instead
    if (busId >= kFirstMotioncoreBus) {
        const int detectedId{A301_AutoDetectDeviceId(busId)};
        if (detectedId != -1) {
            deviceId = detectedId;
        }
    }
    *actualDeviceId = deviceId;

    if (!s_A301_ExistingDeviceIds.InsertDevice(busId, deviceId)) {
        return c_REVLibError_DuplicateCANId;
    }
    return c_REVLibError_None;
}

// c_A301_RegisterId() must be called first
c_A301_handle c_A301_Create(int busId, int deviceId,
                            c_REVLib_ErrorCode* status) {
    // If on a Motioncore, ignore deviceId and try to detect instead
    if (busId >= kFirstMotioncoreBus) {
        const int detectedId{A301_AutoDetectDeviceId(busId)};
        if (detectedId != -1) {
            deviceId = detectedId;
        }
    }

    c_A301_handle handle = A301_Create_Inplace(busId, deviceId, status);

    if (*status != c_REVLibError_None) {
        return handle;
    }

    GetA301StatusPeriodsFromDevice(handle);

    c_A301_SetpointCommand(handle, 0.0f, c_A301_kControlType_DutyCycle, 0.0f);

    // TODO
    getREVLibDriver()->reportDeviceUsage(REVDevice::A301, busId, deviceId);

    return handle;
}

void c_A301_Close(c_A301_handle handle) {
    if (handle == NULL) {
        return;
    }

    s_A301_ExistingDeviceIds.RemoveDevice(
        static_cast<uint8_t>(handle->m_busId),
        static_cast<uint8_t>(handle->m_deviceId));

    REVLib_StopCANPacketRepeating(handle, handle->m_activeSetpointApi,
                                  "A301 destroy: Stop Repeating");

    if (handle->m_simDevice != NULL) {
        c_SIM_A301_Close(handle->m_simDevice);
    }
}

void c_A301_Destroy(c_A301_handle handle) {
    if (handle == NULL) {
        return;
    }

    c_SIM_A301_Destroy(handle->m_simDevice);
    delete handle;
}

c_REVLib_ErrorCode c_A301_GetFirmwareVersion(
    c_A301_handle handle, c_A301_FirmwareVersion* fwVersion) {
    // Start with invalid firmware
    *fwVersion = {};

    if (c_SIM_A301_IsSim(handle->m_simDevice)) {
        return c_REVLibError_None;
    }

    std::array<uint8_t, kMaxPacketLength> packedData;
    const c_REVLib_ErrorCode status = REVLib_WriteAndReadRtrCANPacket(
        handle, packedData, A301_GET_FIRMWARE_VERSION_FRAME_ID,
        A301_GET_FIRMWARE_VERSION_LENGTH, "Get Firmware Version");

    if (status != c_REVLibError_None) {
        // Firmware version not received
        REVLib_SetLastError(handle, c_REVLibError_CantFindFirmware);
        return c_REVLibError_CantFindFirmware;
    }

    a301_get_firmware_version_t frameData;
    a301_get_firmware_version_unpack(&frameData, packedData.data(),
                                     A301_GET_FIRMWARE_VERSION_LENGTH);

    fwVersion->major = frameData.major;
    fwVersion->minor = frameData.minor;
    fwVersion->patch = frameData.patch;
    fwVersion->prerelease = frameData.prerelease;
    fwVersion->versionRaw = (static_cast<uint32_t>(fwVersion->major) << 24) |
                            (static_cast<uint32_t>(fwVersion->minor) << 16) |
                            static_cast<uint32_t>(fwVersion->patch);

    REVLib_SetLastError(handle, c_REVLibError_None);
    return c_REVLibError_None;
}

c_REVLib_ErrorCode c_A301_GetBusId(c_A301_handle handle, int* busId) {
    *busId = handle->m_busId;
    REVLib_SetLastError(handle, c_REVLibError_None);
    return c_REVLibError_None;
}

c_REVLib_ErrorCode c_A301_GetDeviceId(c_A301_handle handle, int* deviceId) {
    *deviceId = handle->m_deviceId;
    REVLib_SetLastError(handle, c_REVLibError_None);
    return c_REVLibError_None;
}

#if 0
void c_A301_SetPeriodicFrameTimeout(c_A301_handle handle, int timeoutMs) {
    if (timeoutMs < 0) {
        return;
    }
    handle->m_periodicFrameTimeout_ms = timeoutMs;
}

void c_A301_SetCANMaxRetries(c_A301_handle handle, int numRetries) {
    if (numRetries < 0) {
        return;
    }
    handle->m_canMaxRetryCount = numRetries;
}

void c_A301_SetControlFramePeriod(c_A301_handle handle, int periodMs) {
    if (periodMs < 0) {
        return;
    }
    handle->m_controlFramePeriod_ms = periodMs;
}

int c_A301_GetControlFramePeriod(c_A301_handle handle) {
    return handle->m_controlFramePeriod_ms;
}

void enableFrameIfNeeded(const c_A301_handle handle,
                         const c_REVLib_ErrorCode revlibError,
                         const int frameId) {
    if (revlibError == c_REVLibError_None) {
        c_A301_DequeueStatusFrame(handle, frameId);
    } else {
        c_A301_QueueStatusFrame(handle, frameId);
    }
}
#endif

namespace {
c_REVLib_ErrorCode SIM_A301_GetPeriodicStatus0(
    c_A301_handle handle, c_A301_PeriodicStatus0* rawFrame) {
    *rawFrame = {};

#if 0
    rawFrame->appliedOutput = c_SIM_A301_GetAppliedOutput(handle->m_simDevice);
    rawFrame->voltage = c_SIM_A301_GetBusVoltage(handle->m_simDevice);
    rawFrame->current = c_SIM_A301_GetOutputCurrent(handle->m_simDevice);
    rawFrame->motorTemperature =
        c_SIM_A301_GetMotorTemperature(handle->m_simDevice);

    rawFrame->inverted = c_SIM_A301_GetInverted(handle->m_simDevice);
    rawFrame->gearboxRPM = c_A301_GearboxRPM_215;
    rawFrame->primaryHeartbeatLock = 0;
#endif

    return c_REVLibError_None;
}
}  // namespace

c_REVLib_ErrorCode c_A301_GetPeriodicStatus0(c_A301_handle handle,
                                             c_A301_PeriodicStatus0* rawFrame) {
    if (c_SIM_A301_IsSim(handle->m_simDevice)) {
        return SIM_A301_GetPeriodicStatus0(handle, rawFrame);
    }

    std::array<uint8_t, kMaxPacketLength> packedData{};

    c_REVLib_ErrorCode revlibError = REVLib_ReadCANPacketTimeout(
        handle, packedData, rawFrame->timestamp, A301_STATUS_0_FRAME_ID,
        handle->m_status0Period_ms, A301_STATUS_0_LENGTH, "Period Status 0");

    // REVLib_ReadCANPacketTimeout will always either return the latest packet
    // data or zero the packet
    a301_status_0_t frameData;
    a301_status_0_unpack(&frameData, packedData.data(), A301_STATUS_0_LENGTH);

    rawFrame->appliedOutput =
        a301_status_0_applied_output_decode(frameData.applied_output) *
        (handle->m_inverted ? -1.0 : 1.0);
    rawFrame->voltage = a301_status_0_voltage_decode(frameData.voltage);
    rawFrame->current = a301_status_0_current_decode(frameData.current);
    rawFrame->motorTemperature = frameData.motor_temperature;
    rawFrame->inverted = handle->m_inverted;
    rawFrame->gearboxRPM = frameData.gearbox_rpm;
    rawFrame->primaryHeartbeatLock = frameData.primary_heartbeat_lock;

    return revlibError;
}

#if 0
namespace {
c_REVLib_ErrorCode SIM_A301_GetPeriodicStatus1(
    c_A301_handle handle, c_A301_PeriodicStatus1* rawFrame) {
    if (c_SIM_A301_GetSimFaultManagerExists(handle->m_simDevice)) {
        c_SIM_A301_FaultManager_handle simFaultManager =
            c_SIM_A301_GetOrCreateSimFaultManager(handle->m_simDevice);
        c_SIM_A301_SetRawframeFromSimFaults(simFaultManager, rawFrame);
    } else {
        // if the fault manager has not been set up, all faults off
        rawFrame->otherFault = 0;
        rawFrame->motorTypeFault = 0;
        rawFrame->sensorFault = 0;
        rawFrame->canFault = 0;
        rawFrame->temperatureFault = 0;
        rawFrame->drvFault = 0;
        rawFrame->escEepromFault = 0;
        rawFrame->firmwareFault = 0;
        rawFrame->brownoutWarning = 0;
        rawFrame->overcurrentWarning = 0;
        rawFrame->escEepromWarning = 0;
        rawFrame->extEepromWarning = 0;
        rawFrame->sensorWarning = 0;
        rawFrame->stallWarning = 0;
        rawFrame->hasResetWarning = 0;
        rawFrame->otherWarning = 0;
        rawFrame->otherStickyFault = 0;
        rawFrame->motorTypeStickyFault = 0;
        rawFrame->sensorStickyFault = 0;
        rawFrame->canStickyFault = 0;
        rawFrame->temperatureStickyFault = 0;
        rawFrame->drvStickyFault = 0;
        rawFrame->escEepromStickyFault = 0;
        rawFrame->firmwareStickyFault = 0;
        rawFrame->brownoutStickyWarning = 0;
        rawFrame->overcurrentStickyWarning = 0;
        rawFrame->escEepromStickyWarning = 0;
        rawFrame->extEepromStickyWarning = 0;
        rawFrame->sensorStickyWarning = 0;
        rawFrame->stallStickyWarning = 0;
        rawFrame->hasResetStickyWarning = 0;
        rawFrame->otherStickyWarning = 0;
    }
    rawFrame->isFollower = c_SIM_A301_IsFollower(handle->m_simDevice);
    return c_REVLibError_None;
}
}  // namespace
#endif

c_REVLib_ErrorCode c_A301_GetPeriodicStatus1(c_A301_handle handle,
                                             c_A301_PeriodicStatus1* rawFrame) {
    // if (c_SIM_A301_IsSim(handle->m_simDevice)) {
    //     return SIM_A301_GetPeriodicStatus1(handle, rawFrame);
    // }

    std::array<uint8_t, kMaxPacketLength> packedData{};

    c_REVLib_ErrorCode revlibError = REVLib_ReadCANPacketTimeout(
        handle, packedData, rawFrame->timestamp, A301_STATUS_1_FRAME_ID,
        handle->m_status1Period_ms, A301_STATUS_1_LENGTH, "Period Status 1");

    //     enableFrameIfNeeded(handle, revlibError, 1);

    a301_status_1_t frameData;
    a301_status_1_unpack(&frameData, packedData.data(), A301_STATUS_1_LENGTH);

    rawFrame->otherFault = frameData.other_fault;
    rawFrame->motorTypeFault = frameData.motor_type_fault;
    rawFrame->sensorFault = frameData.sensor_fault;
    rawFrame->canFault = frameData.can_fault;
    rawFrame->temperatureFault = frameData.temperature_fault;
    rawFrame->drvFault = frameData.drv_fault;
    rawFrame->escEepromFault = frameData.esc_eeprom_fault;
    rawFrame->firmwareFault = frameData.firmware_fault;
    rawFrame->motorStartupFault = frameData.motor_startup_fault;

    rawFrame->brownoutWarning = frameData.brownout_warning;
    rawFrame->overcurrentWarning = frameData.overcurrent_warning;
    rawFrame->escEepromWarning = frameData.esc_eeprom_warning;
    rawFrame->extEepromWarning = frameData.ext_eeprom_warning;
    rawFrame->sensorWarning = frameData.sensor_warning;
    rawFrame->stallWarning = frameData.stall_warning;
    rawFrame->hasResetWarning = frameData.has_reset_warning;
    rawFrame->otherWarning = frameData.other_warning;
    rawFrame->overvoltageWarning = frameData.overvoltage_warning;
    rawFrame->motorLoopSpeedWarning = frameData.motor_loop_speed_warning;

    rawFrame->otherStickyFault = frameData.other_sticky_fault;
    rawFrame->motorTypeStickyFault = frameData.motor_type_sticky_fault;
    rawFrame->sensorStickyFault = frameData.sensor_sticky_fault;
    rawFrame->canStickyFault = frameData.can_sticky_fault;
    rawFrame->temperatureStickyFault = frameData.temperature_sticky_fault;
    rawFrame->drvStickyFault = frameData.drv_sticky_fault;
    rawFrame->escEepromStickyFault = frameData.esc_eeprom_sticky_fault;
    rawFrame->firmwareStickyFault = frameData.firmware_sticky_fault;
    rawFrame->motorStartupStickyFault = frameData.motor_startup_sticky_fault;

    rawFrame->brownoutStickyWarning = frameData.brownout_sticky_warning;
    rawFrame->overcurrentStickyWarning = frameData.overcurrent_sticky_warning;
    rawFrame->escEepromStickyWarning = frameData.esc_eeprom_sticky_warning;
    rawFrame->extEepromStickyWarning = frameData.ext_eeprom_sticky_warning;
    rawFrame->sensorStickyWarning = frameData.sensor_sticky_warning;
    rawFrame->stallStickyWarning = frameData.stall_sticky_warning;
    rawFrame->hasResetStickyWarning = frameData.has_reset_sticky_warning;
    rawFrame->otherStickyWarning = frameData.other_sticky_warning;
    rawFrame->overvoltageStickyWarning = frameData.overvoltage_sticky_warning;
    rawFrame->motorLoopSpeedStickyWarning =
        frameData.motor_loop_speed_sticky_warning;

    return revlibError;
}

#if 0
namespace {
c_REVLib_ErrorCode SIM_A301_GetPeriodicStatus2(
    c_A301_handle handle, c_A301_PeriodicStatus2* rawFrame) {
    c_SIM_A301_RelativeEncoder_handle simRelativeEncoder =
        c_SIM_A301_GetOrCreateSimRelativeEncoder(handle->m_simDevice);

    rawFrame->primaryEncoderPosition = static_cast<float>(
        c_SIM_A301_GetSimRelativeEncoderPosition(simRelativeEncoder));
    rawFrame->relativeEncoderVelocity = static_cast<float>(
        c_SIM_A301_GetSimRelativeEncoderVelocity(simRelativeEncoder));
    return c_REVLibError_None;
}
}  // namespace
#endif

c_REVLib_ErrorCode c_A301_GetPeriodicStatus2(c_A301_handle handle,
                                             c_A301_PeriodicStatus2* rawFrame) {
    // if (c_SIM_A301_IsSim(handle->m_simDevice)) {
    //     return SIM_A301_GetPeriodicStatus2(handle, rawFrame);
    // }

    std::array<uint8_t, kMaxPacketLength> packedData{};

    c_REVLib_ErrorCode revlibError = REVLib_ReadCANPacketTimeout(
        handle, packedData, rawFrame->timestamp, A301_STATUS_2_FRAME_ID,
        handle->m_status2Period_ms, A301_STATUS_2_LENGTH, "Period Status 2");

    //     enableFrameIfNeeded(handle, revlibError, 2);

    a301_status_2_t frameData;
    a301_status_2_unpack(&frameData, packedData.data(), A301_STATUS_2_LENGTH);

    rawFrame->encoderVelocity =
        a301_status_2_encoder_velocity_decode(frameData.encoder_velocity) *
        (handle->m_inverted ? -1.0 : 1.0);
    rawFrame->relativeEncoderPosition =
        a301_status_2_relative_encoder_position_decode(
            frameData.relative_encoder_position);

    return revlibError;
}

#if 0
namespace {
c_REVLib_ErrorCode SIM_A301_GetPeriodicStatus3(
    c_A301_handle handle, c_A301_PeriodicStatus3* rawFrame) {
    c_SIM_A301_AbsoluteEncoder_handle simAbsoluteEncoder =
        c_SIM_A301_GetOrCreateSimAbsoluteEncoder(handle->m_simDevice);

    rawFrame->dutyCycleEncoderPosition =
        c_SIM_A301_GetSimAbsoluteEncoderPosition(simAbsoluteEncoder);
    rawFrame->dutyCycleEncoderVelocity =
        c_SIM_A301_GetSimAbsoluteEncoderVelocity(simAbsoluteEncoder);

    return c_REVLibError_None;
}
}  // namespace
#endif

c_REVLib_ErrorCode c_A301_GetPeriodicStatus3(c_A301_handle handle,
                                             c_A301_PeriodicStatus3* rawFrame) {
    // if (c_SIM_A301_IsSim(handle->m_simDevice)) {
    //     return SIM_A301_GetPeriodicStatus3(handle, rawFrame);
    // }

    std::array<uint8_t, kMaxPacketLength> packedData{};

    c_REVLib_ErrorCode revlibError = REVLib_ReadCANPacketTimeout(
        handle, packedData, rawFrame->timestamp, A301_STATUS_3_FRAME_ID,
        handle->m_status3Period_ms, A301_STATUS_3_LENGTH, "Period Status 3");

    //     enableFrameIfNeeded(handle, revlibError, 3);

    a301_status_3_t frameData;
    a301_status_3_unpack(&frameData, packedData.data(), A301_STATUS_3_LENGTH);

    rawFrame->absoluteEncoderPosition =
        a301_status_3_absolute_encoder_position_decode(
            frameData.absolute_encoder_position);

    return revlibError;
}

c_REVLib_ErrorCode c_A301_SetAbsoluteEncoderPosition(c_A301_handle handle,
                                                     float position) {
    // if (c_SIM_Spark_IsSim(handle->m_simDevice)) {
    //     c_SIM_Spark_AbsoluteEncoder_handle simAbsoluteEncoder =
    //         c_SIM_Spark_GetOrCreateSimAbsoluteEncoder(handle->m_simDevice);
    //     c_SIM_Spark_SetSimAbsoluteEncoderPosition(
    //         simAbsoluteEncoder, static_cast<double>(position));
    //     return c_REVLibError_None;
    // }

    a301_set_absolute_encoder_position_t frameData;
    uint8_t packedData[A301_SET_ABSOLUTE_ENCODER_POSITION_LENGTH];

    frameData.position = position;
    a301_set_absolute_encoder_position_pack(
        packedData, &frameData, A301_SET_ABSOLUTE_ENCODER_POSITION_LENGTH);

    c_REVLib_ErrorCode status = REVLib_WriteCANPacket(
        handle, packedData, A301_SET_ABSOLUTE_ENCODER_POSITION_FRAME_ID,
        "Set Absolute Encoder Position");
    return status;
}

c_REVLib_ErrorCode c_A301_SetRelativeEncoderPosition(c_A301_handle handle,
                                                     float position) {
    // if (c_SIM_Spark_IsSim(handle->m_simDevice)) {
    //     c_SIM_Spark_RelativeEncoder_handle simRelativeEncoder =
    //         c_SIM_Spark_GetOrCreateSimRelativeEncoder(handle->m_simDevice);
    //     c_SIM_Spark_SetSimRelativeEncoderPosition(
    //         simRelativeEncoder, static_cast<double>(position));
    //     return c_REVLibError_None;
    // }

    a301_set_relative_encoder_position_t frameData;
    uint8_t packedData[A301_SET_RELATIVE_ENCODER_POSITION_LENGTH];

    frameData.position = position;
    a301_set_relative_encoder_position_pack(
        packedData, &frameData, A301_SET_RELATIVE_ENCODER_POSITION_LENGTH);

    c_REVLib_ErrorCode status = REVLib_WriteCANPacket(
        handle, packedData, A301_SET_RELATIVE_ENCODER_POSITION_FRAME_ID,
        "Set Relative Encoder Position");
    return status;
}

namespace {

// SetpointCommand uses the same setpoint struct for all of the setpoint frames.
// This works as long as there is a floating points <setpoint> member.
// Velocity is set for the position frames and it ends up in the
// reserved field for the others (which will be ignored).

template <typename T>
concept HasFloatSetpoint = requires(T t) {
    t.setpoint;  // 'setpoint' must exist
    // and be a floating-point type:
    requires std::floating_point<std::remove_cvref_t<decltype(t.setpoint)>>;
};

template <typename T>
concept HasFloatVelocity = requires(T t) {
    t.velocity;  // 'velocity' must exist
    // and be a floating-point type:
    requires std::floating_point<std::remove_cvref_t<decltype(t.velocity)>>;
};

static_assert(HasFloatSetpoint<a301_velocity_setpoint_t>);
static_assert(HasFloatSetpoint<a301_duty_cycle_setpoint_t>);
static_assert(HasFloatSetpoint<a301_relative_position_setpoint_t>);
static_assert(HasFloatSetpoint<a301_current_setpoint_t>);
static_assert(HasFloatSetpoint<a301_absolute_position_setpoint_t>);

static_assert(HasFloatVelocity<a301_relative_position_setpoint_t>);
static_assert(HasFloatVelocity<a301_absolute_position_setpoint_t>);

}  // namespace

c_REVLib_ErrorCode c_A301_SetpointCommand(c_A301_handle handle, float value,
                                          c_A301_ControlType ctrl,
                                          float positionSpeed) {
    if (ctrl >= c_A301_kNumControlTypes) {
        REVLib_SetLastError(handle, c_REVLibError_Invalid);
        return c_REVLibError_Invalid;
    }

    static constexpr std::array a301CtrlToA301ApiId{
        A301_DUTY_CYCLE_SETPOINT_FRAME_ID,  // c_A301_kControlType_DutyCycle
        A301_VELOCITY_SETPOINT_FRAME_ID,    // c_A301_kControlType_Velocity
        A301_DUTY_CYCLE_SETPOINT_FRAME_ID,  // c_A301_kControlType_Voltage
        A301_RELATIVE_POSITION_SETPOINT_FRAME_ID,  // c_A301_kControlType_RelativePosition
        A301_ABSOLUTE_POSITION_SETPOINT_FRAME_ID,  // c_A301_kControlType_AbsolutePosition
        A301_CURRENT_SETPOINT_FRAME_ID,  // c_A301_kControlType_Current
    };
    const int apiId = a301CtrlToA301ApiId[ctrl];

    static constexpr float nominalVoltage{12.0f};

    if (ctrl == c_A301_kControlType_Voltage) {
        value /= nominalVoltage;
    }

    a301_relative_position_setpoint_t a301FrameData{
        .setpoint = value,
        .velocity = positionSpeed,
    };

    if (handle->m_inverted) {
        a301FrameData.setpoint *= -1;
    }

    // TODO
    // if (c_SIM_A301_IsSim(handle->m_simDevice)) {
    //    handle->m_activeSetpointApi = apiId;
    //    return c_SIM_A301_SetSetpoint(handle->m_simDevice, value,
    //                                   static_cast<uint8_t>(ctrl), 0, 0.0f,
    //                                   0);
    //}

    // Stop sending previous setpoint command if it is different
    if (handle->m_activeSetpointApi != apiId) {
        REVLib_StopCANPacketRepeating(handle, handle->m_activeSetpointApi,
                                      "A301 Setpoint command: Stop Repeating");
    }

    handle->m_activeSetpointApi = apiId;

    uint8_t packedData[A301_RELATIVE_POSITION_SETPOINT_LENGTH];
    a301_relative_position_setpoint_pack(
        packedData, &a301FrameData, A301_RELATIVE_POSITION_SETPOINT_LENGTH);

    const c_REVLib_ErrorCode status = REVLib_WriteCANPacketRepeating(
        handle, packedData, handle->m_activeSetpointApi,
        "A301 Setpoint command");

    if (status != c_REVLibError_None) {
        return status;
    }

    REVLib_SetLastError(handle, c_REVLibError_None);
    return c_REVLibError_None;
}

c_REVLib_ErrorCode c_A301_SetInverted(c_A301_handle handle, uint8_t inverted) {
    handle->m_inverted = inverted;
    return c_REVLibError_None;
}

c_REVLib_ErrorCode c_A301_GetInverted(c_A301_handle handle, uint8_t* inverted) {
    *inverted = handle->m_inverted;
    REVLib_SetLastError(handle, c_REVLibError_None);
    return c_REVLibError_None;
}

c_REVLib_ErrorCode c_A301_ClearFaults(c_A301_handle handle) {
    // if (c_SIM_A301_IsSim(handle->m_simDevice)) {
    //     if (c_SIM_A301_GetSimFaultManagerExists(handle->m_simDevice)) {
    //         c_SIM_A301_FaultManager_handle simFaultManager =
    //             c_SIM_A301_GetOrCreateSimFaultManager(handle->m_simDevice);
    //         c_SIM_A301_ClearSimFaults(simFaultManager);
    //     }
    // }

    c_REVLib_ErrorCode status =
        REVLib_WriteCANPacket(handle, zeroLengthDataPacket,
                              A301_CLEAR_FAULTS_FRAME_ID, "Clear Faults");
    return status;
}

c_REVLib_ErrorCode c_A301_SetIdleMode(c_A301_handle handle, uint8_t idleMode) {
    a301_set_idle_mode_t frameData{.idle_mode = idleMode};
    uint8_t packedData[A301_SET_IDLE_MODE_LENGTH];

    a301_set_idle_mode_pack(packedData, &frameData, A301_SET_IDLE_MODE_LENGTH);

    c_REVLib_ErrorCode status = REVLib_WriteCANPacket(
        handle, packedData, A301_SET_IDLE_MODE_FRAME_ID, "Set Idle Mode");
    return status;
}

c_REVLib_ErrorCode c_A301_GetIdleMode(c_A301_handle handle, uint8_t* idleMode) {
    *idleMode = uint8_t(0);

    std::array<uint8_t, kMaxPacketLength> packedDataIn{};

    c_REVLib_ErrorCode status = REVLib_WriteAndReadCANPacketWithReadTimeout(
        handle, zeroLengthDataPacket, A301_GET_IDLE_MODE_FRAME_ID, packedDataIn,
        A301_GET_IDLE_MODE_RESPONSE_FRAME_ID,
        A301_GET_IDLE_MODE_RESPONSE_LENGTH, kTempReadTimeout, "Get Idle Mode");

    if (status != c_REVLibError_None) {
        return status;
    }

    // Check the status of the response
    a301_get_idle_mode_response_t response;
    a301_get_idle_mode_response_unpack(&response, packedDataIn.data(),
                                       A301_GET_IDLE_MODE_RESPONSE_LENGTH);

    *idleMode = response.idle_mode;

    REVLib_SetLastError(handle, c_REVLibError_None);
    return c_REVLibError_None;
}

c_REVLib_ErrorCode c_A301_SetAbsolutePositionContinuousInput(
    c_A301_handle handle, uint8_t enabled) {
    a301_set_continuous_input_t frameData{
        .enabled = enabled,
        .reserved = 0,
    };
    uint8_t packedData[A301_SET_CONTINUOUS_INPUT_LENGTH];

    a301_set_continuous_input_pack(packedData, &frameData,
                                   A301_SET_CONTINUOUS_INPUT_LENGTH);

    c_REVLib_ErrorCode status = REVLib_WriteCANPacket(
        handle, packedData, A301_SET_CONTINUOUS_INPUT_FRAME_ID,
        "Set Absolute Position Continuous Input");
    return status;
}

c_REVLib_ErrorCode c_A301_GetAbsolutePositionContinuousInput(
    c_A301_handle handle, uint8_t* enabled) {
    *enabled = uint8_t(0);

    std::array<uint8_t, kMaxPacketLength> packedDataIn{};

    c_REVLib_ErrorCode status = REVLib_WriteAndReadCANPacketWithReadTimeout(
        handle, zeroLengthDataPacket, A301_GET_CONTINUOUS_INPUT_FRAME_ID,
        packedDataIn, A301_GET_CONTINUOUS_INPUT_RESPONSE_FRAME_ID,
        A301_GET_CONTINUOUS_INPUT_RESPONSE_LENGTH, kTempReadTimeout,
        "Set Absolute Position Continuous Input");

    if (status != c_REVLibError_None) {
        return status;
    }

    // Check the status of the response
    a301_get_continuous_input_response_t contInputResponse;
    a301_get_continuous_input_response_unpack(
        &contInputResponse, packedDataIn.data(),
        A301_GET_CONTINUOUS_INPUT_RESPONSE_LENGTH);

    *enabled = contInputResponse.enabled;

    REVLib_SetLastError(handle, c_REVLibError_None);
    return c_REVLibError_None;
}

c_REVLib_ErrorCode c_A301_SetAbsoluteEncoderRangeOffset(c_A301_handle handle,
                                                        float offset) {
    a301_set_abs_range_offset_t frameData;
    uint8_t packedData[A301_SET_ABS_RANGE_OFFSET_LENGTH];

    frameData.offset = offset * (handle->m_inverted ? -1.0f : 1.0f);
    a301_set_abs_range_offset_pack(packedData, &frameData,
                                   A301_SET_ABS_RANGE_OFFSET_LENGTH);

    c_REVLib_ErrorCode status = REVLib_WriteCANPacket(
        handle, packedData, A301_SET_ABS_RANGE_OFFSET_FRAME_ID,
        "Set Absolute Encoder Range Offset");
    return status;
}

c_REVLib_ErrorCode c_A301_GetAbsoluteEncoderRangeOffset(c_A301_handle handle,
                                                        float* offset) {
    *offset = 0.0f;

    std::array<uint8_t, kMaxPacketLength> packedDataIn{};

    c_REVLib_ErrorCode status = REVLib_WriteAndReadCANPacketWithReadTimeout(
        handle, zeroLengthDataPacket, A301_GET_ABS_RANGE_OFFSET_FRAME_ID,
        packedDataIn, A301_GET_ABS_RANGE_OFFSET_RESPONSE_FRAME_ID,
        A301_GET_ABS_RANGE_OFFSET_RESPONSE_LENGTH, kTempReadTimeout,
        "Set Absolute Position Range Offset");

    if (status != c_REVLibError_None) {
        return status;
    }

    // Check the status of the response
    a301_get_abs_range_offset_response_t offsetResponse;
    a301_get_abs_range_offset_response_unpack(
        &offsetResponse, packedDataIn.data(),
        A301_GET_ABS_RANGE_OFFSET_RESPONSE_LENGTH);

    *offset = offsetResponse.offset;

    REVLib_SetLastError(handle, c_REVLibError_None);
    return c_REVLibError_None;
}

c_REVLib_ErrorCode c_A301_SetStatusFramePeriod(c_A301_handle handle,
                                               c_A301_PeriodicFrame frame,
                                               uint32_t period_ms) {
    if (c_SIM_A301_IsSim(handle->m_simDevice)) {
        // TODO
        return c_REVLibError_None;
    }

    static constexpr uint32_t kMaxStatusPeriodMs{1000};
    period_ms = std::min(period_ms, kMaxStatusPeriodMs);

    a301_set_status_period_t statusPeriod{
        .periodic_frame = static_cast<uint8_t>(frame), .period = period_ms};
    uint8_t packedDataOut[A301_SET_STATUS_PERIOD_LENGTH]{};

    a301_set_status_period_pack(packedDataOut, &statusPeriod,
                                A301_SET_STATUS_PERIOD_LENGTH);

    std::array<uint8_t, kMaxPacketLength> packedDataIn{};

    static constexpr int32_t kTempReadTimeout{500};

    c_REVLib_ErrorCode status = REVLib_WriteAndReadCANPacketWithReadTimeout(
        handle, packedDataOut, A301_SET_STATUS_PERIOD_FRAME_ID, packedDataIn,
        A301_SET_STATUS_PERIOD_RESPONSE_FRAME_ID,
        A301_SET_STATUS_PERIOD_RESPONSE_LENGTH, kTempReadTimeout,
        fmt::format("Set Status Frame Period {}", static_cast<int>(frame)));

    if (status != c_REVLibError_None) {
        return status;
    }

    // Check the status of the response
    a301_set_status_period_response_t statusPeriodResponse;
    a301_set_status_period_response_unpack(
        &statusPeriodResponse, packedDataIn.data(),
        A301_SET_STATUS_PERIOD_RESPONSE_LENGTH);

    if (statusPeriodResponse.specified_periodic_frame ==
            static_cast<uint8_t>(frame) &&
        statusPeriodResponse.result_code != 0) {
        std::string errMsg{fmt::format(
            "Set Status Frame Period ({}) response returned error: {}",
            static_cast<int>(frame), statusPeriodResponse.result_code)};
        // TODO: error codes
        REVLib_SendErrorText(handle, c_REVLibError_General, errMsg);
        REVLib_SetLastError(handle, c_REVLibError_General);
        return c_REVLibError_General;
    }

    int* handle_statusPeriod{};

    switch (frame) {
        case c_A301_kStatus0:
            handle_statusPeriod = &handle->m_status0Period_ms;
            break;
        case c_A301_kStatus1:
            handle_statusPeriod = &handle->m_status1Period_ms;
            break;
        case c_A301_kStatus2:
            handle_statusPeriod = &handle->m_status2Period_ms;
            break;
        case c_A301_kStatus3:
            handle_statusPeriod = &handle->m_status3Period_ms;
            break;
        default:
            // Do nothing. Shouldn't get here
            break;
    }
    if (handle_statusPeriod) {
        *handle_statusPeriod = static_cast<int32_t>(period_ms);
    }

    REVLib_SetLastError(handle, c_REVLibError_None);
    return c_REVLibError_None;
}

c_REVLib_ErrorCode c_A301_GetStatusFramePeriod(c_A301_handle handle,
                                               c_A301_PeriodicFrame frame,
                                               uint32_t* period_ms) {
    *period_ms = 0u;

    if (c_SIM_A301_IsSim(handle->m_simDevice)) {
        // TODO
        return c_REVLibError_None;
    }

    a301_get_status_period_t statusPeriod{.periodic_frame =
                                              static_cast<uint8_t>(frame)};
    uint8_t packedDataOut[A301_GET_STATUS_PERIOD_LENGTH]{};

    a301_get_status_period_pack(packedDataOut, &statusPeriod,
                                A301_GET_STATUS_PERIOD_LENGTH);

    std::array<uint8_t, kMaxPacketLength> packedDataIn{};

    c_REVLib_ErrorCode status = REVLib_WriteAndReadCANPacketWithReadTimeout(
        handle, packedDataOut, A301_GET_STATUS_PERIOD_FRAME_ID, packedDataIn,
        A301_GET_STATUS_PERIOD_RESPONSE_FRAME_ID,
        A301_GET_STATUS_PERIOD_RESPONSE_LENGTH, kTempReadTimeout,
        fmt::format("Get Status Frame Period {}", static_cast<int>(frame)));

    if (status != c_REVLibError_None) {
        return status;
    }

    // Check the status of the response
    a301_get_status_period_response_t statusPeriodResponse;
    a301_get_status_period_response_unpack(
        &statusPeriodResponse, packedDataIn.data(),
        A301_GET_STATUS_PERIOD_RESPONSE_LENGTH);

    if (statusPeriodResponse.specified_periodic_frame ==
        static_cast<uint8_t>(frame)) {
        *period_ms = statusPeriodResponse.status_period;
    }

    REVLib_SetLastError(handle, c_REVLibError_None);
    return c_REVLibError_None;
}

#if 0
c_REVLib_ErrorCode c_A301_SetCANTimeout(c_A301_handle handle, int timeoutMs) {
    handle->m_canTimeout_ms = timeoutMs;
    REVLib_SetLastError(handle, c_REVLibError_None);
    return c_REVLibError_None;
}

c_REVLib_ErrorCode c_A301_IdentifyUniqueId(int busId, uint32_t uniqueId) {
    constexpr size_t IDENFITY_MSG_SIZE{sizeof(uniqueId)};

    std::span<uint8_t> packet{reinterpret_cast<uint8_t*>(&uniqueId),
                              IDENFITY_MSG_SIZE};

    const c_REVLib_ErrorCode status = REVLib_WriteBroadcastCANPacket(
        packet, busId, A301_IDENTIFY_FRAME_ID, "Identify Unique ID");

    return status;
}

void c_A301_SetSimAppliedOutput(c_A301_handle handle, float appliedOutput) {
    if (c_SIM_A301_IsSim(handle->m_simDevice)) {
        c_SIM_A301_SetAppliedOutput(handle->m_simDevice, appliedOutput);
    }
}

c_REVLib_ErrorCode c_A301_GetLastError(c_A301_handle handle) {
    return REVLib_GetLastError(handle);
}

c_REVLib_ErrorCode c_A301_StartFollowerMode(c_A301_handle handle) {
    if (c_SIM_A301_IsSim(handle->m_simDevice)) {
        c_SIM_A301_StartFollowerMode(handle->m_simDevice);

        return c_REVLibError_None;
    }

    std::array<uint8_t, kMaxPacketLength> packedDataIn{};

    c_REVLib_ErrorCode status = REVLib_WriteAndReadCANPacket(
        handle, zeroLengthDataPacket, A301_START_FOLLOWER_MODE_FRAME_ID,
        packedDataIn, A301_START_FOLLOWER_MODE_RESPONSE_FRAME_ID,
        A301_START_FOLLOWER_MODE_RESPONSE_LENGTH, "Start Follower Mode");

    if (status != c_REVLibError_None) {
        return status;
    }

    // Check response status
    A301_start_follower_mode_response_t frameIn;
    A301_start_follower_mode_response_unpack(
        &frameIn, packedDataIn.data(),
        A301_START_FOLLOWER_MODE_RESPONSE_LENGTH);
    if (frameIn.status != 0) {
        REVLib_SendErrorText(handle, c_REVLibError_FollowConfigMismatch,
                             fmt::format("Start Follower Mode"));
        REVLib_SetLastError(handle, c_REVLibError_FollowConfigMismatch);
        return c_REVLibError_FollowConfigMismatch;
    }

    REVLib_SetLastError(handle, c_REVLibError_None);
    return c_REVLibError_None;
}

c_REVLib_ErrorCode c_A301_StopFollowerMode(c_A301_handle handle) {
    if (c_SIM_A301_IsSim(handle->m_simDevice)) {
        c_SIM_A301_StopFollowerMode(handle->m_simDevice);

        return c_REVLibError_None;
    }

    std::array<uint8_t, kMaxPacketLength> packedDataIn;

    c_REVLib_ErrorCode status = REVLib_WriteAndReadCANPacket(
        handle, zeroLengthDataPacket, A301_STOP_FOLLOWER_MODE_FRAME_ID,
        packedDataIn, A301_STOP_FOLLOWER_MODE_RESPONSE_FRAME_ID,
        A301_STOP_FOLLOWER_MODE_RESPONSE_LENGTH, "Stop Follower Mode");

    if (status != c_REVLibError_None) {
        return status;
    }

    REVLib_SetLastError(handle, c_REVLibError_None);
    return c_REVLibError_None;
}

c_REVLib_ErrorCode c_SIM_A301_GetSimPIDOutput(c_A301_handle handle,
                                               float* value, float setpoint,
                                               float pv, float dt) {
    if (c_SIM_A301_IsSim(handle->m_simDevice)) {
        feedforward_constants_t ffConstants =
            c_SIM_A301_GetFeedforwardConstants(handle->m_simDevice);
        feedforward_state_t ffState;
        feedforward_signals_t ffSignals;
        float feedforward = 0;

        switch (c_SIM_A301_GetControlMode(handle->m_simDevice)) {
            case CTRL_VELOCITY:
                ffState.velocity = setpoint;
                ffState.acceleration = 0;
                ffState.position = 0;
                ffSignals = {1, 1, 0, 0, 0};  // s, v; no a, g, cos
                feedforward = c_SIM_A301_CalculateFeedforward(
                    handle->m_simDevice, &ffConstants, ffState, ffSignals,
                    c_SIM_A301_GetBusVoltage(handle->m_simDevice));
                break;
            case CTRL_POSITION:
                ffState.velocity = setpoint - pv;
                ffState.acceleration = 0;
                ffState.position = setpoint;
                ffSignals = {1, 0, 0, 1, 1};  // s, g, cos; no v, a
                feedforward = c_SIM_A301_CalculateFeedforward(
                    handle->m_simDevice, &ffConstants, ffState, ffSignals,
                    c_SIM_A301_GetBusVoltage(handle->m_simDevice));
                break;
            default:
                break;
        }

        *value = c_SIM_A301_CalculatePID(handle->m_simDevice, setpoint, pv, dt,
                                          feedforward);
    }
    return c_REVLibError_None;
}

void c_SIM_A301_CreateSimFaultManager(c_A301_handle handle) {
    if (c_SIM_A301_IsSim(handle->m_simDevice)) {
        c_SIM_A301_GetOrCreateSimFaultManager(handle->m_simDevice);
    }
}
#endif
