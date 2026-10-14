/*
 * Copyright (c) 2024-2026 REV Robotics
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

#include "rev/CANServoHubDriver.h"

#include <algorithm>
#include <array>
#include <cassert>
#include <chrono>
#include <mutex>
#include <span>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>

#include <fmt/format.h>

// TODO: (dave) get rid of:
#include <rev/driver/REVLibDriver.h>

#include "rev/CANDriverPrivate.h"
#include "rev/CANServoHubFrames.h"
#include "rev/CANServoHubParameters.h"
#include "rev/REVLibDaemon.h"
#include "rev/REVLibErrors.h"
#include "rev/REVLibVersion.h"
#include "rev/REVUtils.h"
#include "rev/sim/CANServoHub.h"

namespace {
constexpr uint16_t kServoHub_kAPIMajorVersion = REVLibMajorVersion;
constexpr uint8_t kServoHub_kAPIMinorVersion = REVLibMinorVersion;
constexpr uint8_t kServoHub_kAPIBuildVersion = REVLibBuildVersion;
constexpr uint32_t kServoHub_kAPIVersion = REVLibVersion;

constexpr int kMaxPacketLength{8};

constexpr std::array kPulseWidthApiIds{
    SERVO_HUB_SET_SERVO_0_TO_2_PULSE_WIDTHS_FRAME_ID,
    SERVO_HUB_SET_SERVO_3_TO_5_PULSE_WIDTHS_FRAME_ID};

constexpr int32_t kConfigTimeout_ms{100};
constexpr int32_t kResetConfigTimeout_ms{150};

constexpr int32_t kDefaultControlFramePeriod_ms{100};

constexpr int32_t kDefaultStatus0Period_ms{20};
constexpr int32_t kDefaultStatus2Period_ms{50};
constexpr int32_t kDefaultStatus3Period_ms{50};
constexpr int32_t kDefaultStatus4Period_ms{50};

enum SetParameterResult {
    kParamSuccess = 0,
    kParamInvalidChannel = 1,
    kParamInvalidValue = 2,
    kParamNotReady = 3
};

}  // namespace

// NOTE: This struct can't be in the unnamed namespace

struct c_ServoHub_Obj : public c_BaseCAN_Obj {
    c_ServoHub_Obj(int busId, int deviceId)
        : c_BaseCAN_Obj(ServoHub, busId, deviceId),
          m_simDevice{c_SIM_ServoHub_Create(busId, deviceId)} {
        m_controlFramePeriod_ms = kDefaultControlFramePeriod_ms;

        m_status0Period_ms = kDefaultStatus0Period_ms;
        m_status2Period_ms = kDefaultStatus2Period_ms;
        m_status3Period_ms = kDefaultStatus3Period_ms;
        m_status4Period_ms = kDefaultStatus4Period_ms;
    }

    c_ServoHub_FirmwareVersion m_firmwareVersion{};

    servo_hub_set_servo_0_to_2_pulse_widths_t m_setpoint02Data{
        .pulse_width_0 = 1500u,
        .pulse_width_1 = 1500u,
        .pulse_width_2 = 1500u,
        .enable_channel_0 = 0u,
        .enable_channel_1 = 0u,
        .enable_channel_2 = 0u,
        .power_channel_0 = 0u,
        .power_channel_1 = 0u,
        .power_channel_2 = 0u,
        .reserved = 0u};
    servo_hub_set_servo_3_to_5_pulse_widths_t m_setpoint35Data{
        .pulse_width_3 = 1500u,
        .pulse_width_4 = 1500u,
        .pulse_width_5 = 1500u,
        .enable_channel_3 = 0u,
        .enable_channel_4 = 0u,
        .enable_channel_5 = 0u,
        .power_channel_3 = 0u,
        .power_channel_4 = 0u,
        .power_channel_5 = 0u,
        .reserved = 0u};

    std::array<uint8_t, SERVO_HUB_SET_SERVO_0_TO_2_PULSE_WIDTHS_LENGTH>
        m_setpoint02Data_Packed{};
    std::array<uint8_t, SERVO_HUB_SET_SERVO_3_TO_5_PULSE_WIDTHS_LENGTH>
        m_setpoint35Data_Packed{};

    c_ServoHub_ProgrammingEnableStates channelProgrammingStates{
        .channel0_programming_enabled = false,
        .channel1_programming_enabled = false,
        .channel2_programming_enabled = false,
        .channel3_programming_enabled = false,
        .channel4_programming_enabled = false,
        .channel5_programming_enabled = false};

    c_SIM_ServoHub_handle m_simDevice{nullptr};
};

namespace {

CAN_ExistingDeviceIds s_servoHub_ExistingDeviceIds;

bool isVersionTooOld(const c_ServoHub_FirmwareVersion& fwVersion) {
    return (fwVersion.fwYear < 25);
}

//  c_ServoHub_RegisterId() must be called first
c_ServoHub_handle ServoHub_Create_Inplace(int busId, int deviceId,
                                          c_REVLib_ErrorCode* status) {
    *status = c_REVLibError_None;

    if (!CAN_IsValidDeviceId(deviceId)) {
        c_REVLib_SendError(c_REVLibError_InvalidCANId, ServoHub, busId,
                           deviceId);
        *status = c_REVLibError_InvalidCANId;
        // Don't allow a nullptr to be returned, invalid deviceId will just
        // fail, error is already sent
        // return nullptr;
    }

    c_ServoHub_handle handle = new c_ServoHub_Obj(busId, deviceId);

    if (!s_servoHub_ExistingDeviceIds.ContainsDevice(busId, deviceId)) {
        // I used sendError directly instead of c_REVLib_SendError because
        // we don't want to expose a REVLib error code for this condition.
        REVLib_SendLowLevelError(1,
                                 "c_ServoHub_RegisterId() was not called "
                                 "before c_ServoHub_Create()");
    }

    REVLib_SetLastError(handle, c_REVLibError_None);

    if (!c_SIM_ServoHub_IsSim(handle->m_simDevice)) {
        c_REVLib_RunDaemon();
    }

    *status = c_REVLibError_None;
    c_ServoHub_FirmwareVersion fwVersion;
    if (c_SIM_ServoHub_IsSim(handle->m_simDevice)) {
        *status = c_REVLibError_None;
    } else if (c_ServoHub_GetFirmwareVersion(handle, &fwVersion) !=
               c_REVLibError_None) {
        *status = c_REVLibError_CantFindFirmware;
    } else {
        handle->m_firmwareVersion = fwVersion;
        if (isVersionTooOld(fwVersion)) {
            *status = c_REVLibError_FirmwareTooOld;
        }
    }

    if (*status != c_REVLibError_None) {
        REVLib_SendError(handle, *status);
    }

    // Make sure no frames are repeating before registering
    for (int32_t apiId : kPulseWidthApiIds) {
        *status = REVLib_StopCANPacketRepeating(
            handle, apiId, "Create in place: Stop Repeating");
        if (*status != c_REVLibError_None) {
            break;
        }
    }

    return handle;
}

c_REVLib_ErrorCode ServoHub_ParamResultToErrorCode(
    SetParameterResult paramStatus) {
    switch (paramStatus) {
        case kParamSuccess:
            return c_REVLibError_None;
        case kParamInvalidChannel:
            return c_REVLibError_ParamInvalidChannel;
        case kParamInvalidValue:
            return c_REVLibError_ParamInvalidValue;
        case kParamNotReady:
            return c_REVLibError_NotReadyToReceiveCommand;
    }

    return c_REVLibError_Invalid;
}

c_REVLib_ErrorCode ServoHub_SetServoPulseRange(c_ServoHub_handle,
                                               c_ServoHub_ConfigParameter,
                                               uint32_t);
c_REVLib_ErrorCode ServoHub_SetDisableBehavior(c_ServoHub_handle,
                                               c_ServoHub_ConfigParameter,
                                               uint32_t);

using parameter_set_function_t = c_REVLib_ErrorCode (*)(
    c_ServoHub_handle, c_ServoHub_ConfigParameter, uint32_t);

struct private_parameter_table_entry_t {
    c_ServoHub_ConfigParameter id;
    parameter_set_function_t setter;
    int32_t frameId;
    int32_t length;
    int32_t responseFrameId;
    int32_t responseLength;
    uint8_t channel;
};

constexpr std::array<private_parameter_table_entry_t,
                     c_ServoHub_kNumConfigParameters>
    s_ServoHub_PrivateParameterTable{
        {{c_ServoHub_kChannel0_MinPulseWidth, ServoHub_SetServoPulseRange,
          SERVO_HUB_SET_SERVO_MIN_PULSE_WIDTH_FRAME_ID,
          SERVO_HUB_SET_SERVO_MIN_PULSE_WIDTH_LENGTH,
          SERVO_HUB_SET_SERVO_MIN_PULSE_RESPONSE_FRAME_ID,
          SERVO_HUB_SET_SERVO_MIN_PULSE_RESPONSE_LENGTH, 0u},
         {c_ServoHub_kChannel0_CenterPulseWidth, ServoHub_SetServoPulseRange,
          SERVO_HUB_SET_SERVO_CENTER_PULSE_WIDTH_FRAME_ID,
          SERVO_HUB_SET_SERVO_CENTER_PULSE_WIDTH_LENGTH,
          SERVO_HUB_SET_SERVO_CENTER_PULSE_RESPONSE_FRAME_ID,
          SERVO_HUB_SET_SERVO_CENTER_PULSE_RESPONSE_LENGTH, 0u},
         {c_ServoHub_kChannel0_MaxPulseWidth, ServoHub_SetServoPulseRange,
          SERVO_HUB_SET_SERVO_MAX_PULSE_WIDTH_FRAME_ID,
          SERVO_HUB_SET_SERVO_MAX_PULSE_WIDTH_LENGTH,
          SERVO_HUB_SET_SERVO_MAX_PULSE_RESPONSE_FRAME_ID,
          SERVO_HUB_SET_SERVO_MAX_PULSE_RESPONSE_LENGTH, 0u},
         {c_ServoHub_kChannel1_MinPulseWidth, ServoHub_SetServoPulseRange,
          SERVO_HUB_SET_SERVO_MIN_PULSE_WIDTH_FRAME_ID,
          SERVO_HUB_SET_SERVO_MIN_PULSE_WIDTH_LENGTH,
          SERVO_HUB_SET_SERVO_MIN_PULSE_RESPONSE_FRAME_ID,
          SERVO_HUB_SET_SERVO_MIN_PULSE_RESPONSE_LENGTH, 1u},
         {c_ServoHub_kChannel1_CenterPulseWidth, ServoHub_SetServoPulseRange,
          SERVO_HUB_SET_SERVO_CENTER_PULSE_WIDTH_FRAME_ID,
          SERVO_HUB_SET_SERVO_CENTER_PULSE_WIDTH_LENGTH,
          SERVO_HUB_SET_SERVO_CENTER_PULSE_RESPONSE_FRAME_ID,
          SERVO_HUB_SET_SERVO_CENTER_PULSE_RESPONSE_LENGTH, 1u},
         {c_ServoHub_kChannel1_MaxPulseWidth, ServoHub_SetServoPulseRange,
          SERVO_HUB_SET_SERVO_MAX_PULSE_WIDTH_FRAME_ID,
          SERVO_HUB_SET_SERVO_MAX_PULSE_WIDTH_LENGTH,
          SERVO_HUB_SET_SERVO_MAX_PULSE_RESPONSE_FRAME_ID,
          SERVO_HUB_SET_SERVO_MAX_PULSE_RESPONSE_LENGTH, 1u},
         {c_ServoHub_kChannel2_MinPulseWidth, ServoHub_SetServoPulseRange,
          SERVO_HUB_SET_SERVO_MIN_PULSE_WIDTH_FRAME_ID,
          SERVO_HUB_SET_SERVO_MIN_PULSE_WIDTH_LENGTH,
          SERVO_HUB_SET_SERVO_MIN_PULSE_RESPONSE_FRAME_ID,
          SERVO_HUB_SET_SERVO_MIN_PULSE_RESPONSE_LENGTH, 2u},
         {c_ServoHub_kChannel2_CenterPulseWidth, ServoHub_SetServoPulseRange,
          SERVO_HUB_SET_SERVO_CENTER_PULSE_WIDTH_FRAME_ID,
          SERVO_HUB_SET_SERVO_CENTER_PULSE_WIDTH_LENGTH,
          SERVO_HUB_SET_SERVO_CENTER_PULSE_RESPONSE_FRAME_ID,
          SERVO_HUB_SET_SERVO_CENTER_PULSE_RESPONSE_LENGTH, 2u},
         {c_ServoHub_kChannel2_MaxPulseWidth, ServoHub_SetServoPulseRange,
          SERVO_HUB_SET_SERVO_MAX_PULSE_WIDTH_FRAME_ID,
          SERVO_HUB_SET_SERVO_MAX_PULSE_WIDTH_LENGTH,
          SERVO_HUB_SET_SERVO_MAX_PULSE_RESPONSE_FRAME_ID,
          SERVO_HUB_SET_SERVO_MAX_PULSE_RESPONSE_LENGTH, 2u},
         {c_ServoHub_kChannel3_MinPulseWidth, ServoHub_SetServoPulseRange,
          SERVO_HUB_SET_SERVO_MIN_PULSE_WIDTH_FRAME_ID,
          SERVO_HUB_SET_SERVO_MIN_PULSE_WIDTH_LENGTH,
          SERVO_HUB_SET_SERVO_MIN_PULSE_RESPONSE_FRAME_ID,
          SERVO_HUB_SET_SERVO_MIN_PULSE_RESPONSE_LENGTH, 3u},
         {c_ServoHub_kChannel3_CenterPulseWidth, ServoHub_SetServoPulseRange,
          SERVO_HUB_SET_SERVO_CENTER_PULSE_WIDTH_FRAME_ID,
          SERVO_HUB_SET_SERVO_CENTER_PULSE_WIDTH_LENGTH,
          SERVO_HUB_SET_SERVO_CENTER_PULSE_RESPONSE_FRAME_ID,
          SERVO_HUB_SET_SERVO_CENTER_PULSE_RESPONSE_LENGTH, 3u},
         {c_ServoHub_kChannel3_MaxPulseWidth, ServoHub_SetServoPulseRange,
          SERVO_HUB_SET_SERVO_MAX_PULSE_WIDTH_FRAME_ID,
          SERVO_HUB_SET_SERVO_MAX_PULSE_WIDTH_LENGTH,
          SERVO_HUB_SET_SERVO_MAX_PULSE_RESPONSE_FRAME_ID,
          SERVO_HUB_SET_SERVO_MAX_PULSE_RESPONSE_LENGTH, 3u},
         {c_ServoHub_kChannel4_MinPulseWidth, ServoHub_SetServoPulseRange,
          SERVO_HUB_SET_SERVO_MIN_PULSE_WIDTH_FRAME_ID,
          SERVO_HUB_SET_SERVO_MIN_PULSE_WIDTH_LENGTH,
          SERVO_HUB_SET_SERVO_MIN_PULSE_RESPONSE_FRAME_ID,
          SERVO_HUB_SET_SERVO_MIN_PULSE_RESPONSE_LENGTH, 4u},
         {c_ServoHub_kChannel4_CenterPulseWidth, ServoHub_SetServoPulseRange,
          SERVO_HUB_SET_SERVO_CENTER_PULSE_WIDTH_FRAME_ID,
          SERVO_HUB_SET_SERVO_CENTER_PULSE_WIDTH_LENGTH,
          SERVO_HUB_SET_SERVO_CENTER_PULSE_RESPONSE_FRAME_ID,
          SERVO_HUB_SET_SERVO_CENTER_PULSE_RESPONSE_LENGTH, 4u},
         {c_ServoHub_kChannel4_MaxPulseWidth, ServoHub_SetServoPulseRange,
          SERVO_HUB_SET_SERVO_MAX_PULSE_WIDTH_FRAME_ID,
          SERVO_HUB_SET_SERVO_MAX_PULSE_WIDTH_LENGTH,
          SERVO_HUB_SET_SERVO_MAX_PULSE_RESPONSE_FRAME_ID,
          SERVO_HUB_SET_SERVO_MAX_PULSE_RESPONSE_LENGTH, 4u},
         {c_ServoHub_kChannel5_MinPulseWidth, ServoHub_SetServoPulseRange,
          SERVO_HUB_SET_SERVO_MIN_PULSE_WIDTH_FRAME_ID,
          SERVO_HUB_SET_SERVO_MIN_PULSE_WIDTH_LENGTH,
          SERVO_HUB_SET_SERVO_MIN_PULSE_RESPONSE_FRAME_ID,
          SERVO_HUB_SET_SERVO_MIN_PULSE_RESPONSE_LENGTH, 5u},
         {c_ServoHub_kChannel5_CenterPulseWidth, ServoHub_SetServoPulseRange,
          SERVO_HUB_SET_SERVO_CENTER_PULSE_WIDTH_FRAME_ID,
          SERVO_HUB_SET_SERVO_CENTER_PULSE_WIDTH_LENGTH,
          SERVO_HUB_SET_SERVO_CENTER_PULSE_RESPONSE_FRAME_ID,
          SERVO_HUB_SET_SERVO_CENTER_PULSE_RESPONSE_LENGTH, 5u},
         {c_ServoHub_kChannel5_MaxPulseWidth, ServoHub_SetServoPulseRange,
          SERVO_HUB_SET_SERVO_MAX_PULSE_WIDTH_FRAME_ID,
          SERVO_HUB_SET_SERVO_MAX_PULSE_WIDTH_LENGTH,
          SERVO_HUB_SET_SERVO_MAX_PULSE_RESPONSE_FRAME_ID,
          SERVO_HUB_SET_SERVO_MAX_PULSE_RESPONSE_LENGTH, 5u},
         {c_ServoHub_kChannel0_DisableBehavior, ServoHub_SetDisableBehavior,
          SERVO_HUB_SET_DISABLE_BEHAVIOR_FRAME_ID,
          SERVO_HUB_SET_DISABLE_BEHAVIOR_LENGTH,
          SERVO_HUB_SET_DISABLE_BEHAVIOR_RESPONSE_FRAME_ID,
          SERVO_HUB_SET_DISABLE_BEHAVIOR_RESPONSE_LENGTH, 0u},
         {c_ServoHub_kChannel1_DisableBehavior, ServoHub_SetDisableBehavior,
          SERVO_HUB_SET_DISABLE_BEHAVIOR_FRAME_ID,
          SERVO_HUB_SET_DISABLE_BEHAVIOR_LENGTH,
          SERVO_HUB_SET_DISABLE_BEHAVIOR_RESPONSE_FRAME_ID,
          SERVO_HUB_SET_DISABLE_BEHAVIOR_RESPONSE_LENGTH, 1u},
         {c_ServoHub_kChannel2_DisableBehavior, ServoHub_SetDisableBehavior,
          SERVO_HUB_SET_DISABLE_BEHAVIOR_FRAME_ID,
          SERVO_HUB_SET_DISABLE_BEHAVIOR_LENGTH,
          SERVO_HUB_SET_DISABLE_BEHAVIOR_RESPONSE_FRAME_ID,
          SERVO_HUB_SET_DISABLE_BEHAVIOR_RESPONSE_LENGTH, 2u},
         {c_ServoHub_kChannel3_DisableBehavior, ServoHub_SetDisableBehavior,
          SERVO_HUB_SET_DISABLE_BEHAVIOR_FRAME_ID,
          SERVO_HUB_SET_DISABLE_BEHAVIOR_LENGTH,
          SERVO_HUB_SET_DISABLE_BEHAVIOR_RESPONSE_FRAME_ID,
          SERVO_HUB_SET_DISABLE_BEHAVIOR_RESPONSE_LENGTH, 3u},
         {c_ServoHub_kChannel4_DisableBehavior, ServoHub_SetDisableBehavior,
          SERVO_HUB_SET_DISABLE_BEHAVIOR_FRAME_ID,
          SERVO_HUB_SET_DISABLE_BEHAVIOR_LENGTH,
          SERVO_HUB_SET_DISABLE_BEHAVIOR_RESPONSE_FRAME_ID,
          SERVO_HUB_SET_DISABLE_BEHAVIOR_RESPONSE_LENGTH, 4u},
         {c_ServoHub_kChannel5_DisableBehavior, ServoHub_SetDisableBehavior,
          SERVO_HUB_SET_DISABLE_BEHAVIOR_FRAME_ID,
          SERVO_HUB_SET_DISABLE_BEHAVIOR_LENGTH,
          SERVO_HUB_SET_DISABLE_BEHAVIOR_RESPONSE_FRAME_ID,
          SERVO_HUB_SET_DISABLE_BEHAVIOR_RESPONSE_LENGTH, 5u}}};

template <typename msg_struct_t>
using pack_function_t = int (*)(uint8_t*, const msg_struct_t*, size_t);

template <typename msg_struct_t>
void PackMessage(std::span<uint8_t> out, const msg_struct_t* msg, size_t length,
                 pack_function_t<msg_struct_t> packer) {
    if (out.size() < length) {
        getREVLibDriver()->sendError(
            c_REVLibError_Invalid,
            "output packet is not large enough for packed message", true);
    }
    packer(out.data(), msg, length);
}

template <typename msg_struct_t>
void PackSetServoWidthMessage(std::span<uint8_t> out,
                              const private_parameter_table_entry_t& param,
                              uint32_t pulse_us,
                              pack_function_t<msg_struct_t> packer) {
    const msg_struct_t msg{.channel = param.channel,
                           .pulseus = static_cast<uint16_t>(pulse_us)};
    PackMessage(out, &msg, param.length, packer);
}

template <typename msg_struct_t>
void PackSetDisableBehaviorMessage(std::span<uint8_t> out,
                                   const private_parameter_table_entry_t& param,
                                   uint32_t behavior,
                                   pack_function_t<msg_struct_t> packer) {
    const msg_struct_t msg{.mask = static_cast<uint8_t>(0x1u << param.channel),
                           .behavior_bitfield = static_cast<uint8_t>(
                               behavior ? 0x1u << param.channel : 0x0u),
                           .reserved{}};
    PackMessage(out, &msg, param.length, packer);
}

template <typename msg_struct_t>
using unpack_function_t = int (*)(msg_struct_t*, const uint8_t*, size_t);

template <typename msg_struct_t>
void UnpackMessage(msg_struct_t* out, const std::span<uint8_t> packedInput,
                   size_t length, unpack_function_t<msg_struct_t> unpacker) {
    if (packedInput.size() < length) {
        getREVLibDriver()->sendError(
            c_REVLibError_Invalid,
            "input packet is not large enough to contain the packed message",
            true);
    }
    unpacker(out, packedInput.data(), length);
}

template <typename resp_struct_t>
SetParameterResult UnpackResponse(const std::span<uint8_t> packedInput,
                                  size_t length,
                                  unpack_function_t<resp_struct_t> unpacker) {
    resp_struct_t response;
    UnpackMessage(&response, packedInput, length, unpacker);

    SetParameterResult paramStatus =
        static_cast<SetParameterResult>(response.result);

    return paramStatus;
}

template <typename resp_struct_t>
SetParameterResult UnpackResponsePulseWidth(
    const std::span<uint8_t> packedInput, size_t length,
    unpack_function_t<resp_struct_t> unpacker) {
    resp_struct_t response;
    UnpackMessage(&response, packedInput, length, unpacker);

    SetParameterResult paramStatus =
        static_cast<SetParameterResult>(response.result);

    return paramStatus;
}

/**
 * Helper function to reset parameters to their default IF they have not
 * been set in Configure().
 */
c_REVLib_ErrorCode ServoHub_ResetParameters(c_ServoHub_handle handle) {
    if (c_SIM_ServoHub_IsSim(handle->m_simDevice)) {
        return c_SIM_ServoHub_ResetParameters(handle->m_simDevice);
    }

    static constexpr uint16_t kResetMagicNumber{0xbeefu};
    servo_hub_reset_configuration_t resetConfig{.magic = kResetMagicNumber};

    uint8_t packedDataOut[SERVO_HUB_RESET_CONFIGURATION_LENGTH];
    servo_hub_reset_configuration_pack(packedDataOut, &resetConfig,
                                       SERVO_HUB_RESET_CONFIGURATION_LENGTH);

    std::array<uint8_t, kMaxPacketLength> packedDataIn{};

    c_REVLib_ErrorCode status = REVLib_WriteAndReadCANPacketWithReadTimeout(
        handle, packedDataOut, SERVO_HUB_RESET_CONFIGURATION_FRAME_ID,
        packedDataIn, SERVO_HUB_RESET_CONFIGURATION_RESPONSE_FRAME_ID,
        SERVO_HUB_RESET_CONFIGURATION_RESPONSE_LENGTH, kResetConfigTimeout_ms,
        "Reset Parameters");

    if (status != c_REVLibError_None) {
        return (status);
    }

    servo_hub_reset_configuration_response_t resetConfigResponse;

    // Check the status of the response
    servo_hub_reset_configuration_response_unpack(
        &resetConfigResponse, packedDataIn.data(),
        SERVO_HUB_RESET_CONFIGURATION_RESPONSE_LENGTH);

    if (resetConfigResponse.magic != kResetMagicNumber) {
        REVLib_SendErrorText(handle, c_REVLibError_HAL,
                             "Reset Parameters response magic number failed");
        REVLib_SetLastError(handle, c_REVLibError_HAL);
        return c_REVLibError_HAL;
    }

    REVLib_SetLastError(handle, c_REVLibError_None);
    return c_REVLibError_None;
}

c_REVLib_ErrorCode ServoHub_SetServoPulseRange(
    c_ServoHub_handle handle, c_ServoHub_ConfigParameter servoPulseWidthParam,
    uint32_t pulseWidth_us) {
    if (c_SIM_ServoHub_IsSim(handle->m_simDevice)) {
        return c_SIM_ServoHub_SetParameter(handle->m_simDevice,
                                           servoPulseWidthParam, pulseWidth_us);
    }

    const size_t paramIdx{static_cast<size_t>(servoPulseWidthParam)};
    const auto& param = s_ServoHub_PrivateParameterTable[paramIdx];

    if (paramIdx != static_cast<size_t>(param.id)) {
        getREVLibDriver()->sendError(c_REVLibError_Invalid,
                                     "param index != param id", true);
        return c_REVLibError_Invalid;
    }

    if (param.channel > 5) {
        return c_REVLibError_ParamInvalid;
    }

    // Define with the max possible length
    std::array<uint8_t, kMaxPacketLength> packedDataOut;
    switch (param.frameId) {
        case SERVO_HUB_SET_SERVO_MIN_PULSE_WIDTH_FRAME_ID:
            PackSetServoWidthMessage(packedDataOut, param, pulseWidth_us,
                                     servo_hub_set_servo_min_pulse_width_pack);
            break;
        case SERVO_HUB_SET_SERVO_CENTER_PULSE_WIDTH_FRAME_ID:
            PackSetServoWidthMessage(
                packedDataOut, param, pulseWidth_us,
                servo_hub_set_servo_center_pulse_width_pack);
            break;
        case SERVO_HUB_SET_SERVO_MAX_PULSE_WIDTH_FRAME_ID:
            PackSetServoWidthMessage(packedDataOut, param, pulseWidth_us,
                                     servo_hub_set_servo_max_pulse_width_pack);
            break;
        default:
            getREVLibDriver()->sendError(
                c_REVLibError_Invalid,
                "Invalid Set Servo ___ Pulse Width frame ID", true);
            return c_REVLibError_Invalid;
    }

    std::array<uint8_t, kMaxPacketLength> packedDataIn{};

    // Set the size according to the actual parameter requirement
    std::span<uint8_t> dataOutPacket(packedDataOut.data(), param.length);

    c_REVLib_ErrorCode status = REVLib_WriteAndReadCANPacketWithReadTimeout(
        handle, dataOutPacket, param.frameId, packedDataIn,
        param.responseFrameId, param.responseLength, kConfigTimeout_ms,
        fmt::format("Set Servo {} Pulse Width", param.channel));

    if (status != c_REVLibError_None) {
        return (status);
    }

    SetParameterResult paramStatus{kParamSuccess};
    switch (param.responseFrameId) {
        case SERVO_HUB_SET_SERVO_MIN_PULSE_RESPONSE_FRAME_ID:
            paramStatus = UnpackResponsePulseWidth(
                packedDataIn, param.responseLength,
                servo_hub_set_servo_min_pulse_response_unpack);
            break;
        case SERVO_HUB_SET_SERVO_CENTER_PULSE_RESPONSE_FRAME_ID:
            paramStatus = UnpackResponsePulseWidth(
                packedDataIn, param.responseLength,
                servo_hub_set_servo_center_pulse_response_unpack);
            break;
        case SERVO_HUB_SET_SERVO_MAX_PULSE_RESPONSE_FRAME_ID:
            paramStatus = UnpackResponsePulseWidth<
                servo_hub_set_servo_max_pulse_response_t>(
                packedDataIn, param.responseLength,
                servo_hub_set_servo_max_pulse_response_unpack);
            break;
        default:
            getREVLibDriver()->sendError(
                c_REVLibError_Invalid,
                "Invalid Set Servo ___ Pulse Response frame ID", true);
            return c_REVLibError_Invalid;
    }

    if (paramStatus != kParamSuccess) {
        c_REVLib_ErrorCode errCode =
            ServoHub_ParamResultToErrorCode(paramStatus);
        REVLib_SendErrorText(handle, errCode,
                             std::to_string(static_cast<int>(param.channel)));
        REVLib_SetLastError(handle, errCode);
        return errCode;
    }

    REVLib_SetLastError(handle, c_REVLibError_None);
    return c_REVLibError_None;
}

c_REVLib_ErrorCode ServoHub_SetDisableBehavior(
    c_ServoHub_handle handle, c_ServoHub_ConfigParameter disableBehaviorParam,
    uint32_t disableBehavior) {
    if (c_SIM_ServoHub_IsSim(handle->m_simDevice)) {
        return c_SIM_ServoHub_SetParameter(
            handle->m_simDevice, disableBehaviorParam, disableBehavior);
    }

    const size_t paramIdx{static_cast<size_t>(disableBehaviorParam)};
    const auto& param = s_ServoHub_PrivateParameterTable[paramIdx];

    if (paramIdx != static_cast<size_t>(param.id)) {
        getREVLibDriver()->sendError(c_REVLibError_Invalid,
                                     "param index != param id", true);
        return c_REVLibError_Invalid;
    }

    if (param.channel > 5) {
        return c_REVLibError_ParamInvalid;
    }

    std::array<uint8_t, kMaxPacketLength> packedDataOut;
    PackSetDisableBehaviorMessage(packedDataOut, param, disableBehavior,
                                  servo_hub_set_disable_behavior_pack);

    std::array<uint8_t, kMaxPacketLength> packedDataIn{};

    // Set the size according to the actual parameter requirement
    std::span<uint8_t> dataOutPacket(packedDataOut.data(), param.length);

    c_REVLib_ErrorCode status = REVLib_WriteAndReadCANPacketWithReadTimeout(
        handle, dataOutPacket, param.frameId, packedDataIn,
        param.responseFrameId, param.responseLength, kConfigTimeout_ms,
        fmt::format("Set Disable Behavior {}", param.channel));

    if (status != c_REVLibError_None) {
        return (status);
    }

    SetParameterResult paramStatus =
        UnpackResponse(packedDataIn, param.responseLength,
                       servo_hub_set_disable_behavior_response_unpack);

    if (paramStatus != kParamSuccess) {
        c_REVLib_ErrorCode errCode =
            ServoHub_ParamResultToErrorCode(paramStatus);
        REVLib_SendErrorText(handle, errCode,
                             std::to_string(static_cast<int>(param.channel)));
        REVLib_SetLastError(handle, errCode);
        return errCode;
    }

    REVLib_SetLastError(handle, c_REVLibError_None);
    return c_REVLibError_None;
}

}  // namespace

// High-level libraries should throw an exception if this
// returns c_REVLibError_DuplicateCANId or c_REVLibError_InvalidCANId
c_REVLib_ErrorCode c_ServoHub_RegisterId(int busId, int deviceId) {
    if (!CAN_IsValidDeviceId(deviceId)) {
        return c_REVLibError_InvalidCANId;
    }
    if (!s_servoHub_ExistingDeviceIds.InsertDevice(busId, deviceId)) {
        return c_REVLibError_DuplicateCANId;
    }
    return c_REVLibError_None;
}

// c_ServoHub_RegisterId() must be called first
c_ServoHub_handle c_ServoHub_Create(int busId, int deviceId,
                                    c_REVLib_ErrorCode* status) {
    c_ServoHub_handle handle = ServoHub_Create_Inplace(busId, deviceId, status);

    if (*status != c_REVLibError_None) {
        return handle;
    }

    servo_hub_set_servo_0_to_2_pulse_widths_pack(
        handle->m_setpoint02Data_Packed.data(), &handle->m_setpoint02Data,
        handle->m_setpoint02Data_Packed.size());

    REVLib_WriteCANPacket(handle, handle->m_setpoint02Data_Packed,
                          SERVO_HUB_SET_SERVO_0_TO_2_PULSE_WIDTHS_FRAME_ID);

    servo_hub_set_servo_3_to_5_pulse_widths_pack(
        handle->m_setpoint35Data_Packed.data(), &handle->m_setpoint35Data,
        handle->m_setpoint35Data_Packed.size());

    REVLib_WriteCANPacket(handle, handle->m_setpoint35Data_Packed,
                          SERVO_HUB_SET_SERVO_3_TO_5_PULSE_WIDTHS_FRAME_ID);

    getREVLibDriver()->reportDeviceUsage(ServoHub, busId, deviceId);

    return handle;
}

void c_ServoHub_Close(c_ServoHub_handle handle) {
    if (handle == nullptr) {
        return;
    }
    s_servoHub_ExistingDeviceIds.RemoveDevice(
        static_cast<uint8_t>(handle->m_busId),
        static_cast<uint8_t>(handle->m_deviceId));

    for (int32_t apiId : kPulseWidthApiIds) {
        REVLib_StopCANPacketRepeating(handle, apiId,
                                      "ServoHub destroy: Stop Repeating");
    }

    c_SIM_ServoHub_Destroy(handle->m_simDevice);

    c_REVLib_StopDaemon();
}

void c_ServoHub_Destroy(c_ServoHub_handle handle) {
    if (handle == nullptr) {
        return;
    }

    delete handle;
}

c_REVLib_ErrorCode c_ServoHub_Configure(c_ServoHub_handle handle,
                                        const char* flattenedConfig,
                                        uint8_t resetSafeParameters) {
    // Unflatten string into processable data
    std::istringstream iss(flattenedConfig);
    std::string line;

    std::unordered_map<c_ServoHub_ConfigParameter, uint32_t> parameters;
    while (std::getline(iss, line)) {
        auto delimiterIndex = line.find(',');
        std::string paramIdStr = line.substr(0, delimiterIndex);
        std::string paramValueStr = line.substr(delimiterIndex + 1);

        c_ServoHub_ConfigParameter paramId =
            static_cast<c_ServoHub_ConfigParameter>(std::stoul(paramIdStr));
        uint32_t paramValue;
        std::stringstream ss;
        ss << std::hex << paramValueStr;
        ss >> paramValue;

        parameters[paramId] = paramValue;
    }

    // Restore defaults if specified by user
    if (resetSafeParameters) {
        ServoHub_ResetParameters(handle);
    }

    // Iterate through parameters and write each one
    for (const auto& [key, value] : parameters) {
        const size_t paramIdx{static_cast<size_t>(key)};

        c_REVLib_ErrorCode status =
            s_ServoHub_PrivateParameterTable[paramIdx].setter(handle, key,
                                                              value);

        // Only return if param-specific error return by device
        if (status == c_REVLibError_ParamInvalidID ||
            status == c_REVLibError_ParamMismatchType ||
            status == c_REVLibError_ParamInvalid ||
            status == c_REVLibError_ParamNotImplementedDeprecated) {
            return status;
        }
    }

    return c_REVLibError_None;
}

void c_ServoHub_SetPeriodicFrameTimeout(c_ServoHub_handle handle,
                                        int timeout_ms) {
    if (timeout_ms < 0) {
        return;
    }
    handle->m_periodicFrameTimeout_ms = timeout_ms;
}

c_REVLib_ErrorCode c_ServoHub_SetCANTimeout(c_ServoHub_handle handle,
                                            int timeout_ms) {
    handle->m_canTimeout_ms = timeout_ms;
    REVLib_SetLastError(handle, c_REVLibError_None);
    return c_REVLibError_None;
}

void c_ServoHub_SetCANMaxRetries(c_ServoHub_handle handle, int numRetries) {
    if (numRetries < 0) {
        return;
    }
    handle->m_canMaxRetryCount = numRetries;
}

void c_ServoHub_SetControlFramePeriod(c_ServoHub_handle handle, int period_ms) {
    if (period_ms < 0) {
        return;
    }
    handle->m_controlFramePeriod_ms = period_ms;
}

int c_ServoHub_GetControlFramePeriod(c_ServoHub_handle handle) {
    return handle->m_controlFramePeriod_ms;
}

c_REVLib_APIVersion c_ServoHub_GetAPIVersion(void) {
    return c_REVLib_APIVersion({.Major = kServoHub_kAPIMajorVersion,
                                .Minor = kServoHub_kAPIMinorVersion,
                                .Build = kServoHub_kAPIBuildVersion,
                                .Version = kServoHub_kAPIVersion});
}

c_REVLib_ErrorCode c_ServoHub_GetFirmwareVersion(
    c_ServoHub_handle handle, c_ServoHub_FirmwareVersion* fwVersion) {
    std::array<uint8_t, kMaxPacketLength> packedData;

    // Start with invalid versions
    *fwVersion = c_ServoHub_FirmwareVersion{};

    if (c_SIM_ServoHub_IsSim(handle->m_simDevice)) {
        return c_REVLibError_None;
    }

    c_REVLib_ErrorCode status = REVLib_WriteAndReadRtrCANPacket(
        handle, packedData, SERVO_HUB_GET_VERSION_FRAME_ID,
        SERVO_HUB_GET_VERSION_LENGTH, "Get Firmware Version");

    if (status != c_REVLibError_None) {
        // Versions not received
        REVLib_SetLastError(handle, c_REVLibError_CantFindFirmware);
        return c_REVLibError_CantFindFirmware;
    }

    servo_hub_get_version_t frameData;
    servo_hub_get_version_unpack(&frameData, packedData.data(),
                                 SERVO_HUB_GET_VERSION_LENGTH);

    fwVersion->fwFix = frameData.firmware_fix;
    fwVersion->fwMinor = frameData.firmware_minor;
    fwVersion->fwYear = frameData.firmware_year;
    fwVersion->hwMinor = frameData.hardware_minor;
    fwVersion->hwMajor = frameData.hardware_major;
    fwVersion->reserved = frameData.reserved;

    REVLib_SetLastError(handle, c_REVLibError_None);
    return c_REVLibError_None;
}

namespace {
c_REVLib_ErrorCode SIM_ServoHub_GetPeriodicStatus0(
    c_SIM_ServoHub_handle simHandle, c_ServoHub_PeriodicStatus0* rawFrame) {
    rawFrame->voltage = c_SIM_ServoHub_GetDeviceVoltage(simHandle);
    rawFrame->servoVoltage = c_SIM_ServoHub_GetServoVoltage(simHandle);
    rawFrame->deviceCurrent = c_SIM_ServoHub_GetDeviceCurrent(simHandle);

    rawFrame->primaryHeartbeatLock = 0;
    rawFrame->systemEnabled = 1;
    rawFrame->communicationMode = 0;
    rawFrame->reserved = 0;

    return c_REVLibError_None;
}
}  // namespace

c_REVLib_ErrorCode c_ServoHub_GetPeriodicStatus0(
    c_ServoHub_handle handle, c_ServoHub_PeriodicStatus0* rawFrame) {
    if (c_SIM_ServoHub_IsSim(handle->m_simDevice)) {
        return SIM_ServoHub_GetPeriodicStatus0(handle->m_simDevice, rawFrame);
    }

    std::array<uint8_t, kMaxPacketLength> packedData{};

    c_REVLib_ErrorCode revlibError = REVLib_ReadCANPacketTimeout(
        handle, packedData, rawFrame->timestamp, SERVO_HUB_STATUS_0_FRAME_ID,
        handle->m_status0Period_ms, SERVO_HUB_STATUS_0_LENGTH,
        "Period Status 0");

    if (revlibError != c_REVLibError_None) {
        return revlibError;
    }

    servo_hub_status_0_t frameData;
    servo_hub_status_0_unpack(&frameData, packedData.data(),
                              SERVO_HUB_STATUS_0_LENGTH);

    rawFrame->voltage = static_cast<float>(
        servo_hub_status_0_voltage_decode(frameData.voltage));
    rawFrame->servoVoltage = static_cast<float>(
        servo_hub_status_0_servo_voltage_decode(frameData.servo_voltage));
    rawFrame->deviceCurrent = static_cast<float>(
        servo_hub_status_0_device_current_decode(frameData.device_current));
    rawFrame->primaryHeartbeatLock = frameData.primary_heartbeat_lock;
    rawFrame->systemEnabled = frameData.system_enabled;
    rawFrame->communicationMode = frameData.communication_mode;
    rawFrame->activelyProgramming = frameData.servo_programming_active;
    rawFrame->programmingEnabled = frameData.programming_mode_enabled;
    rawFrame->reserved = frameData.reserved;

    return c_REVLibError_None;
}

namespace {
c_REVLib_ErrorCode SIM_ServoHub_GetPeriodicStatus1(
    c_SIM_ServoHub_handle simHandle, c_ServoHub_PeriodicStatus1* rawFrame) {
    if (c_SIM_ServoHub_GetSimFaultManagerExists(simHandle)) {
        c_SIM_ServoHub_FaultManager_handle simFaultManager =
            c_SIM_ServoHub_GetOrCreateSimFaultManager(simHandle);
        c_SIM_ServoHub_SetRawFrameFromSimFaults(simFaultManager, rawFrame);
    } else {
        // if the fault manager has not been set up, all faults off
        *rawFrame = c_ServoHub_PeriodicStatus1{};
    }
    return c_REVLibError_None;
}
}  // namespace

c_REVLib_ErrorCode c_ServoHub_GetPeriodicStatus1(
    c_ServoHub_handle handle, c_ServoHub_PeriodicStatus1* rawFrame) {
    if (c_SIM_ServoHub_IsSim(handle->m_simDevice)) {
        return SIM_ServoHub_GetPeriodicStatus1(handle->m_simDevice, rawFrame);
    }

    std::array<uint8_t, kMaxPacketLength> packedData{};

    c_REVLib_ErrorCode revlibError = REVLib_ReadCANPacketTimeout(
        handle, packedData, rawFrame->timestamp, SERVO_HUB_STATUS_1_FRAME_ID,
        handle->m_status1Period_ms, SERVO_HUB_STATUS_1_LENGTH,
        "Period Status 1");

    if (revlibError != c_REVLibError_None) {
        return revlibError;
    }

    servo_hub_status_1_t frameData;
    servo_hub_status_1_unpack(&frameData, packedData.data(),
                              SERVO_HUB_STATUS_1_LENGTH);

    rawFrame->regulatorPowerGoodFault = frameData.regulator_pgood_fault;
    rawFrame->brownout = frameData.brownout;
    rawFrame->canWarning = frameData.can_warning;
    rawFrame->canBusOff = frameData.can_bus_off;
    rawFrame->hardwareFault = frameData.hardware_fault;
    rawFrame->firmwareFault = frameData.firmware_fault;
    rawFrame->hasReset = frameData.has_reset;
    rawFrame->lowBatteryFault = frameData.low_battery_fault;
    rawFrame->channel0Overcurrent = frameData.channel_0_overcurrent;
    rawFrame->channel1Overcurrent = frameData.channel_1_overcurrent;
    rawFrame->channel2Overcurrent = frameData.channel_2_overcurrent;
    rawFrame->channel3Overcurrent = frameData.channel_3_overcurrent;
    rawFrame->channel4Overcurrent = frameData.channel_4_overcurrent;
    rawFrame->channel5Overcurrent = frameData.channel_5_overcurrent;
    rawFrame->stickyRegulatorPowerGoodFault =
        frameData.sticky_regulator_pgood_fault;
    rawFrame->stickyBrownout = frameData.sticky_brownout;
    rawFrame->stickyCanWarning = frameData.sticky_can_warning;
    rawFrame->stickyCanBusOff = frameData.sticky_can_bus_off;
    rawFrame->stickyHardwareFault = frameData.sticky_hardware_fault;
    rawFrame->stickyFirmwareFault = frameData.sticky_firmware_fault;
    rawFrame->stickyHasReset = frameData.sticky_has_reset;
    rawFrame->stickyLowBatteryFault = frameData.sticky_low_battery_fault;
    rawFrame->stickyChannel0Overcurrent =
        frameData.sticky_channel_0_overcurrent;
    rawFrame->stickyChannel1Overcurrent =
        frameData.sticky_channel_1_overcurrent;
    rawFrame->stickyChannel2Overcurrent =
        frameData.sticky_channel_2_overcurrent;
    rawFrame->stickyChannel3Overcurrent =
        frameData.sticky_channel_3_overcurrent;
    rawFrame->stickyChannel4Overcurrent =
        frameData.sticky_channel_4_overcurrent;
    rawFrame->stickyChannel5Overcurrent =
        frameData.sticky_channel_5_overcurrent;

    return c_REVLibError_None;
}

namespace {
c_REVLib_ErrorCode SIM_ServoHub_GetPeriodicStatus2(
    c_SIM_ServoHub_handle simHandle, c_ServoHub_PeriodicStatus2* rawFrame) {
    rawFrame->channel0PulseWidth =
        c_SIM_ServoHub_GetChannelPulseWidth(simHandle, c_ServoHub_kChannel0);
    rawFrame->channel1PulseWidth =
        c_SIM_ServoHub_GetChannelPulseWidth(simHandle, c_ServoHub_kChannel1);
    rawFrame->channel2PulseWidth =
        c_SIM_ServoHub_GetChannelPulseWidth(simHandle, c_ServoHub_kChannel2);

    rawFrame->channel0PulseWidth =
        c_SIM_ServoHub_GetChannelEnabled(simHandle, c_ServoHub_kChannel0);
    rawFrame->channel1PulseWidth =
        c_SIM_ServoHub_GetChannelEnabled(simHandle, c_ServoHub_kChannel1);
    rawFrame->channel2PulseWidth =
        c_SIM_ServoHub_GetChannelEnabled(simHandle, c_ServoHub_kChannel2);

    rawFrame->channel0OutOfRange = false;
    rawFrame->channel1OutOfRange = false;
    rawFrame->channel2OutOfRange = false;

    return c_REVLibError_None;
}
}  // namespace

c_REVLib_ErrorCode c_ServoHub_GetPeriodicStatus2(
    c_ServoHub_handle handle, c_ServoHub_PeriodicStatus2* rawFrame) {
    if (c_SIM_ServoHub_IsSim(handle->m_simDevice)) {
        return SIM_ServoHub_GetPeriodicStatus2(handle->m_simDevice, rawFrame);
    }

    std::array<uint8_t, kMaxPacketLength> packedData{};

    c_REVLib_ErrorCode revLibError = REVLib_ReadCANPacketTimeout(
        handle, packedData, rawFrame->timestamp, SERVO_HUB_STATUS_2_FRAME_ID,
        handle->m_status2Period_ms, SERVO_HUB_STATUS_2_LENGTH,
        "Period Status 2");

    if (revLibError != c_REVLibError_None) {
        return revLibError;
    }

    servo_hub_status_2_t frameData;
    servo_hub_status_2_unpack(&frameData, packedData.data(),
                              SERVO_HUB_STATUS_2_LENGTH);

    rawFrame->channel0PulseWidth = frameData.channel_0_pulse_width;
    rawFrame->channel0Enabled = frameData.channel_0_enabled;
    rawFrame->channel0OutOfRange = frameData.channel_0_out_of_range;
    rawFrame->channel1PulseWidth = frameData.channel_1_pulse_width;
    rawFrame->channel1Enabled = frameData.channel_1_enabled;
    rawFrame->channel1OutOfRange = frameData.channel_1_out_of_range;
    rawFrame->channel2PulseWidth = frameData.channel_2_pulse_width;
    rawFrame->channel2Enabled = frameData.channel_2_enabled;
    rawFrame->channel2OutOfRange = frameData.channel_2_out_of_range;
    rawFrame->reserved = frameData.reserved;

    return c_REVLibError_None;
}

namespace {
c_REVLib_ErrorCode SIM_ServoHub_GetPeriodicStatus3(
    c_SIM_ServoHub_handle simHandle, c_ServoHub_PeriodicStatus3* rawFrame) {
    rawFrame->channel3PulseWidth =
        c_SIM_ServoHub_GetChannelPulseWidth(simHandle, c_ServoHub_kChannel3);
    rawFrame->channel4PulseWidth =
        c_SIM_ServoHub_GetChannelPulseWidth(simHandle, c_ServoHub_kChannel4);
    rawFrame->channel5PulseWidth =
        c_SIM_ServoHub_GetChannelPulseWidth(simHandle, c_ServoHub_kChannel5);

    rawFrame->channel3PulseWidth =
        c_SIM_ServoHub_GetChannelEnabled(simHandle, c_ServoHub_kChannel3);
    rawFrame->channel4PulseWidth =
        c_SIM_ServoHub_GetChannelEnabled(simHandle, c_ServoHub_kChannel4);
    rawFrame->channel5PulseWidth =
        c_SIM_ServoHub_GetChannelEnabled(simHandle, c_ServoHub_kChannel5);

    rawFrame->channel3OutOfRange = false;
    rawFrame->channel4OutOfRange = false;
    rawFrame->channel5OutOfRange = false;

    return c_REVLibError_None;
}
}  // namespace

c_REVLib_ErrorCode c_ServoHub_GetPeriodicStatus3(
    c_ServoHub_handle handle, c_ServoHub_PeriodicStatus3* rawFrame) {
    if (c_SIM_ServoHub_IsSim(handle->m_simDevice)) {
        return SIM_ServoHub_GetPeriodicStatus3(handle->m_simDevice, rawFrame);
    }

    std::array<uint8_t, kMaxPacketLength> packedData{};

    c_REVLib_ErrorCode revlibError = REVLib_ReadCANPacketTimeout(
        handle, packedData, rawFrame->timestamp, SERVO_HUB_STATUS_3_FRAME_ID,
        handle->m_status3Period_ms, SERVO_HUB_STATUS_3_LENGTH,
        "Period Status 3");

    if (revlibError != c_REVLibError_None) {
        return revlibError;
    }

    servo_hub_status_3_t frameData;
    servo_hub_status_3_unpack(&frameData, packedData.data(),
                              SERVO_HUB_STATUS_3_LENGTH);

    rawFrame->channel3PulseWidth = frameData.channel_3_pulse_width;
    rawFrame->channel3Enabled = frameData.channel_3_enabled;
    rawFrame->channel3OutOfRange = frameData.channel_3_out_of_range;
    rawFrame->channel4PulseWidth = frameData.channel_4_pulse_width;
    rawFrame->channel4Enabled = frameData.channel_4_enabled;
    rawFrame->channel4OutOfRange = frameData.channel_4_out_of_range;
    rawFrame->channel5PulseWidth = frameData.channel_5_pulse_width;
    rawFrame->channel5Enabled = frameData.channel_5_enabled;
    rawFrame->channel5OutOfRange = frameData.channel_5_out_of_range;
    rawFrame->reserved = frameData.reserved;

    return c_REVLibError_None;
}

namespace {
c_REVLib_ErrorCode SIM_ServoHub_GetPeriodicStatus4(
    c_SIM_ServoHub_handle handle, c_ServoHub_PeriodicStatus4* rawFrame) {
    rawFrame->channel0Current = 0.50f;
    rawFrame->channel1Current = 0.51f;
    rawFrame->channel2Current = 0.52f;
    rawFrame->channel3Current = 0.53f;
    rawFrame->channel4Current = 0.54f;
    rawFrame->channel5Current = 0.55f;

    return c_REVLibError_None;
}
}  // namespace

c_REVLib_ErrorCode c_ServoHub_GetPeriodicStatus4(
    c_ServoHub_handle handle, c_ServoHub_PeriodicStatus4* rawFrame) {
    if (c_SIM_ServoHub_IsSim(handle->m_simDevice)) {
        return SIM_ServoHub_GetPeriodicStatus4(handle->m_simDevice, rawFrame);
    }

    std::array<uint8_t, kMaxPacketLength> packedData{};

    c_REVLib_ErrorCode revlibError = REVLib_ReadCANPacketTimeout(
        handle, packedData, rawFrame->timestamp, SERVO_HUB_STATUS_4_FRAME_ID,
        handle->m_status4Period_ms, SERVO_HUB_STATUS_4_LENGTH,
        "Period Status 4");

    if (revlibError != c_REVLibError_None) {
        return revlibError;
    }

    servo_hub_status_4_t frameData;
    servo_hub_status_4_unpack(&frameData, packedData.data(),
                              SERVO_HUB_STATUS_4_LENGTH);

    rawFrame->channel0Current =
        static_cast<float>(servo_hub_status_4_channel_0_current_decode(
            frameData.channel_0_current));
    rawFrame->channel1Current =
        static_cast<float>(servo_hub_status_4_channel_1_current_decode(
            frameData.channel_1_current));
    rawFrame->channel2Current =
        static_cast<float>(servo_hub_status_4_channel_2_current_decode(
            frameData.channel_2_current));
    rawFrame->channel3Current =
        static_cast<float>(servo_hub_status_4_channel_3_current_decode(
            frameData.channel_3_current));
    rawFrame->channel4Current =
        static_cast<float>(servo_hub_status_4_channel_4_current_decode(
            frameData.channel_4_current));
    rawFrame->channel5Current =
        static_cast<float>(servo_hub_status_4_channel_5_current_decode(
            frameData.channel_5_current));
    rawFrame->reserved = frameData.reserved;

    return c_REVLibError_None;
}

// TODO: Extract common functionality
c_REVLib_ErrorCode c_ServoHub_GetChannelPulseRange(
    c_ServoHub_handle handle, c_ServoHub_Channel channel,
    c_ServoHub_ChannelPulseRange* pulseRange_us) {
    if (c_SIM_ServoHub_IsSim(handle->m_simDevice)) {
        return c_SIM_ServoHub_GetChannelPulseRange(handle->m_simDevice, channel,
                                                   pulseRange_us);
    }

    // Set output to an invalid state
    *pulseRange_us = {0u, 0u, 0u};

    const uint8_t channelIdx = static_cast<uint8_t>(channel);
    servo_hub_get_pulse_range_t channelPulseRange{.channel = channelIdx};

    uint8_t packedDataOut[SERVO_HUB_GET_PULSE_RANGE_LENGTH];
    servo_hub_get_pulse_range_pack(packedDataOut, &channelPulseRange,
                                   SERVO_HUB_GET_PULSE_RANGE_LENGTH);

    std::array<uint8_t, kMaxPacketLength> packedDataIn{};

    const c_REVLib_ErrorCode status = REVLib_WriteAndReadCANPacket(
        handle, packedDataOut, SERVO_HUB_GET_PULSE_RANGE_FRAME_ID, packedDataIn,
        SERVO_HUB_GET_PULSE_RANGE_RESPONSE_FRAME_ID,
        SERVO_HUB_GET_PULSE_RANGE_RESPONSE_LENGTH, "Get Channel Pulse Range");

    if (status != c_REVLibError_None) {
        return (status);
    }

    servo_hub_get_pulse_range_response_t channelPulseRangeResponse;

    // Obtain the pulse widths
    servo_hub_get_pulse_range_response_unpack(
        &channelPulseRangeResponse, packedDataIn.data(),
        SERVO_HUB_GET_PULSE_RANGE_RESPONSE_LENGTH);

    *pulseRange_us = {.minPulse_us = channelPulseRangeResponse.minpulseus,
                      .centerPulse_us = channelPulseRangeResponse.centerpulseus,
                      .maxPulse_us = channelPulseRangeResponse.maxpulseus};

    REVLib_SetLastError(handle, c_REVLibError_None);
    return c_REVLibError_None;
}

c_REVLib_ErrorCode c_ServoHub_GetChannelDisableBehavior(
    c_ServoHub_handle handle, c_ServoHub_Channel channel,
    bool* disableBehavior) {
    if (c_SIM_ServoHub_IsSim(handle->m_simDevice)) {
        return c_SIM_ServoHub_GetChannelDisableBehavior(
            handle->m_simDevice, channel, disableBehavior);
    }

    *disableBehavior = false;

    if (c_SIM_ServoHub_IsSim(handle->m_simDevice)) {
        return c_REVLibError_None;
    }

    std::array<uint8_t, kMaxPacketLength> packedData;
    const c_REVLib_ErrorCode status = REVLib_WriteAndReadRtrCANPacket(
        handle, packedData, SERVO_HUB_GET_DISABLE_BEHAVIOR_FRAME_ID,
        SERVO_HUB_GET_DISABLE_BEHAVIOR_LENGTH);

    if (status != c_REVLibError_None) {
        return (status);
    }

    servo_hub_get_disable_behavior_t frameData;
    servo_hub_get_disable_behavior_unpack(
        &frameData, packedData.data(), SERVO_HUB_GET_DISABLE_BEHAVIOR_LENGTH);

    *disableBehavior = (frameData.behavior_bitfield &
                        (0x1u << static_cast<int>(channel))) != 0;

    REVLib_SetLastError(handle, c_REVLibError_None);
    return c_REVLibError_None;
}

c_REVLib_ErrorCode c_ServoHub_SetChannelPulseWidth(c_ServoHub_handle handle,
                                                   c_ServoHub_Channel channel,
                                                   int pulseWidth_us) {
    c_REVLib_ErrorCode status{c_REVLibError_None};

    if (c_SIM_ServoHub_IsSim(handle->m_simDevice)) {
        return c_SIM_ServoHub_SetChannelPulseWidth(handle->m_simDevice, channel,
                                                   pulseWidth_us);
    }

    if (channel < 3) {
        if (channel == 0) {
            handle->m_setpoint02Data.pulse_width_0 = pulseWidth_us;
        } else if (channel == 1) {
            handle->m_setpoint02Data.pulse_width_1 = pulseWidth_us;
        } else if (channel == 2) {
            handle->m_setpoint02Data.pulse_width_2 = pulseWidth_us;
        }
        servo_hub_set_servo_0_to_2_pulse_widths_pack(
            handle->m_setpoint02Data_Packed.data(), &handle->m_setpoint02Data,
            handle->m_setpoint02Data_Packed.size());

        status = REVLib_WriteCANPacketRepeating(
            handle, handle->m_setpoint02Data_Packed,
            SERVO_HUB_SET_SERVO_0_TO_2_PULSE_WIDTHS_FRAME_ID);
    } else {
        if (channel == 3) {
            handle->m_setpoint35Data.pulse_width_3 = pulseWidth_us;
        } else if (channel == 4) {
            handle->m_setpoint35Data.pulse_width_4 = pulseWidth_us;
        } else if (channel == 5) {
            handle->m_setpoint35Data.pulse_width_5 = pulseWidth_us;
        }
        servo_hub_set_servo_3_to_5_pulse_widths_pack(
            handle->m_setpoint35Data_Packed.data(), &handle->m_setpoint35Data,
            handle->m_setpoint35Data_Packed.size());

        status = REVLib_WriteCANPacketRepeating(
            handle, handle->m_setpoint35Data_Packed,
            SERVO_HUB_SET_SERVO_3_TO_5_PULSE_WIDTHS_FRAME_ID);
    }

    return status;
}

c_REVLib_ErrorCode c_ServoHub_SetChannelEnabled(c_ServoHub_handle handle,
                                                c_ServoHub_Channel channel,
                                                bool enabled) {
    if (c_SIM_ServoHub_IsSim(handle->m_simDevice)) {
        return c_SIM_ServoHub_SetChannelEnabled(handle->m_simDevice, channel,
                                                enabled);
    }

    c_REVLib_ErrorCode status{c_REVLibError_None};

    if (channel < 3) {
        if (channel == 0) {
            handle->m_setpoint02Data.enable_channel_0 = enabled;
        } else if (channel == 1) {
            handle->m_setpoint02Data.enable_channel_1 = enabled;
        } else if (channel == 2) {
            handle->m_setpoint02Data.enable_channel_2 = enabled;
        }
        servo_hub_set_servo_0_to_2_pulse_widths_pack(
            handle->m_setpoint02Data_Packed.data(), &handle->m_setpoint02Data,
            handle->m_setpoint02Data_Packed.size());

        status = REVLib_WriteCANPacketRepeating(
            handle, handle->m_setpoint02Data_Packed,
            SERVO_HUB_SET_SERVO_0_TO_2_PULSE_WIDTHS_FRAME_ID);
    } else {
        if (channel == 3) {
            handle->m_setpoint35Data.enable_channel_3 = enabled;
        } else if (channel == 4) {
            handle->m_setpoint35Data.enable_channel_4 = enabled;
        } else if (channel == 5) {
            handle->m_setpoint35Data.enable_channel_5 = enabled;
        }
        servo_hub_set_servo_3_to_5_pulse_widths_pack(
            handle->m_setpoint35Data_Packed.data(), &handle->m_setpoint35Data,
            handle->m_setpoint35Data_Packed.size());

        status = REVLib_WriteCANPacketRepeating(
            handle, handle->m_setpoint35Data_Packed,
            SERVO_HUB_SET_SERVO_3_TO_5_PULSE_WIDTHS_FRAME_ID);
    }

    return status;
}

c_REVLib_ErrorCode c_ServoHub_SetChannelPowered(c_ServoHub_handle handle,
                                                c_ServoHub_Channel channel,
                                                bool powered) {
    if (c_SIM_ServoHub_IsSim(handle->m_simDevice)) {
        return c_SIM_ServoHub_SetChannelPowered(handle->m_simDevice, channel,
                                                powered);
    }

    c_REVLib_ErrorCode status{c_REVLibError_None};

    if (channel < 3) {
        if (channel == 0) {
            handle->m_setpoint02Data.power_channel_0 = powered;
        } else if (channel == 1) {
            handle->m_setpoint02Data.power_channel_1 = powered;
        } else if (channel == 2) {
            handle->m_setpoint02Data.power_channel_2 = powered;
        }
        servo_hub_set_servo_0_to_2_pulse_widths_pack(
            handle->m_setpoint02Data_Packed.data(), &handle->m_setpoint02Data,
            handle->m_setpoint02Data_Packed.size());

        status = REVLib_WriteCANPacketRepeating(
            handle, handle->m_setpoint02Data_Packed,
            SERVO_HUB_SET_SERVO_0_TO_2_PULSE_WIDTHS_FRAME_ID);
    } else {
        if (channel == 3) {
            handle->m_setpoint35Data.power_channel_3 = powered;
        } else if (channel == 4) {
            handle->m_setpoint35Data.power_channel_4 = powered;
        } else if (channel == 5) {
            handle->m_setpoint35Data.power_channel_5 = powered;
        }
        servo_hub_set_servo_3_to_5_pulse_widths_pack(
            handle->m_setpoint35Data_Packed.data(), &handle->m_setpoint35Data,
            handle->m_setpoint35Data_Packed.size());

        status = REVLib_WriteCANPacketRepeating(
            handle, handle->m_setpoint35Data_Packed,
            SERVO_HUB_SET_SERVO_3_TO_5_PULSE_WIDTHS_FRAME_ID);
    }

    return status;
}

c_REVLib_ErrorCode c_ServoHub_SetBankPulsePeriod(c_ServoHub_handle handle,
                                                 c_ServoHub_Bank bank,
                                                 int pulsePeriod_us) {
    if (c_SIM_ServoHub_IsSim(handle->m_simDevice)) {
        return c_SIM_ServoHub_SetBankPulsePeriod(handle->m_simDevice, bank,
                                                 pulsePeriod_us);
    }

    servo_hub_set_banks_pulse_period_t banksPulsePeriod;

    if (bank == c_ServoHub_kBank0_2) {
        banksPulsePeriod.channel_banks_bitfield = (0x1 << 0);
    } else if (bank == c_ServoHub_kBank3_5) {
        banksPulsePeriod.channel_banks_bitfield = (0x1 << 1);
    } else {
        return c_REVLibError_ParamInvalid;
    }

    banksPulsePeriod.pulse_period = pulsePeriod_us;

    uint8_t packedDataOut[SERVO_HUB_SET_BANKS_PULSE_PERIOD_LENGTH];
    servo_hub_set_banks_pulse_period_pack(
        packedDataOut, &banksPulsePeriod,
        SERVO_HUB_SET_BANKS_PULSE_PERIOD_LENGTH);

    std::array<uint8_t, kMaxPacketLength> packedDataIn{};

    c_REVLib_ErrorCode status = REVLib_WriteAndReadCANPacket(
        handle, packedDataOut, SERVO_HUB_SET_BANKS_PULSE_PERIOD_FRAME_ID,
        packedDataIn, SERVO_HUB_SET_BANKS_PULSE_PERIOD_RESPONSE_FRAME_ID,
        SERVO_HUB_SET_BANKS_PULSE_PERIOD_RESPONSE_LENGTH,
        fmt::format("Set Bank {} Pulse Period",
                    bank == c_ServoHub_kBank0_2 ? "0-2" : "3-5"));

    if (status != c_REVLibError_None) {
        return (status);
    }

    servo_hub_set_banks_pulse_period_response_t banksPulsePeriodResponse;

    // Check the status of the response
    servo_hub_set_banks_pulse_period_response_unpack(
        &banksPulsePeriodResponse, packedDataIn.data(),
        SERVO_HUB_SET_BANKS_PULSE_PERIOD_RESPONSE_LENGTH);

    SetParameterResult paramStatus =
        static_cast<SetParameterResult>(banksPulsePeriodResponse.result);

    if (paramStatus != kParamSuccess) {
        c_REVLib_ErrorCode errCode =
            ServoHub_ParamResultToErrorCode(paramStatus);
        REVLib_SendErrorText(handle, errCode,
                             std::to_string(static_cast<int>(bank)));
        REVLib_SetLastError(handle, errCode);
        return errCode;
    }

    REVLib_SetLastError(handle, c_REVLibError_None);
    return c_REVLibError_None;
}

// Lookup table for programming states, specific to object instance
static constexpr std::array<bool c_ServoHub_ProgrammingEnableStates::*, 6>
    programmingStatePtrs = {
        &c_ServoHub_ProgrammingEnableStates::channel0_programming_enabled,
        &c_ServoHub_ProgrammingEnableStates::channel1_programming_enabled,
        &c_ServoHub_ProgrammingEnableStates::channel2_programming_enabled,
        &c_ServoHub_ProgrammingEnableStates::channel3_programming_enabled,
        &c_ServoHub_ProgrammingEnableStates::channel4_programming_enabled,
        &c_ServoHub_ProgrammingEnableStates::channel5_programming_enabled,
};

// NOTE: For Drive Mode and Set Servo Limits, a given channel should be
// completely configured before attempting to configure another one when issuing
// multiple programming commands per channel (ex. chan 1 -> chan 1 -> chan 2 ->
// etc.). If a user can program chan 1 -> chan 2 -> chan 1 within a second, it
// will not work the second time for chan 1.

c_REVLib_ErrorCode c_ServoHub_SetDriveMode(c_ServoHub_handle handle,
                                           uint8_t channel,
                                           c_ServoHub_DriveMode driveMode) {
    c_REVLib_ErrorCode status;

    if (c_SIM_ServoHub_IsSim(handle->m_simDevice)) {
        status = c_REVLibError_Invalid;
        return status;
    }

    uint64_t timeout = getREVLibDriver()->getPreciseTime();
    c_ServoHub_PeriodicStatus0 rawFrame;

    handle->channelProgrammingStates.*(programmingStatePtrs[channel]) = true;

    do {  // When this exits we will be ready to program
        status = c_ServoHub_EnableProgrammingMode(
            handle, handle->channelProgrammingStates);

        c_ServoHub_GetPeriodicStatus0(handle, &rawFrame);

        if (getREVLibDriver()->getPreciseTime() - timeout >
            1000000) {  // 1000ms timeout
            return c_REVLibError_CANTimeout;
        }
    } while ((!rawFrame.programmingEnabled ||
              status == c_REVLibError_NotReadyToReceiveCommand));

    servo_hub_srs_set_servo_drive_mode_t setDriveModeData;
    setDriveModeData.channel = static_cast<uint8_t>(channel);
    setDriveModeData.mode = static_cast<uint8_t>(driveMode);
    uint8_t packedDataOut[SERVO_HUB_SRS_SET_SERVO_DRIVE_MODE_LENGTH];

    servo_hub_srs_set_servo_drive_mode_pack(
        packedDataOut, &setDriveModeData,
        SERVO_HUB_SRS_SET_SERVO_DRIVE_MODE_LENGTH);

    std::array<uint8_t, kMaxPacketLength> packedDataIn{};

    // Firmware takes really long
    status = REVLib_WriteAndReadCANPacketWithReadTimeout(
        handle, packedDataOut, SERVO_HUB_SRS_SET_SERVO_DRIVE_MODE_FRAME_ID,
        packedDataIn, SERVO_HUB_SRS_SET_DRIVE_MODE_RESPONSE_FRAME_ID,
        SERVO_HUB_SRS_SET_DRIVE_MODE_RESPONSE_LENGTH, 50,
        "Set The Servo Drive Mode");

    if (status != c_REVLibError_None) {
        return (status);
    }

    servo_hub_srs_set_drive_mode_response_t setDriveModeResponse;

    // Check the status of the response
    servo_hub_srs_set_drive_mode_response_unpack(
        &setDriveModeResponse, packedDataIn.data(),
        SERVO_HUB_SRS_SET_DRIVE_MODE_RESPONSE_LENGTH);

    SetParameterResult paramStatus =
        static_cast<SetParameterResult>(setDriveModeResponse.result);

    if (paramStatus != kParamSuccess) {
        c_REVLib_ErrorCode errCode =
            ServoHub_ParamResultToErrorCode(paramStatus);
        REVLib_SendErrorText(handle, errCode,
                             "\nWhen Setting Servo Drive Mode");
        REVLib_SetLastError(handle, errCode);
        return errCode;
    }

    REVLib_SetLastError(handle, c_REVLibError_None);
    return c_REVLibError_None;
}

c_REVLib_ErrorCode c_ServoHub_SetServoLimits(c_ServoHub_handle handle,
                                             uint8_t channel,
                                             uint16_t leftLimit,
                                             uint16_t rightLimit) {
    c_REVLib_ErrorCode status;

    if (c_SIM_ServoHub_IsSim(handle->m_simDevice)) {
        status = c_REVLibError_Invalid;
        return status;
    }

    uint64_t timeout = getREVLibDriver()->getPreciseTime();
    c_ServoHub_PeriodicStatus0 rawFrame;

    handle->channelProgrammingStates.*(programmingStatePtrs[channel]) = true;

    do {  // When this exits we will be ready to program
        status = c_ServoHub_EnableProgrammingMode(
            handle, handle->channelProgrammingStates);

        c_ServoHub_GetPeriodicStatus0(handle, &rawFrame);

        if (getREVLibDriver()->getPreciseTime() - timeout >
            1000000) {  // 1000ms timeout
            return c_REVLibError_CANTimeout;
        }
    } while ((!rawFrame.programmingEnabled ||
              status == c_REVLibError_NotReadyToReceiveCommand));

    servo_hub_srs_set_servo_limits_t setServoLimits;
    setServoLimits.channel = static_cast<uint8_t>(channel);
    setServoLimits.left_limit = leftLimit;
    setServoLimits.right_limit = rightLimit;
    uint8_t packedDataOut[SERVO_HUB_SRS_SET_SERVO_LIMITS_LENGTH];

    servo_hub_srs_set_servo_limits_pack(packedDataOut, &setServoLimits,
                                        SERVO_HUB_SRS_SET_SERVO_LIMITS_LENGTH);

    std::array<uint8_t, kMaxPacketLength> packedDataIn{};

    // Firmware takes really long, but HAL_send skips that
    status = REVLib_WriteAndReadCANPacketWithReadTimeout(
        handle, packedDataOut, SERVO_HUB_SRS_SET_SERVO_LIMITS_FRAME_ID,
        packedDataIn, SERVO_HUB_SRS_SET_LIMITS_RESPONSE_FRAME_ID,
        SERVO_HUB_SRS_SET_LIMITS_RESPONSE_LENGTH, 50,
        "Set hard limits for servo");

    if (status != c_REVLibError_None) {
        return (status);
    }

    servo_hub_srs_set_limits_response_t setLimitsResponse;

    // Check the status of the response
    servo_hub_srs_set_limits_response_unpack(
        &setLimitsResponse, packedDataIn.data(),
        SERVO_HUB_SRS_SET_LIMITS_RESPONSE_LENGTH);

    SetParameterResult paramStatus =
        static_cast<SetParameterResult>(setLimitsResponse.result);

    if (paramStatus != kParamSuccess) {
        c_REVLib_ErrorCode errCode =
            ServoHub_ParamResultToErrorCode(paramStatus);
        REVLib_SendErrorText(handle, errCode, ": When Setting Servo Limits");
        REVLib_SetLastError(handle, errCode);
        return errCode;
    }

    REVLib_SetLastError(handle, c_REVLibError_None);
    return c_REVLibError_None;
}

// NOTE: If we enable other channels than the ones being used, 2 second timeout
// will run fully

// NOTE: If used, this must have a corresponding call at some
//  point with enable states at 0 to put servo bank into normal function (or
//  else PWM won't work)
c_REVLib_ErrorCode c_ServoHub_EnableProgrammingMode(
    c_ServoHub_handle handle,
    c_ServoHub_ProgrammingEnableStates programmingStates) {
    c_REVLib_ErrorCode status;
    uint8_t enableStates = 0;
    handle->channelProgrammingStates = programmingStates;

    // Packing struct of bools back into uint8_t
    for (int i = 0; i < 6; i++) {
        enableStates |= (static_cast<uint8_t>(handle->channelProgrammingStates.*
                                              (programmingStatePtrs[i])))
                        << i;
    }

    if (c_SIM_ServoHub_IsSim(handle->m_simDevice)) {
        status = c_REVLibError_Invalid;
        return status;
    }

    if (enableStates == 0) {  // Ensuring programming of channels finishes
                              // before exiting programming mode
        c_ServoHub_PeriodicStatus0 rawFrame;
        uint64_t timeout = getREVLibDriver()->getPreciseTime();

        do {
            c_ServoHub_GetPeriodicStatus0(handle, &rawFrame);
        } while (
            rawFrame.activelyProgramming &&
            (getREVLibDriver()->getPreciseTime() - timeout <
             4000000));  // 4 second timeout, should be more than ever needed
    }

    servo_hub_srs_enable_programming_mode_t enableProgramming;
    enableProgramming.channel_0_enabled =
        static_cast<uint8_t>((enableStates >> 0) & 0x01);
    enableProgramming.channel_1_enabled =
        static_cast<uint8_t>((enableStates >> 1) & 0x01);
    enableProgramming.channel_2_enabled =
        static_cast<uint8_t>((enableStates >> 2) & 0x01);
    enableProgramming.channel_3_enabled =
        static_cast<uint8_t>((enableStates >> 3) & 0x01);
    enableProgramming.channel_4_enabled =
        static_cast<uint8_t>((enableStates >> 4) & 0x01);
    enableProgramming.channel_5_enabled =
        static_cast<uint8_t>((enableStates >> 5) & 0x01);

    uint8_t packedDataOut[SERVO_HUB_SRS_ENABLE_PROGRAMMING_MODE_LENGTH];

    servo_hub_srs_enable_programming_mode_pack(
        packedDataOut, &enableProgramming,
        SERVO_HUB_SRS_ENABLE_PROGRAMMING_MODE_LENGTH);

    std::array<uint8_t, kMaxPacketLength> packedDataIn{};

    // Firmware takes really long, but HAL_send skips that
    status = REVLib_WriteAndReadCANPacketWithReadTimeout(
        handle, packedDataOut, SERVO_HUB_SRS_ENABLE_PROGRAMMING_MODE_FRAME_ID,
        packedDataIn, SERVO_HUB_SRS_PROGRAMMING_MODE_RESPONSE_FRAME_ID,
        SERVO_HUB_SRS_PROGRAMMING_MODE_RESPONSE_LENGTH, 50,
        "Setting servo programming mode");

    if (status != c_REVLibError_None) {
        return (status);
    }

    servo_hub_srs_programming_mode_response_t enableProgrammingResponse;

    // Check the status of the response
    servo_hub_srs_programming_mode_response_unpack(
        &enableProgrammingResponse, packedDataIn.data(),
        SERVO_HUB_SRS_PROGRAMMING_MODE_RESPONSE_LENGTH);

    SetParameterResult paramStatus =
        static_cast<SetParameterResult>(enableProgrammingResponse.result);

    if (paramStatus != kParamSuccess) {
        c_REVLib_ErrorCode errCode =
            ServoHub_ParamResultToErrorCode(paramStatus);
        if (paramStatus != kParamNotReady) {
            REVLib_SendErrorText(handle, errCode,
                                 ": When Setting Servo Programming Mode");
            REVLib_SetLastError(handle, errCode);
        }

        return errCode;
    }

    REVLib_SetLastError(handle, c_REVLibError_None);
    return c_REVLibError_None;
}

c_REVLib_ErrorCode c_ServoHub_ClearFaults(c_ServoHub_handle handle) {
    if (c_SIM_ServoHub_IsSim(handle->m_simDevice)) {
        if (c_SIM_ServoHub_GetSimFaultManagerExists(handle->m_simDevice)) {
            c_SIM_ServoHub_FaultManager_handle simFaultManager =
                c_SIM_ServoHub_GetOrCreateSimFaultManager(handle->m_simDevice);
            c_SIM_ServoHub_ClearSimFaults(simFaultManager);
        }
    }

    std::array<uint8_t, 0> zeroLengthDataPacket;
    c_REVLib_ErrorCode status =
        REVLib_WriteCANPacket(handle, zeroLengthDataPacket,
                              SERVO_HUB_CLEAR_FAULTS_FRAME_ID, "Clear Faults");

    return status;
}

c_REVLib_ErrorCode c_ServoHub_ConfigureAsync(c_ServoHub_handle handle,
                                             const char* flattenedConfig,
                                             uint8_t resetSafeParameters) {
    c_REVLib_RegisterAsyncCall([=]() {
        c_ServoHub_Configure(handle, flattenedConfig, resetSafeParameters);
    });

    return c_REVLibError_None;
}

void c_SIM_ServoHub_CreateSimFaultManager(c_ServoHub_handle handle) {
    if (c_SIM_ServoHub_IsSim(handle->m_simDevice)) {
        c_SIM_ServoHub_GetOrCreateSimFaultManager(handle->m_simDevice);
    }
}
