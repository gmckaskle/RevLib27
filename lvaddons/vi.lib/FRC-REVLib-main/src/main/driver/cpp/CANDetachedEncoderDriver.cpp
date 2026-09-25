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

#include "rev/CANDetachedEncoderDriver.h"

#include <rev/driver/REVLibDriver.h>

#include <array>
#include <cassert>
#include <cstring>
#include <span>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>

#include <fmt/format.h>

#include "rev/CANDetachedEncoderFrames.h"
#include "rev/CANDriverPrivate.h"
#include "rev/REVLibVersion.h"
#include "rev/sim/CANDetachedEncoder.h"

namespace {

constexpr uint16_t kDetached_kAPIMajorVersion = REVLibMajorVersion;
constexpr uint8_t kDetached_kAPIMinorVersion = REVLibMinorVersion;
constexpr uint8_t kDetached_kAPIBuildVersion = REVLibBuildVersion;
constexpr uint32_t kDetached_kAPIVersion = REVLibVersion;

constexpr int kMaxPacketLength{8};

constexpr int32_t kConfigTimeout_ms{100};
constexpr int32_t kResetConfigTimeout_ms{150};

constexpr int32_t kDefaultControlFramePeriod_ms{100};

constexpr int32_t kDefaultStatus0Period_ms{250};
constexpr int32_t kDefaultStatus1Period_ms{100};
constexpr int32_t kDefaultStatus2Period_ms{20};
constexpr int32_t kDefaultStatus3Period_ms{20};
constexpr int32_t kDefaultStatus4Period_ms{20};

enum ParameterResult { kParamSuccess = 0, kParamError = 1 };

std::array<uint8_t, 0> zeroLengthDataPacket;

}  // namespace

// NOTE: This struct can't be in the unnamed namespace

struct c_Detached_Obj : public c_BaseCAN_Obj {
    c_Detached_Obj(int busId, int deviceId,
                   c_Detached_EncoderModel unconfirmedModel)
        : c_BaseCAN_Obj(SplineEncoder, busId, deviceId),
          m_simDevice{
              c_SIM_Detached_Create(busId, deviceId, unconfirmedModel)} {
        m_controlFramePeriod_ms = kDefaultControlFramePeriod_ms;

        m_status0Period_ms = kDefaultStatus0Period_ms;
        m_status1Period_ms = kDefaultStatus1Period_ms;
        m_status2Period_ms = kDefaultStatus2Period_ms;
        m_status3Period_ms = kDefaultStatus3Period_ms;
        m_status4Period_ms = kDefaultStatus4Period_ms;
    }

    c_Detached_EncoderModel m_encoderModel{c_Detached_kUnknown};

    c_SIM_Detached_handle m_simDevice{nullptr};
};

namespace {

// TODO: This may need to move to a common place if/when we support multiple
//       encoder devices.
CAN_ExistingDeviceIds s_encoder_ExistingDeviceIds;

//  c_Detached_RegisterId() must be called first
c_Detached_handle Detached_Create_Inplace(
    int busId, int deviceId, c_Detached_EncoderModel unconfirmedModel,
    c_REVLib_ErrorCode* status) {
    *status = c_REVLibError_None;

    if (!CAN_IsValidDeviceId(deviceId)) {
        c_REVLib_SendError(c_REVLibError_InvalidCANId, SplineEncoder, busId,
                           deviceId);
        *status = c_REVLibError_InvalidCANId;

        // Don't allow a nullptr to be returned, invalid deviceId will just
        // fail, error is already sent
        // return nullptr;
    }

    c_Detached_handle handle =
        new c_Detached_Obj(busId, deviceId, unconfirmedModel);

    if (!s_encoder_ExistingDeviceIds.ContainsDevice(busId, deviceId)) {
        // I used sendError directly instead of c_REVLib_SendError because
        // we don't want to expose a REVLib error code for this condition.
        REVLib_SendLowLevelError(1,
                                 "c_Detached_RegisterId() was not called "
                                 "before c_Detached_Create()");
    }

    REVLib_SetLastError(handle, c_REVLibError_None);

    c_Detached_GetEncoderModel(handle, &handle->m_encoderModel);

    return handle;
}

c_REVLib_ErrorCode Detached_ParamResultToErrorCode(
    ParameterResult paramStatus) {
    switch (paramStatus) {
        case kParamSuccess:
            return c_REVLibError_None;
        case kParamError:
            return c_REVLibError_ParamInvalid;
    }

    return c_REVLibError_Invalid;
}

// Functions to 'set' a parameter's value(s)
using parameter_set_function_t = c_REVLib_ErrorCode (*)(c_Detached_handle, int,
                                                        uint32_t);

c_REVLib_ErrorCode InternalParamSetterFunction(c_Detached_handle, int,
                                               uint32_t) {
    return c_REVLibError_ParamInvalid;
}

c_REVLib_ErrorCode Detached_SetGenericParameter(c_Detached_handle handle,
                                                int paramId, uint32_t value);

// Functions to 'pack' value(s) into the specified message structure (defined
// in CANDetachedEncoderFrames.h)
template <typename msg_struct_t>
using fw_pack_function_t = int (*)(uint8_t* out, const msg_struct_t* msg,
                                   size_t length);

// The signature of all packing functions. This is the 'type' used to store
// function pointers in the s_Detached_PrivateSetParameterTable array.
// There is only a single uint32_t input value. For multi-value structs
// we need to get creative and use bit-masks and offsets to store multiple
// values in the uint32_t.
// If this proves inadequate, in the future, this will need to be
// re-architected.
using driver_pack_function_t = void (*)(std::span<uint8_t> out, uint32_t value,
                                        size_t length);

template <typename T, typename msg_struct_t,
          fw_pack_function_t<msg_struct_t> fw_pack_func>
void PackMessageUintT(std::span<uint8_t> out, T value, size_t length);

template <typename msg_struct_t, fw_pack_function_t<msg_struct_t> fw_pack_func>
void PackMessageFloat(std::span<uint8_t> out, uint32_t value, size_t length);

// Functions to 'unpack' value(s) from the specified response structure (defined
// in CANDetachedEncoderFrames.h)
template <typename msg_struct_t>
using fw_unpack_function_t = int (*)(msg_struct_t*, const uint8_t*, size_t);

// Signature for functions that unpack setter responses.
using driver_unpack_setter_function_t =
    ParameterResult (*)(const std::span<uint8_t>, size_t);

template <typename resp_struct_t,
          fw_unpack_function_t<resp_struct_t> fw_unpack_func>
ParameterResult UnpackSetterResponse(const std::span<uint8_t> packedInput,
                                     size_t length);

// Internal parameters that allow the use of the generic CAN message sending
// code (i.e., an entry in s_Detached_PrivateSetParameterTable).
enum Detached_InternalConfigParameter {
    c_Detached_kInternal_ResetParameters = c_Detached_NumParameters,
    c_Detached_NumTotalSetterParameters,

    c_Detached_kInternal_GetVersion
};

struct private_set_parameter_table_entry_t {
    int id;
    parameter_set_function_t setter;
    int32_t frameId;
    int32_t length;
    int32_t responseFrameId;
    int32_t responseLength;
    driver_pack_function_t packer;
    driver_unpack_setter_function_t unpacker;
};

constexpr std::array<private_set_parameter_table_entry_t,
                     c_Detached_NumTotalSetterParameters>
    s_Detached_PrivateSetParameterTable{{
        {c_Detached_kEncoderAverageDepth, Detached_SetGenericParameter,
         ENCODER_SET_VELOCITY_AVG_SAMPLES_FRAME_ID,
         ENCODER_SET_VELOCITY_AVG_SAMPLES_LENGTH,
         ENCODER_SET_VELOCITY_AVG_SAMPLES_RESP_FRAME_ID,
         ENCODER_SET_VELOCITY_AVG_SAMPLES_RESP_LENGTH,
         [](std::span<uint8_t> o, uint32_t v, size_t l) {
             PackMessageUintT<uint8_t, encoder_set_velocity_avg_samples_t,
                              encoder_set_velocity_avg_samples_pack>(o, v, l);
         },
         [](const std::span<uint8_t> pI, size_t l) {
             return UnpackSetterResponse<
                 encoder_set_velocity_avg_samples_resp_t,
                 encoder_set_velocity_avg_samples_resp_unpack>(pI, l);
         }},
        {c_Detached_kEncoderInverted, Detached_SetGenericParameter,
         ENCODER_SET_INVERSION_FRAME_ID, ENCODER_SET_INVERSION_LENGTH,
         ENCODER_SET_INVERSION_RESP_FRAME_ID, ENCODER_SET_INVERSION_RESP_LENGTH,
         [](std::span<uint8_t> o, uint32_t v, size_t l) {
             PackMessageUintT<uint8_t, encoder_set_inversion_t,
                              encoder_set_inversion_pack>(o, v, l);
         },
         [](const std::span<uint8_t> pI, size_t l) {
             return UnpackSetterResponse<encoder_set_inversion_resp_t,
                                         encoder_set_inversion_resp_unpack>(pI,
                                                                            l);
         }},
        {c_Detached_kPositionConversionFactor, Detached_SetGenericParameter,
         ENCODER_SET_POSITION_CONV_FACTOR_FRAME_ID,
         ENCODER_SET_POSITION_CONV_FACTOR_LENGTH,
         ENCODER_SET_POSITION_CONV_FACTOR_RESP_FRAME_ID,
         ENCODER_SET_POSITION_CONV_FACTOR_RESP_LENGTH,
         [](std::span<uint8_t> o, uint32_t v, size_t l) {
             PackMessageFloat<encoder_set_position_conv_factor_t,
                              encoder_set_position_conv_factor_pack>(o, v, l);
         },
         [](const std::span<uint8_t> pI, size_t l) {
             return UnpackSetterResponse<
                 encoder_set_position_conv_factor_resp_t,
                 encoder_set_position_conv_factor_resp_unpack>(pI, l);
         }},
        {c_Detached_kVelocityConversionFactor, Detached_SetGenericParameter,
         ENCODER_SET_VELOCITY_CONV_FACTOR_FRAME_ID,
         ENCODER_SET_VELOCITY_CONV_FACTOR_LENGTH,
         ENCODER_SET_VELOCITY_CONV_FACTOR_RESP_FRAME_ID,
         ENCODER_SET_VELOCITY_CONV_FACTOR_RESP_LENGTH,
         [](std::span<uint8_t> o, uint32_t v, size_t l) {
             PackMessageFloat<encoder_set_velocity_conv_factor_t,
                              encoder_set_velocity_conv_factor_pack>(o, v, l);
         },
         [](const std::span<uint8_t> pI, size_t l) {
             return UnpackSetterResponse<
                 encoder_set_velocity_conv_factor_resp_t,
                 encoder_set_velocity_conv_factor_resp_unpack>(pI, l);
         }},
        {c_Detached_kDutyCycleZeroCentered, Detached_SetGenericParameter,
         ENCODER_SET_ZERO_CENTERED_FRAME_ID, ENCODER_SET_ZERO_CENTERED_LENGTH,
         ENCODER_SET_ZERO_CENTERED_RESP_FRAME_ID,
         ENCODER_SET_ZERO_CENTERED_RESP_LENGTH,
         [](std::span<uint8_t> o, uint32_t v, size_t l) {
             PackMessageUintT<uint8_t, encoder_set_zero_centered_t,
                              encoder_set_zero_centered_pack>(o, v, l);
         },
         [](const std::span<uint8_t> pI, size_t l) {
             return UnpackSetterResponse<encoder_set_zero_centered_resp_t,
                                         encoder_set_zero_centered_resp_unpack>(
                 pI, l);
         }},
        {c_Detached_kDutyCycleAverageDepth, Detached_SetGenericParameter,
         ENCODER_SET_ABS_AVG_DEPTH_FRAME_ID, ENCODER_SET_ABS_AVG_DEPTH_LENGTH,
         ENCODER_SET_ABS_AVG_DEPTH_RESP_FRAME_ID,
         ENCODER_SET_ABS_AVG_DEPTH_RESP_LENGTH,
         [](std::span<uint8_t> o, uint32_t v, size_t l) {
             PackMessageUintT<uint8_t, encoder_set_abs_avg_depth_t,
                              encoder_set_abs_avg_depth_pack>(o, v, l);
         },
         [](const std::span<uint8_t> pI, size_t l) {
             return UnpackSetterResponse<encoder_set_abs_avg_depth_resp_t,
                                         encoder_set_abs_avg_depth_resp_unpack>(
                 pI, l);
         }},
        {c_Detached_kDutyCycleOffset, Detached_SetGenericParameter,
         ENCODER_SET_ANGLE_OFFSET_FRAME_ID, ENCODER_SET_ANGLE_OFFSET_LENGTH,
         ENCODER_SET_ANGLE_OFFSET_RESP_FRAME_ID,
         ENCODER_SET_ANGLE_OFFSET_RESP_LENGTH,
         [](std::span<uint8_t> o, uint32_t v, size_t l) {
             PackMessageFloat<encoder_set_angle_offset_t,
                              encoder_set_angle_offset_pack>(o, v, l);
         },
         [](const std::span<uint8_t> pI, size_t l) {
             return UnpackSetterResponse<encoder_set_angle_offset_resp_t,
                                         encoder_set_angle_offset_resp_unpack>(
                 pI, l);
         }},
        {c_Detached_kStatus0Period, Detached_SetGenericParameter,
         ENCODER_SET_PERIODIC_FRAME_0_PERIOD_FRAME_ID,
         ENCODER_SET_PERIODIC_FRAME_0_PERIOD_LENGTH,
         ENCODER_SET_PERIODIC_FRAME_0_PERIOD_RESP_FRAME_ID,
         ENCODER_SET_PERIODIC_FRAME_0_PERIOD_RESP_LENGTH,
         [](std::span<uint8_t> o, uint32_t v, size_t l) {
             PackMessageUintT<uint16_t, encoder_set_periodic_frame_0_period_t,
                              encoder_set_periodic_frame_0_period_pack>(o, v,
                                                                        l);
         },
         [](const std::span<uint8_t> pI, size_t l) {
             return UnpackSetterResponse<
                 encoder_set_periodic_frame_0_period_resp_t,
                 encoder_set_periodic_frame_0_period_resp_unpack>(pI, l);
         }},
        {c_Detached_kStatus1Period, Detached_SetGenericParameter,
         ENCODER_SET_PERIODIC_FRAME_1_PERIOD_FRAME_ID,
         ENCODER_SET_PERIODIC_FRAME_1_PERIOD_LENGTH,
         ENCODER_SET_PERIODIC_FRAME_1_PERIOD_RESP_FRAME_ID,
         ENCODER_SET_PERIODIC_FRAME_1_PERIOD_RESP_LENGTH,
         [](std::span<uint8_t> o, uint32_t v, size_t l) {
             PackMessageUintT<uint16_t, encoder_set_periodic_frame_1_period_t,
                              encoder_set_periodic_frame_1_period_pack>(o, v,
                                                                        l);
         },
         [](const std::span<uint8_t> pI, size_t l) {
             return UnpackSetterResponse<
                 encoder_set_periodic_frame_1_period_resp_t,
                 encoder_set_periodic_frame_1_period_resp_unpack>(pI, l);
         }},
        {c_Detached_kStatus2Period, Detached_SetGenericParameter,
         ENCODER_SET_PERIODIC_FRAME_2_PERIOD_FRAME_ID,
         ENCODER_SET_PERIODIC_FRAME_2_PERIOD_LENGTH,
         ENCODER_SET_PERIODIC_FRAME_2_PERIOD_RESP_FRAME_ID,
         ENCODER_SET_PERIODIC_FRAME_2_PERIOD_RESP_LENGTH,
         [](std::span<uint8_t> o, uint32_t v, size_t l) {
             PackMessageUintT<uint16_t, encoder_set_periodic_frame_2_period_t,
                              encoder_set_periodic_frame_2_period_pack>(o, v,
                                                                        l);
         },
         [](const std::span<uint8_t> pI, size_t l) {
             return UnpackSetterResponse<
                 encoder_set_periodic_frame_2_period_resp_t,
                 encoder_set_periodic_frame_2_period_resp_unpack>(pI, l);
         }},
        {c_Detached_kStatus3Period, Detached_SetGenericParameter,
         ENCODER_SET_PERIODIC_FRAME_3_PERIOD_FRAME_ID,
         ENCODER_SET_PERIODIC_FRAME_3_PERIOD_LENGTH,
         ENCODER_SET_PERIODIC_FRAME_3_PERIOD_RESP_FRAME_ID,
         ENCODER_SET_PERIODIC_FRAME_3_PERIOD_RESP_LENGTH,
         [](std::span<uint8_t> o, uint32_t v, size_t l) {
             PackMessageUintT<uint16_t, encoder_set_periodic_frame_3_period_t,
                              encoder_set_periodic_frame_3_period_pack>(o, v,
                                                                        l);
         },
         [](const std::span<uint8_t> pI, size_t l) {
             return UnpackSetterResponse<
                 encoder_set_periodic_frame_3_period_resp_t,
                 encoder_set_periodic_frame_3_period_resp_unpack>(pI, l);
         }},
        {c_Detached_kStatus4Period, Detached_SetGenericParameter,
         ENCODER_SET_PERIODIC_FRAME_4_PERIOD_FRAME_ID,
         ENCODER_SET_PERIODIC_FRAME_4_PERIOD_LENGTH,
         ENCODER_SET_PERIODIC_FRAME_4_PERIOD_RESP_FRAME_ID,
         ENCODER_SET_PERIODIC_FRAME_4_PERIOD_RESP_LENGTH,
         [](std::span<uint8_t> o, uint32_t v, size_t l) {
             PackMessageUintT<uint16_t, encoder_set_periodic_frame_4_period_t,
                              encoder_set_periodic_frame_4_period_pack>(o, v,
                                                                        l);
         },
         [](const std::span<uint8_t> pI, size_t l) {
             return UnpackSetterResponse<
                 encoder_set_periodic_frame_4_period_resp_t,
                 encoder_set_periodic_frame_4_period_resp_unpack>(pI, l);
         }},
        {c_Detached_kAngleConversionFactor, Detached_SetGenericParameter,
         ENCODER_SET_ANGLE_CONV_FACTOR_FRAME_ID,
         ENCODER_SET_ANGLE_CONV_FACTOR_LENGTH,
         ENCODER_SET_ANGLE_CONV_FACTOR_RESP_FRAME_ID,
         ENCODER_SET_ANGLE_CONV_FACTOR_RESP_LENGTH,
         [](std::span<uint8_t> o, uint32_t v, size_t l) {
             PackMessageFloat<encoder_set_angle_conv_factor_t,
                              encoder_set_angle_conv_factor_pack>(o, v, l);
         },
         [](const std::span<uint8_t> pI, size_t l) {
             return UnpackSetterResponse<
                 encoder_set_angle_conv_factor_resp_t,
                 encoder_set_angle_conv_factor_resp_unpack>(pI, l);
         }},
        {c_Detached_kDutyCycleStartPulseUs, Detached_SetGenericParameter,
         ENCODER_SET_ABS_MIN_HIGH_TIME_FRAME_ID,
         ENCODER_SET_ABS_MIN_HIGH_TIME_LENGTH,
         ENCODER_SET_ABS_MIN_HIGH_TIME_RESP_FRAME_ID,
         ENCODER_SET_ABS_MIN_HIGH_TIME_RESP_LENGTH,
         [](std::span<uint8_t> o, uint32_t v, size_t l) {
             PackMessageFloat<encoder_set_abs_min_high_time_t,
                              encoder_set_abs_min_high_time_pack>(o, v, l);
         },
         [](const std::span<uint8_t> pI, size_t l) {
             return UnpackSetterResponse<
                 encoder_set_abs_min_high_time_resp_t,
                 encoder_set_abs_min_high_time_resp_unpack>(pI, l);
         }},
        {c_Detached_kDutyCycleEndPulseUs, Detached_SetGenericParameter,
         ENCODER_SET_ABS_MIN_LOW_TIME_FRAME_ID,
         ENCODER_SET_ABS_MIN_LOW_TIME_LENGTH,
         ENCODER_SET_ABS_MIN_LOW_TIME_RESP_FRAME_ID,
         ENCODER_SET_ABS_MIN_LOW_TIME_RESP_LENGTH,
         [](std::span<uint8_t> o, uint32_t v, size_t l) {
             PackMessageFloat<encoder_set_abs_min_low_time_t,
                              encoder_set_abs_min_low_time_pack>(o, v, l);
         },
         [](const std::span<uint8_t> pI, size_t l) {
             return UnpackSetterResponse<
                 encoder_set_abs_min_low_time_resp_t,
                 encoder_set_abs_min_low_time_resp_unpack>(pI, l);
         }},
        {c_Detached_kDutyCyclePeriodUs, Detached_SetGenericParameter,
         ENCODER_SET_ABS_PERIOD_FRAME_ID, ENCODER_SET_ABS_PERIOD_LENGTH,
         ENCODER_SET_ABS_PERIOD_RESP_FRAME_ID,
         ENCODER_SET_ABS_PERIOD_RESP_LENGTH,
         [](std::span<uint8_t> o, uint32_t v, size_t l) {
             PackMessageFloat<encoder_set_abs_period_t,
                              encoder_set_abs_period_pack>(o, v, l);
         },
         [](const std::span<uint8_t> pI, size_t l) {
             return UnpackSetterResponse<encoder_set_abs_period_resp_t,
                                         encoder_set_abs_period_resp_unpack>(pI,
                                                                             l);
         }},

        // Internal (to the Driver) Parameters
        {c_Detached_kInternal_ResetParameters, InternalParamSetterFunction,
         ENCODER_RESTORE_DEFAULTS_FRAME_ID, ENCODER_RESTORE_DEFAULTS_LENGTH,
         ENCODER_RESTORE_DEFAULTS_RESP_FRAME_ID,
         ENCODER_RESTORE_DEFAULTS_RESP_LENGTH,
         [](std::span<uint8_t> o, uint32_t v, size_t l) {
             PackMessageUintT<uint32_t, encoder_restore_defaults_t,
                              encoder_restore_defaults_pack>(o, v, l);
         },
         [](const std::span<uint8_t> pI, size_t l) {
             return UnpackSetterResponse<encoder_restore_defaults_resp_t,
                                         encoder_restore_defaults_resp_unpack>(
                 pI, l);
         }},
    }};

/////////////////////////////////////////////////////////////
// Helpers to ensure the <id> matches the index in the array

// Helper to check a single index at compile time
template <auto& Arr, size_t I>
consteval bool check_element() {
    static_assert(I == Arr[I].id, "Element failed validation!");
    return (I == Arr[I].id);
}

// Expand indices using an integer sequence (C++17+)
template <auto& Arr, size_t... Is>
consteval bool validate_all_impl(std::integer_sequence<size_t, Is...>) {
    return (check_element<Arr, Is>() && ...);
}

// Public interface
template <auto& Arr>
consteval bool validate_all() {
    return validate_all_impl<Arr>(std::make_index_sequence<Arr.size()>{});
}

// Triggers the compile-time loop and static_assert checks
static_assert(validate_all<s_Detached_PrivateSetParameterTable>());

/////////////////////////////////////////////////////////////

template <typename T, typename msg_struct_t,
          fw_pack_function_t<msg_struct_t> fw_pack_func>
void PackMessageUintT(std::span<uint8_t> out, T value, size_t length) {
    assert(out.size() >= length &&
           "output packet is not large enough for packed message");

    const msg_struct_t msg{value};
    fw_pack_func(out.data(), &msg, length);
}

template <typename msg_struct_t, fw_pack_function_t<msg_struct_t> fw_pack_func>
void PackMessageFloat(std::span<uint8_t> out, uint32_t value, size_t length) {
    assert(out.size() >= length &&
           "output packet is not large enough for packed message");

    float floatValue;
    std::memcpy(&floatValue, &value, sizeof(floatValue));

    const msg_struct_t msg{floatValue};
    fw_pack_func(out.data(), &msg, length);
}

template <typename msg_struct_t>
void UnpackMessage(msg_struct_t* out, const std::span<uint8_t> packedInput,
                   size_t length, fw_unpack_function_t<msg_struct_t> unpacker) {
    assert(packedInput.size() >= length &&
           "input packet is not large enough to contain the packed message");
    unpacker(out, packedInput.data(), length);
}

template <typename resp_struct_t,
          fw_unpack_function_t<resp_struct_t> fw_unpack_func>
ParameterResult UnpackSetterResponse(const std::span<uint8_t> packedInput,
                                     size_t length) {
    resp_struct_t response;
    UnpackMessage(&response, packedInput, length, fw_unpack_func);

    ParameterResult paramStatus = static_cast<ParameterResult>(response.error);

    return paramStatus;
}

c_REVLib_ErrorCode Detached_SetCore(c_Detached_handle handle, int paramId,
                                    uint32_t value, int32_t timeout_ms,
                                    std::string_view description) {
    const size_t paramIdx{static_cast<size_t>(paramId)};
    const auto& param = s_Detached_PrivateSetParameterTable[paramIdx];

    assert((paramIdx == static_cast<size_t>(param.id)) &&
           "param index != param id");

    std::array<uint8_t, kMaxPacketLength> packedDataOut;
    param.packer(packedDataOut, value, param.length);

    std::array<uint8_t, kMaxPacketLength> packedDataIn{};

    // Set the size according to the actual parameter requirement
    std::span<uint8_t> dataOutPacket(packedDataOut.data(), param.length);

    const std::string setDescription{fmt::format("Set: {}", description)};

    c_REVLib_ErrorCode status = REVLib_WriteAndReadCANPacketWithReadTimeout(
        handle, dataOutPacket, param.frameId, packedDataIn,
        param.responseFrameId, param.responseLength, timeout_ms,
        setDescription);

    if (status != c_REVLibError_None) {
        return (status);
    }

    ParameterResult paramStatus =
        param.unpacker(packedDataIn, param.responseLength);

    if (paramStatus != kParamSuccess) {
        c_REVLib_ErrorCode errCode =
            Detached_ParamResultToErrorCode(paramStatus);
        REVLib_SendErrorText(handle, errCode, setDescription);
        REVLib_SetLastError(handle, errCode);
        return errCode;
    }

    REVLib_SetLastError(handle, c_REVLibError_None);
    return c_REVLibError_None;
}

c_REVLib_ErrorCode Detached_SetGenericParameter(c_Detached_handle handle,
                                                int paramId, uint32_t value) {
    if (c_SIM_Detached_IsSim(handle->m_simDevice)) {
        return c_SIM_Detached_SetParameter(
            handle->m_simDevice,
            static_cast<c_Detached_ConfigParameter>(paramId), value);
    }

    return Detached_SetCore(
        handle, paramId, value, kConfigTimeout_ms,
        c_Detached_GetParameterName(
            static_cast<c_Detached_ConfigParameter>(paramId)));
}

// Parameter Getter functions

struct private_get_parameter_context_t {
    int paramId;
    int32_t frameId;
    int32_t responseFrameId;
    int32_t responseLength;
};

template <typename resp_struct_t>
ParameterResult UnpackGetterResponse(
    resp_struct_t* response, const std::span<uint8_t> packedInput,
    private_get_parameter_context_t context,
    fw_unpack_function_t<resp_struct_t> unpacker) {
    UnpackMessage(response, packedInput, context.responseLength, unpacker);

    ParameterResult paramStatus{kParamSuccess};
    if constexpr (!std::is_same_v<resp_struct_t,
                                  encoder_get_versioning_resp_t>) {
        paramStatus = static_cast<ParameterResult>(response->error);
    }

    return paramStatus;
}

template <typename resp_struct_t>
c_REVLib_ErrorCode Detached_GetCoreWithArgument(
    c_Detached_handle handle, private_get_parameter_context_t context,
    std::span<const uint8_t> writePacket,
    fw_unpack_function_t<resp_struct_t> unpacker, resp_struct_t* response,
    std::string_view description) {
    std::array<uint8_t, kMaxPacketLength> packedDataIn{};

    const std::string getDescription{fmt::format("Get: {}", description)};

    const c_REVLib_ErrorCode status = REVLib_WriteAndReadCANPacket(
        handle, writePacket, context.frameId, packedDataIn,
        context.responseFrameId, context.responseLength, getDescription);

    if (status != c_REVLibError_None) {
        return (status);
    }

    const ParameterResult result =
        UnpackGetterResponse(response, packedDataIn, context, unpacker);

    if (result != kParamSuccess) {
        c_REVLib_ErrorCode errCode = Detached_ParamResultToErrorCode(
            static_cast<ParameterResult>(result));
        REVLib_SendErrorText(handle, errCode, getDescription);
        REVLib_SetLastError(handle, errCode);
        return errCode;
    }

    REVLib_SetLastError(handle, c_REVLibError_None);
    return c_REVLibError_None;
}

template <typename resp_struct_t>
c_REVLib_ErrorCode Detached_GetCore(
    c_Detached_handle handle, private_get_parameter_context_t context,
    fw_unpack_function_t<resp_struct_t> unpacker, resp_struct_t* response,
    std::string_view description) {
    return Detached_GetCoreWithArgument(handle, context, zeroLengthDataPacket,
                                        unpacker, response, description);
}

template <typename resp_struct_t>
c_REVLib_ErrorCode Detached_GetGenericParameter(
    c_Detached_handle handle, private_get_parameter_context_t context,
    fw_unpack_function_t<resp_struct_t> unpacker, resp_struct_t* response) {
    const char* description = c_Detached_GetParameterName(
        static_cast<c_Detached_ConfigParameter>(context.paramId));

    return Detached_GetCore(handle, context, unpacker, response, description);
}

c_REVLib_ErrorCode Detached_GetStatusPeriod(c_Detached_handle handle,
                                            c_Detached_ConfigParameter periodId,
                                            uint32_t* period) {
    if ((periodId < c_Detached_kStatus0Period) ||
        (periodId > c_Detached_kStatus4Period)) {
        return c_REVLibError_ParamInvalid;
    }

    if (c_SIM_Detached_IsSim(handle->m_simDevice)) {
        return c_SIM_Detached_GetParameter(handle->m_simDevice, periodId,
                                           period);
    }

    const int32_t frame_idx{periodId - c_Detached_kStatus0Period};

    c_REVLib_ErrorCode status{c_REVLibError_ParamInvalid};

    // TODO (dave): This sucks. Do better (later)!
    switch (frame_idx) {
        case 0: {
            encoder_get_periodic_frame_0_period_resp_t periodResponse;
            status = Detached_GetGenericParameter(
                handle,
                {periodId, ENCODER_GET_PERIODIC_FRAME_0_PERIOD_FRAME_ID,
                 ENCODER_GET_PERIODIC_FRAME_0_PERIOD_RESP_FRAME_ID,
                 ENCODER_GET_PERIODIC_FRAME_0_PERIOD_RESP_LENGTH},
                encoder_get_periodic_frame_0_period_resp_unpack,
                &periodResponse);

            if (status != c_REVLibError_None) {
                return (status);
            }

            *period = periodResponse.period;
            break;
        }

        case 1: {
            encoder_get_periodic_frame_1_period_resp_t periodResponse;
            status = Detached_GetGenericParameter(
                handle,
                {periodId, ENCODER_GET_PERIODIC_FRAME_1_PERIOD_FRAME_ID,
                 ENCODER_GET_PERIODIC_FRAME_1_PERIOD_RESP_FRAME_ID,
                 ENCODER_GET_PERIODIC_FRAME_1_PERIOD_RESP_LENGTH},
                encoder_get_periodic_frame_1_period_resp_unpack,
                &periodResponse);

            if (status != c_REVLibError_None) {
                return (status);
            }

            *period = periodResponse.period;
            break;
        }

        case 2: {
            encoder_get_periodic_frame_2_period_resp_t periodResponse;
            status = Detached_GetGenericParameter(
                handle,
                {periodId, ENCODER_GET_PERIODIC_FRAME_2_PERIOD_FRAME_ID,
                 ENCODER_GET_PERIODIC_FRAME_2_PERIOD_RESP_FRAME_ID,
                 ENCODER_GET_PERIODIC_FRAME_2_PERIOD_RESP_LENGTH},
                encoder_get_periodic_frame_2_period_resp_unpack,
                &periodResponse);

            if (status != c_REVLibError_None) {
                return (status);
            }

            *period = periodResponse.period;
            break;
        }

        case 3: {
            encoder_get_periodic_frame_3_period_resp_t periodResponse;
            status = Detached_GetGenericParameter(
                handle,
                {periodId, ENCODER_GET_PERIODIC_FRAME_3_PERIOD_FRAME_ID,
                 ENCODER_GET_PERIODIC_FRAME_3_PERIOD_RESP_FRAME_ID,
                 ENCODER_GET_PERIODIC_FRAME_3_PERIOD_RESP_LENGTH},
                encoder_get_periodic_frame_3_period_resp_unpack,
                &periodResponse);

            if (status != c_REVLibError_None) {
                return (status);
            }

            *period = periodResponse.period;
            break;
        }

        case 4: {
            encoder_get_periodic_frame_4_period_resp_t periodResponse;
            status = Detached_GetGenericParameter(
                handle,
                {periodId, ENCODER_GET_PERIODIC_FRAME_4_PERIOD_FRAME_ID,
                 ENCODER_GET_PERIODIC_FRAME_4_PERIOD_RESP_FRAME_ID,
                 ENCODER_GET_PERIODIC_FRAME_4_PERIOD_RESP_LENGTH},
                encoder_get_periodic_frame_4_period_resp_unpack,
                &periodResponse);

            if (status != c_REVLibError_None) {
                return (status);
            }

            *period = periodResponse.period;
            break;
        }

        default:
            return c_REVLibError_ParamInvalid;
    }

    return c_REVLibError_None;
}

struct status_period_info_t {
    c_Detached_ConfigParameter periodId;
    int* handle_statusPeriod_ms;
};

void Detached_GetStatusPeriodsFromDevice(c_Detached_handle handle) {
    const std::array<status_period_info_t, 5> statusPeriods{
        {{c_Detached_kStatus0Period, &handle->m_status0Period_ms},
         {c_Detached_kStatus1Period, &handle->m_status1Period_ms},
         {c_Detached_kStatus2Period, &handle->m_status2Period_ms},
         {c_Detached_kStatus3Period, &handle->m_status3Period_ms},
         {c_Detached_kStatus4Period, &handle->m_status4Period_ms}}};

    for (const auto& sp : statusPeriods) {
        uint32_t period{100};
        Detached_GetStatusPeriod(handle, sp.periodId, &period);
        *sp.handle_statusPeriod_ms = static_cast<int32_t>(period);
    }
}

}  // namespace

// High-level libraries should throw an exception if this
// returns c_REVLibError_DuplicateCANId or c_REVLibError_InvalidCANId
c_REVLib_ErrorCode c_Detached_RegisterId(int busId, int deviceId) {
    if (!CAN_IsValidDeviceId(deviceId)) {
        return c_REVLibError_InvalidCANId;
    }

    if (!s_encoder_ExistingDeviceIds.InsertDevice(busId, deviceId)) {
        return c_REVLibError_DuplicateCANId;
    }

    return c_REVLibError_None;
}

// c_Detached_RegisterId() must be called first
c_Detached_handle c_Detached_Create(int busId, int deviceId,
                                    c_Detached_EncoderModel unconfirmedModel,
                                    c_REVLib_ErrorCode* status) {
    c_Detached_handle handle =
        Detached_Create_Inplace(busId, deviceId, unconfirmedModel, status);

    if (*status != c_REVLibError_None) {
        return handle;
    }

    Detached_GetStatusPeriodsFromDevice(handle);

    getREVLibDriver()->reportDeviceUsage(REVDevice::SplineEncoder, busId,
                                         deviceId);

    c_Detached_ClearFaults(handle);

    return handle;
}

void c_Detached_Close(c_Detached_handle handle) {
    if (handle == nullptr) {
        return;
    }

    s_encoder_ExistingDeviceIds.RemoveDevice(
        static_cast<uint8_t>(handle->m_busId),
        static_cast<uint8_t>(handle->m_deviceId));

    if (handle->m_simDevice != NULL) {
        c_SIM_Detached_Close(handle->m_simDevice);
    }
}

void c_Detached_Destroy(c_Detached_handle handle) {
    if (handle == nullptr) {
        return;
    }

    c_SIM_Detached_Destroy(handle->m_simDevice);

    delete handle;
}

namespace {
/**
 * Helper function to validate parameters before processing them.
 */
c_REVLib_ErrorCode Detached_ValidateParameters(
    c_Detached_handle handle,
    std::unordered_map<c_Detached_ConfigParameter, uint32_t>& parameters) {
    return c_REVLibError_None;
}

/**
 * Helper function to handle additional actions with parameters before sending
 * them to the device. Parameters are passed by reference and can be modified by
 * this function which can be useful for any kind of sanitizing.
 */
c_REVLib_ErrorCode Detached_PreProcessParameters(
    c_Detached_handle handle,
    std::unordered_map<c_Detached_ConfigParameter, uint32_t>& parameters) {
    return c_REVLibError_None;
}

/**
 * Helper function to reset parameters to their default IF they have not
 * been set in Configure().
 */
c_REVLib_ErrorCode Detached_ResetParameters(c_Detached_handle handle) {
    if (c_SIM_Detached_IsSim(handle->m_simDevice)) {
        return c_SIM_Detached_ResetParameters(handle->m_simDevice);
    }

    static constexpr uint32_t kResetMagicNumber{0x4A11B335};

    return Detached_SetCore(handle, c_Detached_kInternal_ResetParameters,
                            kResetMagicNumber, kResetConfigTimeout_ms,
                            "Reset Parameters");
}

/**
 * Helper function to handle additional actions with parameters after sending
 * them to the device.
 */
c_REVLib_ErrorCode Detached_PostProcessParameters(
    c_Detached_handle handle,
    std::unordered_map<c_Detached_ConfigParameter, uint32_t>& parameters) {
    const std::array<status_period_info_t, 5> statusPeriods{{
        {c_Detached_kStatus0Period, &handle->m_status0Period_ms},
        {c_Detached_kStatus1Period, &handle->m_status1Period_ms},
        {c_Detached_kStatus2Period, &handle->m_status2Period_ms},
        {c_Detached_kStatus3Period, &handle->m_status3Period_ms},
        {c_Detached_kStatus4Period, &handle->m_status4Period_ms},
    }};

    for (auto& sp : statusPeriods) {
        if (parameters.contains(sp.periodId)) {
            const int periodMs = parameters[sp.periodId];
            *(sp.handle_statusPeriod_ms) = periodMs;
        }
    }

    return c_REVLibError_None;
}

}  // namespace

c_REVLib_ErrorCode c_Detached_Configure(c_Detached_handle handle,
                                        const char* flattenedConfig,
                                        uint8_t resetSafeParameters) {
    // Unflatten string into processable data
    std::istringstream iss(flattenedConfig);
    std::string line;

    std::unordered_map<c_Detached_ConfigParameter, uint32_t> parameters;
    while (std::getline(iss, line)) {
        auto delimiterIndex = line.find(',');
        std::string paramIdStr = line.substr(0, delimiterIndex);
        std::string paramValueStr = line.substr(delimiterIndex + 1);

        c_Detached_ConfigParameter paramId =
            static_cast<c_Detached_ConfigParameter>(std::stoul(paramIdStr));
        uint32_t paramValue;
        std::stringstream ss;
        ss << std::hex << paramValueStr;
        ss >> paramValue;

        parameters[paramId] = paramValue;
    }

    c_REVLib_ErrorCode status = Detached_ValidateParameters(handle, parameters);
    if (status != c_REVLibError_None) return status;

    status = Detached_PreProcessParameters(handle, parameters);
    if (status != c_REVLibError_None) return status;

    // Restore defaults if specified by user
    if (resetSafeParameters) {
        Detached_ResetParameters(handle);
    }

    // Iterate through parameters and write each one
    for (const auto& [key, value] : parameters) {
        const size_t paramIdx{static_cast<size_t>(key)};

        status = s_Detached_PrivateSetParameterTable[paramIdx].setter(
            handle, key, value);

        // Only return if param-specific error return by device
        if (status == c_REVLibError_ParamInvalid) {
            return status;
        }
    }

    status = Detached_PostProcessParameters(handle, parameters);
    if (status != c_REVLibError_None) return status;

    return c_REVLibError_None;
}

void c_Detached_SetPeriodicFrameTimeout(c_Detached_handle handle,
                                        int timeout_ms) {
    if (timeout_ms < 0) {
        return;
    }
    handle->m_periodicFrameTimeout_ms = timeout_ms;
}

c_REVLib_ErrorCode c_Detached_SetCANTimeout(c_Detached_handle handle,
                                            int timeout_ms) {
    handle->m_canTimeout_ms = timeout_ms;
    REVLib_SetLastError(handle, c_REVLibError_None);
    return c_REVLibError_None;
}

void c_Detached_SetCANMaxRetries(c_Detached_handle handle, int numRetries) {
    if (numRetries < 0) {
        return;
    }
    handle->m_canMaxRetryCount = numRetries;
}

void c_Detached_SetControlFramePeriod(c_Detached_handle handle, int period_ms) {
    if (period_ms < 0) {
        return;
    }
    handle->m_controlFramePeriod_ms = period_ms;
}

int c_Detached_GetControlFramePeriod(c_Detached_handle handle) {
    return handle->m_controlFramePeriod_ms;
}

c_REVLib_APIVersion c_Detached_GetAPIVersion(void) {
    return c_REVLib_APIVersion{.Major = kDetached_kAPIMajorVersion,
                               .Minor = kDetached_kAPIMinorVersion,
                               .Build = kDetached_kAPIBuildVersion,
                               .Version = kDetached_kAPIVersion};
}

c_REVLib_ErrorCode c_Detached_GetFirmwareVersion(
    c_Detached_handle handle, c_Detached_FirmwareVersion* fwVersion) {
    if (c_SIM_Detached_IsSim(handle->m_simDevice)) {
        fwVersion->hwMinor = 0;
        fwVersion->hwMajor = 0;
        fwVersion->fwPrerelease = 0;
        fwVersion->fwFix = 0;
        fwVersion->fwMinor = 0;
        fwVersion->fwYear = 0;
        return c_REVLibError_None;
    }

    encoder_get_versioning_resp_t versionResponse;
    const c_REVLib_ErrorCode status = Detached_GetCore(
        handle,
        {c_Detached_kInternal_GetVersion, ENCODER_GET_VERSIONING_FRAME_ID,
         ENCODER_GET_VERSIONING_RESP_FRAME_ID,
         ENCODER_GET_VERSIONING_RESP_LENGTH},
        encoder_get_versioning_resp_unpack, &versionResponse,
        "Firmware Version");

    if (status != c_REVLibError_None) {
        return (status);
    }

    fwVersion->hwMinor = versionResponse.hw_minor;
    fwVersion->hwMajor = versionResponse.hw_minor;
    fwVersion->fwPrerelease = versionResponse.sw_prerelease;
    fwVersion->fwFix = versionResponse.sw_fix;
    fwVersion->fwMinor = versionResponse.sw_minor;
    fwVersion->fwYear = versionResponse.sw_major;

    return c_REVLibError_None;
}

c_REVLib_ErrorCode c_Detached_GetEncoderModel(c_Detached_handle handle,
                                              c_Detached_EncoderModel* model) {
    if (c_SIM_Detached_IsSim(handle->m_simDevice)) {
        uint8_t value = c_SIM_Detached_GetEncoderModel(handle->m_simDevice);
        *model = static_cast<c_Detached_EncoderModel>(value);
        return c_REVLibError_None;
    }

    c_Detached_PeriodicStatus0 statusFrame;
    const c_REVLib_ErrorCode status =
        c_Detached_GetPeriodicStatus0(handle, &statusFrame);

    if (status != c_REVLibError_None) {
        *model = c_Detached_kUnknown;
        REVLib_SendErrorText(
            handle, status,
            "Getting Detached Encoder Model failed. Unable to account for "
            "device-specific behavior differences.\n");

        return status;
    }

    *model = static_cast<c_Detached_EncoderModel>(statusFrame.encoderModel);

    return status;
}

c_REVLib_ErrorCode c_Detached_GetInverted(c_Detached_handle handle,
                                          bool* inverted) {
    if (c_SIM_Detached_IsSim(handle->m_simDevice)) {
        uint32_t value;
        c_REVLib_ErrorCode status = c_SIM_Detached_GetParameter(
            handle->m_simDevice, c_Detached_kEncoderInverted, &value);
        *inverted = value != 0;
        return status;
    }

    encoder_get_inversion_resp_t inversionResponse;
    const c_REVLib_ErrorCode status = Detached_GetGenericParameter(
        handle,
        {c_Detached_kEncoderInverted, ENCODER_GET_INVERSION_FRAME_ID,
         ENCODER_GET_INVERSION_RESP_FRAME_ID,
         ENCODER_GET_INVERSION_RESP_LENGTH},
        encoder_get_inversion_resp_unpack, &inversionResponse);

    if (status != c_REVLibError_None) {
        return (status);
    }

    *inverted = inversionResponse.inverted != 0;

    return c_REVLibError_None;
}

c_REVLib_ErrorCode c_Detached_GetPositionConversionFactor(
    c_Detached_handle handle, float* positionConversionFactor) {
    if (c_SIM_Detached_IsSim(handle->m_simDevice)) {
        uint32_t value;
        c_REVLib_ErrorCode status = c_SIM_Detached_GetParameter(
            handle->m_simDevice, c_Detached_kPositionConversionFactor, &value);
        std::memcpy(positionConversionFactor, &value,
                    sizeof(*positionConversionFactor));
        return status;
    }

    encoder_get_position_conv_factor_resp_t posConvFactorResponse;
    const c_REVLib_ErrorCode status = Detached_GetGenericParameter(
        handle,
        {c_Detached_kPositionConversionFactor,
         ENCODER_GET_POSITION_CONV_FACTOR_FRAME_ID,
         ENCODER_GET_POSITION_CONV_FACTOR_RESP_FRAME_ID,
         ENCODER_GET_POSITION_CONV_FACTOR_RESP_LENGTH},
        encoder_get_position_conv_factor_resp_unpack, &posConvFactorResponse);

    if (status != c_REVLibError_None) {
        return (status);
    }

    *positionConversionFactor = posConvFactorResponse.factor;

    return c_REVLibError_None;
}

c_REVLib_ErrorCode c_Detached_GetVelocityConversionFactor(
    c_Detached_handle handle, float* velocityConversionFactor) {
    if (c_SIM_Detached_IsSim(handle->m_simDevice)) {
        uint32_t value;
        c_REVLib_ErrorCode status = c_SIM_Detached_GetParameter(
            handle->m_simDevice, c_Detached_kVelocityConversionFactor, &value);
        std::memcpy(velocityConversionFactor, &value,
                    sizeof(*velocityConversionFactor));
        return status;
    }

    encoder_get_velocity_conv_factor_resp_t velConvFactorResponse;
    const c_REVLib_ErrorCode status = Detached_GetGenericParameter(
        handle,
        {c_Detached_kVelocityConversionFactor,
         ENCODER_GET_VELOCITY_CONV_FACTOR_FRAME_ID,
         ENCODER_GET_VELOCITY_CONV_FACTOR_RESP_FRAME_ID,
         ENCODER_GET_VELOCITY_CONV_FACTOR_RESP_LENGTH},
        encoder_get_velocity_conv_factor_resp_unpack, &velConvFactorResponse);

    if (status != c_REVLibError_None) {
        return (status);
    }

    *velocityConversionFactor = velConvFactorResponse.factor;

    return c_REVLibError_None;
}

c_REVLib_ErrorCode c_Detached_GetAngleConversionFactor(
    c_Detached_handle handle, float* angleConversionFactor) {
    if (c_SIM_Detached_IsSim(handle->m_simDevice)) {
        uint32_t value;
        c_REVLib_ErrorCode status = c_SIM_Detached_GetParameter(
            handle->m_simDevice, c_Detached_kAngleConversionFactor, &value);
        std::memcpy(angleConversionFactor, &value,
                    sizeof(*angleConversionFactor));
        return status;
    }

    encoder_get_angle_conv_factor_resp_t angleConvFactorResponse;
    const c_REVLib_ErrorCode status = Detached_GetGenericParameter(
        handle,
        {c_Detached_kAngleConversionFactor,
         ENCODER_GET_ANGLE_CONV_FACTOR_FRAME_ID,
         ENCODER_GET_ANGLE_CONV_FACTOR_RESP_FRAME_ID,
         ENCODER_GET_ANGLE_CONV_FACTOR_RESP_LENGTH},
        encoder_get_angle_conv_factor_resp_unpack, &angleConvFactorResponse);

    if (status != c_REVLibError_None) {
        return (status);
    }

    *angleConversionFactor = angleConvFactorResponse.factor;

    return c_REVLibError_None;
}

c_REVLib_ErrorCode c_Detached_GetQuadratureAverageDepth(
    c_Detached_handle handle, uint32_t* quadratureAverageDepth) {
    if (c_SIM_Detached_IsSim(handle->m_simDevice)) {
        c_REVLib_ErrorCode status = c_SIM_Detached_GetParameter(
            handle->m_simDevice, c_Detached_kEncoderAverageDepth,
            quadratureAverageDepth);
        return status;
    }

    encoder_get_velocity_avg_samples_resp_t averageDepthResponse;
    const c_REVLib_ErrorCode status = Detached_GetGenericParameter(
        handle,
        {c_Detached_kEncoderAverageDepth,
         ENCODER_GET_VELOCITY_AVG_SAMPLES_FRAME_ID,
         ENCODER_GET_VELOCITY_AVG_SAMPLES_RESP_FRAME_ID,
         ENCODER_GET_VELOCITY_AVG_SAMPLES_RESP_LENGTH},
        encoder_get_velocity_avg_samples_resp_unpack, &averageDepthResponse);

    if (status != c_REVLibError_None) {
        return (status);
    }

    *quadratureAverageDepth = averageDepthResponse.samples;

    return c_REVLibError_None;
}

c_REVLib_ErrorCode c_Detached_GetZeroOffset(c_Detached_handle handle,
                                            float* zeroOffset) {
    if (c_SIM_Detached_IsSim(handle->m_simDevice)) {
        uint32_t value;
        c_REVLib_ErrorCode status = c_SIM_Detached_GetParameter(
            handle->m_simDevice, c_Detached_kDutyCycleOffset, &value);
        std::memcpy(zeroOffset, &value, sizeof(*zeroOffset));
        return status;
    }

    encoder_get_angle_offset_resp_t zeroOffsetResponse;
    const c_REVLib_ErrorCode status = Detached_GetGenericParameter(
        handle,
        {c_Detached_kDutyCycleOffset, ENCODER_GET_ANGLE_OFFSET_FRAME_ID,
         ENCODER_GET_ANGLE_OFFSET_RESP_FRAME_ID,
         ENCODER_GET_ANGLE_OFFSET_RESP_LENGTH},
        encoder_get_angle_offset_resp_unpack, &zeroOffsetResponse);

    if (status != c_REVLibError_None) {
        return (status);
    }

    *zeroOffset = zeroOffsetResponse.offset;

    return c_REVLibError_None;
}

c_REVLib_ErrorCode c_Detached_GetZeroCentered(c_Detached_handle handle,
                                              bool* zeroCentered) {
    if (c_SIM_Detached_IsSim(handle->m_simDevice)) {
        uint32_t value;
        c_REVLib_ErrorCode status = c_SIM_Detached_GetParameter(
            handle->m_simDevice, c_Detached_kDutyCycleZeroCentered, &value);
        *zeroCentered = value != 0;
        return status;
    }

    encoder_get_zero_centered_resp_t zeroCenteredResponse;
    const c_REVLib_ErrorCode status = Detached_GetGenericParameter(
        handle,
        {c_Detached_kDutyCycleZeroCentered, ENCODER_GET_ZERO_CENTERED_FRAME_ID,
         ENCODER_GET_ZERO_CENTERED_RESP_FRAME_ID,
         ENCODER_GET_ZERO_CENTERED_RESP_LENGTH},
        encoder_get_zero_centered_resp_unpack, &zeroCenteredResponse);

    if (status != c_REVLibError_None) {
        return (status);
    }

    *zeroCentered = zeroCenteredResponse.centered != 0;

    return c_REVLibError_None;
}

c_REVLib_ErrorCode c_Detached_GetAbsoluteAverageDepth(
    c_Detached_handle handle, uint32_t* absoluteAverageDepth) {
    if (c_SIM_Detached_IsSim(handle->m_simDevice)) {
        return c_SIM_Detached_GetParameter(handle->m_simDevice,
                                           c_Detached_kDutyCycleAverageDepth,
                                           absoluteAverageDepth);
    }

    encoder_get_abs_avg_depth_resp_t absAvgDepthResponse;
    const c_REVLib_ErrorCode status = Detached_GetGenericParameter(
        handle,
        {c_Detached_kDutyCycleAverageDepth, ENCODER_GET_ABS_AVG_DEPTH_FRAME_ID,
         ENCODER_GET_ABS_AVG_DEPTH_RESP_FRAME_ID,
         ENCODER_GET_ABS_AVG_DEPTH_RESP_LENGTH},
        encoder_get_abs_avg_depth_resp_unpack, &absAvgDepthResponse);

    if (status != c_REVLibError_None) {
        return (status);
    }

    *absoluteAverageDepth = absAvgDepthResponse.depth;

    return c_REVLibError_None;
}

c_REVLib_ErrorCode c_Detached_GetStartPulseUs(c_Detached_handle handle,
                                              float* startPulseUs) {
    if (c_SIM_Detached_IsSim(handle->m_simDevice)) {
        uint32_t value;
        c_REVLib_ErrorCode status = c_SIM_Detached_GetParameter(
            handle->m_simDevice, c_Detached_kDutyCycleStartPulseUs, &value);
        std::memcpy(startPulseUs, &value, sizeof(*startPulseUs));
        return status;
    }

    encoder_get_abs_min_high_time_resp_t startPulseResponse;
    const c_REVLib_ErrorCode status = Detached_GetGenericParameter(
        handle,
        {c_Detached_kDutyCycleStartPulseUs,
         ENCODER_GET_ABS_MIN_HIGH_TIME_FRAME_ID,
         ENCODER_GET_ABS_MIN_HIGH_TIME_RESP_FRAME_ID,
         ENCODER_GET_ABS_MIN_HIGH_TIME_RESP_LENGTH},
        encoder_get_abs_min_high_time_resp_unpack, &startPulseResponse);

    if (status != c_REVLibError_None) {
        return (status);
    }

    *startPulseUs = startPulseResponse.time;

    return c_REVLibError_None;
}

c_REVLib_ErrorCode c_Detached_GetEndPulseUs(c_Detached_handle handle,
                                            float* endPulseUs) {
    if (c_SIM_Detached_IsSim(handle->m_simDevice)) {
        uint32_t value;
        c_REVLib_ErrorCode status = c_SIM_Detached_GetParameter(
            handle->m_simDevice, c_Detached_kDutyCycleEndPulseUs, &value);
        std::memcpy(endPulseUs, &value, sizeof(*endPulseUs));
        return status;
    }

    encoder_get_abs_min_low_time_resp_t endPulseResponse;
    const c_REVLib_ErrorCode status = Detached_GetGenericParameter(
        handle,
        {c_Detached_kDutyCycleEndPulseUs, ENCODER_GET_ABS_MIN_LOW_TIME_FRAME_ID,
         ENCODER_GET_ABS_MIN_LOW_TIME_RESP_FRAME_ID,
         ENCODER_GET_ABS_MIN_LOW_TIME_RESP_LENGTH},
        encoder_get_abs_min_low_time_resp_unpack, &endPulseResponse);

    if (status != c_REVLibError_None) {
        return (status);
    }

    *endPulseUs = endPulseResponse.time;

    return c_REVLibError_None;
}

c_REVLib_ErrorCode c_Detached_GetAbsolutePeriodUs(c_Detached_handle handle,
                                                  float* periodUs) {
    if (c_SIM_Detached_IsSim(handle->m_simDevice)) {
        uint32_t value;
        c_REVLib_ErrorCode status = c_SIM_Detached_GetParameter(
            handle->m_simDevice, c_Detached_kDutyCyclePeriodUs, &value);
        std::memcpy(periodUs, &value, sizeof(*periodUs));
        return status;
    }

    encoder_get_abs_period_resp_t periodResponse;
    const c_REVLib_ErrorCode status = Detached_GetGenericParameter(
        handle,
        {c_Detached_kDutyCyclePeriodUs, ENCODER_GET_ABS_PERIOD_FRAME_ID,
         ENCODER_GET_ABS_PERIOD_RESP_FRAME_ID,
         ENCODER_GET_ABS_PERIOD_RESP_LENGTH},
        encoder_get_abs_period_resp_unpack, &periodResponse);

    if (status != c_REVLibError_None) {
        return (status);
    }

    *periodUs = periodResponse.period;

    return c_REVLibError_None;
}

c_REVLib_ErrorCode c_Detached_ClearFaults(c_Detached_handle handle) {
    if (c_SIM_Detached_IsSim(handle->m_simDevice)) {
        if (c_SIM_Detached_GetSimFaultManagerExists(handle->m_simDevice)) {
            c_SIM_Detached_FaultManager_handle simFaultManager =
                c_SIM_Detached_GetOrCreateSimFaultManager(handle->m_simDevice);
            c_SIM_Detached_ClearSimFaults(simFaultManager);
        }
    }

    c_REVLib_ErrorCode status =
        REVLib_WriteCANPacket(handle, zeroLengthDataPacket,
                              ENCODER_CLEAR_FAULTS_FRAME_ID, "Clear Faults");

    return status;
}

// Status Frames

namespace {
c_REVLib_ErrorCode SIM_Detached_GetPeriodicStatus0(
    c_Detached_handle handle, c_Detached_PeriodicStatus0* rawFrame) {
    rawFrame->encoderModel = c_Detached_kMAXSplineEncoder;
    rawFrame->timestamp = 0u;

    return c_REVLibError_None;
}
}  // namespace

c_REVLib_ErrorCode c_Detached_GetPeriodicStatus0(
    c_Detached_handle handle, c_Detached_PeriodicStatus0* rawFrame) {
    if (c_SIM_Detached_IsSim(handle->m_simDevice)) {
        return SIM_Detached_GetPeriodicStatus0(handle, rawFrame);
    }

    std::array<uint8_t, kMaxPacketLength> packedData{};

    c_REVLib_ErrorCode revlibError = REVLib_ReadCANPacketTimeout(
        handle, packedData, rawFrame->timestamp,
        ENCODER_PERIODIC_FRAME_0_FRAME_ID, handle->m_status0Period_ms,
        ENCODER_PERIODIC_FRAME_0_LENGTH, "Period Status 0");

    if (revlibError != c_REVLibError_None) {
        return revlibError;
    }

    encoder_periodic_frame_0_t frameData;
    encoder_periodic_frame_0_unpack(&frameData, packedData.data(),
                                    ENCODER_PERIODIC_FRAME_0_LENGTH);

    rawFrame->encoderModel =
        static_cast<c_Detached_EncoderModel>(frameData.device_model);

    return revlibError;
}

namespace {
c_REVLib_ErrorCode SIM_Detached_GetPeriodicStatus1(
    c_Detached_handle handle, c_Detached_PeriodicStatus1* rawFrame) {
    if (c_SIM_Detached_GetSimFaultManagerExists(handle->m_simDevice)) {
        c_SIM_Detached_FaultManager_handle simFaultManager =
            c_SIM_Detached_GetOrCreateSimFaultManager(handle->m_simDevice);
        c_SIM_Detached_SetRawframeFromSimFaults(simFaultManager, rawFrame);
    } else {
        // if the fault manager has not been set up, all faults off
        rawFrame->unexpectedFault = 0;
        rawFrame->hasResetFault = 0;
        rawFrame->canTxFault = 0;
        rawFrame->canRxFault = 0;
        rawFrame->eepromFault = 0;

        rawFrame->unexpectedStickyFault = 0;
        rawFrame->hasResetStickyFault = 0;
        rawFrame->canTxStickyFault = 0;
        rawFrame->canRxStickyFault = 0;
        rawFrame->eepromStickyFault = 0;
    }
    return c_REVLibError_None;
}
}  // namespace

c_REVLib_ErrorCode c_Detached_GetPeriodicStatus1(
    c_Detached_handle handle, c_Detached_PeriodicStatus1* rawFrame) {
    if (c_SIM_Detached_IsSim(handle->m_simDevice)) {
        return SIM_Detached_GetPeriodicStatus1(handle, rawFrame);
    }

    std::array<uint8_t, kMaxPacketLength> packedData{};

    c_REVLib_ErrorCode revlibError = REVLib_ReadCANPacketTimeout(
        handle, packedData, rawFrame->timestamp,
        ENCODER_PERIODIC_FRAME_1_FRAME_ID, handle->m_status1Period_ms,
        ENCODER_PERIODIC_FRAME_1_LENGTH, "Period Status 1");

    encoder_periodic_frame_1_t frameData;
    encoder_periodic_frame_1_unpack(&frameData, packedData.data(),
                                    ENCODER_PERIODIC_FRAME_1_LENGTH);

    rawFrame->unexpectedFault = frameData.unexpected;
    rawFrame->hasResetFault = frameData.has_reset;
    rawFrame->canTxFault = frameData.can_tx;
    rawFrame->canRxFault = frameData.can_rx;
    rawFrame->eepromFault = frameData.eeprom;

    rawFrame->unexpectedStickyFault = frameData.sticky_unexpected;
    rawFrame->hasResetStickyFault = frameData.sticky_has_reset;
    rawFrame->canTxStickyFault = frameData.sticky_can_tx;
    rawFrame->canRxStickyFault = frameData.sticky_can_rx;
    rawFrame->eepromStickyFault = frameData.sticky_eeprom;

    return revlibError;
}

namespace {
c_REVLib_ErrorCode SIM_Detached_GetPeriodicStatus2(
    c_Detached_handle handle, c_Detached_PeriodicStatus2* rawFrame) {
    rawFrame->absoluteAngle =
        static_cast<float>(c_SIM_Detached_GetEncoderAngle(handle->m_simDevice));
    rawFrame->rawAbsoluteAngle = static_cast<float>(
        c_SIM_Detached_GetEncoderRawAngle(handle->m_simDevice));
    rawFrame->timestamp = 0u;

    return c_REVLibError_None;
}
}  // namespace

c_REVLib_ErrorCode c_Detached_GetPeriodicStatus2(
    c_Detached_handle handle, c_Detached_PeriodicStatus2* rawFrame) {
    if (c_SIM_Detached_IsSim(handle->m_simDevice)) {
        return SIM_Detached_GetPeriodicStatus2(handle, rawFrame);
    }

    std::array<uint8_t, kMaxPacketLength> packedData{};

    c_REVLib_ErrorCode revlibError = REVLib_ReadCANPacketTimeout(
        handle, packedData, rawFrame->timestamp,
        ENCODER_PERIODIC_FRAME_2_FRAME_ID, handle->m_status2Period_ms,
        ENCODER_PERIODIC_FRAME_2_LENGTH, "Period Status 2");

    encoder_periodic_frame_2_t frameData;
    encoder_periodic_frame_2_unpack(&frameData, packedData.data(),
                                    ENCODER_PERIODIC_FRAME_2_LENGTH);

    rawFrame->absoluteAngle =
        encoder_periodic_frame_2_angle_decode(frameData.angle);
    rawFrame->rawAbsoluteAngle =
        encoder_periodic_frame_2_rawangle_decode(frameData.rawangle);

    return revlibError;
}

namespace {
c_REVLib_ErrorCode SIM_Detached_GetPeriodicStatus3(
    c_Detached_handle handle, c_Detached_PeriodicStatus3* rawFrame) {
    rawFrame->relativePosition = (static_cast<float>(
        c_SIM_Detached_GetEncoderPosition(handle->m_simDevice)));
    rawFrame->encoderTimestamp = 0u;
    rawFrame->timestamp = 0u;

    return c_REVLibError_None;
}
}  // namespace

c_REVLib_ErrorCode c_Detached_GetPeriodicStatus3(
    c_Detached_handle handle, c_Detached_PeriodicStatus3* rawFrame) {
    if (c_SIM_Detached_IsSim(handle->m_simDevice)) {
        return SIM_Detached_GetPeriodicStatus3(handle, rawFrame);
    }

    std::array<uint8_t, kMaxPacketLength> packedData{};

    c_REVLib_ErrorCode revlibError = REVLib_ReadCANPacketTimeout(
        handle, packedData, rawFrame->timestamp,
        ENCODER_PERIODIC_FRAME_3_FRAME_ID, handle->m_status3Period_ms,
        ENCODER_PERIODIC_FRAME_3_LENGTH, "Period Status 3");

    encoder_periodic_frame_3_t frameData;
    encoder_periodic_frame_3_unpack(&frameData, packedData.data(),
                                    ENCODER_PERIODIC_FRAME_3_LENGTH);

    rawFrame->relativePosition =
        encoder_periodic_frame_3_position_decode(frameData.position);
    rawFrame->encoderTimestamp =
        encoder_periodic_frame_3_timestamp_decode(frameData.timestamp);

    return revlibError;
}

namespace {
c_REVLib_ErrorCode SIM_Detached_GetPeriodicStatus4(
    c_Detached_handle handle, c_Detached_PeriodicStatus4* rawFrame) {
    rawFrame->encoderVelocity = static_cast<float>(
        c_SIM_Detached_GetEncoderVelocity(handle->m_simDevice));
    rawFrame->encoderTimestamp = 0u;
    rawFrame->timestamp = 0u;

    return c_REVLibError_None;
}
}  // namespace

c_REVLib_ErrorCode c_Detached_GetPeriodicStatus4(
    c_Detached_handle handle, c_Detached_PeriodicStatus4* rawFrame) {
    if (c_SIM_Detached_IsSim(handle->m_simDevice)) {
        return SIM_Detached_GetPeriodicStatus4(handle, rawFrame);
    }

    std::array<uint8_t, kMaxPacketLength> packedData{};

    c_REVLib_ErrorCode revlibError = REVLib_ReadCANPacketTimeout(
        handle, packedData, rawFrame->timestamp,
        ENCODER_PERIODIC_FRAME_4_FRAME_ID, handle->m_status4Period_ms,
        ENCODER_PERIODIC_FRAME_4_LENGTH, "Period Status 4");

    encoder_periodic_frame_4_t frameData;
    encoder_periodic_frame_4_unpack(&frameData, packedData.data(),
                                    ENCODER_PERIODIC_FRAME_4_LENGTH);

    rawFrame->encoderVelocity =
        encoder_periodic_frame_4_velocity_decode(frameData.velocity);
    rawFrame->encoderTimestamp =
        encoder_periodic_frame_4_timestamp_decode(frameData.timestamp);

    return revlibError;
}

// All of the other Getters don't have any input parameters. The Status Period
// requires the frame_idx, so it requires special handling.
c_REVLib_ErrorCode c_Detached_GetStatusPeriod(
    c_Detached_handle handle, c_Detached_ConfigParameter periodId,
    uint32_t* period) {
    return Detached_GetStatusPeriod(handle, periodId, period);
}

c_REVLib_ErrorCode c_Detached_SetEncoderPosition(c_Detached_handle handle,
                                                 float position) {
    if (c_SIM_Detached_IsSim(handle->m_simDevice)) {
        c_SIM_Detached_SetEncoderPosition(handle->m_simDevice,
                                          static_cast<double>(position));
        return c_REVLibError_None;
    }

    encoder_set_encoder_position_t frameData{.position = position};
    uint8_t packedData[ENCODER_SET_ENCODER_POSITION_LENGTH];

    encoder_set_encoder_position_pack(packedData, &frameData,
                                      ENCODER_SET_ENCODER_POSITION_LENGTH);

    c_REVLib_ErrorCode status = REVLib_WriteCANPacket(
        handle, packedData, ENCODER_SET_ENCODER_POSITION_FRAME_ID,
        "Set Encoder Position");
    return status;
}

// Simulation helpers

void c_SIM_Detached_CreateSimFaultManager(c_Detached_handle handle) {
    if (c_SIM_Detached_IsSim(handle->m_simDevice)) {
        c_SIM_Detached_GetOrCreateSimFaultManager(handle->m_simDevice);
    }
}
