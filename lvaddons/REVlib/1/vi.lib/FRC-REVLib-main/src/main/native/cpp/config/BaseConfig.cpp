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

#include "rev/config/BaseConfig.h"

#include <string>

#include "rev/CANCommonParameters.h"

using namespace rev;

namespace {  // unnamed

// std::variant overloaded visitor pattern:
//   see https://en.cppreference.com/w/cpp/utility/variant/visit

// helper type for the visitor
template <typename... Ts>
struct overloaded : Ts... {
    using Ts::operator()...;
};
// explicit deduction guide (not needed as of C++20)
template <typename... Ts>
overloaded(Ts...) -> overloaded<Ts...>;

}  // namespace

std::string BaseConfig::Flatten() {
    std::string flattenedString;

    constexpr uint32_t kMaxStringLength = 16;
    char flattenedCharArray[kMaxStringLength];

    for (auto& param : m_Parameters) {
        std::visit(
            overloaded{[&flattenedCharArray, key = param.first](int32_t arg) {
                           c_REVLib_FlattenParameterInt32(
                               key, arg, flattenedCharArray, kMaxStringLength);
                       },
                       [&flattenedCharArray, key = param.first](uint32_t arg) {
                           c_REVLib_FlattenParameterUint32(
                               key, arg, flattenedCharArray, kMaxStringLength);
                       },
                       [&flattenedCharArray, key = param.first](float arg) {
                           c_REVLib_FlattenParameterFloat(
                               key, arg, flattenedCharArray, kMaxStringLength);
                       },
                       [&flattenedCharArray, key = param.first](bool arg) {
                           c_REVLib_FlattenParameterBool(
                               key, arg, flattenedCharArray, kMaxStringLength);
                       }},
            param.second);

        flattenedString += flattenedCharArray;
    }

    return flattenedString;
}

std::optional<BaseConfig::ParameterType_t> BaseConfig::GetParameter(
    uint8_t parameterId) {
    auto search = m_Parameters.find(parameterId);
    if (search == m_Parameters.end()) {
        return std::nullopt;
    } else {
        return std::optional<BaseConfig::ParameterType_t>(search->second);
    }
}

std::optional<BaseConfig::ParameterType_t> BaseConfig::GetParameter(
    BaseConfig& fromConfig, uint8_t parameterId) {
    return fromConfig.GetParameter(parameterId);
}

void BaseConfig::RemoveParameter(uint8_t parameterId) {
    m_Parameters.erase(parameterId);
}

void BaseConfig::RemoveParameter(BaseConfig& fromConfig, uint8_t parameterId) {
    fromConfig.RemoveParameter(parameterId);
}

void BaseConfig::Apply(BaseConfig& config) {
    for (auto [key, value] : config.m_Parameters) {
        PutParameter(key, value);
    }
}
