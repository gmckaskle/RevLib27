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

#pragma once

#include <stdint.h>

#include <concepts>
#include <utility>

#include "rev/REVLibError.h"

namespace rev {
namespace util {

template <std::copy_constructible T>
class Signal {
public:
    Signal(T value, REVLibError error, uint64_t timestamp)
        : m_value(std::move(value)), m_error(error), m_timestamp(timestamp) {}

    template <std::invocable<T> F>
    auto Map(F&& f) const -> Signal<std::invoke_result_t<F, T>> {
        using U = std::invoke_result_t<F, T>;
        return Signal<U>(f(m_value), m_error, m_timestamp);
    }

    T Get() const { return m_value; }

    T Get(T defaultValue) const { return IsValid() ? m_value : defaultValue; }

    uint64_t GetTimestamp() const { return m_timestamp; }
    REVLibError GetError() const { return m_error; }
    bool IsValid() const { return m_error == REVLibError::kOk; }

private:
    T m_value{};
    REVLibError m_error{REVLibError::kError};
    uint64_t m_timestamp{0};
};

}  // namespace util
}  // namespace rev
