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

#include "rev/sim/MovingAverageFilterSim.h"

#include <utility>

using namespace rev::spark;

MovingAverageFilterSim::MovingAverageFilterSim(int taps, double sampleRate)
    : m_filter(wpi::math::LinearFilter<double>::MovingAverage(taps)),
      m_sampleRate(sampleRate) {}

double MovingAverageFilterSim::lerp(const std::pair<double, double>& p1,
                                    const std::pair<double, double>& p2,
                                    double x) const {
    if (p2.first == p1.first) {
        return p2.second;
    }
    return p1.second +
           (x - p1.first) * ((p2.second - p1.second) / (p2.first - p1.first));
}

void MovingAverageFilterSim::put(double value, double delta) {
    double newDelta = m_state.first + delta;

    while (newDelta >= m_sampleRate) {
        m_value =
            m_filter.Calculate(lerp(m_state, {newDelta, value}, m_sampleRate));
        newDelta -= m_sampleRate;
    }
    m_state = {newDelta, value};
}

double MovingAverageFilterSim::get() const { return m_value; }
