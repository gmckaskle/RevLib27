/*
 * Copyright (c) 2018-2025 REV Robotics
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

#include "rev/SparkFrameManager.h"

#include <rev/driver/REVLibDriver.h>

#include <atomic>
#include <queue>
#include <thread>
#include <unordered_map>
#include <vector>

#include "rev/CANDriverPrivate.h"
#include "rev/CANSparkFrames.h"
#include "rev/REVUtils.h"

// Replacement for wpi functions
#include "SafeThread.h"
#include <mutex>

namespace {
class FrameDaemon : public rev::SafeThread {
public:
    FrameDaemon() {}

    void QueueFrame(c_BaseCAN_handle handle, uint8_t frameId) {
        std::lock_guard<std::mutex> lock(m_statusFramesMutex);
        EnqueueQ.push({handle, frameId});
    }

    void DequeueFrame(c_BaseCAN_handle handle, uint8_t frameId) {
        std::lock_guard<std::mutex> lock(m_statusFramesMutex);
        DequeueQ.push({handle, frameId});
    }

private:
    std::queue<std::pair<c_BaseCAN_handle, uint16_t>> EnqueueQ{};
    std::queue<std::pair<c_BaseCAN_handle, uint16_t>> DequeueQ{};
    std::unordered_map<c_BaseCAN_handle, uint16_t> m_statusFrames{};
    std::mutex m_statusFramesMutex{};

    void Main() override {
        // 100ms thread rate, units in microseconds
        constexpr uint64_t kThreadRate = 100000;
        uint32_t timer = getREVLibDriver()->allocatePreciseTimer();

        while (m_active) {
            uint64_t startTime = getREVLibDriver()->getPreciseTime();

            // Queue received handle and Id
            {
                std::lock_guard<std::mutex> lock(m_statusFramesMutex);

                for (; !EnqueueQ.empty(); EnqueueQ.pop()) {
                    auto& [handle, frameId] = EnqueueQ.front();

                    if (m_statusFrames.find(handle) == m_statusFrames.end()) {
                        m_statusFrames[handle] = 0x0;
                    }

                    m_statusFrames[handle] |= 0x1 << frameId;
                }
            }

            for (auto it = m_statusFrames.begin();
                 it != m_statusFrames.end();) {
                c_BaseCAN_handle handle = it->first;

                // Read Status 0 and break if it is not found. Status 0 should
                // always be enabled, and if it is not found, it is possible the
                // device is completely offline.
                uint8_t data[8]{0};
                const uint32_t timeout_ms{
                    500};  // 500ms is the maximum period for status 0
                uint64_t timestamp;

                c_REVLib_ErrorCode status = REVLib_ReadCANPacketTimeout(
                    handle, data, timestamp, SPARK_STATUS_0_FRAME_ID,
                    timeout_ms, SPARK_STATUS_0_LENGTH,
                    "Frame Mgr: Period Status 0");

                if (status != c_REVLibError_None) {
                    it++;
                    continue;
                }

                // Take a snapshot of the mask and clear it from the map. This
                // is done to allow additional status frames to be added to the
                // mask that will be processed in the next iteration without
                // blocking the main thread.
                uint16_t mask;
                {
                    std::lock_guard<std::mutex> lock(m_statusFramesMutex);
                    if (it->second == 0) {
                        it = m_statusFrames.erase(it);
                        continue;
                    }
                    mask = it->second;
                    it->second = 0;
                }

                // Send enable frame command
                uint8_t packedData[SPARK_SET_STATUSES_ENABLED_LENGTH] = {0};
                spark_set_statuses_enabled_t frameData;
                frameData.mask = mask;
                frameData.enabled_bitfield = 0xffff;

                spark_set_statuses_enabled_pack(
                    packedData, &frameData, SPARK_SET_STATUSES_ENABLED_LENGTH);

                status = REVLib_WriteAndReadCANPacket(
                    handle, packedData, SPARK_SET_STATUSES_ENABLED_FRAME_ID,
                    data, SPARK_SET_STATUSES_ENABLED_RESPONSE_FRAME_ID,
                    SPARK_SET_STATUSES_ENABLED_RESPONSE_LENGTH,
                    "Frame Mgr: Set statues enabled");

                if (status != c_REVLibError_None) {
                    // TODO: ???
                }

                it++;
            }

            // Dequeue received handle and Id
            {
                std::lock_guard<std::mutex> lock(m_statusFramesMutex);

                for (; !DequeueQ.empty(); DequeueQ.pop()) {
                    auto& [handle, frameId] = DequeueQ.front();

                    if (m_statusFrames.find(handle) != m_statusFrames.end()) {
                        m_statusFrames[handle] &= ~(0x1 << frameId);

                        if (m_statusFrames[handle] == 0x00) {
                            m_statusFrames.erase(handle);
                        }
                    }
                }
            }

            uint64_t currTime = getREVLibDriver()->getPreciseTime();
            getREVLibDriver()->preciseDelayMicroseconds(
                timer,
                static_cast<uint64_t>((kThreadRate - (currTime - startTime))));
        }

        getREVLibDriver()->closePreciseTimer(timer);
    }
};
}  // namespace

static rev::SafeThreadOwner<FrameDaemon>& FrameThread() {
    static rev::SafeThreadOwner<FrameDaemon> inst;
    return inst;
}

static std::atomic_uint initialized = {0};

void c_Spark_RunStatusFrameManager(void) {
    if (!initialized) {
        FrameThread().Start();
    }

    initialized++;
}

void c_Spark_StopStatusFrameManager(void) {
    if (initialized) {
        initialized--;

        if (initialized <= 0) {
            FrameThread().Join();
            initialized = 0;
        }
    }
}

void c_Spark_QueueStatusFrame(c_BaseCAN_handle handle, uint8_t frameNumber) {
    FrameThread().GetThreadSharedPtr()->QueueFrame(handle, frameNumber);
}

void c_Spark_DequeueStatusFrame(c_BaseCAN_handle handle, uint8_t frameNumber) {
    FrameThread().GetThreadSharedPtr()->DequeueFrame(handle, frameNumber);
}
