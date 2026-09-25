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

#include "rev/REVUtils.h"

#include <rev/driver/FRCCanSpec.h>
#include <rev/driver/REVLibDriver.h>

#include <chrono>
#include <cstdlib>
#include <cstring>
#include <thread>

#define REV_MANUFACTURER_CANID 5

extern "C" {

static int32_t REV_BaseMotorCtrlID =
    ((static_cast<int32_t>(rev::FRCDeviceType::MotorController) & 0x1F) << 24) |
    ((REV_MANUFACTURER_CANID & 0xFF) << 16);

int32_t CreateCANID(int32_t deviceID, int32_t apiID) {
    return REV_BaseMotorCtrlID | (apiID & 0x3FF) << 6 | (deviceID & 0x3F);
}

uint64_t ArrToUint64(const uint8_t* data8) {
    uint64_t tmpData = 0;
    for (int i = 0; i < 8; i++) {
        tmpData |= static_cast<uint64_t>(data8[i]) << (i * 8);
    }
    return tmpData;
}

void Uint64ToArr(uint64_t data, uint8_t* arr) {
    for (int i = 0; i < 8; i++) {
        arr[i] = (data >> (i * 8)) & 0xFF;
    }
}

}  // extern "C"
