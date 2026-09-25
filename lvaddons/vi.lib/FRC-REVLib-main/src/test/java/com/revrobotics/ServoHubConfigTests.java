/*
 * Copyright (c) 2024 REV Robotics
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

package com.revrobotics;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.revrobotics.servohub.ServoChannel;
import com.revrobotics.servohub.config.ServoChannelConfig;
import com.revrobotics.servohub.config.ServoHubConfig;
import com.revrobotics.servohub.config.ServoHubParameter;
import org.junit.jupiter.api.Test;

public class ServoHubConfigTests {
  private class ChanParams {
    public ServoChannel.ChannelId id;
    public int min;
    public int center;
    public int max;
    public int disable;

    ChanParams(ServoChannel.ChannelId id, int min, int center, int max, int disable) {
      this.id = id;
      this.min = min;
      this.center = center;
      this.max = max;
      this.disable = disable;
    }
  }

  private final ChanParams[] ChanTestParams = {
    new ChanParams(
        ServoChannel.ChannelId.kChannelId0,
        ServoHubParameter.kChannel0_MinPulseWidth.value,
        ServoHubParameter.kChannel0_CenterPulseWidth.value,
        ServoHubParameter.kChannel0_MaxPulseWidth.value,
        ServoHubParameter.kChannel0_DisableBehavior.value),
    new ChanParams(
        ServoChannel.ChannelId.kChannelId1,
        ServoHubParameter.kChannel1_MinPulseWidth.value,
        ServoHubParameter.kChannel1_CenterPulseWidth.value,
        ServoHubParameter.kChannel1_MaxPulseWidth.value,
        ServoHubParameter.kChannel1_DisableBehavior.value),
    new ChanParams(
        ServoChannel.ChannelId.kChannelId2,
        ServoHubParameter.kChannel2_MinPulseWidth.value,
        ServoHubParameter.kChannel2_CenterPulseWidth.value,
        ServoHubParameter.kChannel2_MaxPulseWidth.value,
        ServoHubParameter.kChannel2_DisableBehavior.value),
    new ChanParams(
        ServoChannel.ChannelId.kChannelId3,
        ServoHubParameter.kChannel3_MinPulseWidth.value,
        ServoHubParameter.kChannel3_CenterPulseWidth.value,
        ServoHubParameter.kChannel3_MaxPulseWidth.value,
        ServoHubParameter.kChannel3_DisableBehavior.value),
    new ChanParams(
        ServoChannel.ChannelId.kChannelId4,
        ServoHubParameter.kChannel4_MinPulseWidth.value,
        ServoHubParameter.kChannel4_CenterPulseWidth.value,
        ServoHubParameter.kChannel4_MaxPulseWidth.value,
        ServoHubParameter.kChannel4_DisableBehavior.value),
    new ChanParams(
        ServoChannel.ChannelId.kChannelId5,
        ServoHubParameter.kChannel5_MinPulseWidth.value,
        ServoHubParameter.kChannel5_CenterPulseWidth.value,
        ServoHubParameter.kChannel5_MaxPulseWidth.value,
        ServoHubParameter.kChannel5_DisableBehavior.value)
  };

  private class SubServoChannelConfig extends ServoChannelConfig {
    SubServoChannelConfig(ServoChannel.ChannelId channelId) {
      super(channelId);
    }

    Object getParam(int parameterId) {
      return getParameter(parameterId);
    }
  }

  @Test
  void ServoChannel_SetsPulseRange() {
    for (ChanParams chan : ChanTestParams) {
      SubServoChannelConfig config = new SubServoChannelConfig(chan.id);

      int idx = chan.id.value;
      config.pulseRange(525 + idx, 1500 + idx, 2475 + idx);
      Object minPulse = config.getParam(chan.min);
      Object centerPulse = config.getParam(chan.center);
      Object maxPulse = config.getParam(chan.max);
      assertTrue(minPulse != null, "idx: " + Integer.toString(idx) + ": \n" + config.flatten());
      assertTrue(centerPulse != null);
      assertTrue(maxPulse != null);
      assertEquals((int) minPulse, 525 + idx);
      assertEquals((int) centerPulse, 1500 + idx);
      assertEquals((int) maxPulse, 2475 + idx);
    }
  }

  @Test
  void ServoChannel_SetsPulseRangeClass() {
    for (ChanParams chan : ChanTestParams) {
      SubServoChannelConfig config = new SubServoChannelConfig(chan.id);

      int idx = chan.id.value;
      ServoChannelConfig.PulseRange pulseRange =
          new ServoChannelConfig.PulseRange(525 + idx, 1500 + idx, 2475 + idx);
      config.pulseRange(pulseRange);

      Object minPulse = config.getParam(chan.min);
      Object centerPulse = config.getParam(chan.center);
      Object maxPulse = config.getParam(chan.max);
      assertTrue(minPulse != null, "idx: " + Integer.toString(idx) + ": \n" + config.flatten());
      assertTrue(centerPulse != null);
      assertTrue(maxPulse != null);
      assertEquals((int) minPulse, 525 + idx);
      assertEquals((int) centerPulse, 1500 + idx);
      assertEquals((int) maxPulse, 2475 + idx);
    }
  }

  @Test
  void ServoChannel_SetsDisableBehavior() {
    for (ChanParams chan : ChanTestParams) {
      SubServoChannelConfig config = new SubServoChannelConfig(chan.id);

      config.disableBehavior(ServoChannelConfig.BehaviorWhenDisabled.kSupplyPower);
      Object disableBehavior = config.getParam(chan.disable);
      assertTrue(
          disableBehavior != null,
          "idx: " + Integer.toString(chan.id.value) + ": \n" + config.flatten());
      assertEquals((boolean) disableBehavior, true);
    }

    for (ChanParams chan : ChanTestParams) {
      SubServoChannelConfig config = new SubServoChannelConfig(chan.id);

      config.disableBehavior(ServoChannelConfig.BehaviorWhenDisabled.kDoNotSupplyPower);
      Object disableBehavior = config.getParam(chan.disable);
      assertTrue(
          disableBehavior != null,
          "idx: " + Integer.toString(chan.id.value) + ": \n" + config.flatten());
      assertEquals((boolean) disableBehavior, false);
    }
  }

  @Test
  void ServoChannel_Applies() {
    for (ChanParams srcChan : ChanTestParams) {
      SubServoChannelConfig srcConfig = new SubServoChannelConfig(srcChan.id);

      int srcIdx = srcChan.id.value;
      srcConfig.pulseRange(545 + srcIdx, 1540 + srcIdx, 2445 + srcIdx);

      for (ChanParams destChan : ChanTestParams) {
        if (srcChan.id == destChan.id) continue;

        SubServoChannelConfig destConfig = new SubServoChannelConfig(destChan.id);
        destConfig.apply(srcConfig);

        Object minPulse = destConfig.getParam(destChan.min);
        Object centerPulse = destConfig.getParam(destChan.center);
        Object maxPulse = destConfig.getParam(destChan.max);

        assertTrue(
            minPulse != null,
            Integer.toString(srcChan.id.value)
                + ": "
                + Integer.toString(srcChan.min)
                + "."
                + Integer.toString(srcChan.center)
                + "."
                + Integer.toString(srcChan.max)
                + " -> "
                + Integer.toString(destChan.id.value)
                + ": "
                + Integer.toString(destChan.min)
                + "."
                + Integer.toString(destChan.center)
                + "."
                + Integer.toString(destChan.max)
                + "\n"
                + srcConfig.flatten()
                + "\n\n"
                + destConfig.flatten());

        assertTrue(centerPulse != null);
        assertTrue(maxPulse != null);
        assertEquals((int) minPulse, 545 + srcIdx);
        assertEquals((int) centerPulse, 1540 + srcIdx);
        assertEquals((int) maxPulse, 2445 + srcIdx);
      }
    }
  }

  @Test
  void ServoHub_Flattens() {
    ServoHubConfig config = new ServoHubConfig();

    config
        .channel0
        .pulseRange(505, 1234, 1856)
        .disableBehavior(ServoChannelConfig.BehaviorWhenDisabled.kSupplyPower);
    config
        .channel1
        .pulseRange(605, 1235, 1956)
        .disableBehavior(ServoChannelConfig.BehaviorWhenDisabled.kDoNotSupplyPower);
    config
        .channel2
        .pulseRange(705, 1236, 1766)
        .disableBehavior(ServoChannelConfig.BehaviorWhenDisabled.kSupplyPower);
    config
        .channel3
        .pulseRange(805, 1237, 1776)
        .disableBehavior(ServoChannelConfig.BehaviorWhenDisabled.kDoNotSupplyPower);
    config
        .channel4
        .pulseRange(555, 1238, 1757)
        .disableBehavior(ServoChannelConfig.BehaviorWhenDisabled.kSupplyPower);
    config
        .channel5
        .pulseRange(565, 1239, 1758)
        .disableBehavior(ServoChannelConfig.BehaviorWhenDisabled.kDoNotSupplyPower);

    StringBuilder flattened = new StringBuilder(config.flatten());

    TestUtils.validateFlattenedSubString(flattened, "0,1F9\n");
    TestUtils.validateFlattenedSubString(flattened, "1,4D2\n");
    TestUtils.validateFlattenedSubString(flattened, "2,740\n");
    TestUtils.validateFlattenedSubString(flattened, "3,25D\n");
    TestUtils.validateFlattenedSubString(flattened, "4,4D3\n");
    TestUtils.validateFlattenedSubString(flattened, "5,7A4\n");
    TestUtils.validateFlattenedSubString(flattened, "6,2C1\n");
    TestUtils.validateFlattenedSubString(flattened, "7,4D4\n");
    TestUtils.validateFlattenedSubString(flattened, "8,6E6\n");
    TestUtils.validateFlattenedSubString(flattened, "9,325\n");
    TestUtils.validateFlattenedSubString(flattened, "10,4D5\n");
    TestUtils.validateFlattenedSubString(flattened, "11,6F0\n");
    TestUtils.validateFlattenedSubString(flattened, "12,22B\n");
    TestUtils.validateFlattenedSubString(flattened, "13,4D6\n");
    TestUtils.validateFlattenedSubString(flattened, "14,6DD\n");
    TestUtils.validateFlattenedSubString(flattened, "15,235\n");
    TestUtils.validateFlattenedSubString(flattened, "16,4D7\n");
    TestUtils.validateFlattenedSubString(flattened, "17,6DE\n");
    TestUtils.validateFlattenedSubString(flattened, "18,1\n");
    TestUtils.validateFlattenedSubString(flattened, "19,0\n");
    TestUtils.validateFlattenedSubString(flattened, "20,1\n");
    TestUtils.validateFlattenedSubString(flattened, "21,0\n");
    TestUtils.validateFlattenedSubString(flattened, "22,1\n");
    TestUtils.validateFlattenedSubString(flattened, "23,0\n");
    assertEquals(flattened.length(), 0, flattened.toString());
  }

  @Test
  void ServoHub_Applies() {
    ServoHubConfig first = new ServoHubConfig();
    ServoHubConfig second = new ServoHubConfig();

    first
        .channel0
        .pulseRange(505, 1234, 1856)
        .disableBehavior(ServoChannelConfig.BehaviorWhenDisabled.kSupplyPower);
    first
        .channel1
        .pulseRange(605, 1235, 1956)
        .disableBehavior(ServoChannelConfig.BehaviorWhenDisabled.kDoNotSupplyPower);
    first
        .channel2
        .pulseRange(705, 1236, 1766)
        .disableBehavior(ServoChannelConfig.BehaviorWhenDisabled.kSupplyPower);
    first
        .channel3
        .pulseRange(805, 1237, 1776)
        .disableBehavior(ServoChannelConfig.BehaviorWhenDisabled.kDoNotSupplyPower);

    first
        .channel4
        .pulseRange(555, 1238, 1757)
        .disableBehavior(ServoChannelConfig.BehaviorWhenDisabled.kSupplyPower);
    first.apply(ServoChannel.ChannelId.kChannelId5, first.channel4);

    TestUtils.ValidateFlattenedStringsExcludingIds(
        first.channel4.flatten(), first.channel5.flatten());

    second.apply(first);

    TestUtils.validateFlattenedStrings(first.flatten(), second.flatten());
  }
}
