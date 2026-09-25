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

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;

import java.util.Arrays;

public class TestUtils {
  public static void validateFlattenedSubString(StringBuilder flattened, String sub) {
    int pos = flattened.indexOf(sub);
    assertNotEquals(pos, -1);

    if (pos != -1) {
      flattened = flattened.replace(pos, pos + sub.length(), "");
    }
  }

  public static void validateFlattenedStrings(String fs1, String fs2) {
    String[] results1 = fs1.split("\\n+");
    Arrays.sort(results1);

    String[] results2 = fs2.split("\\n+");
    Arrays.sort(results2);

    assertArrayEquals(results1, results2);
  }

  public static void ValidateFlattenedStringsExcludingIds(String fs1, String fs2) {
    String[] results1 = fs1.split("\\n+");
    Arrays.sort(results1);
    for (int i = 0; i < results1.length; ++i) {
      results1[i] = results1[i].substring(results1[i].indexOf(","));
    }

    String[] results2 = fs2.split("\\n+");
    Arrays.sort(results2);
    for (int i = 0; i < results2.length; ++i) {
      results2[i] = results2[i].substring(results2[i].indexOf(","));
    }

    assertArrayEquals(results1, results2);
  }
}
