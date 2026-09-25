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

package com.revrobotics.util;

import com.revrobotics.REVLibError;
import com.revrobotics.StatusFrame;
import java.util.function.Function;

public final class Signal<T> {
  private final T value;
  private final REVLibError error;
  private final long timestamp;

  private Signal(T value, REVLibError error, long timestamp) {
    this.value = value;
    this.error = error;
    this.timestamp = timestamp;
  }

  /**
   * Get the last known value. This value may or may not be recent. Check isValid() to know if this
   * value was recently collected. Prefer get(T defaultValue) if you have a reasonable default.
   *
   * @return The last known value.
   */
  public T get() {
    return value;
  }

  /**
   * Get the last known value if it is recent, or the provided default otherwise.
   *
   * @param defaultValue the default to use if the value is not valid
   * @return The last known value, or the default.
   */
  public T get(T defaultValue) {
    if (isValid()) {
      return value;
    }
    return defaultValue;
  }

  public long getTimestamp() {
    return timestamp;
  }

  /**
   * Returns whether the value is recent and was collected without error. You can still get the last
   * known value through 'get' if this method returns false, but the value may be outdated.
   *
   * @return Whether the value returned by 'get' is valid and recent.
   */
  public boolean isValid() {
    return error == REVLibError.kOk;
  }

  public REVLibError getError() {
    return error;
  }

  /**
   * Create a new Signal with a value returned by mapper, and the same error and timestamp as this
   * object.
   *
   * @param mapper a function that converts this object's signal to another signal
   * @return A new signal object with the same error and timestamp, but a new value
   * @param <R> the new object's signal type
   */
  public <R> Signal<R> map(Function<T, R> mapper) {
    var newValue = mapper.apply(value);

    return Signal.ofError(newValue, error, timestamp);
  }

  /**
   * Create a Signal with no error. isValid will return true
   *
   * @param value known good value
   * @param timestamp timestamp the value was collected
   * @return A known good signal with the given value
   */
  public static <T> Signal<T> of(T value, long timestamp) {
    return new Signal<>(value, REVLibError.kOk, timestamp);
  }

  /**
   * Create a signal that has an error. isValid will return false if the error was not kOk.
   *
   * @param value The latest known good value
   * @param error error associated with the signal
   * @param timestamp timestamp the value was collected
   * @return A signal that may be good or have an error
   */
  public static <T> Signal<T> ofError(T value, REVLibError error, long timestamp) {
    return new Signal<>(value, error, timestamp);
  }

  public static <T extends StatusFrame> Signal<T> create(T statusFrame) {
    return Signal.ofError(statusFrame, statusFrame.getRevlibError(), statusFrame.timestamp());
  }
}
