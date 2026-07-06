#!/usr/bin/env python3
"""Read a servo-style PWM signal from an RC receiver on the Jetson Orin Nano (Super).

Wiring (RC receiver -> Jetson 40-pin header):
    - Signal -> physical pin 19 (BOARD 19; SPI0_MOSI on the Orin Nano)
    - GND    -> any GND pin (e.g. pin 6, 9, 14, 20, 25, ...)
    - The receiver is usually powered separately (5V from an ESC/BEC or pin 2/4).

The 40-pin header keeps BOARD (physical) pin numbering across Jetson boards, so
pin 19 works the same here as on the classic Nano -- Jetson.GPIO handles the
underlying gpiochip/line differences for the Orin.

An RC channel is a ~50 Hz pulse train. The information is in the HIGH pulse
width, not the duty cycle:
    ~1000 us -> stick/switch minimum
    ~1500 us -> neutral / center
    ~2000 us -> stick/switch maximum

NOTE: The Jetson header is 3.3V logic. Most RC receivers output 3.3V-safe
signals, but if yours drives a full 5V pulse, use a level shifter or a simple
resistor divider on the signal line to avoid damaging the pin.

Why readings can "bounce": measuring a single ~1-2 ms pulse from Python on a
non-realtime OS is noisy -- the interpreter can be paused mid-pulse (GIL,
scheduler) and report a badly wrong width. On the Orin Nano each GPIO.input()
read goes through libgpiod and is comparatively slow, which limits edge-timing
resolution, so the outlier rejection + median + EMA below matter even more:
  1. Rejecting physically-impossible pulses (outlier rejection).
  2. Taking the MEDIAN of several pulses per reading (kills one-off spikes).
  3. Smoothing the result with an exponential moving average (EMA).

Run with sudo (GPIO access requires root, unless your user is in the gpio group):
    sudo python3 signals.py
"""

import time

import Jetson.GPIO as GPIO

# Physical pin 19 on the 40-pin header (BOARD numbering).
SIGNAL_PIN = 19

# Bounds of a standard RC pulse, in microseconds.
PULSE_MIN_US = 1000.0
PULSE_MAX_US = 2000.0

# Anything outside this range is treated as a bad measurement (glitch / jitter)
# and thrown away. Wider than the nominal 1000-2000 to allow for trims/endpoints.
PULSE_VALID_MIN_US = 700.0
PULSE_VALID_MAX_US = 2300.0

# If no edge shows up within this window, treat it as "no signal".
# A 50 Hz frame is 20 ms, so 30 ms is a safe timeout.
EDGE_TIMEOUT_S = 0.03

# Pulses to collect per reading; we take the median of these.
SAMPLES_PER_READ = 5

# EMA smoothing factor (0..1). Lower = smoother but laggier.
EMA_ALPHA = 0.3


def _wait_for_level(pin, level, timeout_s):
    """Busy-wait until `pin` reads `level`. Returns the timestamp, or None on timeout.

    Busy polling (rather than GPIO event callbacks) gives us far better timing
    resolution for the short ~1-2 ms pulses we need to measure.
    """
    deadline = time.perf_counter() + timeout_s
    while GPIO.input(pin) != level:
        if time.perf_counter() > deadline:
            return None
    return time.perf_counter()


def read_pulse_us(pin, timeout_s=EDGE_TIMEOUT_S):
    """Measure one HIGH pulse width on `pin`, in microseconds.

    Returns the pulse width in us, or None if no clean/plausible pulse was seen.
    """
    # Make sure we start from a known low state so we catch a full rising edge,
    # rather than latching onto a pulse that is already in progress.
    if _wait_for_level(pin, GPIO.LOW, timeout_s) is None:
        return None

    rising = _wait_for_level(pin, GPIO.HIGH, timeout_s)
    if rising is None:
        return None

    falling = _wait_for_level(pin, GPIO.LOW, timeout_s)
    if falling is None:
        return None

    pulse_us = (falling - rising) * 1_000_000.0

    # Reject physically-impossible widths caused by the interpreter being
    # paused mid-measurement. These are the main source of "bouncing".
    if not (PULSE_VALID_MIN_US <= pulse_us <= PULSE_VALID_MAX_US):
        return None

    return pulse_us


def read_pulse_median(pin, samples=SAMPLES_PER_READ, timeout_s=EDGE_TIMEOUT_S):
    """Collect several pulses and return their median width in us.

    The median is robust to the occasional wildly-wrong sample, which a mean
    is not. Returns None if not enough valid pulses were captured.
    """
    values = []
    # Allow a few extra attempts so a couple of rejected glitches don't starve us.
    for _ in range(samples * 2):
        pulse_us = read_pulse_us(pin, timeout_s)
        if pulse_us is not None:
            values.append(pulse_us)
        if len(values) >= samples:
            break

    if not values:
        return None

    values.sort()
    mid = len(values) // 2
    if len(values) % 2:
        return values[mid]
    return (values[mid - 1] + values[mid]) / 2.0


def pulse_to_normalized(pulse_us, deadband=0.02):
    """Map a pulse width (us) to -1.0 .. +1.0, centered at 1500 us.

    A small deadband snaps values near center to exactly 0.0.
    """
    if pulse_us is None:
        return None
    center = (PULSE_MIN_US + PULSE_MAX_US) / 2.0
    half_range = (PULSE_MAX_US - PULSE_MIN_US) / 2.0
    value = (pulse_us - center) / half_range
    value = max(-1.0, min(1.0, value))
    if abs(value) < deadband:
        value = 0.0
    return value


def main():
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(SIGNAL_PIN, GPIO.IN)

    print(f"Reading RC PWM on physical pin {SIGNAL_PIN}. Press Ctrl+C to stop.")
    smoothed_us = None
    try:
        while True:
            pulse_us = read_pulse_median(SIGNAL_PIN)
            if pulse_us is None:
                print("no signal            ", end="\r", flush=True)
                smoothed_us = None  # reset so we don't glide from a stale value
            else:
                # Exponential moving average for a steady, low-jitter output.
                if smoothed_us is None:
                    smoothed_us = pulse_us
                else:
                    smoothed_us += EMA_ALPHA * (pulse_us - smoothed_us)

                norm = pulse_to_normalized(smoothed_us)
                print(f"pulse: {smoothed_us:7.1f} us   value: {norm:+.2f}    ",
                      end="\r", flush=True)
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        GPIO.cleanup()


if __name__ == "__main__":
    main()
