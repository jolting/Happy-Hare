# Happy Hare MMU Software
#
# Copyright (C) 2026  Hunter Laux <hunterlaux@gmail.com>
#
# Goal: Encoder pulse counter from one analog input
#
# A counter backend for MmuEncoder, in place of Klipper's pulse_counter.MCU_counter, for a sensor
# whose reading swings as the filament moves: for example a linear Hall sensor on a magnet wheel,
# or an optical sensor read without a comparator. While the filament moves the reading runs
# through its range once per wheel turn (or vane); it stays put while the filament stands still.
#
# One physical input, one counter. It counts a pulse each time the reading crosses `analog_high`
# going up or `analog_low` going down (hysteresis between them, so noise inside the band counts
# nothing), i.e. two pulses per turn, and reports the running count and the time of the last
# pulse the way MCU_counter does. The hysteresis state is kept from the first sample on, whatever
# the MMU is doing, so no crossing is ever lost or invented by a change elsewhere.
#
# SAMPLING LIMITS. The ADC is sampled on the host side of the link: one sample every
# `analog_report_time` (default 20ms), delivered `analog_batch` samples at a time on Klipper with
# batched ADC reports. So:
#   - a pulse is only seen if the reading spends at least one sample past each threshold, which
#     bounds the filament speed to about resolution / (2 * analog_report_time), e.g. 125mm/s for
#     5mm per pulse at 20ms (max_speed() and the encoder's status report the figure)
#   - counts arrive up to report_latency() after the movement that made them. MmuEncoder waits
#     that long for samples before it trusts a reading, but anything that needs the count while a
#     move is still running (encoder homing, the encoder endstop) cannot be supported
#
# INTERPOLATION (analog_interpolate: 1, off by default). Between the pulses the reading says
# where in the wheel's cycle the filament is. The counter learns the cycle's extremes (the mean
# peak and trough of each half cycle, seeded by analog_min/analog_max) and maps each reading to a
# phase with acos, i.e. as a sine. One analog input has no second (quadrature) channel, so a
# reading fits a rising and a falling point of the cycle: the one nearest the last phase in the
# direction of travel is taken, so the filament is assumed to keep going through a peak or a
# trough (it only reverses between moves, while standing). The count is then the distance
# travelled, in pulses and fractions of one, either way like a pulse counter. The phase follows
# the reading through a backlash of +-4 * analog_noise: standing noise adds nothing, and near a
# peak or trough, where the reading hardly changes, the phase waits until the reading has moved
# clear. On the Flashforge AD5X IFS wheel
# (5.3mm per pulse) that resolves about 0.3mm over most of the cycle. The speed limit above is
# unchanged.
#
# [mmu_encoder] options when analog:
#   analog_pin:         the ADC pin (replaces encoder_pin)
#   analog_high:        upper threshold, fraction of ADC full scale (default 0.72)
#   analog_low:         lower threshold, below analog_high (default 0.58)
#   analog_report_time: seconds between samples (default 0.02)
#   analog_batch:       samples per report from the MCU, on Klipper with batched ADC reports
#                       (default 8; keeps the traffic low on a slow MCU link)
#   encoder_resolution: mm per pulse, as for any encoder (MMU_CALIBRATE_ENCODER sets it)
#   analog_interpolate: 1 to count fractions of a pulse (default 0)
#   analog_noise:       standing noise of the reading, fraction of full scale (default 0.002)
#   analog_min, analog_max: the reading's trough and peak to start from before they are learned
#                       (default: learned from the first full swing)
#
# (\_/)
# ( *,*)
# (")_(") Happy Hare Ready
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
import logging
import math

# Happy Hare imports
from ..mmu_sensor_utils import MmuAdcHelper


class MmuAnalogCounter:

    SAMPLE_TIME = 0.001     # Between the MCU's conversions within one sample
    SAMPLE_COUNT = 4        # Conversions averaged per sample

    def __init__(self, config, pin):
        self.printer = config.get_printer()
        self.pin = pin
        self.high = config.getfloat('analog_high', 0.72, minval=0., maxval=1.)
        self.low = config.getfloat('analog_low', 0.58, minval=0., below=self.high)
        self.report_time = config.getfloat('analog_report_time', 0.02, minval=0.005)
        self.batch = config.getint('analog_batch', 8, minval=1, maxval=24)
        self.interpolate = bool(config.getint('analog_interpolate', 0, minval=0, maxval=1))
        self.noise = config.getfloat('analog_noise', 0.002, above=0., maxval=0.1)
        self.peak = config.getfloat('analog_max', None, minval=0., maxval=1.)
        self.trough = config.getfloat('analog_min', None, minval=0., maxval=1.)
        if self.peak is not None and self.trough is not None and self.trough >= self.peak:
            raise config.error("[%s] analog_min must be below analog_max" % config.get_name())

        self._callback = None
        self.value = None           # Latest reading, fraction of full scale
        self._state = None          # 1 above high, 0 below low, None until the first reading past one
        self.count = 0              # Pulses (threshold crossings)
        self.count_time = 0.        # Print time of the last pulse (or counted fraction)
        self.position = 0.          # Interpolated count: distance in pulses and fractions
        self._half_max = -1.        # Extremes of the current half cycle
        self._half_min = 2.
        self._theta = None          # Phase, in cycles, unwrapped
        self._direction = 0         # Of travel through the phase, +1/-1, 0 not yet known
        self._phase_value = None    # The reading the phase was last taken from
        self.last_sample_time = 0.  # Print time of the latest sample received

        ppins = self.printer.lookup_object('pins')
        self.mcu_adc = ppins.setup_pin('adc', pin)
        self._batched = True
        try:
            self.mcu_adc.setup_adc_sample(self.report_time, self.SAMPLE_TIME, self.SAMPLE_COUNT,
                                          batch_num=self.batch)
            self.mcu_adc.setup_adc_callback(self._adc_callback)
        except TypeError:
            # Klipper without batched ADC reports: one sample per report
            self._batched = False
            MmuAdcHelper.setup_adc_compat(self.mcu_adc, self.report_time, self.SAMPLE_TIME,
                                          self.SAMPLE_COUNT, self._adc_callback)
        logging.info("MMU: Created analog encoder counter on %s" % pin)

    # Same interface as pulse_counter.MCU_counter
    def setup_callback(self, cb):
        self._callback = cb

    def report_latency(self):
        """Longest time between a movement and its count reaching the host."""
        return self.report_time * (self.batch if self._batched else 1) + self.report_time

    def max_speed(self, resolution):
        """Fastest filament speed (mm/s) at which every pulse of `resolution` mm is still seen."""
        return resolution / (2. * self.report_time)

    PEAK_ALPHA = 0.3            # Weight of each half cycle's extreme in the learned peak/trough

    def interpolating(self):
        """Counting fractions of a pulse: enabled and the cycle's extremes are known."""
        return (self.interpolate and self.peak is not None and self.trough is not None
                and self.peak - self.trough > 8 * self.noise)

    def _learn(self, extreme, learned):
        return extreme if learned is None else learned + (extreme - learned) * self.PEAK_ALPHA

    def _interpolate(self, value, read_time):
        # The phase follows the reading through a backlash of +-4 * analog_noise: it moves only by
        # as much as the reading leaves that band, so noise (an outlier, then its return) moves it
        # not at all, and nothing is smoothed away at speed. A reversal loses the band's width
        band = 4. * self.noise
        if self._phase_value is None:
            self._phase_value = value
        else:
            excess = value - self._phase_value
            if abs(excess) <= band:
                return      # Noise, or too close to a peak or trough to tell
            self._phase_value = value - math.copysign(band, excess)
        span = self.peak - self.trough
        u = min(max((self._phase_value - self.trough) / span, 0.), 1.)
        a = math.acos(1. - 2. * u) / (2. * math.pi)    # 0 at the trough .. 0.5 at the peak
        if self._theta is None:
            self._theta = a
            return
        base = math.floor(self._theta)
        candidates = [base + k + c for k in (-1, 0, 1, 2) for c in (a, 1. - a)]
        nearest = min(candidates, key=lambda c: abs(c - self._theta))
        ahead = [c for c in candidates if (c - self._theta) * self._direction >= 0] if self._direction else []
        onward = min(ahead, key=lambda c: abs(c - self._theta)) if ahead else nearest
        # Keep the direction of travel through a peak or trough; reverse only rather than jump
        theta = onward if abs(onward - self._theta) <= 0.25 else nearest
        if theta != self._theta:
            self._direction = 1 if theta > self._theta else -1
        self.position += 2. * abs(theta - self._theta)     # two pulses per cycle
        self._theta = theta
        self.count_time = read_time

    def _adc_callback(self, *args):
        if len(args) == 1:
            samples = args[0]   # Batched: [(read_time, value), ...]
        else:
            samples = [args]    # Older Klipper: (read_time, value)
        if not samples:
            return
        for read_time, value in samples:
            self.value = value
            self.last_sample_time = read_time
            self._half_max = max(self._half_max, value)
            self._half_min = min(self._half_min, value)
            pulse = False
            if self._state is None:
                if value >= self.high:
                    self._state = 1
                elif value <= self.low:
                    self._state = 0
            elif self._state == 0 and value >= self.high:
                self._state = 1
                pulse = True
                self.trough = self._learn(self._half_min, self.trough)
                self._half_min = 2.
            elif self._state == 1 and value <= self.low:
                self._state = 0
                pulse = True
                self.peak = self._learn(self._half_max, self.peak)
                self._half_max = -1.
            if pulse:
                self.count += 1
                if not self.interpolating() or self._theta is None:
                    # Until fractions are counted (and from where they start) a pulse is a pulse
                    self.position += 1.
                    self.count_time = read_time
            if self.interpolating():
                self._interpolate(value, read_time)
        if self._callback is not None:
            print_time = samples[-1][0]
            reported = self.position if self.interpolate else self.count
            self._callback(print_time, reported, self.count_time or print_time)

    def get_status(self, eventtime):
        status = {
            'value': self.value,
            'count': self.count,
        }
        if self.interpolate:
            status.update(position=round(self.position, 3), interpolating=self.interpolating(),
                          peak=self.peak, trough=self.trough)
        return status
