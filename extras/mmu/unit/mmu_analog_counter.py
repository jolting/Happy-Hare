# Happy Hare MMU Software
#
# Goal: Encoder pulse counter from per-gate analog wheel sensors
#
# Some MMUs (the Flashforge IFS) have no encoder pin but one analog sensor
# per gate: a Hall sensor reading a magnet wheel that the filament turns
# (the IFS "stall" sensor). While filament moves, the reading runs through
# its whole range once per wheel turn; it stays put while filament stands
# still. This counts a pulse each time the selected gate's reading crosses
# the high threshold going up or the low threshold going down (hysteresis
# between them), so about two pulses per wheel turn, and reports the count
# like Klipper's MCU_counter, so MmuEncoder works unchanged on top of it.
#
# Config, in [mmu_encoder]:
#   analog_pins: one ADC pin per gate of the unit, in gate order
#   analog_high, analog_low: thresholds (0-1 of ADC full scale)
#   analog_report_time: seconds between samples (default 0.02)
#   analog_batch: samples per report from the MCU (default 8)
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#

import logging

from ..mmu_sensor_utils import MmuAdcHelper


class MmuAnalogCounter:

    def __init__(self, config, mmu_unit):
        self.printer = config.get_printer()
        self.mmu_unit = mmu_unit
        self._pins = config.getlist('analog_pins')
        self.high = config.getfloat('analog_high', 0.72, minval=0., maxval=1.)
        self.low = config.getfloat('analog_low', 0.58, minval=0., below=self.high)
        report_time = config.getfloat('analog_report_time', 0.02, minval=0.005)
        batch = config.getint('analog_batch', 8, minval=1, maxval=24)
        sample_time = 0.001
        sample_count = 4

        self._callback = None
        self.values_raw = [0.] * len(self._pins)
        self.active_gate = 0        # Local gate whose wheel is counted
        self._state = None          # 1 above high, 0 below low, None not yet known
        self.count = 0
        self.count_time = 0.        # Print time of the last pulse

        ppins = self.printer.lookup_object('pins')
        self.mcu_adcs = []
        for idx, pin in enumerate(self._pins):
            adc = ppins.setup_pin('adc', pin)
            cb = lambda *args, g=idx: self._adc_callback(g, *args)
            try:
                adc.setup_adc_sample(report_time, sample_time, sample_count,
                                     batch_num=batch)
                adc.setup_adc_callback(cb)
            except TypeError:
                # Klipper without batched ADC reports: one sample per report
                MmuAdcHelper.setup_adc_compat(adc, report_time, sample_time,
                                              sample_count, cb)
            self.mcu_adcs.append(adc)

        self.printer.register_event_handler("mmu:gate_selected", self._handle_gate_selected)
        logging.info("MMU: Created analog encoder counter on %s" % ", ".join(self._pins))

    # Same interface as pulse_counter.MCU_counter
    def setup_callback(self, cb):
        self._callback = cb

    def _handle_gate_selected(self, gate, prev_gate):
        if gate is None or gate < 0 or not self.mmu_unit.manages_gate(gate):
            return
        lgate = self.mmu_unit.local_gate(gate)
        if 0 <= lgate < len(self._pins) and lgate != self.active_gate:
            self.active_gate = lgate
            self._state = None  # The new wheel's position counts only from here

    def _adc_callback(self, gate_idx, *args):
        if len(args) == 1:
            samples = args[0]   # Batched: [(read_time, value), ...]
        else:
            samples = [args]    # Old Klipper: (read_time, value)
        if not samples:
            return
        for read_time, value in samples:
            self.values_raw[gate_idx] = value
            if gate_idx != self.active_gate:
                continue
            if self._state is None:
                if value >= self.high:
                    self._state = 1
                elif value <= self.low:
                    self._state = 0
                if self._state is not None and not self.count_time:
                    self.count_time = read_time
            elif self._state == 0 and value >= self.high:
                self._state = 1
                self.count += 1
                self.count_time = read_time
            elif self._state == 1 and value <= self.low:
                self._state = 0
                self.count += 1
                self.count_time = read_time
        if gate_idx == self.active_gate and self._callback is not None:
            print_time = samples[-1][0]
            self._callback(print_time, self.count, self.count_time or print_time)

    def get_status(self, eventtime):
        return {
            'active_gate': self.active_gate,
            'values_raw': list(self.values_raw),
            'count': self.count,
        }
