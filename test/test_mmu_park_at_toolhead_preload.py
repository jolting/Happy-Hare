# Happy Hare test harness - preload on a park_at_toolhead unit.
#
# park_at_toolhead is for a unit with one bowden per gate running all the way to a combiner at
# the toolhead (the Flashforge AD5X IFS). A gate is "parked" with its strand in its own bowden,
# toolhead_unload_safety_margin short of the toolhead sensor, and its next load only homes to
# that sensor. Preload leaves a gate in the same place:
#
#   - toolhead sensor clear: feed until it triggers, then back off the margin
#   - another gate loaded: the shared sensor already reads filament, so feed until the encoder
#     shows the strand has stopped against the occupied combiner, then back off the margin
#
# Never by feed distance: the calibrated bowden length only bounds the feed. The combiner is
# modelled as a stop COMBINER_STOP short of the toolhead sensor that no strand passes while
# another gate is loaded.
#
# Other gates can be preloaded while one is loaded, which is what lets a user insert a spool
# into any free slot and have it fed in automatically.
#
#   ./venv/bin/python -m unittest test.test_mmu_park_at_toolhead_preload
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import logging
import unittest

from test.hh import profiles, session

logging.getLogger().setLevel(logging.CRITICAL)

TIP_AT_GATE = -40.0
COMBINER_STOP = 30.0
FILAMENT_POS_UNLOADED = 0
FILAMENT_POS_LOADED = 10
GATE_EMPTY = 0
GATE_AVAILABLE = 1

PROFILE = profiles.get('tradrack').derive(
    'tradrack_park_at_toolhead',
    syms=dict({'MMU_HAS_SENSOR_TOOLHEAD': True, 'PIN_TOOLHEAD_SENSOR': 'PG13',
               'MMU_HAS_SENSOR_ENTRY': True,
               'MMU_HAS_ENCODER': True, 'PIN_ENCODER': 'unit0:PA6'},
              **profiles._per_gate_pins(range(4), 100, 'ENTRY_SENSOR', prefix='^unit0:')))


class ParkAtToolheadPreloadTestCase(unittest.TestCase):

    def setUp(self):
        self.hh = session(PROFILE)
        self.hh.build()
        self.hh.boot(calibrate=True)
        self.addCleanup(self.hh.close)
        self.mmu = self.hh.mmu
        self.unit = self.mmu.mmu_unit(0)
        # The IFS has no gate endstop: preload is defined by the toolhead sensor alone
        self.hh.run_gcode('MMU_TEST_CONFIG UNIT=0 gate_preload_parking_distance=0')
        self.hh.run_gcode('MMU_TEST_CONFIG UNIT=0 gate_preload_endstop=none')
        self.hh.run_gcode('MMU_TEST_CONFIG UNIT=0 gate_parking_distance=0')
        self.hh.run_gcode('MMU_TEST_CONFIG UNIT=0 gate_homing_endstop=none')
        self.hh.run_gcode('MMU_TEST_CONFIG UNIT=0 park_at_toolhead=1')
        self.assertEqual(self.hh.errors, [], 'setup was not clean')
        self.fil = self.hh.filament()
        th = self.unit.toolhead_wrapper.p
        self.margin = self.unit.p.toolhead_unload_safety_margin
        self.to_sensor = (self.unit.calibrator.get_bowden_length()
                          + th.toolhead_extruder_to_nozzle - th.toolhead_sensor_to_nozzle)
        self.park = self.fil.layout['toolhead'] - self.margin
        self.combiner = self.fil.layout['toolhead'] - COMBINER_STOP
        self.stall_feeds = 0
        advance = self.fil.advance

        def blocked_by_combiner(gate, delta, reason=''):
            # A strand stops at the combiner while another gate's filament runs through it
            others_loaded = any(self.fil.tip[g] >= self.fil.layout['toolhead']
                                for g in range(len(self.fil.tip)) if g != gate)
            if others_loaded and delta > 0 and self.fil.tip[gate] + delta > self.combiner:
                delta = max(0., self.combiner - self.fil.tip[gate])
                self.stall_feeds += 1
            return advance(gate, delta, reason)

        self.fil.advance = blocked_by_combiner

    def load_gate(self, gate):
        # This profile calibrated its bowden length from mmu_shared_exit; a full load starts there
        self.hh.place_filament(gate, position=self.fil.layout['mmu_shared_exit'])
        self.hh.heat_extruder(220)
        self.hh.run_gcode('MMU_SELECT GATE=%d' % gate)
        self.hh.run_gcode('MMU_LOAD')
        self.assertEqual(self.mmu.filament_pos, FILAMENT_POS_LOADED, self.hh.errors)

    def test_clear_toolhead_preload_stops_at_the_toolhead_sensor_and_parks(self):
        self.hh.place_filament(1, position=TIP_AT_GATE)
        self.fil.history.clear()

        self.hh.run_gcode('MMU_PRELOAD GATE=1')

        self.assertEqual(self.hh.errors, [])
        legs = [(d, r) for g, d, r in self.fil.history if g == 1]
        self.assertTrue(any('toolhead' in r for _d, r in legs), legs)
        self.assertAlmostEqual(self.fil.tip[1], self.park, places=1)
        self.assertFalse(self.hh.sensor('toolhead').present)
        self.assertEqual(self.mmu.gate_status[1], GATE_AVAILABLE)
        self.assertTrue(self.mmu.gate_parked_at_toolhead[1])
        self.assertEqual(self.mmu.filament_pos, FILAMENT_POS_UNLOADED)

    def test_preload_while_another_gate_is_loaded_stops_at_the_combiner(self):
        self.load_gate(0)
        self.hh.place_filament(2, position=TIP_AT_GATE)
        self.fil.history.clear()

        self.hh.run_gcode('MMU_PRELOAD GATE=2')

        self.assertEqual(self.hh.errors, [])
        self.assertGreater(self.stall_feeds, 0)
        self.assertAlmostEqual(self.fil.tip[2], self.combiner - self.margin, places=1)
        self.assertTrue(self.mmu.gate_parked_at_toolhead[2])
        self.assertEqual(self.mmu.gate_status[2], GATE_AVAILABLE)
        # Gate 0 is untouched and still loaded
        self.assertEqual(self.mmu.gate_selected, 0)
        self.assertEqual(self.mmu.filament_pos, FILAMENT_POS_LOADED)
        self.assertFalse(any(g == 0 for g, _d, _r in self.fil.history), self.fil.history)

    def test_stop_does_not_depend_on_the_bowden_length(self):
        """Pushed in further than usual, the strand still stops at the combiner, not short of it."""
        self.load_gate(0)
        self.hh.place_filament(2, position=TIP_AT_GATE + 100.)

        self.hh.run_gcode('MMU_PRELOAD GATE=2')

        self.assertEqual(self.hh.errors, [])
        self.assertAlmostEqual(self.fil.tip[2], self.combiner - self.margin, places=1)

    def test_no_stall_within_the_feed_limit_is_an_error_not_a_park(self):
        self.load_gate(0)
        self.combiner = 1e9     # nothing in the way
        self.hh.place_filament(2, position=TIP_AT_GATE)

        self.hh.run_gcode('MMU_PRELOAD GATE=2')

        self.assertTrue(any('did not stop at the toolhead' in e for e in self.hh.errors), self.hh.errors)
        self.assertFalse(self.mmu.gate_parked_at_toolhead[2])

    def test_inserting_a_spool_while_loaded_preloads_it(self):
        self.load_gate(0)
        self.fil.history.clear()

        self.hh.place_filament(3, position=TIP_AT_GATE, quiet=False)
        self.hh.settle()

        self.assertEqual(self.hh.errors, [])
        self.assertTrue(self.mmu.gate_parked_at_toolhead[3])
        self.assertEqual(self.mmu.gate_status[3], GATE_AVAILABLE)
        self.assertEqual(self.mmu.filament_pos, FILAMENT_POS_LOADED)
        self.assertEqual(self.mmu.gate_selected, 0)

    def test_preloaded_gate_loads_by_homing_only_to_the_toolhead_sensor(self):
        self.hh.place_filament(1, position=TIP_AT_GATE)
        self.hh.run_gcode('MMU_PRELOAD GATE=1')
        self.hh.heat_extruder(220)
        self.fil.history.clear()

        self.hh.run_gcode('MMU_SELECT GATE=1')
        self.hh.run_gcode('MMU_LOAD')

        self.assertEqual(self.hh.errors, [])
        self.assertEqual(self.mmu.filament_pos, FILAMENT_POS_LOADED)
        moved = sum(d for g, d, _r in self.fil.history if g == 1 and d > 0)
        self.assertLess(moved, self.margin + self.unit.p.toolhead_homing_max + 100, self.fil.history)

    def test_pulling_a_parked_strand_out_clears_its_parked_flag(self):
        self.hh.place_filament(1, position=TIP_AT_GATE)
        self.hh.run_gcode('MMU_PRELOAD GATE=1')
        self.assertTrue(self.mmu.gate_parked_at_toolhead[1])

        self.hh.place_filament(1, position=-100000.0, quiet=False)
        self.hh.settle()

        self.assertEqual(self.mmu.gate_status[1], GATE_EMPTY)
        self.assertFalse(self.mmu.gate_parked_at_toolhead[1])

    def test_empty_gate_is_refused(self):
        self.hh.place_filament(1, position=-100000.0)
        self.hh.run_gcode('MMU_PRELOAD GATE=1')

        self.assertEqual(self.mmu.gate_status[1], GATE_EMPTY)
        self.assertFalse(self.mmu.gate_parked_at_toolhead[1])
        self.assertFalse(any(g == 1 for g, _d, _r in self.fil.history), self.fil.history)


if __name__ == '__main__':
    unittest.main()
