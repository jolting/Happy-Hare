# Happy Hare test harness - preload on a park_at_toolhead unit.
#
# park_at_toolhead is for a unit with one bowden per gate running all the way to a combiner at
# the toolhead (the Flashforge AD5X IFS). A gate is "parked" with its strand in its own bowden,
# toolhead_unload_safety_margin short of the toolhead sensor, and its next load only homes to
# that sensor. Preload leaves a gate in the same place:
#
#   - toolhead sensor clear: feed until it triggers, then back off the margin
#   - another gate loaded: the shared sensor already reads filament, so make the same trip as
#     a fixed move from the calibrated bowden length
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
FILAMENT_POS_UNLOADED = 0
FILAMENT_POS_LOADED = 10
GATE_EMPTY = 0
GATE_AVAILABLE = 1

PROFILE = profiles.get('tradrack').derive(
    'tradrack_park_at_toolhead',
    syms=dict({'MMU_HAS_SENSOR_TOOLHEAD': True, 'PIN_TOOLHEAD_SENSOR': 'PG13',
               'MMU_HAS_SENSOR_ENTRY': True},
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

    def test_preload_while_another_gate_is_loaded_parks_by_fixed_move(self):
        self.load_gate(0)
        self.hh.place_filament(2, position=TIP_AT_GATE)
        self.fil.history.clear()

        self.hh.run_gcode('MMU_PRELOAD GATE=2')

        self.assertEqual(self.hh.errors, [])
        legs = [d for g, d, r in self.fil.history if g == 2]
        self.assertEqual(len(legs), 1, self.fil.history)
        self.assertAlmostEqual(legs[0], self.to_sensor - self.margin, places=1)
        self.assertLess(self.fil.tip[2], self.fil.layout['toolhead'])
        self.assertTrue(self.mmu.gate_parked_at_toolhead[2])
        self.assertEqual(self.mmu.gate_status[2], GATE_AVAILABLE)
        # Gate 0 is untouched and still loaded
        self.assertEqual(self.mmu.gate_selected, 0)
        self.assertEqual(self.mmu.filament_pos, FILAMENT_POS_LOADED)
        self.assertFalse(any(g == 0 for g, _d, _r in self.fil.history), self.fil.history)

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
