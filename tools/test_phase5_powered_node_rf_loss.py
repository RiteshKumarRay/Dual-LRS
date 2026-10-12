#!/usr/bin/env python3
"""Host tests for the phase-5 observation gates and bench safety text."""

import unittest
from pathlib import Path

from tools.bench_phase5_powered_node_rf_loss import FCTruthAuditor


class TestPhase5ObservationGates(unittest.TestCase):
    def test_baseline_health_cannot_count_as_post_restore_recovery(self):
        auditor = FCTruthAuditor()
        auditor.rc_healthy_history.append((10.0, True))
        auditor.failsafe_cleared_events.append((10.0, "baseline healthy"))

        self.assertFalse(auditor.failsafe_observed_since(20.0))
        self.assertFalse(auditor.recovery_observed_since(20.0))

    def test_recovery_requires_an_observation_after_restore(self):
        auditor = FCTruthAuditor()
        auditor.rc_healthy_history.extend([(10.0, True), (30.0, False)])
        auditor.failsafe_events.append((30.0, "radio failsafe"))

        self.assertTrue(auditor.failsafe_observed_since(25.0))
        self.assertFalse(auditor.recovery_observed_since(40.0))

        auditor.rc_healthy_history.append((50.0, True))
        auditor.failsafe_cleared_events.append((50.0, "radio failsafe cleared"))
        self.assertTrue(auditor.recovery_observed_since(40.0))

    def test_phase5_never_recommends_powered_antenna_disconnect(self):
        source = Path(__file__).with_name("bench_phase5_powered_node_rf_loss.py").read_text()
        self.assertIn("NEVER disconnect an antenna", source)
        self.assertNotIn("or disconnect antenna", source)
        self.assertNotIn("restore the antenna", source)


if __name__ == "__main__":
    unittest.main()
