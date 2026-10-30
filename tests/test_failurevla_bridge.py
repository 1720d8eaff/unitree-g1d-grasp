import unittest
from types import SimpleNamespace
from integrations.failurevla_bridge import FailureVLABridge


class BridgeTest(unittest.TestCase):
    def test_rank_filter_plan_and_budget(self):
        items = [SimpleNamespace(score=3, pose='collision'), SimpleNamespace(score=2, pose='valid')]
        bridge = FailureVLABridge(lambda: 'obs', lambda o: items,
                                  lambda c: c.pose != 'collision', lambda c: True,
                                  lambda c, o: (c.pose, o))
        request = SimpleNamespace(failure_type='grasp', remaining_budget=1, observation=bridge.observe())
        self.assertEqual(bridge.plan_recovery(request), ('valid', 'obs'))
        request.remaining_budget = 0
        self.assertIsNone(bridge.plan_recovery(request))
        with self.assertRaises(RuntimeError):
            bridge.execute_reviewed_plan('plan')
