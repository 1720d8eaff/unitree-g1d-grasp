import unittest
from vla.action_buffer import ActionBuffer, Chunk


class BufferTests(unittest.TestCase):
    def test_replacement_and_replanning(self):
        b = ActionBuffer(max_age=.5, max_state_error=.1)
        c = Chunk(1, 10., (0., 1.), ((1., 2.), (3., 4.)))
        self.assertEqual(b.replace(c, (0.05, 1.), 10.2), (True, "accepted"))
        self.assertFalse(b.needs_replan())
        self.assertEqual(b.pop(), (1., 2.))
        self.assertTrue(b.needs_replan())
        self.assertEqual(b.replace(c, (0., 1.), 10.3)[1], "out_of_order")
        self.assertEqual(b.pop(), (3., 4.))
        self.assertIsNone(b.pop())

    def test_rejected_chunks_do_not_replace_current_actions(self):
        b = ActionBuffer(max_age=1, max_state_error=.1)
        self.assertTrue(b.replace(Chunk(0, 0., (0.,), ((1.,),)), (0.,), .1)[0])
        self.assertEqual(b.replace(Chunk(1, 0., (0.,), ((2.,),)), (0.,), 2.)[1], "stale")
        self.assertEqual(b.replace(Chunk(2, 2., (0.,), ((2.,),)), (1.,), 2.)[1], "state_mismatch")
        self.assertEqual(b.pop(), (1.,))

    def test_action_expires_when_consumed_with_clock(self):
        b = ActionBuffer(max_age=.5, max_state_error=.1)
        self.assertTrue(b.replace(Chunk(0, 1., (0.,), ((1.,), (2.,))), (0.,), 1.1)[0])
        self.assertEqual(b.pop(now=1.2), (1.,))
        self.assertIsNone(b.pop(now=1.6))
        self.assertTrue(b.needs_replan())
        with self.assertRaises(ValueError):
            b.pop(now=float('nan'))


try:
    import torch
    from vla.failure_adapter import FailureAdapter, sampling_weights, training_loss
except ImportError:
    torch = None


@unittest.skipIf(torch is None, "PyTorch not installed")
class AdapterTests(unittest.TestCase):
    def test_forward_and_gradient(self):
        model = FailureAdapter(4, 2, 8)
        out = model(torch.randn(3, 5, 4), torch.randn(3, 6, 2))
        self.assertEqual(out['actions'].shape, (3, 6, 2))
        loss = training_loss(out, torch.randn(3, 6, 2), torch.tensor([0, 1, 2]), torch.tensor([0., .5, 1.]))
        loss.backward()
        self.assertTrue(torch.isfinite(loss))
        self.assertIsNotNone(model.encoder.weight_ih_l0.grad)
        self.assertEqual(sampling_weights([0, 1]).tolist(), [1., 3.])
