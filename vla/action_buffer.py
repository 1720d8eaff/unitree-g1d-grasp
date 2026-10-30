"""State-aligned, thread-safe action chunk replacement.

All timestamps must share the same monotonic clock. Actions are returned to the
caller; this module never sends robot commands.
"""
from dataclasses import dataclass
import math
from threading import Lock


@dataclass(frozen=True)
class Chunk:
    request_id: int
    observed_at: float
    reference_state: tuple[float, ...]
    actions: tuple[tuple[float, ...], ...]


class ActionBuffer:
    def __init__(self, *, max_age: float, max_state_error: float):
        if not math.isfinite(max_age) or not math.isfinite(max_state_error) or max_age < 0 or max_state_error < 0:
            raise ValueError("thresholds must be finite and nonnegative")
        self.max_age, self.max_state_error = max_age, max_state_error
        self._lock = Lock()
        self._last_id = -1
        self._actions = ()

    def replace(self, chunk: Chunk, state, now: float) -> tuple[bool, str]:
        state = tuple(state)
        with self._lock:
            if chunk.request_id <= self._last_id:
                return False, "out_of_order"
            if not math.isfinite(now) or not math.isfinite(chunk.observed_at) or not 0 <= now - chunk.observed_at <= self.max_age:
                return False, "stale"
            if not state or len(state) != len(chunk.reference_state):
                return False, "state_shape"
            if not all(math.isfinite(x) for x in (*state, *chunk.reference_state)):
                return False, "invalid_state"
            if max(abs(a-b) for a, b in zip(state, chunk.reference_state)) > self.max_state_error:
                return False, "state_mismatch"
            if not chunk.actions or not chunk.actions[0] or any(len(a) != len(chunk.actions[0]) or not all(math.isfinite(x) for x in a) for a in chunk.actions):
                return False, "invalid_actions"
            self._last_id = chunk.request_id
            self._actions = tuple(tuple(a) for a in chunk.actions)
            return True, "accepted"

    def pop(self):
        with self._lock:
            if not self._actions:
                return None
            action, self._actions = self._actions[0], self._actions[1:]
            return action

    def needs_replan(self, remaining_threshold: int = 1) -> bool:
        if remaining_threshold < 0:
            raise ValueError("remaining_threshold must be nonnegative")
        with self._lock:
            return len(self._actions) <= remaining_threshold
