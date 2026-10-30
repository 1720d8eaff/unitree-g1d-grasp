"""Dependency-injected bridge to perception and planning; does not import SDK2."""


class FailureVLABridge:
    def __init__(self, observe, candidates, collision_free, reachable, plan, execute=None):
        self._observe, self._candidates = observe, candidates
        self._collision_free, self._reachable, self._plan = collision_free, reachable, plan
        self._execute = execute

    def observe(self):
        return self._observe()

    def plan_recovery(self, request):
        if request.failure_type != "grasp" or request.remaining_budget <= 0:
            return None
        candidates = sorted(self._candidates(request.observation), key=lambda c: c.score, reverse=True)
        for candidate in candidates:
            if self._collision_free(candidate) and self._reachable(candidate):
                plan = self._plan(candidate, request.observation)
                if plan is not None:
                    return plan
        return None

    def execute_reviewed_plan(self, plan):
        if self._execute is None:
            raise RuntimeError("Execution callback is not configured")
        return self._execute(plan)
