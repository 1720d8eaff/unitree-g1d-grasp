# Safety contract

- Every new session starts by verifying the live ARM/DISARM state.
- ARM and DISARM are performed only by the onsite operator on the board.
- Plan-only is the default and the only motion-planning mode exposed by
  `g1dctl` version 1.
- The active table geometry must be operator-confirmed, marked verified, added
  to PlanningScene, and read back before Plan-only starts. The current profile
  is the handoff manual's compensated `torso_link` table confirmed on
  2026-09-07.
- A column-height change invalidates every saved trajectory and table profile.
- Action success is not endpoint acceptance.  The left-arm nominal endpoint
  gate remains 0.015 rad together with the V2 low-speed stability condition
  and physical TCP confirmation.
- No hand CLOSE, visual adjustment, lift, or place is permitted before endpoint
  V2 acceptance.
- Do not relax collision, joint-limit, start-state freshness, target-motion,
  speed, duration, tracking-error, or endpoint checks to make a test pass.
