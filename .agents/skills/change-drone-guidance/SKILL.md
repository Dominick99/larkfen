---
name: change-drone-guidance
description: Modify or review Larkfen's bounding-box steering and terminal interception policy. Use for changes to pursuit.py, box-center steering, box-area ranging, target-loss behavior, approach speed, descent timing, or guidance outputs.
---

# Change drone guidance

Read `references/guidance-boundary.md` before editing guidance.

Follow this workflow:

1. Inspect `simulation/drone_sim/pursuit.py`, its caller in `runner.py`, and the
   bounding-box contract in `perception.py`.
2. State which observable signals drive each new behavior.
3. Keep target ground truth outside the guidance path.
4. Change one control behavior at a time when tuning.
5. Run `.agents/skills/verify-interception/scripts/verify.py`.
6. Compare impact, detection rate, impact time, and relative impact speed with
   the prior baseline.

Prefer named configuration values over unexplained literals. Preserve a safe
reacquisition behavior when the box is temporarily absent. Avoid optimizing
only one video frame or seed unless the task explicitly requests that scope.
