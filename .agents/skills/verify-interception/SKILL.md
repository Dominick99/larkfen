---
name: verify-interception
description: Build, run, and assess the deterministic Larkfen bounding-box interception baseline. Use after changing guidance, perception, camera geometry, target motion, airframe physics, flight control, simulation timing, Docker configuration, or telemetry.
---

# Verify interception

Run the repository verifier from the repository root:

```text
python .agents/skills/verify-interception/scripts/verify.py
```

The script builds the supported Docker image, runs the seeded quadcopter
scenario, validates telemetry, and runs `git diff --check`.

Treat verification as successful only when:

- Docker commands exit successfully.
- Telemetry reports `impact: true`.
- Detection rate is at least `0.95`.
- Impact time and relative impact speed are positive.
- The Git diff contains no whitespace errors.

Report impact status, detection rate, impact time, relative impact speed, and
artifact paths. If behavior fails, inspect the generated JSON before changing
controller constants. Do not commit generated artifacts.
