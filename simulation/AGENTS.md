# Simulation guidance

This subtree owns PyBullet-specific world, rendering, perception, vehicle, and
evaluation code.

- Do not give target world coordinates to `pursuit.py`.
- Keep camera observations timestamp-compatible and image-space based.
- Run the `verify-interception` skill after behavioral changes.
- Preserve the default quadcopter scenario unless a requested change explicitly
  replaces it.
- Do not commit files generated under `artifacts/`.
