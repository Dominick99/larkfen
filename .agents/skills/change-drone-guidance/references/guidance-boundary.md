# Guidance boundary

The current rule-based guidance path is:

```text
PyBullet segmentation -> BoundingBox -> BoundingBoxPursuitController
    -> world-space waypoint -> airframe control -> PyBullet forces/torques
```

Permitted guidance inputs:

- Bounding-box coordinates, image dimensions, center error, and area.
- A short history of prior bounding boxes.
- The drone's estimated pose, velocity, and orientation.
- Configuration, timestamps, and target-loss duration.

Forbidden guidance inputs:

- Target world position, orientation, or velocity.
- PyBullet body IDs, contact data, or segmentation pixels.
- Evaluation-only distance and collision metrics.

PyBullet ground truth remains valid inside perception and evaluation. When the
companion-computer boundary is introduced, move guidance without changing its
observation contract; keep simulator-specific conversion in the simulation
adapter.
