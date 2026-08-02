# Companion computer

This component owns target tracking and high-level guidance. It consumes the
shared `BoundingBoxObservation` and `VehicleState` contracts and emits a
`PositionSetpoint`. It deliberately has no PyBullet dependency, so the same
guidance package can later be placed behind a camera detector and MAVLink
adapter on an onboard Linux computer.

The current policy is rule-based. It centers the target horizontally, closes
range, and uses bounding-box growth to command the terminal descent.
