# Flight controller

This component owns low-level vehicle control. It consumes a shared
`PositionSetpoint` plus `VehicleState` and emits a `WrenchCommand` containing
forces and torques. It has no PyBullet dependency.

In this lightweight simulation the PyBullet adapter applies that wrench to the
airframe. A later PX4 or ArduPilot integration would replace this controller
with SITL and translate the same companion setpoint over MAVLink. On physical
hardware, the autopilot firmware—not this Python implementation—would run the
stabilization loop.
