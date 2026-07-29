from __future__ import annotations

import argparse
import json
import math
from dataclasses import asdict, dataclass
from pathlib import Path

import imageio.v2 as imageio
import numpy as np
import pybullet as p
import pybullet_data


@dataclass(frozen=True)
class DroneConfig:
    mass: float = 1.0
    arm_length: float = 0.28
    max_force: float = 22.0
    max_torque: float = 5.0
    linear_drag: float = 0.18
    angular_drag: float = 0.12
    position_gain: float = 2.8
    velocity_gain: float = 2.2
    attitude_gain: float = 7.0
    angular_gain: float = 2.4


def _create_drone(config: DroneConfig) -> int:
    body_half = [0.13, 0.09, 0.045]
    collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=body_half)
    visuals = [
        p.createVisualShape(
            p.GEOM_BOX, halfExtents=body_half, rgbaColor=[0.12, 0.17, 0.22, 1]
        )
    ]
    positions = [[0, 0, 0]]
    orientations = [[0, 0, 0, 1]]

    arm_half = [config.arm_length, 0.018, 0.015]
    visuals.append(
        p.createVisualShape(
            p.GEOM_BOX, halfExtents=arm_half, rgbaColor=[0.3, 0.34, 0.38, 1]
        )
    )
    positions.append([0, 0, 0])
    orientations.append([0, 0, 0, 1])
    visuals.append(
        p.createVisualShape(
            p.GEOM_BOX, halfExtents=arm_half, rgbaColor=[0.3, 0.34, 0.38, 1]
        )
    )
    positions.append([0, 0, 0])
    orientations.append(p.getQuaternionFromEuler([0, 0, math.pi / 2]))

    for x, y, color in [
        (config.arm_length, 0, [0.9, 0.2, 0.12, 1]),
        (-config.arm_length, 0, [0.1, 0.45, 0.9, 1]),
        (0, config.arm_length, [0.1, 0.45, 0.9, 1]),
        (0, -config.arm_length, [0.1, 0.45, 0.9, 1]),
    ]:
        visuals.append(
            p.createVisualShape(
                p.GEOM_CYLINDER,
                radius=0.095,
                length=0.012,
                rgbaColor=color,
            )
        )
        positions.append([x, y, 0.025])
        orientations.append([0, 0, 0, 1])

    drone = p.createMultiBody(
        baseMass=config.mass,
        baseCollisionShapeIndex=collision,
        baseVisualShapeIndex=-1,
        basePosition=[0, 0, 0.7],
        linkMasses=[0] * len(visuals),
        linkCollisionShapeIndices=[-1] * len(visuals),
        linkVisualShapeIndices=visuals,
        linkPositions=positions,
        linkOrientations=orientations,
        linkInertialFramePositions=[[0, 0, 0]] * len(visuals),
        linkInertialFrameOrientations=[[0, 0, 0, 1]] * len(visuals),
        linkParentIndices=[0] * len(visuals),
        linkJointTypes=[p.JOINT_FIXED] * len(visuals),
        linkJointAxis=[[0, 0, 0]] * len(visuals),
    )
    p.changeDynamics(
        drone,
        -1,
        linearDamping=0,
        angularDamping=0,
        lateralFriction=0.8,
        restitution=0.1,
    )
    return drone


def _create_scene() -> int:
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    plane = p.loadURDF("plane.urdf")
    p.changeVisualShape(plane, -1, rgbaColor=[0.23, 0.31, 0.22, 1])

    for position, size, color in [
        ([2.2, 1.2, 0.55], [0.35, 0.35, 0.55], [0.55, 0.3, 0.16, 1]),
        ([-1.7, 1.8, 0.8], [0.25, 0.25, 0.8], [0.22, 0.42, 0.55, 1]),
        ([0.2, -2.0, 0.4], [0.6, 0.25, 0.4], [0.5, 0.47, 0.2, 1]),
    ]:
        shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=size)
        visual = p.createVisualShape(p.GEOM_BOX, halfExtents=size, rgbaColor=color)
        p.createMultiBody(0, shape, visual, position)

    target_visual = p.createVisualShape(
        p.GEOM_SPHERE, radius=0.22, rgbaColor=[0.95, 0.12, 0.2, 1]
    )
    return p.createMultiBody(0, -1, target_visual, [2.8, -1.5, 1.6])


def _control(drone: int, target: np.ndarray, config: DroneConfig) -> None:
    position, quaternion = p.getBasePositionAndOrientation(drone)
    velocity, angular_velocity = p.getBaseVelocity(drone)
    position = np.asarray(position)
    velocity = np.asarray(velocity)
    angular_velocity = np.asarray(angular_velocity)

    desired_accel = (
        config.position_gain * (target - position)
        - config.velocity_gain * velocity
        + np.array([0.0, 0.0, 9.81])
    )
    desired_force = config.mass * desired_accel
    force_norm = float(np.linalg.norm(desired_force))
    force = desired_force * min(1.0, config.max_force / max(force_norm, 1e-6))

    yaw = math.atan2(target[1] - position[1], target[0] - position[0])
    desired_z = force / max(np.linalg.norm(force), 1e-6)
    desired_x_hint = np.array([math.cos(yaw), math.sin(yaw), 0.0])
    desired_y = np.cross(desired_z, desired_x_hint)
    desired_y /= max(np.linalg.norm(desired_y), 1e-6)
    desired_x = np.cross(desired_y, desired_z)
    desired_rotation = np.column_stack((desired_x, desired_y, desired_z))

    rotation = np.asarray(p.getMatrixFromQuaternion(quaternion)).reshape(3, 3)
    error_matrix = 0.5 * (
        desired_rotation.T @ rotation - rotation.T @ desired_rotation
    )
    attitude_error = np.array(
        [error_matrix[2, 1], error_matrix[0, 2], error_matrix[1, 0]]
    )
    torque_body = (
        -config.attitude_gain * attitude_error
        - config.angular_gain * angular_velocity
    )
    torque_body = np.clip(torque_body, -config.max_torque, config.max_torque)

    p.applyExternalForce(drone, -1, force.tolist(), position.tolist(), p.WORLD_FRAME)
    p.applyExternalTorque(drone, -1, torque_body.tolist(), p.LINK_FRAME)
    p.applyExternalForce(
        drone,
        -1,
        (-config.linear_drag * velocity).tolist(),
        position.tolist(),
        p.WORLD_FRAME,
    )
    p.applyExternalTorque(
        drone,
        -1,
        (-config.angular_drag * angular_velocity).tolist(),
        p.WORLD_FRAME,
    )


def _render(drone: int, width: int = 640, height: int = 368) -> np.ndarray:
    position, _ = p.getBasePositionAndOrientation(drone)
    position = np.asarray(position)
    camera = position + np.array([-5.2, -5.2, 3.2])
    view = p.computeViewMatrix(camera, position + [0, 0, 0.25], [0, 0, 1])
    projection = p.computeProjectionMatrixFOV(58, width / height, 0.1, 100)
    _, _, rgba, _, _ = p.getCameraImage(
        width,
        height,
        view,
        projection,
        shadow=1,
        renderer=p.ER_TINY_RENDERER,
    )
    return np.asarray(rgba, dtype=np.uint8).reshape(height, width, 4)[:, :, :3]


def run(duration: float, seed: int, output: Path, config: DroneConfig) -> dict:
    np.random.seed(seed)
    client = p.connect(p.DIRECT)
    if client < 0:
        raise RuntimeError("Could not start PyBullet")

    physics_hz = 240
    video_fps = 30
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(1 / physics_hz)
    p.setPhysicsEngineParameter(numSolverIterations=80, deterministicOverlappingPairs=1)
    _create_scene()
    drone = _create_drone(config)
    waypoints = np.array(
        [[0, 0, 1.2], [2.0, 0, 1.8], [2.2, 2.0, 2.2], [0, 2.2, 1.5], [-1.5, 0, 1.8]]
    )

    output.parent.mkdir(parents=True, exist_ok=True)
    writer = imageio.get_writer(output, fps=video_fps, codec="libx264", quality=7)
    waypoint_index = 0
    try:
        for step in range(int(duration * physics_hz)):
            position, _ = p.getBasePositionAndOrientation(drone)
            if np.linalg.norm(waypoints[waypoint_index] - position) < 0.35:
                waypoint_index = (waypoint_index + 1) % len(waypoints)
            _control(drone, waypoints[waypoint_index], config)
            p.stepSimulation()
            if step % (physics_hz // video_fps) == 0:
                writer.append_data(_render(drone))
    finally:
        writer.close()
        final_position, final_orientation = p.getBasePositionAndOrientation(drone)
        p.disconnect()

    metadata = {
        "duration_seconds": duration,
        "seed": seed,
        "physics_hz": physics_hz,
        "video_fps": video_fps,
        "config": asdict(config),
        "final_position": final_position,
        "final_orientation_xyzw": final_orientation,
    }
    metadata_path = output.with_suffix(".json")
    metadata_path.write_text(json.dumps(metadata, indent=2), encoding="utf-8")
    return metadata


def main() -> None:
    parser = argparse.ArgumentParser(description="Run a headless PyBullet drone demo.")
    parser.add_argument("--duration", type=float, default=12.0)
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--output", type=Path, default=Path("/artifacts/flight.mp4"))
    parser.add_argument("--mass", type=float, default=1.0)
    args = parser.parse_args()
    if args.duration <= 0:
        parser.error("--duration must be positive")
    if args.mass <= 0:
        parser.error("--mass must be positive")

    metadata = run(args.duration, args.seed, args.output, DroneConfig(mass=args.mass))
    print(json.dumps(metadata, indent=2))
    print(f"Video written to {args.output}")
