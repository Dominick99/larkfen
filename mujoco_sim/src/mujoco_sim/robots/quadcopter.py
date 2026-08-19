from __future__ import annotations

import numpy as np
import mujoco

from robotics.contracts import VehicleState
from robotics.flight_controllers.simple import rotation_matrix_xyzw
from robotics.robot_models.quadcopter import (
    QuadcopterConfig,
    RotorModel,
    rotor_positions,
)


def build_world_xml(config: QuadcopterConfig, physics_hz: int) -> str:
    """Create MuJoCo's engine-specific representation of the shared model."""
    inertia = " ".join(str(value) for value in config.inertia_diagonal_kg_m2)
    rotor_geoms = "\n".join(
        f'<geom type="cylinder" pos="{x} {y} {z}" size="0.075 0.006" '
        f'rgba="{("0.95 0.25 0.12 1" if index == 0 else "0.15 0.45 0.9 1")}" '
        'contype="0" conaffinity="0"/>'
        for index, (x, y, z) in enumerate(rotor_positions(config))
    )
    return f"""
<mujoco model="shared_quadcopter">
  <compiler angle="degree" inertiafromgeom="false"/>
  <option timestep="{1.0 / physics_hz}" gravity="0 0 -9.81" integrator="RK4"/>
  <asset>
    <texture name="checker" type="2d" builtin="checker" rgb1="0.85 0.88 0.92"
             rgb2="0.35 0.48 0.72" width="256" height="256"/>
    <material name="ground" texture="checker" texrepeat="12 12" reflectance="0.05"/>
  </asset>
  <worldbody>
    <light pos="0 -2 6" dir="0 0 -1" diffuse="0.9 0.9 0.9"/>
    <geom name="ground" type="plane" size="20 20 0.1" material="ground"/>
    <geom name="target" type="sphere" pos="3 0 0.18" size="0.16"
          rgba="0.95 0.18 0.12 1" contype="0" conaffinity="0"/>
    <body name="quadcopter" pos="0 0 1">
      <freejoint/>
      <inertial pos="0 0 0" mass="{config.mass_kg}" diaginertia="{inertia}"/>
      <geom name="body_collision" type="box" size="0.11 0.08 0.04"
            rgba="0.12 0.16 0.2 1"/>
      <geom type="box" size="{config.arm_length_m} 0.015 0.012"
            euler="0 0 45" rgba="0.35 0.38 0.42 1" contype="0" conaffinity="0"/>
      <geom type="box" size="{config.arm_length_m} 0.015 0.012"
            euler="0 0 -45" rgba="0.35 0.38 0.42 1" contype="0" conaffinity="0"/>
      {rotor_geoms}
    </body>
  </worldbody>
</mujoco>
"""


class Quadcopter:
    """Thin MuJoCo adapter for the portable quadcopter dynamics model."""

    _spin_directions = np.array([-1.0, 1.0, -1.0, 1.0])

    def __init__(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        config: QuadcopterConfig,
    ) -> None:
        self.model = model
        self.data = data
        self.config = config
        self.rotors = RotorModel(config)
        self.rotor_positions = rotor_positions(config)
        self.body_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_BODY, "quadcopter"
        )

    @classmethod
    def create(cls, physics_hz: int) -> tuple[Quadcopter, mujoco.MjModel, mujoco.MjData]:
        config = QuadcopterConfig()
        model = mujoco.MjModel.from_xml_string(build_world_xml(config, physics_hz))
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        return cls(model, data, config), model, data

    def state(self) -> VehicleState:
        quaternion_wxyz = self.data.xquat[self.body_id].copy()
        velocity = np.zeros(6)
        mujoco.mj_objectVelocity(
            self.model,
            self.data,
            mujoco.mjtObj.mjOBJ_BODY,
            self.body_id,
            velocity,
            0,
        )
        return VehicleState(
            position=self.data.xpos[self.body_id].copy(),
            orientation_xyzw=quaternion_wxyz[[1, 2, 3, 0]],
            linear_velocity=velocity[3:6].copy(),
            angular_velocity=velocity[0:3].copy(),
        )

    def apply_motor_command(self, throttle: np.ndarray, timestep: float) -> None:
        thrusts = self.rotors.step(throttle, timestep)
        state = self.state()
        rotation = rotation_matrix_xyzw(state.orientation_xyzw)
        body_force = np.array([0.0, 0.0, float(thrusts.sum())])
        lever_torques = np.sum(
            np.cross(
                self.rotor_positions,
                np.column_stack((np.zeros(4), np.zeros(4), thrusts)),
            ),
            axis=0,
        )
        yaw_torque = float(
            np.dot(self._spin_directions, thrusts)
            * self.config.yaw_torque_per_thrust_m
        )
        body_angular_velocity = rotation.T @ state.angular_velocity
        body_torque = lever_torques + np.array([0.0, 0.0, yaw_torque])
        body_torque -= (
            self.config.angular_drag_nm_per_rad_s * body_angular_velocity
        )
        world_force = rotation @ body_force
        world_force -= self.config.linear_drag_n_per_mps * state.linear_velocity
        world_torque = rotation @ body_torque
        self.data.xfrc_applied[self.body_id, :3] = world_force
        self.data.xfrc_applied[self.body_id, 3:] = world_torque
