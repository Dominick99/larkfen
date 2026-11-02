"""Image-box following for a world-aligned, stabilized downward camera.

    Camera right = world +X, image up = world +Y, optical axis = world -Z.
    Altitude hold is independent of box size; this is overhead following.
"""
import math
import numpy as np

from robotics.contracts import VelocityCommand
from robotics.flight_controllers.simple import rotation_matrix_xyzw


class VisualFollower:
    def __init__(self, altitude_m=9.0, fovy_deg=70.0, max_speed_mps=3.0):
        self.altitude_m = altitude_m
        self.fovy_deg = fovy_deg
        self.max_speed_mps = max_speed_mps
        self.previous_observation = None
        self.filtered_velocity = np.zeros(2)

    def update(self, state, box, image_size, timestamp):
        world_velocity = np.zeros(2)
        if box is not None:
            width, height = image_size
            focal = height / (2 * math.tan(math.radians(self.fovy_deg / 2)))
            center = np.array([(box[0]+box[2])/2-width/2, height/2-(box[1]+box[3])/2])
            displacement = center * max(state.position[2], 1) / focal
            observed_point = state.position[:2] + displacement
            if self.previous_observation is not None:
                old_time, old_point = self.previous_observation
                elapsed = timestamp - old_time
                if 0 < elapsed < .5:
                    measured_velocity = np.clip((observed_point-old_point)/elapsed, -3, 3)
                    self.filtered_velocity = .85*self.filtered_velocity + .15*measured_velocity
            self.previous_observation = (timestamp, observed_point)
            world_velocity = .8*displacement + self.filtered_velocity
            speed = np.linalg.norm(world_velocity)
            if speed > self.max_speed_mps:
                world_velocity *= self.max_speed_mps/speed
        else:
            # Brake to zero horizontal velocity. Do not chase stale predictions.
            self.previous_observation = None
            self.filtered_velocity.fill(0)
        rotation = rotation_matrix_xyzw(state.orientation_xyzw)
        yaw = math.atan2(rotation[1, 0], rotation[0, 0])
        forward = math.cos(yaw)*world_velocity[0] + math.sin(yaw)*world_velocity[1]
        right = -math.sin(yaw)*world_velocity[0] + math.cos(yaw)*world_velocity[1]
        up = float(np.clip(1.5*(self.altitude_m-state.position[2]), -1, 1))
        return VelocityCommand(float(forward), float(right), up, 0.0)
