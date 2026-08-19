"""Portable physical definitions and actuator models for robots."""

from .quadcopter import QuadcopterConfig, RotorModel, allocate_rotor_throttle

__all__ = ["QuadcopterConfig", "RotorModel", "allocate_rotor_throttle"]
