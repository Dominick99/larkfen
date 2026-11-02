"""Engine-independent geographic navigation and synthetic sensor estimation.

Distances are metres, angles degrees, velocities north/east m/s. Geographic
coordinates are WGS84 geodetic latitude/longitude. Altitude is MSL metres.
"""
from dataclasses import dataclass
import math

import numpy as np
from geographiclib.geodesic import Geodesic


def inverse(a, b):
    result = Geodesic.WGS84.Inverse(*a, *b)
    return result['s12'], result['azi1'] % 360


def advance(position, north, east, dt):
    result = Geodesic.WGS84.Direct(
        *position, math.degrees(math.atan2(east, north)), math.hypot(north, east) * dt
    )
    return result['lat2'], result['lon2']


def wrap(angle):
    return (angle + 180) % 360 - 180


@dataclass(frozen=True)
class Scenario:
    # Representative coordinates, not a verified business or private address.
    origin: tuple[float, float] = (35.1068, -106.6298)
    destination: tuple[float, float] = (35.6595, -105.9770)
    altitude_m: float = 3000.0
    duration_s: float = 2400.0
    outage_start_s: float = 450.0
    outage_end_s: float = 750.0
    delivery_radius_m: float = 350.0
    seed: int = 7

    def validate(self):
        for lat, lon in (self.origin, self.destination):
            if not (-90 < lat < 90 and -180 <= lon <= 180):
                raise ValueError('Coordinates must be valid WGS84 latitude/longitude')
        if not all(math.isfinite(v) for v in (
            self.altitude_m, self.duration_s, self.outage_start_s,
            self.outage_end_s, self.delivery_radius_m,
        )):
            raise ValueError('Scenario values must be finite')
        if self.duration_s < 1 or self.altitude_m < 500 or self.delivery_radius_m <= 0:
            raise ValueError('Duration >= 1 s, positive radius, and altitude >= 500 m required')
        if not 0 <= self.outage_start_s <= self.outage_end_s:
            raise ValueError('GPS outage must have ordered nonnegative times')
        if inverse(self.origin, self.destination)[0] < self.delivery_radius_m * 2:
            raise ValueError('Destination must be outside the starting delivery zone')


class Navigator:
    """GPS-aided airspeed/heading dead reckoning; never reads aircraft position.

    GPS is 1 Hz with 4 m horizontal noise. Air-data is sampled by the runner
    at 10 Hz. Persistent heading/airspeed biases and unobserved wind changes
    make outages observable. This is a simple estimator, not an INS/EKF.
    """
    def __init__(self, scenario):
        self.position = scenario.origin
        self.rng = np.random.default_rng(scenario.seed)
        self.wind = np.zeros(2)
        self.airspeed_mps = 46.0
        self.last_gps = None
        self.last_gps_time = None

    def update(self, time, dt, air_north, air_east, gps_fix):
        speed = math.hypot(air_north, air_east) * 1.008 + self.rng.normal(0, 0.15)
        self.airspeed_mps = speed
        heading = math.atan2(air_east, air_north) + math.radians(0.6 + self.rng.normal(0, 0.1))
        measured_air = np.array([speed * math.cos(heading), speed * math.sin(heading)])
        self.position = advance(self.position, *(measured_air + self.wind), dt)
        if gps_fix is not None:
            noisy = advance(gps_fix, *self.rng.normal(0, 4, 2), 1)
            if self.last_gps is not None:
                elapsed = time - self.last_gps_time
                distance, bearing = inverse(self.last_gps, noisy)
                gps_velocity = distance / elapsed * np.array([
                    math.cos(math.radians(bearing)), math.sin(math.radians(bearing))
                ])
                self.wind = 0.9 * self.wind + 0.1 * (gps_velocity - measured_air)
            self.position = noisy
            self.last_gps = noisy
            self.last_gps_time = time
        return self.position
