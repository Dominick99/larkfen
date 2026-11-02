"""JSBSim adapter and deterministic mission runner. No map/network dependency."""
from dataclasses import asdict
import math
import tempfile
import time

import jsbsim
import numpy as np

from .navigation import Navigator, Scenario, inverse

FT_TO_M = 0.3048
DT = 0.02
SENSOR_DT = 0.1


def run(scenario: Scenario):
    scenario.validate()
    wall_start = time.perf_counter()
    fdm = jsbsim.FGFDMExec(None)
    fdm.set_debug_level(0)
    fdm.set_output_path(tempfile.gettempdir())
    if not fdm.load_model('c172x'):
        raise RuntimeError('Bundled JSBSim c172x model could not be loaded')
    fdm.disable_output()
    fdm.set_dt(DT)
    _, bearing = inverse(scenario.origin, scenario.destination)
    for key, value in {
        'ic/lat-geod-deg': scenario.origin[0],
        'ic/long-gc-deg': scenario.origin[1],
        'ic/h-sl-ft': scenario.altitude_m / FT_TO_M,
        'ic/terrain-elevation-ft': 0,
        'ic/vt-fps': 150,
        'ic/psi-true-deg': bearing,
        'propulsion/set-running': -1,
    }.items():
        fdm[key] = value
    if not fdm.run_ic():
        raise RuntimeError('Aircraft initialization failed')
    fdm.get_propulsion().init_running(-1)
    fdm.do_trim(0)
    trim_throttle = fdm['fcs/throttle-cmd-norm']
    trim_speed = fdm['velocities/vtrue-fps']
    fdm['ap/heading_hold'] = 1
    fdm['ap/altitude_hold'] = 1
    fdm['ap/altitude_setpoint'] = scenario.altitude_m / FT_TO_M
    fdm['ap/heading_setpoint'] = bearing

    navigator = Navigator(scenario)
    samples = []
    errors = []
    outage_errors = []
    recovery_errors = []
    altitude_errors = []
    minimum_distance = math.inf
    arrived_at = None
    loiter_at = None
    failure = None
    next_gps = 0.0
    last_time = 0.0
    previous_true = scenario.origin
    path_length = 0.0
    # Mission decisions use the estimated position, never JSBSim's true position.
    for step in range(int(scenario.duration_s / DT)):
        t = step * DT
        # Deterministic slowly varying wind in local NED. JSBSim takes ft/s.
        wind_n = 3 + 2 * math.sin(t / 180)
        wind_e = 5 + 2 * math.sin(t / 130)
        fdm['atmosphere/wind-north-fps'] = wind_n / FT_TO_M
        fdm['atmosphere/wind-east-fps'] = wind_e / FT_TO_M
        if step % 5 == 0:
            true = (fdm['position/lat-geod-deg'], fdm['position/long-gc-deg'])
            alt = fdm['position/h-sl-ft'] * FT_TO_M
            vn = fdm['velocities/v-north-fps'] * FT_TO_M
            ve = fdm['velocities/v-east-fps'] * FT_TO_M
            if not all(math.isfinite(v) for v in (*true, alt, vn, ve)):
                failure = 'Nonfinite aircraft state'
                break
            gps_on = not scenario.outage_start_s <= t < scenario.outage_end_s
            fix = None
            if t + 1e-6 >= next_gps:
                next_gps += 1
                if gps_on:
                    fix = true
            estimated = navigator.update(t, SENSOR_DT, vn - wind_n, ve - wind_e, fix)
            remaining, course = inverse(estimated, scenario.destination)
            if loiter_at is None and remaining <= scenario.delivery_radius_m:
                loiter_at = t
            if loiter_at is not None:
                # Tangential orbit plus radial correction, radius 700 m.
                course = (course - 90 + np.clip((remaining - 700) * 0.06, -60, 60)) % 360
            # Compensate crosswind using only the estimator's wind estimate.
            course_rad = math.radians(course)
            crosswind = -navigator.wind[0] * math.sin(course_rad) + navigator.wind[1] * math.cos(course_rad)
            airspeed = max(navigator.airspeed_mps, 20)
            correction = math.asin(float(np.clip(-crosswind / airspeed, -0.7, 0.7)))
            fdm['ap/heading_setpoint'] = (course + math.degrees(correction)) % 360
            # Ideal airspeed feedback for the low-level controller, not navigation.
            fdm['fcs/throttle-cmd-norm'] = float(np.clip(
                trim_throttle + 0.025 * (trim_speed - fdm['velocities/vtrue-fps']), 0, 1
            ))
            error = inverse(true, estimated)[0]
            distance = inverse(true, scenario.destination)[0]
            minimum_distance = min(minimum_distance, distance)
            if arrived_at is None and distance <= scenario.delivery_radius_m:
                arrived_at = t
            errors.append(error)
            altitude_errors.append(abs(alt - scenario.altitude_m))
            if not gps_on:
                outage_errors.append(error)
            if scenario.outage_end_s + 5 <= t < scenario.outage_end_s + 35:
                recovery_errors.append(error)
            path_length += inverse(previous_true, true)[0]
            previous_true = true
            if step % 50 == 0:
                samples.append(dict(time_s=round(t, 2), true=list(true), estimated=list(estimated),
                                    altitude_m=alt, heading_deg=fdm['attitude/psi-deg'],
                                    speed_mps=math.hypot(vn, ve), gps_available=gps_on,
                                    error_m=error, remaining_m=distance,
                                    phase='loiter' if loiter_at is not None else 'cruise'))
            if not math.isfinite(error):
                failure = 'Nonfinite navigation state'
                break
            if alt < 500 or abs(fdm['attitude/phi-deg']) > 75:
                failure = 'Aircraft exceeded altitude/bank safety envelope'
                break
            if loiter_at is not None and t >= loiter_at + 90:
                last_time = t
                break
        if not fdm.run():
            failure = 'JSBSim stopped unexpectedly'
            break
        last_time = (step + 1) * DT

    elapsed = time.perf_counter() - wall_start
    route_distance = inverse(scenario.origin, scenario.destination)[0]
    metrics = dict(
        success=arrived_at is not None and failure is None,
        failure=failure, arrival_time_s=arrived_at, loiter_start_s=loiter_at,
        closest_approach_m=minimum_distance,
        route_distance_m=route_distance, path_length_m=path_length,
        position_rmse_m=float(np.sqrt(np.mean(np.square(errors)))),
        max_position_error_m=max(errors), max_outage_error_m=max(outage_errors, default=0),
        recovery_mean_error_m=float(np.mean(recovery_errors)) if recovery_errors else None,
        max_altitude_error_m=max(altitude_errors), simulated_seconds=last_time,
        physics_wall_seconds=elapsed, simulation_speedup=last_time / elapsed,
        sample_count=len(samples),
    )
    return dict(schema_version=1, aircraft='JSBSim c172x', scenario=asdict(scenario),
                metrics=metrics, samples=samples)
