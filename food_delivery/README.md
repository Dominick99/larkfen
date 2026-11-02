# Food delivery flight prototype

A standalone JSBSim experiment, independent of Larkfen's shared quadcopter code.
Fly between real WGS84 coordinates near Albuquerque and Santa Fe, interrupt GPS,
measure dead-reckoning error, and replay the result on a map or in an MP4.

The included endpoints are **representative locations**, not a verified restaurant
or a person's home. The default route is approximately 85 km. No geocoding API,
account, or API key is required. Supply coordinates to choose actual endpoints.

## Run from the repository root

```powershell
docker build -t larkfen-food-delivery food_delivery
docker run --rm -v "${PWD}/food_delivery/artifacts:/artifacts" larkfen-food-delivery --output /artifacts --evaluate
```

The first command downloads dependencies. Subsequent flights run offline. The
simulation runs as fast as the CPU allows; rendering happens afterward. Expect
the MP4 to take longer to generate than the flight physics.

Outputs (ignored by Git) go in `food_delivery/artifacts/`:

| File | Contents |
| --- | --- |
| `flight.mp4` | Up to 45 seconds of accelerated 2D replay, actual/estimated tracks and navigation error |
| `replay.html` | Self-contained offline geographic replay, play/pause, time scrubber, speed selection, optional street map |
| `overview.png` | Final flight track and error chart |
| `flight.json` | Scenario, 1 Hz trajectory samples, and evaluation metrics |
| `evaluation.json` | GPS baseline, outage, alternate seed, and deterministic repeat checks (`--evaluate`) |

The MP4 uses a geographic grid, **not street-map tiles or a 3D aircraft camera**.
The HTML replay offers a street-map background. Serve it over HTTP so the map
provider receives normal browser referrer headers:

```powershell
docker run --rm -p 127.0.0.1:8765:8000 -v "${PWD}/food_delivery/artifacts:/artifacts:ro" --entrypoint python larkfen-food-delivery -m http.server 8000 --directory /artifacts
```

Open <http://localhost:8765/replay.html>, then click **Load street map**. The
optional map loads Leaflet 1.9.4 from unpkg and OpenStreetMap tiles on demand.
Attribution remains visible. No tile scraping, prefetching, or offline tile
bundles are implemented. See the [OSM tile policy](https://operations.osmfoundation.org/policies/tiles/).
If internet is unavailable, the geographic canvas replay still works, including
when `replay.html` is opened directly from disk.

## Customize a flight

```powershell
docker run --rm -v "${PWD}/food_delivery/artifacts/custom:/artifacts" larkfen-food-delivery --output /artifacts --origin 35.1068 -106.6298 --destination 35.6595 -105.9770 --seed 11 --outage 450 750 --duration 2400
```

- `--outage START END`: GPS loss interval in simulated seconds; `0 0` disables it.
- `--duration SECONDS`: maximum flight duration, not replay duration.
- `--seed INTEGER`: repeatable sensor noise.
- `--no-video`: save HTML, PNG, and JSON without MP4 encoding.
- A flight that does not reach the destination before timeout returns exit code 1.
- Use separate output directories for comparisons; a run replaces its named outputs.
- `--evaluate` is intended for the default scenario. Custom routes/outages may
  intentionally fail its default arrival, drift, and recovery thresholds.

Local Python 3.11+ is also supported:

```powershell
python -m pip install -r food_delivery/requirements.txt
python -m food_delivery --output food_delivery/artifacts --evaluate
python -m unittest discover -s food_delivery/tests -v
```

Container tests:

```powershell
docker run --rm --entrypoint python larkfen-food-delivery -m unittest discover -s /app/food_delivery/tests -v
```

## What is simulated

- JSBSim's bundled `c172x` Cessna 172 model is a **fixed-wing surrogate**, not a
  calibrated delivery drone. JSBSim owns aerodynamics, propulsion, and rigid-body
  motion. The bundled heading/altitude autopilot stabilizes the aircraft.
- Starts already airborne at 3000 m MSL and about 46 m/s true airspeed. Dynamics
  step at 50 Hz. Navigation/air-data updates at 10 Hz; GPS and saved tracks at 1 Hz.
- Coordinates and integration use WGS84 geodesics; JSBSim conversions explicitly
  account for its feet and local north/east conventions.
- GPS fixes have 4 m standard-deviation noise on each horizontal axis. GPS is
  unavailable between 450 and 750 seconds by default.
- Synthetic horizontal air-data measurements are derived by the sensor adapter
  from true air-relative velocity, with airspeed noise/bias and heading noise/bias.
  GPS velocity estimates wind while fixes are available. In an outage, estimated
  wind is frozen and measured air motion is integrated. This is **GPS-aided
  air-data dead reckoning**, not a full accelerometer/gyro INS or Kalman filter.
- The navigator receives measurements, never true aircraft position except
  through noisy GPS fixes. True position is used separately by the evaluator.
  Attitude, altitude, and airspeed feedback to the low-level autopilot are ideal;
  low-level sensor failures are outside this first experiment.
- Wind varies smoothly over time. Navigation compensates using estimated wind.
- On estimated entry into a 350 m destination zone, the controller commands an
  approximately 700 m orbit and continues for 90 seconds. Evaluation independently
  checks whether the **actual aircraft** entered the zone. Arrival is not landing
  or physical delivery, and the orbit is not an independently certified hold.

## Evaluation

`--evaluate` runs four flights: the requested outage case, uninterrupted GPS,
an exact repeat, and a second sensor seed. It exits nonzero unless all checks pass:

- All three distinct flights reach the actual delivery zone without a simulated
  envelope failure.
- Outage-flight altitude error stays below 100 m.
- GPS-baseline horizontal position RMSE stays below 30 m.
- Outage error exceeds 50 m, exercising dead reckoning rather than hidden truth.
- Mean error 5–35 seconds after GPS restoration is below 30 m.
- The same seed produces identical trajectory samples within this environment.

Metrics include arrival time, closest approach, route/path length, position RMSE,
maximum outage error, GPS recovery error, maximum altitude error, and physics
simulation speedup. Timing excludes plotting and MP4 encoding. Exact numeric
reproducibility across different JSBSim/platform versions is not promised.

There is currently no real elevation model, obstacle avoidance, airspace routing,
takeoff/landing, battery model, or package release. Terrain is explicitly set to
zero elevation in the physics world; the map is a display and supplies no terrain
or building collision geometry. Consequently, this evaluates navigation and
control in an idealized world, not the real-world flyability of the route.

## Code layout

- `navigation.py`: engine-independent geodesy, mission configuration, sensor/estimator model.
- `simulation.py`: JSBSim state adaptation, wind, autopilot commands, flight loop, scoring.
- `rendering.py` and `replay.html`: offline plots, video encoding, and map replay.
- `__main__.py`: CLI and evaluation suite.
- `tests/`: geodesy, outage behavior, seed repeatability, and input validation.

[JSBSim](https://github.com/JSBSim-Team/jsbsim) is installed as a pinned dependency;
its bundled aircraft data is not copied into this repository. Its upstream code
and model licenses continue to apply.
