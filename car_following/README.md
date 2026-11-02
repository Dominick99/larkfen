# Green-car visual following

MuJoCo scene with red, blue, and green moving cars and a physical quadcopter.
The drone recognizes the green car in **rendered RGB pixels** and follows its
tracking box from above. Shared rotor dynamics and flight-controller math remain
in `robotics/`; MuJoCo-specific scene/state/force code remains in adapters.

## Run from repository root

```powershell
docker build -f car_following/Dockerfile -t larkfen-car-following .
docker run --rm -v "${PWD}/car_following/artifacts:/artifacts" larkfen-car-following --output /artifacts --evaluate
```

Outputs under `car_following/artifacts/` (ignored by Git):

- `following.mp4`: 30 seconds at 12 fps, onboard RGB detections beside an external
  view of all three cars and the drone.
- `preview.png`: a frame at five seconds.
- `following.json`: per-frame telemetry, metrics, thresholds, and pass/fail.
- `evaluation.json`: primary/alternate route-speed metrics and deterministic
  repeat comparison. Repeat and alternate telemetry have their own subfolders.

`--no-video` skips encoding, but still renders camera pixels for perception and
segmentation for independent scoring. `--no-dropout` disables the scripted camera
blackout from 12–14 seconds. `--seed 8` changes the scripted route speed; this
prototype otherwise contains no random sensor noise. `--duration` must be at least
15 seconds so the default dropout/recovery can be evaluated.

Tests:

```powershell
docker run --rm --entrypoint python larkfen-car-following -m unittest discover -s /app/car_following/tests -v
python scripts/verify.py
```

## Perception and control boundary

```text
MuJoCo RGB camera
   -> Detector.detect(rgb): list[Detection(box, label, confidence)]
   -> GreenTracker: Track(detection or None, track_id, state)
   -> shared VisualFollower: body-relative VelocityCommand
   -> shared SimpleVelocityFlightController: rotor throttle
   -> MuJoCo Quadcopter adapter: forces/torques
```

`Detector` is a Python protocol in `perception.py`. Replace `ColorDetector` with
an object-detection model that returns the same pixel-XYXY contracts. The label
currently carries a color (`green`, `red`, `blue`); a semantic detector should
select cars and provide an equivalent green-car attribute or adapt its output.
`GreenTracker` can separately be replaced with a learned or motion-based tracker.

Current ATR is an HSV color threshold and connected-component detector, **not AI
or a learned car classifier**. Confidence is a fixed prototype value, not a
calibrated probability. It works because the scene's colored objects are cars;
other green objects can fool it. The tracker associates green boxes by nearest
image center, retains identity through gaps up to three seconds, and emits no
stale detection during loss. It does not solve general multi-object re-ID.

The camera is an **ideal world-aligned stabilized downward camera**, attached by
position to a rig 0.15 m below the drone. Image right points world +X and image up
points world +Y. It does not tilt with the airframe. FOV is 70 degrees vertically,
resolution 640×480. No gimbal mechanics, camera delay, or pose-estimation noise
are modeled. Drone pose and velocity feedback are ideal simulator state.

The follower uses box center, camera intrinsics, and drone altitude to approximate
horizontal displacement by a pinhole projection onto a flat ground plane. A
smoothed finite difference of image-derived positions estimates target motion.
It commands proportional centering plus velocity feedforward, capped at 3 m/s.
Altitude is held independently at 9 m. Ground-plane projection is approximate
because car roofs are above ground. This is overhead following, not pursuit to
contact or box-area ranging.

On target loss, horizontal velocity commands become zero, braking toward hover.
Altitude hold continues. There is no blind chase, descent, or persistent search
pattern; a target that permanently leaves the field of view may remain lost.

## Evaluation

Ground-truth car positions and segmentation masks are used **only after guidance
has produced its command**, for scoring and visualization. A separate MuJoCo
segmentation pass produces the visible green-car box; it is never supplied to
the detector, tracker, or follower.

The default evaluation checks:

- Drone altitude remains within 1 m of 9 m and within the finite-state envelope.
- The target is visible in >90% of scored frames.
- Green-car detection in >95% of target-visible frames.
- >95% of emitted tracks overlap the green-car reference box with IoU >0.3.
- Mean horizontal following error below 2 m.
- Zero horizontal commands during the camera blackout.
- Reacquisition within one second of restored camera frames.
- Both configured route speeds pass, and an identical seed repeats exactly in
  the same execution environment.

Initial acquisition (first three seconds) and intentional dropout frames are
excluded from detection/mean-following scoring. Maximum offset/altitude metrics
include the full flight. Box IoU is imperfect because the detector sees colored
panels while segmentation includes wheels. The dropout is a synthetic whole-frame
camera blackout, **not a physical occlusion test**.

Physics/control/perception run at 240/120/12 Hz respectively. Cars are kinematic
props following a smooth oval at different phases; they do not simulate tires,
traffic, collision response, or road rules. There are no sharp-turn, realistic
occlusion, or clutter robustness guarantees yet. This is a deterministic baseline
for plugging in better perception and tracking models.
