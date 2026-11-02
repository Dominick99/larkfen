import argparse
import json
from pathlib import Path
import time

import imageio.v2 as imageio
import mujoco
import numpy as np
from PIL import Image, ImageDraw

from robotics.contracts import VelocityCommand
from robotics.flight_controllers import SimpleVelocityFlightController
from robotics.flight_controllers.visual_follow import VisualFollower
from .perception import ColorDetector, GreenTracker, iou
from .world import create_world, move_scene, truth_box


def run(output, duration=30, seed=7, video=True, dropout=True):
    output.mkdir(parents=True, exist_ok=True)
    drone, model, data = create_world()
    controller = SimpleVelocityFlightController(drone.config)
    follower = VisualFollower()
    detector, tracker = ColorDetector(), GreenTracker()
    renderer = mujoco.Renderer(model, height=480, width=640)
    # Warm-start motors at hover; altitude control handles the remaining transient.
    drone.rotors.throttle[:] = np.sqrt(drone.config.mass_kg*9.81/(4*drone.config.max_thrust_per_rotor_n))
    command = VelocityCommand()
    throttle = controller.update(drone.state(), command, 1/120)
    samples = []
    writer = imageio.get_writer(output/'following.mp4', fps=12, codec='libx264', quality=7) if video else None
    start = time.perf_counter()
    failure = None
    try:
        for step in range(round(duration*240)):
            t = step/240
            if step % 20 == 0:
                move_scene(model, data, drone, t, seed)
                renderer.update_scene(data, camera='onboard')
                rgb = renderer.render().copy()
                blacked_out = dropout and 12 <= t < 14
                if blacked_out:
                    rgb[:] = 12
                detections = detector.detect(rgb)
                track = tracker.update(detections, t)
                box = track.detection.box if track.detection else None
                state = drone.state()
                command = follower.update(state, box, (640,480), t)
                # Everything below this line is independent scoring/visualization.
                reference = truth_box(renderer, model, data)
                target = data.xpos[model.body('green_car').id].copy()
                horizontal_error = float(np.linalg.norm(state.position[:2]-target[:2]))
                overlap = iou(box, reference)
                samples.append(dict(time_s=t, drone=state.position.tolist(), target=target.tolist(),
                                    detected=box is not None, track_id=track.track_id,
                                    state=track.state, dropout=blacked_out,
                                    visible=reference is not None, iou=overlap,
                                    horizontal_error_m=horizontal_error,
                                    altitude_m=float(state.position[2]),
                                    command=[command.forward_mps, command.right_mps, command.up_mps]))
                if writer:
                    renderer.update_scene(data, camera='overview')
                    overview = renderer.render().copy()
                    canvas = Image.new('RGB', (1280,560), '#0c1420')
                    canvas.paste(Image.fromarray(rgb),(0,50))
                    canvas.paste(Image.fromarray(overview),(640,50))
                    draw = ImageDraw.Draw(canvas)
                    for d in detections:
                        color = {'green':'#4ade80','red':'#fb7185','blue':'#38bdf8'}[d.label]
                        x1,y1,x2,y2=d.box
                        draw.rectangle((x1,y1+50,x2,y2+50),outline=color,width=2)
                        draw.text((x1,max(50,y1+35)),d.label+' car',fill=color)
                    draw.line((310,290,330,290),fill='white')
                    draw.line((320,280,320,300),fill='white')
                    draw.text((16,10),'LARKFEN / GREEN CAR FOLLOWING',fill='#67e8c7')
                    draw.text((16,30),f'Onboard RGB + color ATR | {track.state.upper()} | t={t:.1f}s',fill='white')
                    draw.text((656,10),'MuJoCo overview / physical quadcopter / kinematic cars',fill='white')
                    draw.text((656,30),f'Horizontal offset {horizontal_error:.2f} m | altitude {state.position[2]:.2f} m',fill='white')
                    camera = model.camera('overview').id
                    local = (state.position-data.cam_xpos[camera]) @ data.cam_xmat[camera].reshape(3,3)
                    if local[2] < 0:
                        focal = 240/np.tan(np.deg2rad(65/2))
                        px, py = 960+focal*local[0]/-local[2], 290-focal*local[1]/-local[2]
                        draw.ellipse((px-10,py-10,px+10,py+10),outline='#67e8c7',width=2)
                        draw.text((px+14,py-6),'DRONE',fill='#67e8c7')
                    draw.text((16,537),'CAMERA DROPOUT / BRAKING TO HOVER' if blacked_out else 'Green boxes drive guidance. Ground truth is evaluation only.',fill='#fbbf24')
                    writer.append_data(np.asarray(canvas))
                    if step == 20*60:
                        canvas.save(output/'preview.png')
                if not np.isfinite(state.position).all() or state.position[2] < 4 or state.position[2] > 14:
                    failure = 'Aircraft exceeded finite-state/altitude envelope'
                    break
            if step % 2 == 0:
                throttle = controller.update(drone.state(), command, 1/120)
            drone.apply_motor_command(throttle, 1/240)
            mujoco.mj_step(model, data)
    finally:
        if writer:
            writer.close()
        renderer.close()
    scored = [s for s in samples if s['time_s'] >= 3 and not s['dropout']]
    visible = [s for s in scored if s['visible']]
    detections = [s for s in scored if s['detected']]
    recovered = next((s['time_s']-14 for s in samples if s['time_s']>=14 and s['detected']), None) if dropout and duration>14 else None
    metrics = dict(
        failure=failure, duration_s=duration, seed=seed,
        detection_rate=sum(s['detected'] for s in visible)/max(len(visible),1),
        target_visible_fraction=len(visible)/max(len(scored),1),
        correct_target_fraction=sum(s['iou']>.3 for s in detections)/max(len(detections),1),
        mean_box_iou=float(np.mean([s['iou'] for s in detections])) if detections else 0,
        mean_horizontal_error_m=float(np.mean([s['horizontal_error_m'] for s in scored])) if scored else None,
        max_horizontal_error_m=max(s['horizontal_error_m'] for s in samples),
        max_altitude_error_m=max(abs(s['altitude_m']-9) for s in samples),
        reacquisition_delay_s=recovered,
        dropout_hover_command=all(s['command'][:2]==[0.0,0.0] for s in samples if s['dropout']),
        wall_seconds=time.perf_counter()-start,
    )
    checks = dict(stable=failure is None and metrics['max_altitude_error_m']<1,
                  target_visible=metrics['target_visible_fraction']>.9,
                  detection=metrics['detection_rate']>.95,
                  correct_identity=metrics['correct_target_fraction']>.95,
                  following=metrics['mean_horizontal_error_m'] is not None and metrics['mean_horizontal_error_m']<2,
                  loss_hover=metrics['dropout_hover_command'],
                  reacquired=not dropout or (recovered is not None and recovered<1))
    result=dict(metrics=metrics,checks=checks,passed=all(checks.values()),samples=samples)
    (output/'following.json').write_text(json.dumps(result,indent=2,allow_nan=False),encoding='utf-8')
    print(json.dumps(dict(metrics=metrics,checks=checks,passed=result['passed']),indent=2))
    return result


def main():
    parser=argparse.ArgumentParser(description='MuJoCo RGB green-car following')
    parser.add_argument('--output',type=Path,default=Path('car_following/artifacts'))
    parser.add_argument('--duration',type=float,default=30)
    parser.add_argument('--seed',type=int,default=7)
    parser.add_argument('--no-video',action='store_true')
    parser.add_argument('--no-dropout',action='store_true')
    parser.add_argument('--evaluate',action='store_true')
    args=parser.parse_args()
    if not np.isfinite(args.duration) or args.duration<15:
        parser.error('duration must be finite and at least 15 seconds')
    result=run(args.output,args.duration,args.seed,not args.no_video,not args.no_dropout)
    if args.evaluate:
        repeat=run(args.output/'repeat',args.duration,args.seed,False,not args.no_dropout)
        alternate=run(args.output/'alternate',args.duration,args.seed+1,False,not args.no_dropout)
        report=dict(passed=result['passed'] and repeat['passed'] and alternate['passed'] and result['samples']==repeat['samples'],
                    deterministic=result['samples']==repeat['samples'],
                    primary=result['metrics'],alternate=alternate['metrics'])
        (args.output/'evaluation.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
        return 0 if report['passed'] else 1
    return 0 if result['passed'] else 1


if __name__=='__main__':
    raise SystemExit(main())
