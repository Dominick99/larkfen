"""Engine-specific scene, car motion, and camera adaptation."""
import math
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from mujoco_sim.robots.quadcopter import Quadcopter, build_world_xml
from robotics.robot_models.quadcopter import QuadcopterConfig


def create_world():
    config = QuadcopterConfig()
    root = ET.fromstring(build_world_xml(config, 240))
    root.set('model', 'visual_car_following')
    ET.SubElement(root, 'visual')
    ET.SubElement(root.find('visual'), 'global', offwidth='640', offheight='480')
    ET.SubElement(root.find('visual'), 'headlight', ambient='.5 .5 .5', diffuse='.5 .5 .5', specular='0 0 0')
    world = root.find('worldbody')
    light = world.find('light')
    light.set('pos', '0 0 20')
    light.set('dir', '-.3 .2 -1')
    light.set('directional', 'true')
    light.set('ambient', '.3 .3 .3')
    light.set('specular', '0 0 0')
    world.remove(world.find("geom[@name='target']"))
    ground = world.find("geom[@name='ground']")
    ground.attrib.pop('material')
    ground.set('rgba', '.22 .25 .29 1')
    ground.set('size', '40 40 .1')
    drone = world.find("body[@name='quadcopter']")
    drone.set('pos', '0 -3 9')
    # Neutral rotors avoid false color detections when the drone enters another view.
    for geom in drone.findall('geom'):
        geom.set('rgba', '.65 .7 .76 1')
    for x in range(-24, 25, 4):
        ET.SubElement(world, 'geom', type='box', pos=f'{x} 4 .003', size='.025 22 .003',
                      rgba='.38 .41 .45 1', contype='0', conaffinity='0')
    for y in range(-16, 25, 4):
        ET.SubElement(world, 'geom', type='box', pos=f'0 {y} .004', size='24 .025 .003',
                      rgba='.38 .41 .45 1', contype='0', conaffinity='0')
    for name, color in [('red', '.95 .04 .03 1'), ('blue', '.03 .15 .95 1'), ('green', '.05 .85 .12 1')]:
        body = ET.SubElement(world, 'body', name=f'{name}_car', mocap='true', pos='0 0 .35')
        ET.SubElement(body, 'geom', name=f'{name}_chassis', type='box', size='.8 .4 .2',
                      rgba=color, contype='0', conaffinity='0')
        ET.SubElement(body, 'geom', name=f'{name}_roof', type='box', pos='-.1 0 .27',
                      size='.4 .32 .15', rgba=color, contype='0', conaffinity='0')
        for x in [-.5, .5]:
            for y in [-.43, .43]:
                ET.SubElement(body, 'geom', name=f'{name}_wheel_{x}_{y}', type='cylinder',
                              pos=f'{x} {y} -.15', size='.17 .08', euler='90 0 0',
                              rgba='.03 .035 .04 1', contype='0', conaffinity='0')
    # Ideal stabilized camera rig: follows only the drone, never the target.
    rig = ET.SubElement(world, 'body', name='camera_rig', mocap='true', pos='0 -3 8.85')
    ET.SubElement(rig, 'camera', name='onboard', pos='0 0 0', xyaxes='1 0 0 0 1 0', fovy='70')
    ET.SubElement(world, 'camera', name='overview', pos='13 -15 22', xyaxes='.83 .56 0 -.38 .56 .74', fovy='65')
    model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding='unicode'))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    return Quadcopter(model, data, config), model, data


def move_scene(model, data, drone, time, seed):
    # Kinematic props, not tire/steering physics. Seed changes route phase/speed.
    speed = .14 * (1 + .04 * (seed % 3))
    for i, color in enumerate(['green', 'red', 'blue']):
        phase = speed*time + i*2.1
        x = 8*math.sin(phase)
        y = 5-5*math.cos(phase)
        heading = math.atan2(5*math.sin(phase), 8*math.cos(phase))
        body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f'{color}_car')
        mocap = model.body_mocapid[body]
        data.mocap_pos[mocap] = [x, y, .35]
        data.mocap_quat[mocap] = [math.cos(heading/2), 0, 0, math.sin(heading/2)]
    rig = model.body('camera_rig').mocapid[0]
    data.mocap_pos[rig] = drone.state().position + [0, 0, -.15]
    data.mocap_quat[rig] = [1, 0, 0, 0]
    mujoco.mj_forward(model, data)


def truth_box(renderer, model, data):
    """Evaluation-only visible green body mask. Never passed to detector/guidance."""
    green_id = model.body('green_car').id
    renderer.enable_segmentation_rendering()
    try:
        renderer.update_scene(data, camera='onboard')
        seg = renderer.render().copy()
    finally:
        renderer.disable_segmentation_rendering()
    ids = np.where(model.geom_bodyid == green_id)[0]
    mask = np.isin(seg[:, :, 0], ids) & (seg[:, :, 1] == int(mujoco.mjtObj.mjOBJ_GEOM))
    yy, xx = np.where(mask)
    if not len(xx):
        return None
    return (int(xx.min()), int(yy.min()), int(xx.max()+1), int(yy.max()+1))
