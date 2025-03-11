import argparse
import omni
import json
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

from pxr import Usd, UsdGeom,UsdLux
import numpy as np
import open3d as o3d
import os
import sys
from urllib.parse import quote
from usd_utils import remove_empty_prims, recursive_parse_new, get_mesh_from_points_and_faces, sample_points_from_mesh, sample_points_from_prim
from usd_utils import fix_mdls
from usd_utils import IsEmpty, IsObjXform

from omni.isaac.sensor import Camera
from omni.isaac.core import World

from omni.isaac.core.utils.stage import open_stage
from omni.isaac.core.utils.stage import add_reference_to_stage
import omni.isaac.core.utils.numpy.rotations as rot_utils
from utils.usd_utils import filter_free_noise, strip_world_prim
from utils.geometry_tools import extract_floor_heights, fix_floorheight, generate_intrinsic


# turn on the camera light
action_registry = omni.kit.actions.core.get_action_registry()
action = action_registry.get_action("omni.kit.viewport.menubar.lighting", "set_lighting_mode_camera")
action.execute()


def find_path(dir, endswith):
    paths = os.listdir(dir)
    for p in paths:
        if endswith in p:
            target_path = os.path.join(dir,p)
            if endswith == '.usd':
                target_path = quote(target_path, safe=':/')
            return target_path


def convert_usd_to_points(stage, meters_per_unit, json_data, use_json_data=True):
    remove_empty_prims(stage)
    world_prim = stage.GetPrimAtPath("/World/scene")
    scene_root = strip_world_prim(world_prim)
    noise_list = []
    if use_json_data:
        for prim_info in json_data:
            prim_name = prim_info['name'].replace("_MightBeGlass", "")
            prim_type = prim_info['feedback']
            if prim_type == 'confirm':
                noise_list.append(prim_name)
        print(noise_list)
    prims_all = [p for p in scene_root.GetAllChildren() if p.IsA(UsdGeom.Mesh) or p.IsA(UsdGeom.Xform) and not IsEmpty(p) and IsObjXform(p)]
    pcs_all = []
    sample_points_number = 100000
    for prim in prims_all:
        if prim.GetName() in noise_list:
            continue
        try:
            pcs, mesh = sample_points_from_prim(prim, sample_points_number)
            pcs_all.append(pcs)
            print(prim.GetName())
        except:
            prims_all.remove(prim)
            continue
    pcs_all = np.concatenate(pcs_all, axis=0) * meters_per_unit
    scene_pcd = o3d.geometry.PointCloud()
    scene_pcd.points = o3d.utility.Vector3dVector(pcs_all)
    scene_pcd.colors = o3d.utility.Vector3dVector(np.ones_like(pcs_all)*0.4)
    scene_pcd = scene_pcd.voxel_down_sample(0.05)
    return scene_pcd, prims_all



# data path
data_path = ''
output_path = ''
mdl_path = ''

all_list = sorted(os.listdir(data_path))
loop_list = all_list

for house_id in loop_list:
    # if house_id != '0050':
    #     continue
    # if house_id not in test_list:
    #     continue
    print(house_id)
    if not os.path.exists("%s/%s"%(data_path,house_id)):
        usd_path = find_path(os.path.join(data_path, house_id), endswith='.usd')
        usd_file_name = os.path.split(usd_path)[-1].replace(".usd", "")
        json_path = find_path(os.path.join(data_path, house_id), endswith='.json')
        json_data = None
        if json_path is not None:
            print(json_path)
            with open(json_path, 'r') as f:
                json_data = json.load(f)
        print(usd_path)
        world = World(physics_dt=0.01, rendering_dt=0.01, stage_units_in_meters=1.0)
        fix_mdls(usd_path, mdl_path)
        add_reference_to_stage(usd_path, "/World/scene")
        stage = omni.usd.get_context().get_stage()
        meters_per_unit = Usd.Stage.Open(usd_path).GetMetadata('metersPerUnit')
        print(meters_per_unit)
        scene_pcd_without_labeled_noise, prims_all = convert_usd_to_points(stage, meters_per_unit, json_data, False)
        scene_pcd_without_xy_free_noise = filter_free_noise(scene_pcd_without_labeled_noise)    
        scene_pcd_without_free_noise = filter_free_noise(scene_pcd_without_xy_free_noise)     
        scene_pcd = scene_pcd_without_free_noise
        points = np.array(scene_pcd.points)
        floor_heights = extract_floor_heights(points)
        # initialize the camera function
        camera = Camera(
            prim_path="/World/camera",
            position=np.array([0,0,0]),
            dt=0.05,
            resolution=(1280, 720),
            orientation=rot_utils.euler_angles_to_quats(np.array([0.0, 0.0, 90.0]),degrees=True))
        camera.set_focal_length(1.4)
        camera.set_focus_distance(0.205)
        camera.set_clipping_range(0.01,10000000)
        world.reset()
        camera.initialize()
        camera.add_motion_vectors_to_frame()
        camera.add_distance_to_image_plane_to_frame()
        for i in range(30):
            world.step(render=True)

        ceiling_height = 2.5
        for floor_index,current_floor in enumerate(floor_heights):

            current_floor = current_floor[0]
            current_floor = fix_floorheight(current_floor, prims_all, meters_per_unit)
            floor_upper_bound = current_floor + 0.1
            floor_lower_bound = current_floor - 0.1
            floor_points = points[np.where((points[:,2]<floor_upper_bound) & (points[:,2]>floor_lower_bound))]
            floor_xy = floor_points[:,:2]
            floor_x_max, floor_x_min = np.max(floor_xy[:,0]), np.min(floor_xy[:,0])
            floor_y_max, floor_y_min = np.max(floor_xy[:,1]), np.min(floor_xy[:,1])
            camera_intrinsics = generate_intrinsic(1280, 720, 90, 90)

