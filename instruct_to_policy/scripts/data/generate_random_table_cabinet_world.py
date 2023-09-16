#!/usr/bin/env python
"""
This script generates a random world by: 
- import the table_cabinet_base.world gazebo world file, which contains only the table and the cabinet (panda urdf loaded by a stand-alone spawner)
- randomly select a number of containers from the container model list, and place them on the table
- randomly select a number of pickable objects from the object model list, and place them on the table or in the containers
- generate the new world file by inserting the new models into the table_cabinet_base.world file and save it to a folder named "worlds" in the current package
"""

import os
import sys
from typing import List, Tuple, Dict
import random
import numpy as np
import rospy
import rospkg
import json
import argparse
from xml.etree import ElementTree as ET

# Require ROS workspace environment variables 
# import pysdf 

def parse_args():
    """
    Parse the arguments of the program.
    """
    parser = argparse.ArgumentParser(description="Generate a random world.")
    parser.add_argument("--container_metadata_file", type=str, default="data/google_scanned_object/container_metadata.json", 
                        help="Path to the container metadata file")
    parser.add_argument("--container_models_dir", type=str, default="data/google_scanned_object/models", 
                        help="Path to the folder containing the container models")
    parser.add_argument("--ycb_object_metadata_file", type=str, default="data/ycb/metadata.json", 
                        help="Path to the pickable object metadata file from YCB dataset")
    parser.add_argument("--ycb_object_models_dir", type=str, default="data/ycb/models", 
                        help="Path to the folder containing the pickable object models")
    parser.add_argument("--google_scanned_object_metadata_file", type=str, default="data/google_scanned_object/object_metadata.json",
                        help="Path to the pickable object metadata file from google scanned object dataset")
    parser.add_argument("--google_scanned_object_models_dir", type=str, default="data/google_scanned_object/models",
                        help="Path to the folder containing the pickable object models")
    parser.add_argument("--world_metadata_dir", type=str, default="data/world_metadata", 
                        help="Path to the folder containing the world metadata files")
    
    # world generation parameters
    parser.add_argument("--num_containers", type=int, default=2, help="Number of containers to place on the table")
    parser.add_argument("--num_objects", type=int, default=3, help="Number of pickable objects to place on the table or in containers")
    parser.add_argument("--num_worlds", type=int, default=10, help="Number of worlds to generate")
    parser.add_argument("--base_world_file", type=str, default="worlds/table_cabinet_base.world", help="World file to use as base")
    parser.add_argument("--output_folder", type=str, default="worlds", help="Output folder")
    parser.add_argument("--model_margin", type=int, default=0.01, help="Margin to add to the model bounding box")
    parser.add_argument("--container_margin", type=int, default= 0.05, help="Margin when placing objects in containers")
    parser.add_argument("--table_margin", type=int, default= 0.15, help="Margin when placing objects on the table")
    parser.add_argument("--seed", type=int, default=42, help="Random seed")
    
    # base world constants, since the base world uses <include> tag to load the models, positions need to be manually specified here
    parser.add_argument("--table_xy_bbox", type=float, nargs=4, default=[-0.6, 0.6, -0.6, 0.6], help="Table xy bounding box, [x_min, x_max, y_min, y_max]")
    parser.add_argument("--table_surface_z", type=float, default=1.02, help="Table surface z position")
    parser.add_argument("--panda_base_xy", type=float, nargs=2, default=[-0.45, 0.0], help="Panda base xy position")
    parser.add_argument("--panda_xy_bbox", type=float, nargs=4, default=[-0.55, -0.35, -0.1, 0.1], help="Panda arm xy bounding box, [x_min, x_max, y_min, y_max]")
    parser.add_argument("--panda_max_range", type=float, default=1.0, help="Panda arm max reachable range from its arm base, upper limit is 1.3")
    
    
    args, unknown_args = parser.parse_known_args()

    print("Generating random world... Please make sure that the script is executed from the instruct_to_policy pacakge root folder.")

    # Check for unknown arguments
    if unknown_args:
        print(f"Warning: Unknown arguments: {unknown_args}")
    
    return args

def load_models_from_metadata(metadata_file) -> dict:
    """
    Load the selected models from the metadata file.
    """
    models = {}
    with open (metadata_file, "r") as f:
        metadata = json.load(f)
        for model in metadata["objects"].keys():
            models[model] = metadata["objects"][model]
            models[model]["model_id"] = model

    return models

def collides_with_bbox(bbox1: List[np.ndarray], bbox2: List[np.ndarray]):
    """
    Check if the bounding box collides with another bounding box .
    """
    if bbox1[0][0] > bbox2[1][0] or bbox1[1][0] < bbox2[0][0]:
        return False
    if bbox1[0][1] > bbox2[1][1] or bbox1[1][1] < bbox2[0][1]:
        return False
    if bbox1[0][2] > bbox2[1][2] or bbox1[1][2] < bbox2[0][2]:
        return False
    return True

def collides_with_other_objects(bbox: List[np.ndarray], models_in_world: Dict):
    """
    Check if the bounding box collides with other objects in the world.
    """
    for model in models_in_world.keys():
        # check if the bounding box collides with the model 
        if len(models_in_world[model]["bbox"]) > 0 and \
            collides_with_bbox(bbox, models_in_world[model]["bbox"]):
            return True
    return False

def reachable_by_panda(bbox: List[np.ndarray], panda_base_xy: np.ndarray, panda_max_range: float):
    """
    Check if all area of the bounding box is reachable by the panda arm.
    """
    # check if the bounding box is within the panda arm reachable range
    if bbox[0][0] < panda_base_xy[0] - panda_max_range or bbox[1][0] > panda_base_xy[0] + panda_max_range:
        return False
    if bbox[0][1] < panda_base_xy[1] - panda_max_range or bbox[1][1] > panda_base_xy[1] + panda_max_range:
        return False
    return True

def sample_container_position(container_id, container_models, models_in_world, args, num_trial=100):
    """
    Randomly generate containers positions on the table with contraints:
    - container should not collide with other containers
    - container should not collide with the panda arm
    - container should be within the panda arm reachable range
    - container bottom should be on the table surface
    """
    table_xy_bbox = args.table_xy_bbox
    table_surface_z = args.table_surface_z
    panda_base_xy = args.panda_base_xy
    panda_max_range = args.panda_max_range
    margin = args.model_margin
    c_margin = args.container_margin
    t_margin = args.table_margin

    # get the container dimension
    container_dimension = np.array(container_models[container_id]["bbox_size"])

    for _ in range(num_trial):
            
        # If the container collides with other containers or the panda arm, sample a new position
        # table also seen as a container, apply the container_margin constraint
        x = np.random.uniform(table_xy_bbox[0] + t_margin, table_xy_bbox[1] - t_margin)
        y = np.random.uniform(table_xy_bbox[2] + t_margin, table_xy_bbox[3] - t_margin)
        # bounding box bottom should be on surface
        z = table_surface_z + container_models[container_id]["bbox_size"][2] / 2.0 + margin
        container_position = np.array([x, y, z])

        # NOTE: Do NOT sample orientation for now, to use axis aligned bounding box
        # Sample a container orientation
        # roll = 0.0
        # pitch = 0.0
        # yaw = np.random.uniform(-np.pi, np.pi)

        # calculate the container bounding box
        container_bbox = [container_position - container_dimension/2, container_position + container_dimension/2]

        # Check if the container collides with other containers
        if not collides_with_other_objects(container_bbox, models_in_world) and \
            reachable_by_panda(container_bbox, panda_base_xy, panda_max_range):

            # add the container to the world
            container_name = container_models[container_id]["model_name"]
            models_in_world[container_id] = {"type": "container", "name": container_name, "bbox": container_bbox, "objects_in": []}
            models_in_world["table"]["objects_on"].append(container_id)
            return container_position
        
    # If no valid container position is found, return None
    print(f"Warning: No valid position found for container {container_id}")
    return None

def sample_object_position(object_id, object_models, container_models, models_in_world, args, 
                           ratio_in_container=0.6, max_objects_in_container=2, num_trial=100):
    """
    Randomly generate pickable object positions on the table with contraints:
    - object should not collide with other models
    - object can be placed on the table or in containers, with a certain ratio of being placed in containers
    - object should be within the panda arm reachable range
    - object bottom should be on the table surface or container bounding box top 
    """
    table_xy_bbox = args.table_xy_bbox
    table_surface_z = args.table_surface_z
    panda_base_xy = args.panda_base_xy
    panda_max_range = args.panda_max_range
    margin = args.model_margin
    c_margin = args.container_margin
    t_margin = args.table_margin

    # get the object dimension
    object_dimension = np.array(object_models[object_id]["bbox_size"])

    # containers available for placing objects
    all_containers = [model for model in models_in_world.keys() 
                      if models_in_world[model]["type"] == "container"]
    # valid containers for placing this object:
    # - not full
    # - xy-bounding box should be larger than the object
    valid_containers = [container for container in all_containers 
                            if len(models_in_world[container]["objects_in"]) < max_objects_in_container and
                               container_models[container]["bbox_size"][0] > object_models[object_id]["bbox_size"][0] + c_margin and
                                 container_models[container]["bbox_size"][1] > object_models[object_id]["bbox_size"][1] + c_margin] 
    

    for _ in range(num_trial):
            
        # object z position should be on the table surface or container bounding box top
        # randomly decide if the object should be placed in a container
        if np.random.uniform() < ratio_in_container and len(valid_containers) > 0:
            # randomly select a container
            container_id = random.choice(valid_containers)
            container_name = container_models[container_id]["model_name"]
            # randomly select a position on the container
            container_bbox_bottom = models_in_world[container_id]["bbox"][0]
            container_bbox_top = models_in_world[container_id]["bbox"][1]
            x = np.random.uniform(container_bbox_bottom[0] + object_models[object_id]["bbox_size"][0], 
                                  container_bbox_top[0] - object_models[object_id]["bbox_size"][0])
            y = np.random.uniform(container_bbox_bottom[1] + object_models[object_id]["bbox_size"][1], 
                                  container_bbox_top[1] - object_models[object_id]["bbox_size"][1])
            z = container_bbox_top[2] + object_models[object_id]["bbox_size"][2] / 2.0 + margin
        else:
            # sample a position on the table
            container_id = None
            container_name = None
            # table also seen as a container, apply the container_margin constraint
            x = np.random.uniform(table_xy_bbox[0] + t_margin, table_xy_bbox[1] - t_margin)
            y = np.random.uniform(table_xy_bbox[2] + t_margin, table_xy_bbox[3] - t_margin)
            z = table_surface_z + object_models[object_id]["bbox_size"][2] / 2.0 + margin

        object_position = np.array([x, y, z])

        # NOTE: Do NOT sample orientation for now, to use axis aligned bounding box
        # Sample a object orientation
        # roll = 0.0
        # pitch = 0.0
        # yaw = np.random.uniform(-np.pi, np.pi)

        # calculate the object bounding box
        object_bbox = [object_position - object_dimension/2, object_position + object_dimension/2]

        # Check if the object collides with other models
        if not collides_with_other_objects(object_bbox, models_in_world) and \
            reachable_by_panda(object_bbox, panda_base_xy, panda_max_range):

            # add the object to the world
            object_name = object_models[object_id]["model_name"]
            models_in_world[object_id] = {"type": "object", "name": object_name, "bbox": object_bbox}
            # add the object to the container if it is placed in a container
            if container_name is not None:
                models_in_world[container_id]["objects_in"].append(object_id)
            else:
                models_in_world["table"]["objects_on"].append(object_id)

            return object_position
        
    # If no valid object position is found, return None
    print(f"Warning: No valid position found for object {object_id}")
    return None


def add_model_to_xml(xml_root: ET.Element, model_id: str, model_name: str, model_pose: np.ndarray):
    """
    Add a model to the world tag in xml.

    Args:
        xml_root: The root of the xml tree
        model_id: The model id
        model_name: The model name
        model_pose: The model pose, [x, y, z, roll, pitch, yaw]
    """
    # Create a new <include> element
    new_include = ET.Element("include")

    # Create <uri>, <name>, <pose> subelements for the new <include> element
    uri = ET.Element("uri")
    uri.text = f"model://{model_id}"  
    name = ET.Element("name")
    name.text = model_name
    pose = ET.Element("pose")
    pose.text = " ".join([str(p) for p in model_pose])
    
    new_include.append(uri)
    new_include.append(name)
    new_include.append(pose)

    # Append the new <include> element to the root
    xml_root.find(".//world").append(new_include)
    

def generate_random_world(args):
    """
    Generate random worlds.
    """
    # Create the output folder if it doesn't exist
    if not os.path.exists(args.output_folder):
        os.makedirs(args.output_folder)

    # Read candidate models from metadata files
    container_models = load_models_from_metadata(args.container_metadata_file)
    ycb_object_models = load_models_from_metadata(args.ycb_object_metadata_file)
    google_scanned_object_models = load_models_from_metadata(args.google_scanned_object_metadata_file)
    object_models = {**ycb_object_models, **google_scanned_object_models}

    for world_idx in range(args.num_worlds):
        # Read the base world file with xml 
        # NOTE: pysdf library will automatically load models in <include> tag, which will dilate the world file.
        # Therefore, we use xml.etree.ElementTree to read the world file and manually insert new models into the world file. 
        world_file = args.base_world_file
        tree = ET.parse(world_file)
        root = tree.getroot()

        # initialize a dictionary of bouding boxes for all models added to the world
        models_in_world = {}
        # add table and robot arm collison bounding boxes to the dictionary
        models_in_world["table"] = {
            "type": "env", 
            "name": "table",
            "bbox": [np.array([args.table_xy_bbox[0], args.table_xy_bbox[2], 0.0]), 
                np.array([args.table_xy_bbox[1], args.table_xy_bbox[3], args.table_surface_z])],
            "objects_on": []
        }
        # TODO: add cabinet and drawer bounding box
        # TODO: randomly spawn objects inside cabinet drawers 
        models_in_world["cabinet"] = {"type": "env", "name": "cabinet", "bbox": [], "has_drawers": []}
        for i in range(0, 4):
            drawer_name = f"cabinet.drawer{i}"
            models_in_world[drawer_name] = {"type": "drawer", "name": f"cabinet.drawer{i}", "bbox": [], "objects_in": []}
            models_in_world["cabinet"]["has_drawers"].append(drawer_name)

        models_in_world["panda"] = {"type": "robot", "name": "panda_robot", "bbox": 
                                    [np.array([args.panda_xy_bbox[0], args.panda_xy_bbox[2], args.table_surface_z]), 
                                     np.array([args.panda_xy_bbox[1], args.panda_xy_bbox[3], args.table_surface_z + 1.0])]}

        # Randomly select containers and place them on the table without collision 
        selected_containers = random.sample(list(container_models.keys()), args.num_containers)
        print("Selected containers:")
        for container in selected_containers:
            print(f"- {container}")

        # Randomly generate container positions on the table, and add them to the world
        for container in selected_containers:
            sampled_position = sample_container_position(container, container_models, models_in_world, args)
            if sampled_position is not None:
                # add the container to the world
                # NOTE: currently random orientation is not used, to use axis aligned bounding box
                sampled_pose = np.concatenate((sampled_position, np.array([0.0, 0.0, 0.0])))
                add_model_to_xml(root, container, container_models[container]["model_name"], sampled_pose)
             

        # Randomly select pickable objects and place them on the table or in containers
        selected_objects = random.sample(list(object_models.keys()), args.num_objects)
        print("Selected pickable objects:")
        for obj in selected_objects:
            print(f"- {obj}")

        # Randomly generate object positions on the table or in containers, and add them to the world
        for obj in selected_objects:
            sampled_position = sample_object_position(obj, object_models, container_models, models_in_world, args)
            if sampled_position is not None:
                # add the object to the world
                # NOTE: currently random orientation is not used, to use axis aligned bounding box
                sampled_pose = np.concatenate((sampled_position, np.array([0.0, 0.0, 0.0])))
                add_model_to_xml(root, obj, object_models[obj]["model_name"], sampled_pose)

        # Write the new world file
        output_world_file = os.path.join(args.output_folder, f"table_cabinet_{world_idx}.world")
        tree.write(output_world_file)

        # Write the world metadata file based on models_in_world dictionary
        output_metadata_file = os.path.join(args.world_metadata_dir, f"table_cabinet_{world_idx}.json")
        def numpy2list(obj):
            if isinstance(obj, np.ndarray):
                return obj.tolist()
            raise TypeError('Not serializable')
        with open(output_metadata_file, "w") as f:
            json.dump(models_in_world, f, default=numpy2list, indent=2)



if __name__ == "__main__":

    args = parse_args()
    random.seed(args.seed)
    np.random.seed(args.seed)
    generate_random_world(args)