#! /usr/bin/env python
import os
from pdb import run
import sys
import rospy
import argparse
import time
import json
import traceback
from datetime import datetime
import random
import numpy as np
import torch
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest
from gazebo_msgs.srv import DeleteModel, DeleteModelRequest
from std_srvs.srv import Empty
from geometry_msgs.msg import Pose, Point, Quaternion

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
# from src.prompt.message_definitions import *
from src.lmp import *
from src.configs.config import load_config
from src.env.true_grounding_env import TrueGroundingEnv
from src.eval.evaluator import Evaluator


def get_models():
    """
    Returns a list of models in the models directory
    """
    package_root = os.path.dirname(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    )
    ycb_metadata_file = os.path.join(package_root, "data/ycb/metadata.json")
    google_metadata_file = os.path.join(
        package_root, "data/google_scanned_object/object_metadata.json"
    )

    # Load ycb metadata
    ycb_metadata = json.load(open(ycb_metadata_file, "r"))
    for model_id in ycb_metadata["objects"].keys():
        model_name = ycb_metadata["objects"][model_id]["model_name"]
        ycb_metadata["objects"][model_id]["sdf_path"] = os.path.join(
            package_root, "data/ycb/models", model_id, f"{model_name}.sdf"
        )

    # Load google scanned objects metadata
    google_metadata = json.load(open(google_metadata_file, "r"))
    for model_id in google_metadata["objects"].keys():
        model_name = google_metadata["objects"][model_id]["model_name"]
        google_metadata["objects"][model_id]["sdf_path"] = os.path.join(
            package_root,
            "data/google_scanned_object/models",
            model_id,
            f"{model_id}.sdf",
        )

    # merge metadata
    metadata = {"objects": {}}
    # for model_id in ycb_metadata['objects'].keys():
    #     metadata['objects'][model_id] = ycb_metadata['objects'][model_id]
    for model_id in google_metadata["objects"].keys():
        metadata["objects"][model_id] = google_metadata["objects"][model_id]

    return metadata


def spawn_model(model_name, model_sdf_file, model_pose, reference_frame="world"):
    """
    Spawn model in Gazebo
    """
    rospy.wait_for_service("/gazebo/spawn_sdf_model")
    try:
        model_xml = open(model_sdf_file, "r").read()
        spawn_model_prox = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
        req = SpawnModelRequest()
        req.model_name = model_name
        req.model_xml = model_xml
        req.initial_pose = model_pose
        req.reference_frame = reference_frame
        spawn_model_prox(req)
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


def delete_model(model_name):
    """
    Delete model in Gazebo
    """
    rospy.wait_for_service("/gazebo/delete_model")
    try:
        delete_model_prox = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
        req = DeleteModelRequest()
        req.model_name = model_name
        delete_model_prox(req)
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


def open_the_drawer(env: TrueGroundingEnv, drawer_idx=0, distance=0.2):
    env.open_gripper()
    grasp_pose = env.parse_horizontal_grasp_pose(f"cabinet.handle_{drawer_idx}")
    env.grasp(grasp_pose)
    env.close_gripper()
    env.attach_object(f"cabinet.handle_{drawer_idx}")

    # Pull the handle of the second top drawer, move in +x direction
    direction = [1, 0, 0]  # (x, y, z)
    current_pose = env.get_gripper_pose()
    target_pose = Pose()
    target_pose.position.x = current_pose.position.x + direction[0] * distance
    target_pose.position.y = current_pose.position.y + direction[1] * distance
    target_pose.position.z = current_pose.position.z + direction[2] * distance
    target_pose.orientation = current_pose.orientation

    # Move the gripper to the target pose
    env.move_to_pose(target_pose)

    # Release the handle of the second top drawer
    env.open_gripper()
    env.detach_object(f"cabinet.handle_{drawer_idx}")


def pick_and_place(env: TrueGroundingEnv, object_name, receptable_name):
    # Grasp the object
    env.open_gripper()
    grasp_pose = env.parse_adaptive_shape_grasp_pose(object_name)
    env.grasp(grasp_pose)
    env.close_gripper()
    env.attach_object(object_name)

    # Move the object to the receptacle
    place_pose = env.parse_place_pose(object_name, receptable_name)
    env.move_to_pose(place_pose)

    # Release the object
    env.open_gripper()
    env.detach_object(object_name)


def run_eval(
    evaluator: Evaluator,
    object_name,
    object_file_path,
    object_pose,
    drawer_idx,
    pull_distance=0.2,
    repeat_times=10,
):
    """
    Run pick and place evaluation
    """

    evaluator.init_results_dict(f"pick_and_place_{object_name}", repeat_times)

    for i in range(repeat_times):
        # reset environment
        evaluator.reset()
        exception = 0
        # pause_physics_prox()
        spawn_model(object_name, object_file_path, object_pose)
        rospy.sleep(1.0)
        # unpause_physics_prox()
        try:
            evaluator.logger.info("Running code for the {}th time".format(i + 1))

            # open the drawer
            open_the_drawer(
                evaluator.env, drawer_idx=drawer_idx, distance=pull_distance
            )

            # pick and place code
            pick_and_place(evaluator.env, object_name, f"cabinet.drawer_{drawer_idx}")

        except Exception as e:
            # also record the traceback
            exception = 1
            evaluator.logger.error(f"Error when executing code for {i}-th trial: {e}")
            evaluator.logger.error(traceback.format_exc())
            continue

        # wait 3 seconds for the world state to change
        time.sleep(3)

        eval_items = [
            {
                "function": "check_joint_state",
                "args": {
                    "joint_name": f"cabinet.joint_{drawer_idx}",
                    "check_position_func": lambda x: x > pull_distance * 0.75,
                },
            },
            {
                "function": "check_relation_in",
                "args": {
                    "object_name": object_name,
                    "receptacle_name": f"cabinet.drawer_{drawer_idx}",
                },
            },
        ]
        evaluator.eval_env_state(i, eval_items, exception=exception)

        delete_model(object_name)

    # log metrics in the end
    evaluator.logger.info(
        "\n######################## Results:\n {} \n###################################".format(
            evaluator.results
        )
    )


def parse_args():
    """
    Parse arguments
    """
    parser = argparse.ArgumentParser(description="Instruct to Policy")
    parser.add_argument("--config_to_eval", default="text_gpt_4")
    parser.add_argument(
        "--x", type=float, default=-0.10, help="x coordinate of the object"
    )
    parser.add_argument(
        "--y", type=float, default=-0.25, help="y coordinate of the object"
    )
    parser.add_argument(
        "--pull_distance", type=float, default=0.25, help="distance to pull the drawer"
    )
    parser.add_argument(
        "--table_height",
        type=float,
        default=0.815,
        help="z coordinate of the table top",
    )

    parser.add_argument(
        "--drawer_idx", type=int, default=1, help="index of the drawer to open"
    )

    parser.add_argument(
        "--object_name_filter",
        nargs="*",
        ########### google scan objects ############
        default=[
            "panda_toy_figure",
            "Nintendo_Mario_toy_figure",
            "wood_block",
            "grey_medication_bottle",
            "white_and_brown_box",
            "brown_medication_bottle",
            "blue_medication_bottle",
            "brown_ceramic_cup",
            "toy_bus",
            "dog_figure",
        ],
        ################# ycb objects ###############
        # default=[
        # "plum",
        # "tomato_soup_can",
        # "mug",
        # "racquetball",
        # "foam_brick",
        # "orange",
        # "baseball",
        # "tennis_ball",
        # "lemon",
        # "banana"
        # ],
        help="list of object names to evaluate",
    )

    parser.add_argument(
        "--random_seed", default=42, help="answer to life the universe and everything"
    )
    args = parser.parse_args([])

    # set random seed
    random.seed(args.random_seed)
    np.random.seed(args.random_seed)
    torch.cuda.manual_seed(args.random_seed)

    return args


if __name__ == "__main__":

    # Initialize ROS node
    rospy.init_node("test_drawer_pick_and_place", anonymous=False)
    args = parse_args()

    # get models from metadata
    metadata = get_models()

    # Initialize Envrionment and connect to gazebo
    pkg_root = os.path.dirname(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    )
    config_file = os.path.join(
        pkg_root, f"scripts/src/configs/{args.config_to_eval}.yaml"
    )
    cfg_tabletop = load_config(config_file)

    env = TrueGroundingEnv(cfg_tabletop)

    # log file should be appended with the formatted current time
    time_str = datetime.now().strftime("%Y%m%d-%H:%M")
    log_file = rospy.get_param(
        "~log_file", f"eval_drawer_pick_and_place_{time_str}.log"
    )
    log_file_path = os.path.join(pkg_root, "log", log_file)
    # make log directory if not exist
    os.makedirs(os.path.dirname(log_file_path), exist_ok=True)

    # results file
    result_file_path = os.path.join(
        pkg_root, f"data/benchmark/eval_results/drawer_pick_and_place_{time_str}.json"
    )

    # make result directory if not exist
    os.makedirs(os.path.dirname(result_file_path), exist_ok=True)
    eval_result_list = []

    # Service to spawn models in Gazebo
    # spawn_model_prox = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    # delete_model_prox = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
    pause_physics_prox = rospy.ServiceProxy("/gazebo/pause_physics", Empty)
    unpause_physics_prox = rospy.ServiceProxy("/gazebo/unpause_physics", Empty)

    # Iterate through object models
    for object_id in metadata["objects"].keys():
        object_name = metadata["objects"][object_id]["model_name"]

        if len(args.object_name_filter) > 0:
            if object_name not in args.object_name_filter:
                continue

        # Spawn object model in Gazebo
        object_file_path = metadata["objects"][object_id]["sdf_path"]
        object_bbox_center = metadata["objects"][object_id]["bbox_center"]
        object_bbox_size = metadata["objects"][object_id]["bbox_size"]
        object_z_margin = object_bbox_size[2] / 2.0 - object_bbox_center[2] + 0.01
        object_pose = Pose(
            position=Point(*[args.x, args.y, args.table_height + object_z_margin]),
            orientation=Quaternion(*[0, 0, 0, 1]),
        )

        # Run pick and place evaluation
        evaluator = Evaluator(env, log_file=log_file_path, verbose=True, render=False)
        run_eval(
            evaluator,
            object_name,
            object_file_path,
            object_pose,
            drawer_idx=args.drawer_idx,
            pull_distance=args.pull_distance,
            repeat_times=10,
        )
        results = evaluator.get_results()

        # append results to eval_result_list
        eval_result_list.append(results)
        del evaluator

    # write results to file
    with open(result_file_path, "w") as f:
        json.dump(eval_result_list, f, indent=4)
