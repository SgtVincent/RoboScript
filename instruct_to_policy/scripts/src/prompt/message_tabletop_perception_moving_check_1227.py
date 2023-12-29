message_tabletop_perception = [
{
"role":"system",
"content": """
You are a strict assistant, translating natural language to a python script to control a robotic arm.
Your output should be a python script that can be executed to perform the task described in the input.
Your reasoning process and detailed planning can be written as python comment in the scripts.
You can use the following imported modules and functions in your script:

'''
# Import rospy, the foundational Python library for ROS (Robot Operating System)
import rospy

# Import move_group, the class handle for moving the robot arm in MoveIt!
import move_group

# Import various ROS message types for handling robot pose and orientation
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion

# Import utility functions for perception
from perception_utils import (
    get_object_center_position,  # Returns the position of an object in the world frame. Returns: position: np.array [x,y,z]
    get_object_pose,              # Returns the pose of an object in the world frame. Returns: pose: Pose
    get_3d_bbox,                 # Returns the 3D bounding box of an object in the world frame. Args: object_name: str. Returns: bbox: np.array [x_min, y_min, z_min, x_max, y_max, z_max]
    get_obj_name_list,           # Returns a list of names of objects present in the scene
    parse_adaptive_shape_grasp_pose, # Args: object_name: str, preferred_position: Optional(np.array) [x,y,z], preferred_direction: Optional(np.array) [vx, vy, vz]. Returns: grasp_pose: Pose
    parse_central_lift_grasp_pose, # Args: object_name: str, description: Optional(str) in ['top', 'center'], Returns: grasp_pose: Pose
    parse_horizontal_grasp_pose, # Args: object_name: str, Returns: grasp_pose: Pose
    parse_place_pose,            # Predict the place pose for an object relative to a receptacle. Args: object_name: str, receptacle_name: Optional(str), position: Optional(np.array) [x,y,z], . Returns: place_pose: Pose
    detect_objects_with_moving,   # Detect and update task-specific objects' status in the environment. Call this function before interaction with environment objects. Args: object_list: Optional(List[str]), objects to detect.
    check,                       # Check if the state is correct Args: object_name: The container used to hold objects, task_query:query
)

There are three functions for predicting grasp poses, each tailored for different kinds of objects. Note that you need to choose the right grasp function for the object carefully!!!
1. 'parse_central_lift_grasp_pose': This method involves a vertical lifting action. The gripper closes at the center of the object and is not suitable for elongated objects and is not suitable for the objects with openings, as the gripper's width is really small. It is optimal for handling spherical and cuboid objects without any opening that are not ideal for off-center grasping.
2. 'parse_horizontal_grasp_pose': This approach is designed for lateral engagement and is ideal for interacting with objects attached to surfaces perpendicular to the tabletop, commonly found in domestic or industrial environments.
3. 'parse_adaptive_shape_grasp_pose': This function utilizes GraspNet for predictive positioning. This technique is particularly effective for securing objects with unconventional shapes or openings, typical in various everyday scenarios.

# Import utility functions for robot motion planning and execution
from motion_utils import (
    attach_object,  # Attaches an object to the robot gripper in the planning space. Call this function right after closing the gripper. Args: object_id: str. 
    detach_object,   # Detaches an object from the robot gripper in the planning space. Call this function right after opening the gripper. Args: object_id: str. 
    open_gripper,    # Open the gripper. No args.
    close_gripper,   # Close the gripper. No args.
    move_to_pose,    # Move the gripper to pose. Args: pose: Pose
    get_gripper_pose, # Get the gripper pose. No args. Returns: pose: Pose
    grasp,           # Executes a grasp motion at the grasp_pose. Args: grasp_pose: Pose
)
'''
You are encouraged to use above APIs to complete the task.
Note that you may always need to create your arbitrary functions to help you complete the task, which will be defined by external scripts.
The robot working space on table top is in range [-0.5, 0.2] in x-axis and [-0.5, 0.5] in y-axis. The height of the table top is 1.05.
You may need to select a safe temporary location by shifing in x,y -axis  to ensure that the operating object can be placed without interfering with other items or the robot's movements. 
Note that you are a strict coder, you can not hard code a predefined value, but you need to use api and tools to detect the value.
Please pay attention to the description and specific requirements of the target object in the task. You may need to write some founctions by yourself to determine whether the requirements are met.

Your generated content should only contain comments starting with '#' and python code!
"""
},
{# Pick up object on the table and place it 
"role":"user",
"content": "objects = ['object_1', 'container_1', 'object_random', 'container_random', 'table'] ; # pick up object_1 and place it into container_1"
},
{
"role": "assistant",
"content": 
'''
# Detailed planning:
# Step 0: Reason task-specific objects
# Step 1: Grasp the object_1
# Step 2: Move the object_1 to the container_1
# Step 3: Release the object_1
# Step 4: Check if object_1 is in container_1


# Reason task-specific objects
object_list  = ['object_1', 'container_1', 'table']
detect_objects_with_moving(object_list=object_list)

# Grasp the object_1
open_gripper()
grasp_pose = parse_adaptive_shape_grasp_pose(object_name='object_1')
grasp(grasp_pose)
close_gripper()
attach_object('object_1')

# Move the cake to the container_1
parse_place_pose(object_name='object_1', receptacle_name='container_1')
move_to_pose(place_pose)

# Release the object_1
open_gripper()
detach_object('object_1')

# Check if object_1 is in container_1
success = check(object_name='object_1', task_query='Move the cake to the container_1')
'''  
},
{# Open a Drawer
"role":"user",
"content": "objects = ['cabinet', 'drawer1', 'table', 'plants'] ; # open the drawer"
},
{
"role":"assistant",
"content":
'''
# Detailed planning: 
# Step 0: Reason task-specific objects
# Step 1: Grasp the drawer handle 
# Step 2: Pull the drawer handle, move in +x direction 
# Step 3: Release the gripper from drawer handle 
# Step 4: Check if the drawer is opened

# Reason task-specific objects
object_list  = ['cabinet', 'drawer1', 'drawer1_handle']
detect_objects_with_moving(object_list=object_list)

# Grasp the drawer handle 
open_gripper()
grasp_pose = parse_horizontal_grasp_pose('drawer1_handle')
grasp(grasp_pose)
close_gripper()
attach_object('drawer1_handle')

# Pull the drawer handle, move in +x direction
direction = figure_direction(beginning = get_object_center_position('drawer1_handle'), ending = get_object_center_position('drawer1_handle'))
move_in_direction(direction, distance=0.2, reuse_dict = {'object_name': 'drawer1_handle', 'choice': 'save'})

# Release the drawer
open_gripper()
move_in_direction(direction, distance=0.1)
detach_object('drawer1_handle')

# Check if the drawer is opened
success = check(object_name='drawer1', task_query='open the drawer')
'''
},
{# Close a Drawer
"role":"user",
"content": "objects = ['cabinet', 'drawer1', 'table', 'plants'] ; # close the drawer"
},
{
"role":"assistant",
"content":
'''
# Detailed planning:
# Step 0: Reason task-specific objects
# Step 1: Grasp the drawer handle
# Step 2: Pull the drawer handle, move in -x direction
# Step 3: Release the gripper from drawer handle
# Step 4: Check if the drawer is closed


# Reason task-specific objects
object_list  = ['cabinet', 'drawer1', 'drawer1_handle']
detect_objects_with_moving(object_list=object_list)

# Grasp the drawer handle
open_gripper()
grasp_pose = parse_horizontal_grasp_pose('drawer1_handle')
grasp(grasp_pose)
close_gripper()
attach_object('drawer1_handle')



# Pull the drawer handle, move in -x direction
direction = figure_direction(beginning = get_object_center_position('drawer1_handle'), ending = get_object_center_position('drawer1_handle'))
move_in_direction(direction, distance = 0.2, reuse_dict = {'object_name': 'drawer1_handle', 'choice': 'reversed'})


# Release the drawer
open_gripper()
move_in_direction(direction, distance=0.05)
detach_object('drawer1_handle')

# Check if the drawer is closed
success = check(object_name='drawer1', task_query='close the drawer')
'''
},
{# Put object into the drawer
"role":"user",
"content": "objects = ['drawer1', 'drawer2', 'table', 'red_box'] ; # put the red box into the top drawer"
},
{
"role":"assistant",
"content":
'''
# Detailed planning:
# Step 0: Reason task-specific objects
# Step 1: Grasp the handle of the top drawer
# Step 2: Pull the handle of the top drawer, move in +x direction
# Step 3: Release the handle of the top drawer 
# Step 4: Check if the drawer is opened
# Step 5: Grasp the red box
# Step 6: Move the red box into the drawer
# Step 7: Release the red box
# Step 8: Check if the red box is in the drawer


# Reason task-specific objects
object_list  = ['drawer1', 'drawer2', 'drawer1_handle', 'red_box']
detect_objects_with_moving(object_list=object_list)

# Grasp the handle of the top drawer
open_gripper()
grasp_pose = parse_horizontal_grasp_pose('drawer1_handle')
grasp(grasp_pose)
close_gripper()
attach_object('drawer1_handle')

# Pull the handle of the top drawer, move in +x direction
direction = figure_direction(beginning = get_object_center_position('drawer1_handle'), ending = get_object_center_position('drawer1_handle'))
move_in_direction(direction, distance=0.2, reuse_dict = {'object_name': 'drawer1_handle', 'choice': 'save'})

# Release the handle of the top drawer
open_gripper()
move_in_direction(direction, distance=0.05)
detach_object('drawer1_handle')

# Check if the drawer is opened
success = check(object_name='drawer1', task_query='open the drawer')

# Grasp the red box
grasp_pose = parse_adaptive_shape_grasp_pose(object_name='red_box')
grasp(grasp_pose)
close_gripper()
attach_object('red_box')

# Move the red box into the drawer
place_pose = parse_place_pose('red_box', 'drawer1')
move_to_pose(place_pose)

# Release the red box
open_gripper()
detach_object('red_box')

# Check if the red box is in the drawer
success = check(object_name='drawer1', task_query='move the red box into the drawer')

'''
},
{# Pick object from the drawer
"role":"user",
"content": "objects = ['drawer0', 'drawer1', 'table', 'knife', 'plate'] ; # pick the knife from the top drawer and put it on the plate"
},
{
"role":"assistant",
"content":
'''
# Detailed planning:
# Step 0: Reason task-specific objects
# Step 1: Grasp the handle of the top drawer
# Step 2: Pull the handle of the top drawer, move in +x direction
# Step 3: Release the handle of the top drawer
# Step 4: Check if the drawer is opened
# Step 5: Grasp the knife
# Step 6: Move the knife to the plate
# Step 7: Release the knife
# Step 8: Check if the knife is in the drawer


# Reason task-specific objects
object_list  = ['drawer0', 'drawer1', 'drawer0_handle', 'knife', 'plate']
detect_objects_with_moving(object_list=object_list)

# Grasp the handle of the top drawer
open_gripper()
grasp_pose = parse_horizontal_grasp_pose('drawer0_handle')
grasp(grasp_pose)
close_gripper()
attach_object('drawer0_handle')

# Pull the handle of the top drawer, move in +x direction
direction = figure_direction(beginning = get_object_center_position('drawer0_handle'), ending = get_object_center_position('drawer0_handle'))
move_in_direction(direction, distance=0.2, reuse_dict = {'object_name': 'drawer1_handle', 'choice': 'save'})

# Release the handle of the top drawer
open_gripper()
move_in_direction(direction, distance=0.05)
detach_object('drawer0_handle')

# Check if the drawer is opened
success = check(object_name='drawer1', task_query='open the drawer')

# Grasp the knife
grasp_pose = parse_adaptive_shape_grasp_pose(object_name='knife')
grasp(grasp_pose)
close_gripper()
attach_object('knife')

# Move the knife to the plate
place_pose = parse_place_pose('knife', 'plate')
move_to_pose(place_pose)

# Release the knife
open_gripper()
detach_object('knife')

# Check if the knife is in the drawer
success = check(object_name='drawer1', task_query='put the knife into the plate')

'''  
},
{# Open a Door 
"role":"user",
"content": "objects = ['door'] ; # open the door"
},
{
"role":"assistant",
"content":
'''
# Detailed planning: 
# Step 0: Reason task-specific objects
# Step 1: Grasp the door handle 
# Step 2: Pull the door handle downwards with an arc path of small radius by 30 degrees
# Step 3: pull the door horizontally with an arc path of large radius by 60 degrees
# Step 4: release the door handle 
# Step 5: Check if the door is opened

# Reason task-specific objects
object_list  = ['door', 'door_handle']
detect_objects_with_moving(object_list=object_list)

# Grasp the door handle
grasp_pose = parse_horizontal_grasp_pose('door_handle')
grasp(grasp_pose)
close_gripper()
attach_object('door_handle')

# Pull the door handle downwards with an arc path of small radius by 30 degrees
current_pose = get_gripper_pose()
waypoints = generate_arc_path(current_pose, center=get_object_center_position('door_handle'), radius=0.1, n=20, angle=30)
(plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.01, 0.0)
move_group.execute(plan, wait=True)

# pull the door horizontally with an arc path of large radius by 60 degrees
current_pose = get_gripper_pose()
waypoints = generate_arc_path(current_pose, center=get_object_center_position('door_handle'), radius=0.5, n=40, angle=60)
(plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.01, 0.0)
move_group.execute(plan, wait=True)

# Release the door handle
open_gripper()
detach_object('door_handle')

# Check if the door is opened
success = check(object_name='door', task_query='open the door')

'''
},
{# Open a Door 
"role":"user",
"content": "objects = ['door', 'door_handle'] ; # open the door"
},
{
"role":"assistant",
"content":
'''
# Detailed planning: 
# Step 0: Reason task-specific objects
# Step 1: Get the door to be opened and its handle position
# Step 2: Get the door revolute joint closest to the handle position
# Step 3: Grasp the door handle with position and joint axis preferences
# Step 4: Generate a rotational motion plan around the revolute joint 
# Step 5: Move the gripper along the motion plan
# Step 6: Release the gripper from door handle
# Reason task-specific objects


objects = ['door', 'door_handle']
detect_objects(object_list=objects)

# Get the door to be opened and its handle position
door_name = 'door'
handle_name = 'door_handle'
handle_position = get_object_center_position(handle_name)

# Get the door revolute joint closest to the handle position
door_joint_info = get_object_joint_info(obj_name=door_name, position=handle_position, type="revolute")
door_joint_position = door_joint_info["joint_position"]
door_joint_axis = door_joint_info["joint_axis"]

# Grasp the door handle with position and joint axis preferences
open_gripper()
grasp_pose = parse_adaptive_shape_grasp_pose(object_name=handle_name, preferred_position=handle_position, preferred_approach_direction=door_joint_axis)
grasp(grasp_pose)
close_gripper()
attach_object(handle_name)

# Generate a rotational motion plan around the revolute joint
motion_plan = generate_arc_path_around_joint(current_pose=get_gripper_pose(),joint_axis=door_joint_axis, joint_position=door_joint_position, n=10, angle=30)

# Move the gripper along the motion plan
follow_path(motion_plan)
 
# Release the door
open_gripper()
detach_object(handle_name)
'''
},
{# Open a Cabinet 
"role":"user",
"content": "objects = ['cabinet', 'cabinet_handle'] ; # open the door"
},
{
"role":"assistant",
"content":
'''
# Detailed planning: 
# Step 0: Reason task-specific objects
# Step 1: Get the door to be opened and its handle position
# Step 2: Get the door revolute joint closest to the handle position
# Step 3: Grasp the door handle with position and joint axis preferences
# Step 4: Generate a rotational motion plan around the revolute joint 
# Step 5: Move the gripper along the motion plan
# Step 6: Release the gripper from door handle
# Reason task-specific objects


objects = ['cabinet', 'cabinet_handle']
detect_objects(object_list=objects)

# Get the door to be opened and its handle position
door_name = 'cabinet'
handle_name = 'cabinet_handle'
handle_position = get_object_center_position(handle_name)

# Get the door revolute joint closest to the handle position
door_joint_info = get_object_joint_info(obj_name=door_name, position=handle_position, type="revolute")
door_joint_position = door_joint_info["joint_position"]
door_joint_axis = door_joint_info["joint_axis"]

# Grasp the door handle with position and joint axis preferences
open_gripper()
grasp_pose = parse_adaptive_shape_grasp_pose(object_name=handle_name, preferred_position=handle_position, preferred_approach_direction=door_joint_axis)
grasp(grasp_pose)
close_gripper()
attach_object(handle_name)

# Generate a rotational motion plan around the revolute joint
motion_plan = generate_arc_path_around_joint(current_pose=get_gripper_pose(),joint_axis=door_joint_axis, joint_position=door_joint_position, n=10, angle=30)

# Move the gripper along the motion plan
follow_path(motion_plan)
 
# Release the door
open_gripper()
detach_object(handle_name)
'''
},
{# Pick all toys in the basket
"role":"user",
"content": "objects = ['toy_car', 'toy_train', 'table', 'wood_basket', 'glass'] ; # pick all toys in the basket and put them on the table"
},
{
"role":"assistant",
"content":
'''
# Detailed planning:
# Step 0: Reason task-specific objects
# Step 1: Check which toys are in the basket
# Step 2: For each toy in the basket, grasp it and put it on the table
# Step 3: Check if all toys in the basket

# Reason task-specific objects
object_list  = ['toy_car', 'toy_train', 'table', 'wood_basket']
detect_objects_with_moving(object_list=object_list)

# Check which toys are in the basket
toy_names = ['toy_car', 'toy_train']
toys_in_basket = []
for toy_name in toy_names:
    if check_object_in_receptacle(object=toy_name, receptacle='wood_basket'):
        toys_in_basket.append(toy_name)
        
# For each toy in the basket, grasp it and put it on the table
for toy_name in toys_in_basket:
    # Grasp the toy
    open_gripper()
    grasp_pose = parse_adaptive_shape_grasp_pose(object_name=toy_name)
    grasp(grasp_pose)
    close_gripper()
    attach_object(toy_name)
    
    # Move the toy to the table
    place_pose = parse_place_pose(toy_name, 'table')
    move_to_pose(place_pose)
    
    # Release the toy
    open_gripper()
    detach_object(toy_name)

# Check if all toys in the basket
success = check(object_name='wood_basket', task_query='put all the toys into the basket')

'''
},
{# Retrieve an item from a high shelf
"role":"user",
"content": "objects = ['shelf', 'bowl', 'table'] ; # retrieve a bowl from the high shelf and put it on the table"
},
{
"role":"assistant",
"content":
'''
# Detailed planning:
# Step 0: Reason task-specific objects
# Step 1: Grasp the bowl 
# Step 2: Move the bowl to the table
# Step 3: Release the bowl
# Step 4: Check if the bowl is on the table


# Reason task-specific objects
object_list  = ['shelf', 'bowl', 'table']
detect_objects_with_moving(object_list=object_list)

# Grasp the bowl
open_gripper()
grasp_bowl_pose = parse_adaptive_shape_grasp_pose(object_name='bowl', description='a bowl from the high shelf')
grasp(grasp_bowl_pose)
close_gripper()
attach_object('bowl')

# Move the bowl to the table
put_bowl_pose = parse_place_pose(object_name='bowl', receptacle_name='table')
move_to_pose(put_bowl_pose)

# Release the bowl
open_gripper()
detach_object('bowl')

# Check if the bowl is on the table
success = check(object_name='table', task_query='move the bowl to the table')

'''
},
{# Move an object away from another object
"role":"user",
"content": "objects = ['bowl', 'table', 'apple', 'shelve', 'plants'] ; # move the apple away from the bowl by 0.1m"
},
{
"role":"assistant",
"content":
'''
# Detailed planning:
# Step 0: Reason task-specific objects
# Step 1: Grasp the apple
# Step 2: Calculate the position to move the apple to
# Step 3: Move the apple to the calculated position
# Step 4: Release the apple
# Step 5: Check if the apple is away from the bowl


# Reason task-specific objects
object_list  = ['bowl', 'table', 'apple']
detect_objects_with_moving(object_list=object_list)

# Grasp the apple
open_gripper()
grasp_apple_pose = parse_central_lift_grasp_pose(object_name='apple')
grasp(grasp_apple_pose)
close_gripper()
attach_object('apple')

# Calculate the position to move the apple to
direction = figure_direction(beginning = get_object_center_position('apple'), ending = get_object_center_position('bowl'))
move_position = apple_position + direction * 0.1

# Move the apple to the calculated position
put_apple_pose = parse_place_pose(object_name='apple', position=move_position)
move_to_pose(put_apple_pose)
 
# Release the apple
open_gripper()
detach_object('apple')

# Check if the apple is away from the bowl
success = check(object_name='bowl', task_query='Move the apple away from the bowl')

'''
},
{# Swap objects in the two containers
"role":"user",
"content": "objects = ['plate', 'fry_pan', 'table', 'peach', 'apple', 'wood_block', 'wine_glass'] ; # swap the positions of the peach in the plate and the apple in the fry pan"
},
{
"role":"assistant",
"content":
'''
# Detailed planning:
# Step 0: Reason task-specific objects
# Step 1: Grasp the peach in the plate
# Step 2: Move the peach onto the table
# Step 3: Release the peach 
# Step 4: Check if the peach is on the table
# Step 5: Wait for environment to be static and detect objects new states 
# Step 6: Grasp the apple in the fry pan
# Step 7: Move the apple into the plate
# Step 8: Release the apple
# Step 9 :Check if the apple is on the plate
# Step 10: Wait for environment to be static and detect objects new states
# Step 11: Grasp the peach on the table
# Step 12: Move the peach into the fry pan
# Step 13: Release the peach
# Step 14 :Check if the peach is on the fry pan



# Reason task-specific objects
object_list  = ['plate', 'fry_pan', 'table', 'peach', 'apple']
detect_objects_with_moving(object_list=object_list)

# Grasp the peach in the plate
peach_grasp_pose = parse_central_lift_grasp_pose(object_name='peach')
open_gripper()
grasp(peach_grasp_pose)
close_gripper()
attach_object('peach')

# Move the peach onto the table
peach_place_pose = parse_place_pose(object_name='peach', receptacle_name='table')
move_to_pose(peach_place_pose)

# Release the peach
open_gripper()
detach_object('peach')

# Check if the peach is on the table
success = check(object_name='table', task_query='Move the peach onto the table')

# Wait for environment to be static and detect objects new states
rospy.sleep(5)
detect_objects_with_moving()

# Grasp the apple in the fry pan
apple_grasp_pose = parse_central_lift_grasp_pose(object_name='apple')
grasp(apple_grasp_pose)
close_gripper()
attach_object('apple')

# Move the apple into the plate
apple_place_pose = parse_place_pose(object_name='apple', receptacle_name='plate')
move_to_pose(apple_place_pose)

# Release the apple
open_gripper()
detach_object('apple')

# Check if the apple is on the plate
success = check(object_name='plate_0', task_query='move the apple into the plate')

# Wait for environment to be static and detect objects new states
rospy.sleep(5)
detect_objects_with_moving()
 
# Grasp the peach on the table
peach_grasp_pose = parse_central_lift_grasp_pose(object_name='peach')
grasp(peach_grasp_pose)
close_gripper()
attach_object('peach')

# Move the peach into the fry pan
peach_place_pose = parse_place_pose(object_name='peach', receptacle_name='fry_pan')
move_to_pose(peach_place_pose)
 
# Release the peach
open_gripper()
detach_object('peach')

# Check if the peach is on the fry pan
success = check(object_name='fry_pan', task_query='move the peach into the fry pan')
'''
},
]
