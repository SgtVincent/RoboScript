pool = [
[# Pick up object on the table and place it 
'''
# Task: Reason task-relevant objects

objects = ['object_1', 'container_1', 'table']
detect_objects(object_list=objects)
''',
'''
# Task: Grasp the object_1

open_gripper()
grasp_pose = parse_adaptive_shape_grasp_pose('object_1')
grasp(grasp_pose)
close_gripper()
attach_object('object_1')
''',
'''
# Task: Move the object_1 to the container_1

parse_place_pose(object_name='object_1', receptacle_name='container_1')
move_to_pose(place_pose)
''',
'''
# Task: Release the object_1

open_gripper()
detach_object('object_1')
'''  
],
[# Open a Drawer 
'''
# Task: Reason task-relevant objects, drawer and its handle, and detect objects

objects = ['drawer','handle']
detect_objects(object_list=objects)
''',
'''
# Task: Get the first-top drawer to be opened and all candidates handles

# Detailed planning:
# Step 0: Get the drawer to be opened and its bounding box
# Step 1: Get all handles of the drawer
# Step 2: Select the handle in the drawer height range as candidate handles

# Get the drawer to be opened and its bounding box
drawer_postion=get_object_center_position("drawer_0")
x_min,y_min,z_min,x_max,y_max,z_max=get_3d_bbox("drawer_0")

# Get all handles of the drawer
detected_objects=get_obj_name_list()
handles= [name for name in detected_objects if 'handle' in name]

# Select the first handle in the drawer height range as candidate handles
drawer_handles=[]
for handle_name in handles:
  if get_object_center_position(handle_name)[2]>z_min and get_object_center_position(handle_name)[2]<z_max:
    drawer_handles.append(handle_name)
''',
'''
# Task: Select the first handle as the target handle

drawer_handle_name=drawer_handles[0]
drawer_handle_position = get_object_center_position(drawer_handle_name)
''',
'''
# Task: Get the drawer prismatic joint closest to the handle position

drawer_joint_info = get_object_joint_info(obj_name='drawer', position=drawer_handle_position, type="prismatic")
drawer_joint_position = drawer_joint_info["joint_position"]
drawer_joint_axis = drawer_joint_info["joint_axis"]
''',
'''
# Task: Grasp the drawer handle with position and joint axis preferences

open_gripper()
grasp_pose = parse_adaptive_shape_grasp_pose(object_name=drawer_handle_name, preferred_position=drawer_handle_position, preferred_approach_direction=drawer_joint_axis)
grasp(grasp_pose)
close_gripper()
attach_object(drawer_handle_name)
''',
'''
# Task: Pull the drawer handle, move in prismatic joint axis direction

direction = drawer_joint_axis
move_in_direction(direction, distance=0.2)
''',
'''
# Task: Release the drawer handle

open_gripper()
detach_object(drawer_handle_name)
'''   
],
[# Close a Drawer
'''
# Task: Reason task-relevant objects, drawer and its handle, and detect objects

objects = ['drawer','handle']
detect_objects(object_list=objects)
''',
'''
# Task: Get the first-top drawer to be opened and all candidates handles

# Detailed planning:
# Step 0: Get the drawer to be opened and its bounding box
# Step 1: Get all handles of the drawer
# Step 2: Select the handle in the drawer height range as candidate handles

# Get the drawer to be opened and its bounding box
drawer_postion=get_object_center_position("drawer_0")
x_min,y_min,z_min,x_max,y_max,z_max=get_3d_bbox("drawer_0")

# Get all handles of the drawer
detected_objects=get_obj_name_list()
handles= [name for name in detected_objects if 'handle' in name]

# Select the first handle in the drawer height range as candidate handles
drawer_handles=[]
for handle_name in handles:
  if get_object_center_position(handle_name)[2]>z_min and get_object_center_position(handle_name)[2]<z_max:
    drawer_handles.append(handle_name)
''',
'''    
# Task: Select the first handle as the target handle

drawer_handle_name=drawer_handles[0]
drawer_handle_position = get_object_center_position(drawer_handle_name)
''',
'''
# Task: Get the drawer prismatic joint closest to the handle position

drawer_handle_position = get_object_center_position(drawer_handle_name)
drawer_joint_info = get_object_joint_info(obj_name=drawer_name, position=drawer_handle_position, type="prismatic")
drawer_joint_position = drawer_joint_info["joint_position"]
drawer_joint_axis = drawer_joint_info["joint_axis"]
''',
'''
# Task: Grasp the drawer handle with position and joint axis preferences

open_gripper()
grasp_pose = parse_adaptive_shape_grasp_pose(object_name=drawer_handle_name, preferred_position=drawer_handle_position, preferred_approach_direction=drawer_joint_axis)
grasp(grasp_pose)
close_gripper()
attach_object(drawer_handle_name)
''',
'''
# Task: Push the drawer handle, move in -prismatic joint axis direction

direction = -drawer_joint_axis
move_in_direction(direction, distance=0.2)
''',
'''
# Task: Release the drawer handle

open_gripper()
detach_object(drawer_handle_name)
'''
],
[# Put object into the drawer
'''
# Task: Reason task-relevant objects, and detect objects
objects = ['drawer','handle', 'red_box']
detect_objects(object_list=objects)
''',
'''
# Task: Get the first-top drawer to be opened and all candidates handles

# Detailed planning:
# Step 0: Get the drawer to be opened and its bounding box
# Step 1: Get all handles of the drawer
# Step 2: Select the handle in the drawer height range as candidate handles

# Get the drawer to be opened and its bounding box
drawer_postion=get_object_center_position("drawer_0")
x_min,y_min,z_min,x_max,y_max,z_max=get_3d_bbox("drawer_0")

# Get all handles of the drawer
detected_objects=get_obj_name_list()
handles= [name for name in detected_objects if 'handle' in name]

# Select the first handle in the drawer height range as candidate handles
drawer_handles=[]
for handle_name in handles:
  if get_object_center_position(handle_name)[2]>z_min and get_object_center_position(handle_name)[2]<z_max:
    drawer_handles.append(handle_name)
''',
'''
# Task: Select the first handle as the target handle

drawer_handle_name=drawer_handles[0]
drawer_handle_position = get_object_center_position(drawer_handle_name)
''',
'''
# Task: Get the drawer prismatic joint closest to the handle position

drawer_joint_info = get_object_joint_info(obj_name='drawer', position=drawer_handle_position, type="prismatic")
drawer_joint_position = drawer_joint_info["joint_position"]
drawer_joint_axis = drawer_joint_info["joint_axis"]
''',
'''
# Task: Grasp the drawer handle with position and joint axis preferences

open_gripper()
grasp_pose = parse_adaptive_shape_grasp_pose(object_name=drawer_handle_name, preferred_position=drawer_handle_position, preferred_approach_direction=drawer_joint_axis)
grasp(grasp_pose)
close_gripper()
attach_object(drawer_handle_name)
''',
'''
# Task: Pull the drawer handle, move in prismatic joint axis direction

direction = drawer_joint_axis
move_in_direction(direction, distance=0.2)
''',
'''
# Task: Release the drawer handle

open_gripper()
detach_object(drawer_handle_name)
''',
'''
# Task: Grasp the red box

open_gripper()
grasp_pose = parse_adaptive_shape_grasp_pose('red_box_0')
grasp(grasp_pose)
close_gripper()
attach_object('red_box_0')
''',
'''
# Task: Move the red box to the drawer

place_pose = parse_place_pose('red_box_0', 'drawer_0')
move_to_pose(place_pose)
''',
'''
# Task: Release the red box

open_gripper()
detach_object('red_box_0')
'''
],
[# Open a Door
'''
# Task: Reason task-relevant objects, and detect objects

objects = ['door', 'door_handle']
detect_objects(object_list=objects)
''',
'''
# Task: Get the door to be opened and its closest handle position

# Detailed planning:
# Step 0: Get the door to be opened and its bounding box
# Step 1: Get all handles of the door
# Step 2: Select the handle closest to the door position as the target handle


# Get the door to be opened and its bounding box 
door_position=get_object_center_position("door_0")
x_min,y_min,z_min,x_max,y_max,z_max=get_3d_bbox("door_0")

# Get all handles of the door
detected_objects=get_obj_name_list()
handles= [name for name in detected_objects if 'handle' in name]

# Select the handle closest to the door position as the target handle
hanle_distances=[]
for handle_name in handles:
    handle_position=get_object_center_position(handle_name)
    hanle_distances.append(np.linalg.norm(handle_position-door_position))
door_handle_name=handles[np.argmin(hanle_distances)]
handle_position=get_object_center_position(door_handle_name)
''',
'''    
# Task: Get the door revolute joint closest to the handle position

door_joint_info = get_object_joint_info(obj_name=door_name, position=handle_position, type="revolute")
door_joint_position = door_joint_info["joint_position"]
door_joint_axis = door_joint_info["joint_axis"]
''',
'''
# Task: Get the door plane normal closest to the handle position

door_plane_normal = get_plane_normal(obj_name=door_name, position=handle_position)
''',
'''
# Task: Grasp the door handle with position and approach preferences

# Detailed planning:
# Step 0: Grasp at the center of the handle 
# Step 1: Approach from the door plane normal

open_gripper()
grasp_pose = parse_adaptive_shape_grasp_pose(object_name=door_handle_name, preferred_position=handle_position, preferred_approach_direction=door_plane_normal)
grasp(grasp_pose)
close_gripper()
attach_object(handle_name)
''',
'''
# Task: Generate a rotational motion plan around the revolute joint

motion_plan = generate_arc_path_around_joint(current_pose=get_gripper_pose(),joint_axis=door_joint_axis, joint_position=door_joint_position, n=10, angle=30)
''',
'''
# Task: Move the gripper along the motion plan

follow_path(motion_plan)
''',
''' 
# Task: Release the door
open_gripper()
detach_object(handle_name)
'''
],
[# Pick all toys in the basket
'''
# Task: Reason task-relevant objects

objects = ['toy_car', 'toy_train', 'table', 'wood_basket']
detect_objects(object_list=objects)
''',
'''
# Task: Check which toys are in the basket

toy_names = ['toy_car', 'toy_train']
toys_in_basket = []
for toy_name in toy_names:
    if check_object_in_receptacle(object=toy_name, receptacle='wood_basket'):
        toys_in_basket.append(toy_name)
''',
'''        
# Task: For each toy in the basket, grasp it and put it on the table

# Detailed planning:
# Step 0: Grasp the toy
# Step 1: Move the toy to the table
# Step 2: Release the toy


for toy_name in toys_in_basket:
    # Grasp the toy
    open_gripper()
    grasp_pose = parse_adaptive_shape_grasp_pose(toy_name)
    grasp(grasp_pose)
    close_gripper()
    attach_object(toy_name)
    
    # Move the toy to the table
    place_pose = parse_place_pose(toy_name, 'table')
    move_to_pose(place_pose)
    
    # Release the toy
    open_gripper()
    detach_object(toy_name)
'''   
],
[# Retrieve an item from a high shelf
'''
# Task: Reason task-relevant objects

objects = ['shelf', 'bowl', 'table']
detect_objects(object_list=objects)
''',
'''
# Task: Grasp the bowl

open_gripper()
grasp_bowl_pose = parse_adaptive_shape_grasp_pose(object='bowl', description='a bowl from the high shelf')
grasp(grasp_bowl_pose)
close_gripper()
attach_object('bowl')
''',
'''
# Task: Move the bowl to the table

put_bowl_pose = parse_place_pose(object_name='bowl', receptacle_name='table')
move_to_pose(put_bowl_pose)
''',
'''
# Task: Release the bowl

open_gripper()
detach_object('bowl')
'''
],
[# Move an object away from another object
'''
# Task: Grasp the apple

open_gripper()
grasp_apple_pose = parse_central_lift_grasp_pose(object='apple')
grasp(grasp_apple_pose)
close_gripper()
attach_object('apple')
''',
'''
# Task: Calculate the position to move the apple to

apple_position = get_object_center_position('apple')
bowl_position = get_object_center_position('bowl')
direction = apple_position - bowl_position
direction = direction / np.linalg.norm(direction)
move_position = apple_position + direction * 0.1
''',
'''
# Task: Move the apple to the calculated position

put_apple_pose = parse_place_pose(object_name='apple', position=move_position)
move_to_pose(put_apple_pose)
''',
'''
# Task: Release the apple

open_gripper()
detach_object('apple')
'''  
],
[
'''
# Task: Reason task-relevant objects

objects = ['plate', 'fry_pan', 'table', 'peach', 'apple']
detect_objects(object_list=objects)
''',
'''
# Task: Grasp the peach in the plate

peach_grasp_pose = parse_central_lift_grasp_pose(object_name='peach')
open_gripper()
grasp(peach_grasp_pose)
close_gripper()
attach_object('peach')
''',
'''
# Task: Move the peach onto the table

peach_place_pose = parse_place_pose(object_name='peach', receptacle_name='table')
move_to_pose(peach_place_pose)
''',
'''
# Task: Release the peach

open_gripper()
detach_object('peach')
''',
'''
# Task: Wait for environment to be static and detect objects new states

rospy.sleep(3)
detect_objects(object_list=objects)
''',
'''
# Task: Grasp the apple in the fry pan

apple_grasp_pose = parse_central_lift_grasp_pose(object_name='apple')
grasp(apple_grasp_pose)
close_gripper()
attach_object('apple')
''',
'''
# Task: Move the apple into the plate

apple_place_pose = parse_place_pose(object_name='apple', receptacle_name='plate')
move_to_pose(apple_place_pose)
''',
'''
# Task: Release the apple

open_gripper()
detach_object('apple')
''',
'''
# Task: Wait for environment to be static and detect objects new states

rospy.sleep(3)
detect_objects(object_list=objects)
''',
'''
# Task: Grasp the peach on the table

peach_grasp_pose = parse_central_lift_grasp_pose(object_name='peach')
grasp(peach_grasp_pose)
close_gripper()
attach_object('peach')
''',
'''
# Task: Move the peach into the fry pan

peach_place_pose = parse_place_pose(object_name='peach', receptacle_name='fry_pan')
move_to_pose(peach_place_pose)
''',
'''
# Task: Release the peach

open_gripper()
detach_object('peach')
'''
],
[# Swap objects in the two containers
'''
# Task: Reason task-relevant objects

objects = ['plate', 'fry_pan', 'table', 'peach', 'apple']
detect_objects(object_list=objects)
''',
'''
# Task: Grasp the peach in the plate

peach_grasp_pose = parse_central_lift_grasp_pose(object_name='peach')
open_gripper()
grasp(peach_grasp_pose)
close_gripper()
attach_object('peach')
''',
'''
# Task: Move the peach onto the table

peach_place_pose = parse_place_pose(object_name='peach', receptacle_name='table')
move_to_pose(peach_place_pose)
''',
'''
# Task: Release the peach

open_gripper()
detach_object('peach')
''',
'''
# Task: Wait for environment to be static and detect objects new states

rospy.sleep(3)
detect_objects(object_list=objects)
''',
'''
# Task: Grasp the apple in the fry pan

apple_grasp_pose = parse_central_lift_grasp_pose(object_name='apple')
grasp(apple_grasp_pose)
close_gripper()
attach_object('apple')
''',
'''
# Task: Move the apple into the plate

apple_place_pose = parse_place_pose(object_name='apple', receptacle_name='plate')
move_to_pose(apple_place_pose)
''',
'''
# Task: Release the apple

open_gripper()
detach_object('apple')
''',
'''
# Task: Wait for environment to be static and detect objects new states

rospy.sleep(3)
detect_objects(object_list=objects)
''',
'''
# Task: Grasp the peach on the table

peach_grasp_pose = parse_central_lift_grasp_pose(object_name='peach')
grasp(peach_grasp_pose)
close_gripper()
attach_object('peach')
''',
'''
# Task: Move the peach into the fry pan

peach_place_pose = parse_place_pose(object_name='peach', receptacle_name='fry_pan')
move_to_pose(peach_place_pose)
''',
'''
# Task: Release the peach

open_gripper()
detach_object('peach')
'''
],
[# Pick up an object with preferrence of position and approach direction
'''
# Task: Reason task-relevant objects

objects = ['table', 'apple', 'plate', 'standing_pink_box', 'lying_banana']
detect_objects(object_list=objects)
''',
'''
# Task: Grasp the apple. Use parse_center_lift_grasp_pose() since the apple is a spherical object.

open_gripper()
grasp_apple_pose = parse_central_lift_grasp_pose(object='apple')
grasp(grasp_apple_pose)
close_gripper()
attach_object('apple')
''',
'''
# Task: Move the apple to the plate

put_apple_pose = parse_place_pose(object_name='apple', receptacle_name='plate')
move_to_pose(put_apple_pose)
''',
'''
# Task: Release the apple

open_gripper()
detach_object('apple')
''',
'''
# Task: Wait for environment to be static and detect objects new states

rospy.sleep(3)
detect_objects(object_list=objects)
''',
'''
# Task: Grasp the standing pink box. Use parse_adaptive_shape_grasp_pose() with preferences.

# Detailed planning: 
# Step 0: Get the table normal
# Step 1: Grasp the standing pink box. The standing pink box has long-axis aligned with table surface normal. So grasp the box with preferred_position and preferred_plane_normal aligned with table surface normal.
# Step 2: Close gripper and attach the box in moveit planning scene.


# Get the table normal
table_normal = np.array([0,0,1])

# Grasp the standing pink box. The standing pink box has long-axis aligned with table surface normal. So grasp the box with preferred_position and preferred_plane_normal aligned with table surface normal.
open_gripper()
grasp_box_pose = parse_adaptive_shape_grasp_pose(object_name='standing_pink_box', preferred_position=get_object_center_position('standing_pink_box'), preferred_plane_normal=table_normal)
grasp(grasp_box_pose)

# Close gripper and attach the box in moveit planning scene.
close_gripper()
attach_object('standing_pink_box')
''',
'''
# Task: Move the standing pink box to the plate

put_box_pose = parse_place_pose(object_name='standing_pink_box', receptacle_name='plate')
move_to_pose(put_box_pose)
''',
'''
# Task: Release the standing pink box

open_gripper()
detach_object('standing_pink_box')
''',
'''
# Task: Wait for environment to be static and detect objects new states

rospy.sleep(3)
detect_objects(object_list=objects)
''',
'''
# Task: Grasp the lying banana. Use parse_adaptive_shape_grasp_pose() with preferences. 

# Detailed planning:
# Step 0: Get the table normal
# Step 1: Grasp the lying banana. The lying banana has long-axis on the table surface, so approach direction of the gripper should be aligned with table surface normal.
# Step 2: Close gripper and attach the banana in moveit planning scene.


# Get the table normal 
table_normal = np.array([0,0,1])

# Grasp the lying banana. The lying banana has long-axis on the table surface, so approach direction of the gripper should be aligned with table surface normal.
open_gripper()
grasp_banana_pose = parse_adaptive_shape_grasp_pose(object_name='lying_banana', preferred_position=get_object_center_position('lying_banana'), preferred_approach_direction=table_normal)
grasp(grasp_banana_pose)

# Close gripper and attach the banana in moveit planning scene.
close_gripper()
attach_object('lying_banana')
''',
'''
# Task: Move the lying banana to the plate

put_banana_pose = parse_place_pose(object_name='lying_banana', receptacle_name='plate')
move_to_pose(put_banana_pose)
''',
'''
# Task: Release the lying banana
open_gripper()
detach_object('lying_banana')
'''  
]
]