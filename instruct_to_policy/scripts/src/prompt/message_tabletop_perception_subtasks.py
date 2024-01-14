train_subtasks = [
{# Pick up object on the table and place it 
"query": "objects = ['object_1', 'container_1', 'object_random', 'container_random', 'table'] ; # pick up object_1 and place it into container_1",
"COT": 
'''
# Detailed planning:
# Step 0: Reason task-relevant objects
# Step 1: Grasp the object_1
# Step 2: Move the object_1 to the container_1
# Step 3: Release the object_1
''',
"chunks": [
'''
# Reason task-relevant objects
objects = ['object_1', 'container_1', 'table']
detect_objects(object_list=objects)
''',
'''
# Grasp the object_1
open_gripper()
grasp_pose = parse_adaptive_shape_grasp_pose('object_1')
grasp(grasp_pose)
close_gripper()
attach_object('object_1')
''',
'''
# Move the object_1 to the container_1
parse_place_pose(object_name='object_1', receptacle_name='container_1')
move_to_pose(place_pose)
''',
'''
# Release the object_1
open_gripper()
detach_object('object_1')
'''  
]
},
{# Open a Drawer
"query": "objects = ['cabinet', 'drawer', 'handle', 'table', 'plants'] ; # open the first-top drawer",
"COT":
'''
# Detailed planning: 
# Step 0: Reason task-relevant object, drawer and its handle, and detect objects 
# Step 1: Get the first-top drawer to be opened and all candidates handles
# Step 2: Select the first handle as the target handle
# Step 3: Get the drawer prismatic joint closest to the handle position
# Step 4: Grasp the drawer handle with position and joint axis preferences
# Step 5: Pull the drawer handle, move in prismatic joint axis direction
# Step 6: Release the gripper from drawer handle 
''',
"chunks": [
'''
# Reason task-relevant objects, drawer and its handle, and detect objects
objects = ['drawer','handle']
detect_objects(object_list=objects)
''',
'''
# Get the first-top drawer to be opened and all candidates handles
drawer_postion=get_object_center_position("drawer_0")
x_min,y_min,z_min,x_max,y_max,z_max=get_3d_bbox("drawer_0")
detected_objects=get_obj_name_list()
handles= [name for name in detected_objects if 'handle' in name]
drawer_handles=[]
for handle_name in handles:
  if get_object_center_position(handle_name)[2]>z_min and get_object_center_position(handle_name)[2]<z_max:
    drawer_handles.append(handle_name)
''',
'''
# Select the first handle as the target handle
drawer_handle_name=drawer_handles[0]
drawer_handle_position = get_object_center_position(drawer_handle_name)
''',
'''
# Get the drawer prismatic joint closest to the handle position
drawer_joint_info = get_object_joint_info(obj_name='drawer', position=drawer_handle_position, type="prismatic")
drawer_joint_position = drawer_joint_info["joint_position"]
drawer_joint_axis = drawer_joint_info["joint_axis"]
''',
'''
# Grasp the drawer handle with position and joint axis preferences
open_gripper()
grasp_pose = parse_adaptive_shape_grasp_pose(object_name=drawer_handle_name, preferred_position=drawer_handle_position, preferred_approach_direction=drawer_joint_axis)
grasp(grasp_pose)
close_gripper()
attach_object(drawer_handle_name)
''',
'''
# Pull the drawer handle, move in prismatic joint axis direction
direction = drawer_joint_axis
move_in_direction(direction, distance=0.2)
''',
'''
# Release the drawer
open_gripper()
detach_object(drawer_handle_name)
'''
]
},
{# Close a Drawer
"query": "objects = ['cabinet', 'drawer', 'handle', 'table', 'plants'] ; # close the first-top drawer",
"COT":
'''
# Detailed planning:
# Step 0: Reason task-relevant object, drawer and its handle, and detect objects 
# Step 1: Get the first-top drawer to be opened and all candidates handles
# Step 3: Select the first handle as the target handle
# Step 4: Get the drawer prismatic joint closest to the handle position
# Step 5: Grasp the drawer handle with position and joint axis preferences
# Step 6: Push the drawer handle, move in -prismatic joint axis direction
# Step 7: Release the gripper from drawer handle
''',
"chunks": [
'''
# Reason task-relevant objects, drawer and its handle, and detect objects
objects = ['drawer','handle']
detect_objects(object_list=objects)
''',
'''
# Get the first-top drawer to be opened and all candidates handles
drawer_postion=get_object_center_position("drawer_0")
x_min,y_min,z_min,x_max,y_max,z_max=get_3d_bbox("drawer_0")
detected_objects=get_obj_name_list()
handles= [name for name in detected_objects if 'handle' in name]
drawer_handles=[]
for handle_name in handles:
    if get_object_center_position(handle_name)[2]>z_min and get_object_center_position(handle_name)[2]<z_max:
        drawer_handles.append(handle_name)
''',
'''    
# Select the first handle as the target handle
drawer_handle_name=drawer_handles[0]
drawer_handle_position = get_object_center_position(drawer_handle_name)
''',
'''
# Get the drawer prismatic joint closest to the handle position
drawer_handle_position = get_object_center_position(drawer_handle_name)
drawer_joint_info = get_object_joint_info(obj_name=drawer_name, position=drawer_handle_position, type="prismatic")
drawer_joint_position = drawer_joint_info["joint_position"]
drawer_joint_axis = drawer_joint_info["joint_axis"]
''',
'''
# Grasp the drawer handle with position and joint axis preferences
open_gripper()
grasp_pose = parse_adaptive_shape_grasp_pose(object_name=drawer_handle_name, preferred_position=drawer_handle_position, preferred_approach_direction=drawer_joint_axis)
grasp(grasp_pose)
close_gripper()
attach_object(drawer_handle_name)
''',
'''
# Push the drawer handle, move in -prismatic joint axis direction
direction = -drawer_joint_axis
move_in_direction(direction, distance=0.2)
''',
'''
# Release the drawer
open_gripper()
detach_object(drawer_handle_name)
'''
]
},
{# Put object into the drawer
"query": "objects = ['cabinet', 'drawer', 'table', 'red_box'] ; # put the red box into the first-top drawer",
"COT":
'''
# Detailed planning: 
# Step 0: Reason task-relevant object, and detect objects 
# Step 1: Get the first-top drawer to be opened and all candidates handles
# Step 2: Select the first handle as the target handle
# Step 3: Get the drawer prismatic joint closest to the handle position
# Step 4: Grasp the drawer handle with position and joint axis preferences
# Step 5: Pull the drawer handle, move in prismatic joint axis direction
# Step 6: Release the gripper from drawer handle 
# Step 6: Grasp the red box
# Step 7: Move the red box to the drawer
# Step 8: Release the red box
''',
"chunks": [
'''
# Reason task-relevant objects, and detect objects
objects = ['drawer','handle', 'red_box']
detect_objects(object_list=objects)
''',
'''
# Get the first-top drawer to be opened and all candidates handles
drawer_postion=get_object_center_position("drawer_0")
x_min,y_min,z_min,x_max,y_max,z_max=get_3d_bbox("drawer_0")
detected_objects=get_obj_name_list()
handles= [name for name in detected_objects if 'handle' in name]
drawer_handles=[]
for handle_name in handles:
  if get_object_center_position(handle_name)[2]>z_min and get_object_center_position(handle_name)[2]<z_max:
    drawer_handles.append(handle_name)
''',
'''
# Select the first handle as the target handle
drawer_handle_name=drawer_handles[0]
drawer_handle_position = get_object_center_position(drawer_handle_name)
''',
'''
# Get the drawer prismatic joint closest to the handle position
drawer_joint_info = get_object_joint_info(obj_name='drawer', position=drawer_handle_position, type="prismatic")
drawer_joint_position = drawer_joint_info["joint_position"]
drawer_joint_axis = drawer_joint_info["joint_axis"]
''',
'''
# Grasp the drawer handle with position and joint axis preferences
open_gripper()
grasp_pose = parse_adaptive_shape_grasp_pose(object_name=drawer_handle_name, preferred_position=drawer_handle_position, preferred_approach_direction=drawer_joint_axis)
grasp(grasp_pose)
close_gripper()
attach_object(drawer_handle_name)
''',
'''
# Pull the drawer handle, move in prismatic joint axis direction
direction = drawer_joint_axis
move_in_direction(direction, distance=0.2)
''',
'''
# Release the drawer
open_gripper()
detach_object(drawer_handle_name)
''',
'''
# Grasp the red box
open_gripper()
grasp_pose = parse_adaptive_shape_grasp_pose('red_box_0')
grasp(grasp_pose)
close_gripper()
attach_object('red_box_0')
''',
'''
# Move the red box to the drawer
place_pose = parse_place_pose('red_box_0', 'drawer_0')
move_to_pose(place_pose)
''',
'''
# Release the red box
open_gripper()
detach_object('red_box_0')
'''
]
},
{# Open a Door 
"query": "objects = ['door', 'door_handle'] ; # open the door",
"COT":
'''
# Detailed planning: 
# Step 0: Reason task-relevant objects, and detect objects
# Step 1: Get the door to be opened and its closest handle position
# Step 2: Get the door revolute joint closest to the handle position
# Step 3: Get the door plane normal closest to the handle position
# Step 4: Grasp the door handle with position and approach preferences: grasp at the center of the handle, approach from the door plane normal
# Step 5: Generate a rotational motion plan around the revolute joint 
# Step 6: Move the gripper along the motion plan
# Step 7: Release the gripper from door handle
''',
"chunks":[
'''
# Reason task-relevant objects, and detect objects
objects = ['door', 'door_handle']
detect_objects(object_list=objects)
''',
'''
# Get the door to be opened and its closest handle position
door_position=get_object_center_position("door_0")
x_min,y_min,z_min,x_max,y_max,z_max=get_3d_bbox("door_0")
detected_objects=get_obj_name_list()
handles= [name for name in detected_objects if 'handle' in name]
hanle_distances=[]
for handle_name in handles:
    handle_position=get_object_center_position(handle_name)
    hanle_distances.append(np.linalg.norm(handle_position-door_position))
door_handle_name=handles[np.argmin(hanle_distances)]
handle_position=get_object_center_position(door_handle_name)
''',
'''    
# Get the door revolute joint closest to the handle position
door_joint_info = get_object_joint_info(obj_name=door_name, position=handle_position, type="revolute")
door_joint_position = door_joint_info["joint_position"]
door_joint_axis = door_joint_info["joint_axis"]
''',
'''
# Get the door plane normal closest to the handle position
door_plane_normal = get_plane_normal(obj_name=door_name, position=handle_position)
''',
'''
# Grasp the door handle with position and approach preferences: grasp at the center of the handle, approach from the door plane normal
open_gripper()
grasp_pose = parse_adaptive_shape_grasp_pose(object_name=door_handle_name, preferred_position=handle_position, preferred_approach_direction=door_plane_normal)
grasp(grasp_pose)
close_gripper()
attach_object(handle_name)
''',
'''
# Generate a rotational motion plan around the revolute joint
motion_plan = generate_arc_path_around_joint(current_pose=get_gripper_pose(),joint_axis=door_joint_axis, joint_position=door_joint_position, n=10, angle=30)
''',
'''
# Move the gripper along the motion plan
follow_path(motion_plan)
''',
''' 
# Release the door
open_gripper()
detach_object(handle_name)
'''
]
},
{# Pick all toys in the basket
"query": "objects = ['toy_car', 'toy_train', 'table', 'wood_basket', 'glass'] ; # pick all toys in the basket and put them on the table",
"COT":
'''
# Detailed planning:
# Step 0: Reason task-relevant objects
# Step 1: Check which toys are in the basket
# Step 2: For each toy in the basket, grasp it and put it on the table
''',
"chunks": [
'''
# Reason task-relevant objects
objects = ['toy_car', 'toy_train', 'table', 'wood_basket']
detect_objects(object_list=objects)
''',
'''
# Check which toys are in the basket
toy_names = ['toy_car', 'toy_train']
toys_in_basket = []
for toy_name in toy_names:
    if check_object_in_receptacle(object=toy_name, receptacle='wood_basket'):
        toys_in_basket.append(toy_name)
''',
'''        
# For each toy in the basket, grasp it and put it on the table
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
]
},
{# Retrieve an item from a high shelf
"query": "objects = ['shelf', 'bowl', 'table'] ; # retrieve a bowl from the high shelf and put it on the table",
"COT":
'''
# Detailed planning:
# Step 0: Reason task-relevant objects
# Step 1: Grasp the bowl 
# Step 2: Move the bowl to the table
# Step 3: Release the bowl
''',
"chunks": [
'''
# Reason task-relevant objects
objects = ['shelf', 'bowl', 'table']
detect_objects(object_list=objects)
''',
'''
# Grasp the bowl
open_gripper()
grasp_bowl_pose = parse_adaptive_shape_grasp_pose(object='bowl', description='a bowl from the high shelf')
grasp(grasp_bowl_pose)
close_gripper()
attach_object('bowl')
''',
'''
# Move the bowl to the table
put_bowl_pose = parse_place_pose(object_name='bowl', receptacle_name='table')
move_to_pose(put_bowl_pose)
''',
'''
# Release the bowl
open_gripper()
detach_object('bowl')
'''
]
},
{# Move an object away from another object
"query": "objects = ['bowl', 'table', 'apple', 'shelve', 'plants'] ; # move the apple away from the bowl by 0.1m",
"COT":
'''
# Detailed planning:
# Step 0: Reason task-relevant objects
# Step 1: Grasp the apple
# Step 2: Calculate the position to move the apple to
# Step 3: Move the apple to the calculated position
# Step 4: Release the apple
''',
"chunks": [
'''
# Reason task-relevant objects
objects = ['bowl', 'table', 'apple']
detect_objects(object_list=objects)
''',
'''
# Grasp the apple
open_gripper()
grasp_apple_pose = parse_central_lift_grasp_pose(object='apple')
grasp(grasp_apple_pose)
close_gripper()
attach_object('apple')
''',
'''
# Calculate the position to move the apple to
apple_position = get_object_center_position('apple')
bowl_position = get_object_center_position('bowl')
direction = apple_position - bowl_position
direction = direction / np.linalg.norm(direction)
move_position = apple_position + direction * 0.1
''',
'''
# Move the apple to the calculated position
put_apple_pose = parse_place_pose(object_name='apple', position=move_position)
move_to_pose(put_apple_pose)
''',
'''
# Release the apple
open_gripper()
detach_object('apple')
'''
]
},
{# Swap objects in the two containers
"query": "objects = ['plate', 'fry_pan', 'table', 'peach', 'apple', 'wood_block', 'wine_glass'] ; # swap the positions of the peach in the plate and the apple in the fry pan",
"COT":
'''
# Detailed planning:
# Step 0: Reason task-relevant objects
# Step 1: Grasp the peach in the plate
# Step 2: Move the peach onto the table
# Step 3: Release the peach 
# Step 4: Wait for environment to be static and detect objects new states 
# Step 5: Grasp the apple in the fry pan
# Step 6: Move the apple into the plate
# Step 7: Release the apple
# Step 8: Wait for environment to be static and detect objects new states
# Step 9: Grasp the peach on the table
# Step 10: Move the peach into the fry pan
''',
"chunks": [
'''
# Reason task-relevant objects
objects = ['plate', 'fry_pan', 'table', 'peach', 'apple']
detect_objects(object_list=objects)
''',
'''
# Grasp the peach in the plate
peach_grasp_pose = parse_central_lift_grasp_pose(object_name='peach')
open_gripper()
grasp(peach_grasp_pose)
close_gripper()
attach_object('peach')
''',
'''
# Move the peach onto the table
peach_place_pose = parse_place_pose(object_name='peach', receptacle_name='table')
move_to_pose(peach_place_pose)
''',
'''
# Release the peach
open_gripper()
detach_object('peach')
''',
'''
# Wait for environment to be static and detect objects new states
rospy.sleep(3)
detect_objects(object_list=objects)
''',
'''
# Grasp the apple in the fry pan
apple_grasp_pose = parse_central_lift_grasp_pose(object_name='apple')
grasp(apple_grasp_pose)
close_gripper()
attach_object('apple')
''',
'''
# Move the apple into the plate
apple_place_pose = parse_place_pose(object_name='apple', receptacle_name='plate')
move_to_pose(apple_place_pose)
''',
'''
# Release the apple
open_gripper()
detach_object('apple')
''',
'''
# Wait for environment to be static and detect objects new states
rospy.sleep(3)
detect_objects(object_list=objects)
''',
'''
# Grasp the peach on the table
peach_grasp_pose = parse_central_lift_grasp_pose(object_name='peach')
grasp(peach_grasp_pose)
close_gripper()
attach_object('peach')
''',
'''
# Move the peach into the fry pan
peach_place_pose = parse_place_pose(object_name='peach', receptacle_name='fry_pan')
move_to_pose(peach_place_pose)
''',
'''
# Release the peach
open_gripper()
detach_object('peach')
'''
]
},
{# Pick up an object with preferrence of position and approach direction
"query": "objects = ['bowl', 'table', 'apple', 'plate', 'plants', 'standing_pink_box, lying_banana'] ; # move fruits and boxes into the plate, table is in xy-plane with z-axis pointing upward",
"COT":
'''
# Detailed planning:
# Step 0: Reason task-relevant objects
# Step 1: Grasp the apple. Use parse_center_lift_grasp_pose() since the apple is a spherical object.
# Step 2: Move the apple to the plate
# Step 3: Release the apple
# Step 4: Wait for environment to be static and detect objects new states
# Step 5: Grasp the standing pink box. Use parse_adaptive_shape_grasp_pose() with preferences. The standing pink box has axis aligned with table surface normal.
# Step 6: Move the standing pink box to the plate
# Step 7: Release the standing pink box
# Step 8: Wait for environment to be static and detect objects new states
# Step 9: Grasp the lying banana. Use parse_adaptive_shape_grasp_pose() with preferences. The lying banana has axis on the table surface, so approach direction of the gripper should be aligned with table surface normal.
# Step 10: Move the lying banana to the plate
# Step 11: Release the lying banana
''',
"chunks": [
'''
# Reason task-relevant objects
objects = ['table', 'apple', 'plate', 'standing_pink_box', 'lying_banana']
detect_objects(object_list=objects)
''',
'''
# Grasp the apple. Use parse_center_lift_grasp_pose() since the apple is a spherical object.
open_gripper()
grasp_apple_pose = parse_central_lift_grasp_pose(object='apple')
grasp(grasp_apple_pose)
close_gripper()
attach_object('apple')
''',
'''
# Move the apple to the plate
put_apple_pose = parse_place_pose(object_name='apple', receptacle_name='plate')
move_to_pose(put_apple_pose)
''',
'''
# Release the apple
open_gripper()
detach_object('apple')
''',
'''
# Wait for environment to be static and detect objects new states
rospy.sleep(3)
detect_objects(object_list=objects)
''',
'''
# Grasp the standing pink box. Use parse_adaptive_shape_grasp_pose() with preferences. The standing pink box has long-axis aligned with table surface normal.
table_normal = np.array([0,0,1])
open_gripper()
grasp_box_pose = parse_adaptive_shape_grasp_pose(object_name='standing_pink_box', preferred_position=get_object_center_position('standing_pink_box'), preferred_plane_normal=table_normal)
grasp(grasp_box_pose)
close_gripper()
attach_object('standing_pink_box')
''',
'''
# Move the standing pink box to the plate
put_box_pose = parse_place_pose(object_name='standing_pink_box', receptacle_name='plate')
move_to_pose(put_box_pose)
''',
'''
# Release the standing pink box
open_gripper()
detach_object('standing_pink_box')
''',
'''
# Wait for environment to be static and detect objects new states
rospy.sleep(3)
detect_objects(object_list=objects)
''',
'''
# Grasp the lying banana. Use parse_adaptive_shape_grasp_pose() with preferences. The lying banana has long-axis on the table surface, so approach direction of the gripper should be aligned with table surface normal.
table_normal = np.array([0,0,1])
open_gripper()
grasp_banana_pose = parse_adaptive_shape_grasp_pose(object_name='lying_banana', preferred_position=get_object_center_position('lying_banana'), preferred_approach_direction=table_normal)
grasp(grasp_banana_pose)
close_gripper()
attach_object('lying_banana')
''',
'''
# Move the lying banana to the plate
put_banana_pose = parse_place_pose(object_name='lying_banana', receptacle_name='plate')
move_to_pose(put_banana_pose)
''',
'''
# Release the lying banana
open_gripper()
detach_object('lying_banana')
'''
]
}
]

