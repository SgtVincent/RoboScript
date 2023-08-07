Below is a comprehensive summary of the code templates, basic API explanations, and example tasks for controlling a robotic arm using the ROS framework and MoveIt.

## Basic Code Template

### Initialize MoveIt and ROS

```python
def initialize():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('pandas_moveit', anonymous=True)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    move_group = moveit_commander.MoveGroupCommander("manipulator")
    gripper_command_group = moveit_commander.MoveGroupCommander("gripper")
    return move_group, gripper_command_group
```

### Set a Pose

```python
def set_pose(w, x, y, z):
    pose = geometry_msgs.msg.Pose()
    pose.orientation.w = w
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    return pose
```

### Open and Close Gripper

```python
def open_gripper(gripper_command_group):
    gripper_command_group.set_named_target("open")
    gripper_command_group.go(wait=True)

def close_gripper(gripper_command_group):
    gripper_command_group.set_named_target("closed")
    gripper_command_group.go(wait=True)
```

## Example Tasks

### Pick and Place

```python
def execute_pick_and_place(move_group, pick_pose, place_pose):
    waypoints = [move_group.get_current_pose().pose, pick_pose, place_pose]
    (plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.01, 0.0)
    move_group.execute(plan, wait=True)
```

### Open a Drawer

```python
def open_drawer(move_group, gripper_command_group):
    grab_handle_pose = set_pose(1.0, 0.4, -0.2, 0.1)
    open_gripper(gripper_command_group)
    move_group.set_pose_target(grab_handle_pose)
    move_group.go(wait=True)
    pull_drawer_pose = set_pose(1.0, 0.4, -0.3, 0.1)
    move_group.set_pose_target(pull_drawer_pose)
    move_group.go(wait=True)
    open_gripper(gripper_command_group)
```

### Turn a Faucet Clockwise

```python
def turn_faucet_clockwise(move_group, gripper_command_group):
    grab_faucet_pose = set_pose(1.0, 0.4, -0.2, 0.1)
    open_gripper(gripper_command_group)
    move_group.set_pose_target(grab_faucet_pose)
    move_group.go(wait=True)
    close_gripper(gripper_command_group)
    waypoints = [move_group.get_current_pose().pose]
    for angle in range(0, 91, 10):
        target_pose = set_pose(math.cos(math.radians(angle/2)), 0, 0, math.sin(math.radians(angle/2)))
        target_pose.position = current_pose.position
        waypoints.append(target_pose)
    (plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.01, 0.0)
    move_group.execute(plan, wait=True)
    open_gripper(gripper_command_group)
```


### Pressing a Button

This function presses a button, such as an elevator button.

```python
def press_button(move_group, gripper_command_group):
    button_pose = set_pose(1.0, 0.4, -0.2, 0.1)
    open_gripper(gripper_command_group)
    move_group.set_pose_target(button_pose)
    move_group.go(wait=True)
    close_gripper(gripper_command_group)
    button_pose.position.z -= 0.05
    move_group.set_pose_target(button_pose)
    move_group.go(wait=True)
    open_gripper(gripper_command_group)
```

### Drawing a Path

Follows a specified path to "draw" a shape or pattern.

```python
def draw_path(move_group, path_points):
    waypoints = [set_pose(1.0, point[0], point[1], point[2]) for point in path_points]
    (plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.01, 0.0)
    move_group.execute(plan, wait=True)
```

### Pouring Liquid

Grasps a bottle and tilts it to pour a liquid.

```python
def pour_water(move_group, gripper_command_group):
    grab_bottle_pose = set_pose(1.0, 0.4, -0.2, 0.1)
    open_gripper(gripper_command_group)
    move_group.set_pose_target(grab_bottle_pose)
    move_group.go(wait=True)
    close_gripper(gripper_command_group)
    pour_water_pose = set_pose(0.7, 0.4, -0.2, 0.1)
    move_group.set_pose_target(pour_water_pose)
    move_group.go(wait=True)
    move_group.set_pose_target(grab_bottle_pose)
    move_group.go(wait=True)
    open_gripper(gripper_command_group)
```

### Scanning an Object

This function simulates scanning an object by aligning the arm with it.

```python
def scan_object(move_group):
    scan_pose = set_pose(1.0, 0.4, -0.2, 0.1)
    move_group.set_pose_target(scan_pose)
    move_group.go(wait=True)
    # Additional code for actual scanning
```

### Waving Hello

This function makes the robotic arm perform a waving gesture.

```python
def wave_hello(move_group):
    wave_start_pose = set_pose(1.0, 0.4, -0.2, 0.1)
    wave_end_pose = set_pose(1.0, 0.4, 0.2, 0.1)
    for _ in range(3): # Wave 3 times
        move_group.set_pose_target(wave_start_pose)
        move_group.go(wait=True)
        move_group.set_pose_target(wave_end_pose)
        move_group.go(wait=True)
```


### Folding Laundry

Folds a piece of clothing laid flat.

```python
def fold_laundry(move_group, gripper_command_group):
    # Grab and fold
    grab_clothing_pose = set_pose(1.0, 0.5, -0.3, 0.1)
    fold_pose = set_pose(1.0, 0.5, -0.3, 0.5)
    open_gripper(gripper_command_group)
    move_group.set_pose_target(grab_clothing_pose)
    move_group.go(wait=True)
    close_gripper(gripper_command_group)
    move_group.set_pose_target(fold_pose)
    move_group.go(wait=True)
    open_gripper(gripper_command_group)
```

### Mixing Ingredients

Stirs ingredients together in a bowl.

```python
def mix_ingredients(move_group, gripper_command_group):
    grab_spoon_pose = set_pose(1.0, 0.4, -0.2, 0.1)
    open_gripper(gripper_command_group)
    move_group.set_pose_target(grab_spoon_pose)
    move_group.go(wait=True)
    close_gripper(gripper_command_group)
    
    for _ in range(5): # Stir 5 times
        stir_pose = set_pose(0.7, 0.4, -0.2, 0.1)
        move_group.set_pose_target(stir_pose)
        move_group.go(wait=True)
        move_group.set_pose_target(grab_spoon_pose)
        move_group.go(wait=True)
    
    open_gripper(gripper_command_group)
```

### Watering Plants

This function waters a house plant with a watering can.

```python
def water_plant(move_group, gripper_command_group):
    grab_can_pose = set_pose(1.0, 0.5, -0.2, 0.1)
    watering_pose = set_pose(0.7, 0.4, -0.2, 0.1)
    open_gripper(gripper_command_group)
    move_group.set_pose_target(grab_can_pose)
    move_group.go(wait=True)
    close_gripper(gripper_command_group)
    move_group.set_pose_target(watering_pose)
    move_group.go(wait=True)
    move_group.set_pose_target(grab_can_pose)
    move_group.go(wait=True)
    open_gripper(gripper_command_group)
```

### Retrieving Items from a High Shelf

This function retrieves an object from a high shelf.

```python
def retrieve_from_shelf(move_group, gripper_command_group):
    shelf_pose = set_pose(1.0, 0.6, 0.4, 0.1)
    open_gripper(gripper_command_group)
    move_group.set_pose_target(shelf_pose)
    move_group.go(wait=True)
    close_gripper(gripper_command_group)
    lower_pose = set_pose(1.0, 0.4, -0.2, 0.1)
    move_group.set_pose_target(lower_pose)
    move_group.go(wait=True)
    open_gripper(gripper_command_group)
```

### Vacuuming the Floor

Simulates the motion of vacuuming the floor.

```python
def vacuum_floor(move_group):
    for x in range(0, 5):
        for y in range(0, 5):
            vacuum_pose = set_pose(1.0, x * 0.1, y * 0.1, 0.05)
            move_group.set_pose_target(vacuum_pose)
            move_group.go(wait=True)
```


These examples demonstrate a wide variety of tasks that can be performed with a robotic arm using the ROS framework and MoveIt. By understanding and modifying these examples, you can create a diverse set of behaviors for your specific robotic application.