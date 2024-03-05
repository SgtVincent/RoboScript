import os
import pybullet as p
from pybullet_utils import urdfEditor
import pybullet_data
import xml.etree.ElementTree as ET
from xml.etree.ElementTree import Element, tostring, SubElement, Comment, ElementTree, XML
import copy
import os.path as osp
import trimesh


def get_controlable_joint(box_id):
    controllable_object_joints = []
    joint_nums = p.getNumJoints(box_id)
    for i in range(joint_nums):
        info = p.getJointInfo(box_id, i)
        jointID = info[0]
        jointType = info[2]
        controllable = (jointType != p.JOINT_FIXED)
        if controllable:
            controllable_object_joints.append(jointID)
    return controllable_object_joints

def urdf_test(urdf_dir):
    arm_file = "urdf/panda.urdf"
    urdf_file = os.path.join(urdf_dir, 'mobility_scaled_mass.urdf')
    print("urdf_file", urdf_file)
    server_id = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -10)
    # p.createCollisionShape()
    mb = p.loadURDF(urdf_file, basePosition=[1.0, 0, 0.0], useFixedBase=True)
    arm = p.loadURDF(arm_file, useFixedBase=True, basePosition=[0.0, 0, 0.0])
    control_joints = get_controlable_joint(mb)

    for _ in range(1000):
        # for joint in control_joints:
        #     pos, vel, _, _ = p.getJointState(mb, joint)
        #     print(pos)
        #     p.setJointMotorControl2(mb, joint, p.POSITION_CONTROL, -1.57)
        for i in range(1000):
            p.stepSimulation()
    p.disconnect()

def density_of_object():
    object_density_list = []
    box_dict = {"name": "box", "body_density": 800, "handle_density": 800}
    object_density_list.append(box_dict)
    door_dict = {"name": "door", "body_density": 800, "handle_density": 2000}
    object_density_list.append(door_dict)
    microwave_dict = {"name": "microwave", "body_density": 2000, "handle_density": 2000}
    object_density_list.append(microwave_dict)
    dishwasher_dict = {"name": "dishwasher", "body_density": 7800, "handle_density": 7800}
    object_density_list.append(dishwasher_dict)
    drawer_dict = {"name": "drawer", "body_density": 800, "handle_density": 3000}
    object_density_list.append(drawer_dict)
    eyeglass_dict = {"name": "eyeglass", "body_density": 1300, "handle_density": 1300}
    object_density_list.append(eyeglass_dict)
    fan_dict = {"name": "fan", "body_density": 7800, "handle_density": 7800}
    object_density_list.append(fan_dict)
    foldingchair_dict = {"name": "foldingchair", "body_density": 800, "handle_density": 800}
    object_density_list.append(foldingchair_dict)
    laptop_dict = {"name": "laptop", "body_density": 1200, "handle_density": 1200}
    object_density_list.append(laptop_dict)
    oven_dict = {"name": "oven", "body_density": 7800, "handle_density": 7800}
    object_density_list.append(oven_dict)
    phone_dict = {"name": "phone", "body_density": 2000, "handle_density": 2000}
    object_density_list.append(phone_dict)
    plier_dict = {"name": "plier", "body_density": 7800, "handle_density": 7800}
    object_density_list.append(plier_dict)
    scissors_dict = {"name": "scissors", "body_density": 7800, "handle_density": 7800}
    object_density_list.append(scissors_dict)
    safe_dict = {"name": "safe", "body_density": 400, "handle_density": 400}
    object_density_list.append(safe_dict)
    stapler_dict = {"name": "stapler", "body_density": 3000, "handle_density": 3000}
    object_density_list.append(stapler_dict)
    table_dict = {"name": "table", "body_density": 800, "handle_density": 800}
    object_density_list.append(table_dict)
    washingmachine_dict = {"name": "washingmachine", "body_density": 2000, "handle_density": 1200}
    object_density_list.append(washingmachine_dict)
    washingmachine_dict = {"name": "refrigerator", "body_density": 2000, "handle_density": 1200}
    object_density_list.append(washingmachine_dict)
    return object_density_list

def urdf_mass(urdf_dir, save_dir, body_density, handle_density):
    """
        select handle
        """
    urdf_file = osp.join(urdf_dir, 'mobility_scaled.urdf')
    tree = ET.parse(urdf_file)
    root = tree.getroot()
    links_name = [link.attrib['name'] for link in root.findall('link')]
    print(links_name)
    # >>>>>>>>>>>>>> change original urdf mesh filename
    root_copy = copy.deepcopy(root)
    for link in root_copy.findall('link'):
        link_mass = 0
        print("*" * 100)
        inertial_mass = 0
        for visual in link.iter("visual"):
            for geometry in visual.iter('geometry'):
                manifold_file_name = "manifold-" + geometry[0].attrib['filename'].split('/')[1]
                trimesh.util.attach_to_log()
                sub_path = os.path.join(urdf_dir, "textured_objs")
                file_name = os.path.join(sub_path, manifold_file_name)
                if os.path.exists(file_name):
                    mesh = trimesh.load(file_name)
                    link_mass = link_mass + abs(mesh.volume)
                else:
                    print("file_name does not exist:", file_name)
                    # time.sleep(2)
                # print("file name", file_name)
        inertial_mass = link_mass * body_density
        if "handle" in link.attrib['name']:
            inertial_mass = link_mass * handle_density
        print("link_mass:", link_mass, inertial_mass)
        if inertial_mass > 20:
            inertial_mass = 20
        if inertial_mass < 0.01:
            inertial_mass = 5

        for inertial in link.iter("inertial"):
                for mass in inertial.iter("mass"):
                    mass.attrib["value"] = str(inertial_mass)
    tree = ET.ElementTree(root_copy)
    # save urdf without mass
    urdf_save_file = osp.join(save_dir, 'mobility_scaled_mass.urdf')
    tree.write(urdf_save_file, encoding='utf-8', xml_declaration=True)
    print("*"*100)
    print("file_name", file_name)

def urdf_joint_modify(urdf_dir):
    original_urdf_file = os.path.join(urdf_dir, 'mobility.urdf')
    tree = ET.parse(original_urdf_file)
    root = tree.getroot()
    links_name = [link.attrib['name'] for link in root.findall('link')]
    print(links_name)
    num = len(links_name)
    # >>>>>>>>>>>>>> change original urdf mesh filename
    root_copy = copy.deepcopy(root)
    joint_list = []
    for joint in root_copy.findall('joint'):
        if joint.attrib['type'] == "revolute" or joint.attrib['type'] == "prismatic":
            joint_list.append(joint)

    pybullet_scale_urdf = os.path.join(urdf_dir, 'mobility_scaled.urdf')
    tree = ET.parse(pybullet_scale_urdf)
    root = tree.getroot()
    links_name = [link.attrib['name'] for link in root.findall('link')]
    print(links_name)
    num = len(links_name)

    # >>>>>>>>>>>>>> change original urdf mesh filename

    root_copy = copy.deepcopy(root)
    for joint in root_copy.findall('joint'):
        if joint.attrib['type'] == "revolute":
            for original_joint in joint_list:
                if original_joint.get("name") == joint.get("name"):
                    for limit in original_joint.iter('limit'):
                        print(limit.attrib['lower'],  limit.attrib['upper'])
                        limit_new = Element("limit", {"lower": limit.attrib['lower'],
                                                      "upper": limit.attrib['upper'],
                                                      "effort": "10", "velocity": "10"})
                        joint.append(limit_new)
                    break
        else:
            joint.attrib['type'] = "fixed"
    print("*"*100, "joint")
    tree = ET.ElementTree(root_copy)
    # # save urdf without mass
    urdf_save_file = os.path.join(urdf_dir, 'mobility_scaled.urdf')
    tree.write(urdf_save_file, encoding='utf-8', xml_declaration=True)



def urdf_obj_path_modify(urdf_dir, prefix="model://"):
    urdf_file = os.path.join(urdf_dir, 'mobility_scaled.urdf')
    tree = ET.parse(urdf_file)
    root = tree.getroot()
    links_name = [link.attrib['name'] for link in root.findall('link')]
    print(links_name)
    num = len(links_name)
    # >>>>>>>>>>>>>> change original urdf mesh filename
    root_copy = copy.deepcopy(root)
    for link in root_copy.findall('link'):
        for visual in link.iter("visual"):
            for geometry in visual.iter('geometry'):
                for mesh in geometry.iter("mesh"):
                    file_name = mesh.attrib["filename"]
                    goal_str = "textured_objs"
                    mesh.attrib["filename"] = prefix + file_name[file_name.index(goal_str):]
        for collision in link.iter('collision'):
            for geometry in collision.iter('geometry'):
                for mesh in geometry.iter("mesh"):
                    file_name = mesh.attrib["filename"]
                    goal_str = "textured_objs"
                    mesh.attrib["filename"] = prefix + file_name[file_name.index(goal_str):]
    tree = ET.ElementTree(root_copy)
    # # save urdf without mass
    urdf_save_file = os.path.join(urdf_dir, 'mobility_scaled.urdf')
    tree.write(urdf_save_file, encoding='utf-8', xml_declaration=True)




def urdf_scale(urdf_dir, scale):
    urdf_file = os.path.join(urdf_dir, 'mobility.urdf')
    print(urdf_file)
    server_id = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -10)
    mb = p.loadURDF(urdf_file, globalScaling=scale,
                    useFixedBase=True)
    urdf_save_file = os.path.join(urdf_dir, 'mobility_scaled.urdf')
    parser = urdfEditor.UrdfEditor()
    parser.initializeFromBulletBody(mb, physicsClientId=server_id)
    parser.saveUrdf(urdf_save_file)
    p.disconnect()

if __name__ == '__main__':
    # path = "./better_mesh/microwave/002/mesh"
    package_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
    path = os.path.join(package_root, "models/cabinet_1076")
    scale = 0.8
    prefix = "model://cabinet_1076/"
    urdf_scale(path, scale)
    # urdf_joint_modify(path)
    urdf_obj_path_modify(path, prefix)


    density_list = density_of_object()
    body_density = 0
    handle_density = 0
    for density_dict in density_list:
        print("root_file:", path, "density_dict:", density_dict["name"])
        if density_dict["name"] in path:
            body_density = density_dict["body_density"]
            handle_density = density_dict["handle_density"]
            break
    urdf_mass(path, path, body_density, handle_density)
    # urdf_test(path)
    # file_list = os.listdir(path)
    # scale = 0.8
    # for i, file in enumerate(file_list):
    #     sub_path = os.path.join(path, file)
    #     urdf_scale(sub_path, scale)




