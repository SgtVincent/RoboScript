import os
import numpy as np
import trimesh
import xml.etree.ElementTree as ET

def scale_pose(pose: ET.Element, scale_factor):

    if pose is not None:
        pose_values = list(map(float, pose.text.split()))
        scaled_values = [v * scale_factor for v in pose_values[:3]] + pose_values[3:]
        pose.text = ' '.join(map(str, scaled_values))

def scale_mesh(mesh: ET.Element, scale_factor):
    if mesh is not None:
        scale = mesh.find('scale')
        if scale is not None:
            scale_values = list(map(float, scale.text.split()))
            scaled_values = [v * scale_factor for v in scale_values]
            scale.text = ' '.join(map(str, scaled_values))

def scale_joint(joint: ET.Element, scale_factor):
    # Assuming that the joint parameters are in the form of <axis>, <limit>, etc.
    # You would need to adjust the scaling based on the actual joint parameters in your SDF
    for limit in joint.findall('axis/limit'):
        lower = limit.find('lower')
        upper = limit.find('upper')
        velocity = limit.find('velocity')
        
        if lower is not None:
            lower.text = str(float(lower.text) * scale_factor)
        if upper is not None:
            upper.text = str(float(upper.text) * scale_factor)
        if velocity is not None:
            velocity.text = str(float(velocity.text) * scale_factor)
             

def scale_sdf_model(file_path, scale_factor):
    tree = ET.parse(file_path)
    root = tree.getroot()

    # Scale all pose tags
    for pose in root.iter('pose'):
        scale_pose(pose, scale_factor)

    # Change mesh scale
    for mesh in root.iter('mesh'):
        scale_mesh(mesh, scale_factor)

    # Scale joint parameters
    for joint in root.iter('joint'):
        scale_joint(joint, scale_factor)

    # Write the modified tree back to the file
    tree.write(file_path)

if __name__ == "__main__": 


    package_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    path = os.path.join(package_root, "models/cabinet_1076")
    scale = 0.8
    # prefix = "model://cabinet_1076/"
    scale_sdf_model(os.path.join(path, "model.sdf"), scale)