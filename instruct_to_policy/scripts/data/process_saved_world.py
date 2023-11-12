"""
This script is used to process the raw world file saved from Gazebo.
- Instead of saving whole model definition, only save model handles
"""

# import lxml.etree as ET
import xml.etree.ElementTree as ET
import xml.dom.minidom as minidom
import numpy as np
from argparse import ArgumentParser
import json
import os
package_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))
default_base_world = os.path.join(package_root, "worlds", "table_cabinet_base.world")
default_ycb_metadata_file = os.path.join(package_root, "data", "ycb", "metadata.json")
default_google_container_metadata_file = os.path.join(package_root, "data", "google_scanned_object", "container_metadata.json")
default_google_object_metadata_file = os.path.join(package_root, "data", "google_scanned_object", "object_metadata.json")


def parse_args():
    parser = ArgumentParser()
    parser.add_argument('--input_world', type=str, required=True)
    parser.add_argument('--output_world', type=str, required=True)
    parser.add_argument('--base_world', type=str, default=default_base_world)
    
    parser.add_argument('--ycb_metadata_file', type=str, default=default_ycb_metadata_file)
    parser.add_argument('--google_container_metadata_file', type=str, default=default_google_container_metadata_file)
    parser.add_argument('--google_object_metadata_file', type=str, default=default_google_object_metadata_file)

    
    parser.add_argument('--exclude_models', type=str, nargs='*', 
                        default=["sun", "table", "ground_plane", "panda", "triple_camera_set", "cabinet"])
    return parser.parse_args()

if __name__ == '__main__':
    args = parse_args()
    
    # Parse metadata files as dictionaries to get mapping from raw model id to model_name 
    with open(args.ycb_metadata_file, 'r') as f:
        ycb_metadata = json.load(f)
    with open(args.google_container_metadata_file, 'r') as f:
        google_container_metadata = json.load(f)
    with open(args.google_object_metadata_file, 'r') as f:
        google_object_metadata = json.load(f)
    # Get model name from raw model id
    raw_model_id_to_model_name = {}
    for model_id, model_info in ycb_metadata['objects'].items():
        raw_model_id_to_model_name[model_id] = model_info['model_name']
    for model_id, model_info in google_container_metadata['objects'].items():
        raw_model_id_to_model_name[model_id] = model_info['model_name']
    for model_id, model_info in google_object_metadata['objects'].items():
        raw_model_id_to_model_name[model_id] = model_info['model_name']
        
    # Parse the input XML file
    tree = ET.parse(args.input_world)
    root = tree.getroot()  

    # Parse base XML file and write with indentations
    out_tree = ET.parse(args.base_world)
    out_root = out_tree.getroot()
    out_world_tag = out_root.find('world')

    # Find all 'model' tags under 'world/state' and get name, pose, scales
    models = []
    poses = {}
    scales = {}
    for model in root.findall('world/state/model'):
        name = model.attrib['name']
        if name in args.exclude_models:
            continue
        
        pose = model.find('pose').text
        scale = model.find('scale').text
        models.append(name)
        poses[name] = pose
        scales[name] = scale   

    # Copy model tags from input XML to output XML and 
    # add/replace pose, scale tags      
    for model in root.findall('world/model'):
        name = model.attrib['name']
        if name in args.exclude_models:
            continue
        # change model name in model tag to model_name
        if name in raw_model_id_to_model_name:
            model.attrib['name'] = raw_model_id_to_model_name[name]
        
        # Copy model tag
        out_world_tag.append(model)
        # replace pose tag if exists, otherwise add pose tag
        pose_tag = model.find('pose')
        if pose_tag is None:
            # pose_tag = ET.SubElement(model, 'pose')
            pose_tag = ET.Element('pose')
            model.insert(0, pose_tag)
        pose_tag.text = poses[name]

        # NOTE: the scale should be inserted in <mesh> tag instead of <model> tag
        # replace scale tag if exists, otherwise add scale tag
        for mesh_tag in model.findall('.//geometry/mesh'):
            scale_tag = mesh_tag.find('scale')
            if scale_tag is None:
                # scale_tag = ET.SubElement(mesh_tag, 'scale')
                scale_tag = ET.Element('scale')
                mesh_tag.insert(0, scale_tag)
                
            scale_tag.text = scales[name]

    # Write output XML with formant     
    xmlstr = minidom.parseString(ET.tostring(out_root)).toprettyxml(indent="  ")
    xmlstr = os.linesep.join([s for s in xmlstr.splitlines() if s.strip()])
    with open(args.output_world, "w") as f:
        f.write(xmlstr)
