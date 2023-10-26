"""
This script is used to process the raw world file saved from Gazebo.
- Instead of saving whole model definition, only save model handles
"""

import lxml.etree as ET
import numpy as np
from argparse import ArgumentParser
import re
import os
package_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))
default_base_world = os.path.join(package_root, "worlds", "table_cabinet_base.world")

def parse_args():
    parser = ArgumentParser()
    parser.add_argument('--input_world', type=str, required=True)
    parser.add_argument('--output_world', type=str, required=True)
    parser.add_argument('--base_world', type=str, default=default_base_world)
    return parser.parse_args()

if __name__ == '__main__':
    args = parse_args()
    # Parse the input XML file
    tree = ET.parse(args.input_world)
    root = tree.getroot()  

    # Find all 'model' tags under 'world/state' and get name, pose, scales
    models = []
    poses = {}
    scales = {}
    for model in root.find('world/state').iter('model'):
        name = model.attrib['name']
        pose = model.find('pose').text
        scale = model.find('scale').text
        models.append(name)
        poses[name] = pose
        scales[name] = scale

    # Find all model URIs    
    uris = {}    
    for model in root.find('world').iter('model'):
        name = model.attrib['name']
        uri_tag = model.find('link/collision/geometry/mesh/uri')
        # all custom models have a link with name "link"
        # if not found, then it should be a environment model, skip it 
        if uri_tag is not None:
            uri = uri_tag.text
            # get root folder of uri (e.g. 'model://013_apple/google_16k/textured.obj' to 'model://013_apple')
            # use regex

            # first pattern
            pattern = r"^(model://[^/]+)/"
            match = re.search(pattern, uri)
            if match is not None:
                root_uri = match.group(1)
                uris[name] = root_uri
            else:
                # try second pattern
                pattern = r"/models/([^/]+)/"
                match = re.search(pattern, uri)
                if match is not None:
                    model_id = match.group(1)
                    root_uri = "model://" + model_id
                    uris[name] = root_uri
                else:
                    print("Error: cannot find root uri for uri {}".format(uri))
  
    # Parse base XML file and write with indentations
    parser = ET.XMLParser(remove_blank_text=True)
    out_tree = ET.parse(args.base_world, parser)
    out_root = out_tree.getroot()
    sdf_tag = out_root.find('world')

    # Add include tags to base XML
    # the uri, name, pose and scale should be added as subtags of include tag 
    for name in models:
        pose = poses[name]
        scale = scales[name]
        if name in uris:
            # print("Adding model {} to world".format(name))
            uri = uris[name]
            include_tag = ET.SubElement(sdf_tag, 'include')
            ET.SubElement(include_tag, 'name').text = name
            ET.SubElement(include_tag, 'uri').text = uri
            ET.SubElement(include_tag, 'pose').text = pose
            ET.SubElement(include_tag, 'scale').text = scale     

    # Write output XML        
    out_tree.write(args.output_world, pretty_print=True)