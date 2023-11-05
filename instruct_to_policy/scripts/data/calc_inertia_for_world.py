"""
This script is used to process the raw world file saved from Gazebo.
- Instead of saving whole model definition, only save model handles
"""

# import lxml.etree as ET
import xml.etree.ElementTree as ET
import xml.dom.minidom as minidom
import numpy as np
from argparse import ArgumentParser
import re
import os
from add_intertial_tag_urdf import compute_intertial_link

package_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))
# set GAZEBO_MODEL_PATH to parse 'model://xxxxx' mesh paths
default_google_model_path = os.path.join(package_root, "data", "google_scanned_object", "models")
default_ycb_model_path = os.path.join(package_root, "data", "ycb", "models")
os.environ["GAZEBO_MODEL_PATH"] = f"{default_google_model_path}:{default_ycb_model_path}"


def parse_args():
    parser = ArgumentParser()
    parser.add_argument('--input_world', type=str, required=True)
    parser.add_argument('--output_world', type=str, required=True)
    parser.add_argument('--exclude_models', type=str, nargs='*', 
                        default=["sun", "ground_plane", "table", "cabinet", "panda", "triple_camera_set"])
    parser.add_argument('--density', type=float, default=10.0)
    parser.add_argument('--min_mass', type=float, default=0.1)
    
    return parser.parse_args()

if __name__ == '__main__':
    args = parse_args()
    debug = False
    # Parse the input XML file
    tree = ET.parse(args.input_world)
    root = tree.getroot()  

    # Find all 'model' tags under 'world' and get name, scales, mesh paths
    models = []
    scales = {}

    # add/replace pose, scale tags      
    for model in root.findall('world/model'):
        name = model.attrib['name']
        if name in args.exclude_models:
            continue
        
        # for each link in the model, compute the inertial tag
        for link in model.findall('.//link'):
            # calculate mass and inertia
            origin, mass, inertia, mesh = compute_intertial_link(
                link, use_visual_mesh=False, density=args.density, min_mass=args.min_mass)
            
            if debug:
                # visualize combined mesh
                link_name = link.attrib['name']
                mesh.export(f"{link_name}.obj")
            
            # remove existing inertial tag
            if link.find('inertial') is not None:
                link.remove(link.find('inertial'))
                
            # add tag to link
            """
            <inertial>
                <pose>-0.032543 0.011076 0.030041 2.04054 -0.742351 2.74464</pose>
                <mass>0.000192218</mass>
                <inertia>
                    <ixx>1.30711e-07</ixx>
                    <ixy>-5.06436e-23</ixy>
                    <ixz>-2.00641e-24</ixz>
                    <iyy>1.28085e-07</iyy>
                    <iyz>-2.01074e-24</iyz>
                    <izz>7.7209e-08</izz>
                </inertia>
            </inertial>
            """
            inertial_tag = ET.Element('inertial')
            link.insert(0, inertial_tag)
            pose_tag = ET.SubElement(inertial_tag, 'pose')
            pose_tag.text = " ".join([str(v) for v in origin] + ["0", "0", "0"])
            mass_tag = ET.SubElement(inertial_tag, 'mass')
            mass_tag.text = str(mass)
            inertia_tag = ET.SubElement(inertial_tag, 'inertia')
            # insert inertia matrix as subtags instead of attributes
            inertia_ixx_tag = ET.SubElement(inertia_tag, 'ixx')
            inertia_ixx_tag.text = str(inertia[0, 0])
            inertia_ixy_tag = ET.SubElement(inertia_tag, 'ixy')
            inertia_ixy_tag.text = str(inertia[0, 1])
            inertia_ixz_tag = ET.SubElement(inertia_tag, 'ixz')
            inertia_ixz_tag.text = str(inertia[0, 2])
            inertia_iyy_tag = ET.SubElement(inertia_tag, 'iyy')
            inertia_iyy_tag.text = str(inertia[1, 1])
            inertia_iyz_tag = ET.SubElement(inertia_tag, 'iyz')
            inertia_iyz_tag.text = str(inertia[1, 2])
            inertia_izz_tag = ET.SubElement(inertia_tag, 'izz')
            inertia_izz_tag.text = str(inertia[2, 2])


    # Write output XML with formant     
    xmlstr = minidom.parseString(ET.tostring(root)).toprettyxml(indent="  ")
    xmlstr = os.linesep.join([s for s in xmlstr.splitlines() if s.strip()])
    with open(args.output_world, "w") as f:
        f.write(xmlstr)
