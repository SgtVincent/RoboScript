#!/usr/bin/env python
# Credit: Modified from https://github.com/gstavrinos/calc-inertia/blob/master/calc_inertia_for_urdf.py to support .obj mesh files
# Update: support .obj mesh fiels and multiple meshes in a single link

import os
import numpy as np
import sys
# add python root to path 
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
import xacro
import collada
import trimesh
# open obj mesh file 
import rospy 
import rosparam

from urdf_parser_py.urdf import URDF, Link, Mesh, Box, Sphere, Cylinder, Collision
import pysdf 
from pysdf.parse import Link as SDFLink, Collision as SDFCollision
from src.utils import getMeshBounds, getColladaBounds, getMeshDimensions, getColladaDimensions

# Based on https://en.wikipedia.org/wiki/List_of_moments_of_inertia#List_of_3D_inertia_tensors
def getLInkInertia_urdf(link: Link):
    print("  Link name:  " + link.name)
    print("  Mass:  " + str(link.inertial.mass))
    mass = link.inertial.mass
    xx = yy = zz = 1.0
    # parse ros package path function based on ROS version
    get_pkg_fn = None
    ROS_VERSION = os.getenv("ROS_VERSION")
    if not ROS_VERSION:
        print("Could not find the ROS_VERSION environment variable, thus, can't determine your ros version. Assuming ROS2!")
        ROS_VERSION = "2"
    if ROS_VERSION == "1":
        import rospkg
        get_pkg_fn = rospkg.RosPack().get_path
    else:
        import ament_index_python
        get_pkg_fn = ament_index_python.get_package_share_path
    
    if len(link.collisions) == 0:
        print("  No collision in this link!  ")
        return

    if len(link.collisions) > 1:
        print("  Calculating inertia for multiple collisions in a single link!  ")
        max_bound_list = []
        min_bound_list = []

        # simple strategy: sum up the bounding box of all the meshes
        for collision in link.collisions:
            collision: Collision
            pkg_tag = "package://"
            file_tag = "file://"
            model_tag = "model://"
            mesh_file = collision.filename

            if collision.geometry.filename.startswith(pkg_tag):
                package, mesh_file = collision.filename.split(pkg_tag)[1].split(os.sep, 1)
                print(get_pkg_fn(package))
                mesh_file = str(get_pkg_fn(package))+os.sep+mesh_file
            elif collision.geometry.filename.startswith(file_tag):
                mesh_file = collision.filename.replace(file_tag, "")
            elif collision.geometry.filename.startswith(model_tag):
                model_dir = os.getenv("GAZEBO_MODEL_PATH")
                mesh_file = collision.filename.replace(model_tag, model_dir+os.sep)
            # collision center relative to link center
            collision_center = collision.pose.xyz

            if mesh_file.endswith((".stl", ".obj")):
                model = trimesh.load_mesh(mesh_file)
                max_bound, min_bound = getMeshBounds(model, collision_center)
            # Assuming .dae
            else:
                model = collada.Collada(mesh_file)
                max_bound, min_bound = getColladaBounds(model, collision_center)

            # update the max and min bound list
            max_bound_list.append(max_bound)
            min_bound_list.append(min_bound)
        
        # calculate the overall bounding box
        max_bound = np.max(np.array(max_bound_list), axis=0)
        min_bound = np.min(np.array(min_bound_list), axis=0)
        x,y,z = max_bound - min_bound
        # scale does not work for complex mesh models 
        xx,yy,zz = getBoxInertia(x, y, z, mass, [1,1,1])

    else:     
        geometry = link.collision.geometry
        scale = geometry.scale
        if type(geometry) == Mesh:
            geometry: Mesh
            print("  Mesh:  " + geometry.filename)
            print("---\nCalculating inertia...\n---")
            pkg_tag = "package://"
            file_tag = "file://"
            model_tag = "model://"
            mesh_file = geometry.filename
            if geometry.filename.startswith(pkg_tag):
                package, mesh_file = geometry.filename.split(pkg_tag)[1].split(os.sep, 1)
                print(get_pkg_fn(package))
                mesh_file = str(get_pkg_fn(package))+os.sep+mesh_file
            elif geometry.filename.startswith(file_tag):
                mesh_file = geometry.filename.replace(file_tag, "")
            elif geometry.filename.startswith(model_tag):
                model_dir = os.getenv("GAZEBO_MODEL_PATH")
                mesh_file = geometry.filename.replace(model_tag, model_dir+os.sep)

            x = y = z = 0
            if mesh_file.endswith((".stl", ".obj")):
                model = trimesh.load_mesh(mesh_file)
                x,y,z = getMeshDimensions(model)
            # Assuming .dae
            else:
                model = collada.Collada(mesh_file)
                x,y,z = getColladaDimensions(model)
            xx,yy,zz = getBoxInertia(x, y, z, mass, scale)
        elif type(geometry) == Box:
            print("  Box:  " + str(geometry.size))
            print("---\nCalculating inertia...\n---")
            x,y,z = geometry.size
            xx,yy,zz = getBoxInertia(x, y, z, mass, scale)
        elif type(geometry) == Sphere:
            print("  Sphere Radius:  " + str(geometry.radius))
            print("---\nCalculating inertia...\n---")
            xx,yy,zz = getSphereInertia(geometry.radius, mass)
        elif type(geometry) == Cylinder:
            print("  Cylinder Radius and Length:  " + str(geometry.radius) + "," + str(geometry.length))
            print("---\nCalculating inertia...\n---")
            xx,yy,zz = getCylinderInertia(geometry.radius, geometry.length, mass)

    print(" ")
    print("<inertia  ixx=\"%s\" ixy=\"0\" ixz=\"0\" iyy=\"%s\" iyz=\"0\" izz=\"%s\" />" % (xx,yy,zz))
    print(" ")

def getLInkInertia_sdf(link: SDFLink):
    print("  Link name:  " + link.name)
    print("  Mass:  " + str(link.inertial.mass))
    mass = float(link.inertial.mass)
    xx = yy = zz = 1.0
    # parse ros package path function based on ROS version
    get_pkg_fn = None
    ROS_VERSION = os.getenv("ROS_VERSION")
    if not ROS_VERSION:
        print("Could not find the ROS_VERSION environment variable, thus, can't determine your ros version. Assuming ROS2!")
        ROS_VERSION = "2"
    if ROS_VERSION == "1":
        import rospkg
        get_pkg_fn = rospkg.RosPack().get_path
    else:
        import ament_index_python
        get_pkg_fn = ament_index_python.get_package_share_path
    
    if len(link.collisions) == 0:
        print("  No collision in this link!  ")
        return
    
    elif len(link.collisions) > 1:
        print("  Calculating inertia for multiple collisions in a single link!  ")
        max_bound_list = []
        min_bound_list = []

        # simple strategy: sum up the bounding box of all the meshes
        for collision in link.collisions:
            collision: SDFCollision
            pkg_tag = "package://"
            file_tag = "file://"
            model_tag = "model://"
            uri = collision.geometry_data['uri']
            mesh_file = uri
            if uri.startswith(pkg_tag):
                package, mesh_file = uri.split(pkg_tag)[1].split(os.sep, 1)
                print(get_pkg_fn(package))
                mesh_file = str(get_pkg_fn(package))+os.sep+mesh_file
            elif uri.startswith(file_tag):
                mesh_file = uri.replace(file_tag, "")
            elif uri.startswith(model_tag):
                model_dir = os.getenv("GAZEBO_MODEL_PATH")
                mesh_file = uri.replace(model_tag, model_dir+os.sep)

            # collision center relative to link center
            collision_center = collision.pose[:3, 3]

            if mesh_file.endswith((".stl", ".obj")):
                model = trimesh.load_mesh(mesh_file)
                max_bound, min_bound = getMeshBounds(model, collision_center)
            # Assuming .dae
            else:
                model = collada.Collada(mesh_file)
                max_bound, min_bound = getColladaBounds(model, collision_center)

            # update the max and min bound list
            max_bound_list.append(max_bound)
            min_bound_list.append(min_bound)
        
        # calculate the overall bounding box
        max_bound = np.max(np.array(max_bound_list), axis=0)
        min_bound = np.min(np.array(min_bound_list), axis=0)
        x,y,z = max_bound - min_bound
        # scale does not work for complex mesh models 
        xx,yy,zz = getBoxInertia(x, y, z, mass, [1,1,1])

    else:     
        geometry_data = link.collisions[0].geometry_data
        geometry_type = link.collisions[0].geometry_type
        uri = geometry_data['uri']
        # string to list of numbers
        scale = [float(i) for i in geometry_data['scale'].split()]
        if geometry_type == 'mesh':
            print("  Mesh:  " + uri)
            print("---\nCalculating inertia...\n---")
            pkg_tag = "package://"
            file_tag = "file://"
            model_tag = "model://"
            mesh_file = uri
            if uri.startswith(pkg_tag):
                package, mesh_file = uri.split(pkg_tag)[1].split(os.sep, 1)
                print(get_pkg_fn(package))
                mesh_file = str(get_pkg_fn(package))+os.sep+mesh_file
            elif uri.startswith(file_tag):
                mesh_file = uri.replace(file_tag, "")
            elif uri.startswith(model_tag):
                model_dir = os.getenv("GAZEBO_MODEL_PATH")
                mesh_file = uri.replace(model_tag, model_dir+os.sep)

            x = y = z = 0
            if mesh_file.endswith((".stl", ".obj")):
                model = trimesh.load_mesh(mesh_file)
                x,y,z = getMeshDimensions(model)
            # Assuming .dae
            else:
                model = collada.Collada(mesh_file)
                x,y,z = getColladaDimensions(model)
            xx,yy,zz = getBoxInertia(x, y, z, mass, scale)
        elif geometry_type == 'box':
            print("  Box:  " + str(geometry_data['size']))
            print("---\nCalculating inertia...\n---")
            x,y,z = geometry_data['size']
            xx,yy,zz = getBoxInertia(x, y, z, mass, scale)
        elif geometry_type == 'sphere':
            print("  Sphere Radius:  " + str(geometry_data['radius']))
            print("---\nCalculating inertia...\n---")
            xx,yy,zz = getSphereInertia(geometry_data['radius'], mass)
        elif geometry_type == 'cylinder':
            print("  Cylinder Radius and Length:  " + str(geometry_data['radius']) + "," + str(geometry_data['length']))
            print("---\nCalculating inertia...\n---")
            xx,yy,zz = getCylinderInertia(geometry_data['radius'], geometry_data['length'], mass)

    print(" ")
    print("<inertia  ixx=\"%s\" ixy=\"0\" ixz=\"0\" iyy=\"%s\" iyz=\"0\" izz=\"%s\" />" % (xx,yy,zz))
    print(" ")

def getBoxInertia(x, y, z, m, s):
    x *= s[0]
    y *= s[1]
    z *= s[2]
    xx = 1./12 * m * (y**2 + z**2)
    yy = 1./12 * m * (x**2 + z**2)
    zz = 1./12 * m * (x**2 + y**2)
    return xx, yy, zz

def getSphereInertia(r, m):
    i = 2./5 * m * r**2
    return i, i, i

def getCylinderInertia(r, h, m):
    xx = yy = 1./12 * m * (3 * r**2 + h**2)
    zz = 1./2 * m * r**2
    return xx, yy, zz

if __name__ == '__main__':
    # path = sys.argv[1]
    # robot = URDF.from_xml_string(xacro.process_file(sys.argv[1]).toprettyxml())
    # robot = URDF.from_xml_file(sys.argv[1])
    # path = "/home/junting/franka_ws/src/franka_fisher/instruct_to_policy/models/cabinet_1076/model.sdf"
    rospy.init_node("calc_inertia")
    path = rosparam.get_param("/calc_inertia/file_path")

    if path.endswith("xacro"):
        model = URDF.from_xml_string(xacro.process_file(path).toprettyxml())
        for link in model.links:
            getLInkInertia_urdf(link)
    elif path.endswith("urdf"):
        model = URDF.from_xml_file(path)
        for link in model.links:
            getLInkInertia_urdf(link)
    elif path.endswith("sdf"):
        sdf = pysdf.SDF(file=path)
        # assume only one model in the sdf file
        model = sdf.world.models[0]
        for link in model.links:
            getLInkInertia_sdf(link)
        


