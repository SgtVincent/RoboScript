import ast
import astunparse
import numpy as np 
from typing import List, Dict, Any, Tuple, Optional
import shapely
from shapely.geometry import *
from shapely.affinity import *

############### Code execution utils #####################

def var_exists(name, all_vars):
    try:
        eval(name, all_vars)
    except:
        exists = False
    else:
        exists = True
    return exists


def merge_dicts(dicts):
    return {
        k : v 
        for d in dicts
        for k, v in d.items()
    }
    
def prepare_vars(env):
    """Prepare variables including APIs and objects for LMPs """
    fixed_vars = {"np": np}
    fixed_vars.update(
        {name: eval(name) for name in shapely.geometry.__all__ + shapely.affinity.__all__}
    )
    variable_vars = {
        k: getattr(env, k)
        for k in [
            "get_3d_bbox",
            "get_object_center_position",
            "get_obj_name_list",
            "get_end_effector_pose",
            "open_gripper",
            "close_gripper",
            "move_to_pose",
            "move_joints_to",
            "detect_new_objects",
            "attach_object",
            "detach_object"
            # DO NOT use mid-level skills?
            # "grasp",
            # "place"
        ]
    }
    variable_vars["say"] = lambda msg: print(f"robot says: {msg}")
    
    # add moveit interfaces to variables
    variable_vars.update(
        {
            k: getattr(env, k)
            for k in [
                "move_group",
                "gripper_group",
            ]
        }
    )

    return fixed_vars, variable_vars


def exec_safe(code_str, gvars=None, lvars=None):
    banned_phrases = ['import', '__']
    for phrase in banned_phrases:
        assert phrase not in code_str
  
    if gvars is None:
        gvars = {}
    if lvars is None:
        lvars = {}
    empty_fn = lambda *args, **kwargs: None
    custom_gvars = merge_dicts([
        gvars,
        {'exec': empty_fn, 'eval': empty_fn}
    ])
    # print(code_str)
    exec(code_str, custom_gvars, lvars)



class FunctionParser(ast.NodeTransformer):

    def __init__(self, fs, f_assigns):
      super().__init__()
      self._fs = fs
      self._f_assigns = f_assigns

    def visit_Call(self, node):
        self.generic_visit(node)
        if isinstance(node.func, ast.Name):
            f_sig = astunparse.unparse(node).strip()
            f_name = astunparse.unparse(node.func).strip()
            self._fs[f_name] = f_sig
        return node

    def visit_Assign(self, node):
        self.generic_visit(node)
        if isinstance(node.value, ast.Call):
            assign_str = astunparse.unparse(node).strip()
            f_name = astunparse.unparse(node.value.func).strip()
            self._f_assigns[f_name] = assign_str
        return node




##################################################
# Text parsing utils 
##############################################

def has_keywords(text: str, keywords: List[str]) -> bool:
    """ Check if text has any of the keywords."""
    return any([kw in text.lower() for kw in keywords])



#################################################################
# mesh utils 
#################################################################
def getMeshBounds(mesh, center=np.array([0,0,0])):
    """Returns the bounds of .obj/.mlt mesh"""
    return mesh.bounds[1, :] + center, mesh.bounds[0, :] + center

def getColladaBounds(model, center=np.array([0,0,0])):
    """Returns the bounds of .dae mesh"""
    minx = miny = minz = float("inf")
    maxx = maxy = maxz = float("-inf")
    for tr_vertex in model.geometries[0].primitives[0].vertex[model.geometries[0].primitives[0].vertex_index]:
        for v in tr_vertex:
            maxx = maxx if v[0] <= maxx else v[0]
            maxy = maxy if v[1] <= maxy else v[1]
            maxz = maxz if v[2] <= maxz else v[2]
            minx = minx if v[0] >= minx else v[0]
            miny = miny if v[1] >= miny else v[1]
            minz = minz if v[2] >= minz else v[2]
    return np.array([maxx, maxy, maxz]) + center, np.array([minx, miny, minz]) + center

def getMeshCenter(mesh):
    """Returns the center of .obj/.mlt mesh"""
    return (mesh.bounds[1, :] + mesh.bounds[0, :]) / 2

def getColladaCenter(model):
    """Returns the center of .dae mesh"""
    minx = miny = minz = float("inf")
    maxx = maxy = maxz = float("-inf")
    for tr_vertex in model.geometries[0].primitives[0].vertex[model.geometries[0].primitives[0].vertex_index]:
        for v in tr_vertex:
            maxx = maxx if v[0] <= maxx else v[0]
            maxy = maxy if v[1] <= maxy else v[1]
            maxz = maxz if v[2] <= maxz else v[2]
            minx = minx if v[0] >= minx else v[0]
            miny = miny if v[1] >= miny else v[1]
            minz = minz if v[2] >= minz else v[2]
    return (np.array([maxx, maxy, maxz]) + np.array([minx, miny, minz])) / 2


def getMeshDimensions(mesh):
    """Returns the dimensions of .obj/.mlt mesh"""
    return mesh.bounds[1, :] - mesh.bounds[0, :]

def getColladaDimensions(model):
    """Returns the dimensions of .dae mesh"""
    minx = miny = minz = float("inf")
    maxx = maxy = maxz = float("-inf")
    for tr_vertex in model.geometries[0].primitives[0].vertex[model.geometries[0].primitives[0].vertex_index]:
        for v in tr_vertex:
            maxx = maxx if v[0] <= maxx else v[0]
            maxy = maxy if v[1] <= maxy else v[1]
            maxz = maxz if v[2] <= maxz else v[2]
            minx = minx if v[0] >= minx else v[0]
            miny = miny if v[1] >= miny else v[1]
            minz = minz if v[2] >= minz else v[2]
    return maxx - minx, maxy - miny, maxz - minz


##################################################
# Pose utils
##################################################

def get_mirrored_pose_to_plane(original_pose_matrix, mirror_normal, mirror_point):
    """Returns the mirrored pose of the original pose across the mirror plane"""

    # Display the original pose transformation matrix
    print("Original Pose Transformation Matrix:")
    print(original_pose_matrix)

    # Function to create a reflection transformation matrix for a given mirror plane
    def reflection_matrix(mirror_normal, mirror_point):
        d = mirror_point
        n = mirror_normal
        # Reflected rotation can be computed by: 
        # Subtract twice this matrix from the identity matrix (I) to obtain the reflection matrix (R):
        # R = I - 2 * N * N^T
        reflection = np.eye(4)
        reflection[:3, :3] = np.eye(3) - 2 * np.outer(n, n)
        reflection[:3, 3] = 2 * np.dot(d, n) * n
        return reflection

    # Create the reflection transformation matrix
    reflection_transform = reflection_matrix(mirror_normal, mirror_point)

    # Apply the reflection transformation to the original pose transformation matrix
    mirrored_pose_matrix = np.dot(reflection_transform, original_pose_matrix)

    print("\nMirrored Pose Transformation Matrix:")
    print(mirrored_pose_matrix)



