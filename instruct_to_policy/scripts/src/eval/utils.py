import numpy as np 
from scipy.spatial.transform import Rotation as R

################## Bounding box utils ##################

def calc_3d_bbox_iob1(bbox_1:np.array, bbox_2:np.array):
    '''
    Calculate the 3D bounding box intersection over bbox_1
    Args:
        bbox_1: (6,) np.array [x1, y1, z1, x2, y2, z2]
        bbox_2: (6,) np.array [x1, y1, z1, x2, y2, z2]
    '''

    # Calculate the coordinates of the intersection of two bounding boxes
    x1 = max(bbox_1[0], bbox_2[0])
    y1 = max(bbox_1[1], bbox_2[1])
    z1 = max(bbox_1[2], bbox_2[2])
    x2 = min(bbox_1[3], bbox_2[3])
    y2 = min(bbox_1[4], bbox_2[4])
    z2 = min(bbox_1[5], bbox_2[5])

    # Calculate the dimensions of the intersection
    width = max(0, x2 - x1)
    height = max(0, y2 - y1)
    depth = max(0, z2 - z1)

    # Calculate the volume of the intersection
    vol_intersection = width * height * depth

    # Calculate the volume of bounding box 1
    vol_bbox1 = (bbox_1[3] - bbox_1[0]) * (bbox_1[4] - bbox_1[1]) * (bbox_1[5] - bbox_1[2])

    # Calculate the intersection over bbox_1
    iob1 = vol_intersection / vol_bbox1 if vol_bbox1 != 0 else 0

    return iob1 

    
def calc_2d_bbox_iob1(bbox_1:np.array, bbox_2:np.array):
    '''
    Calculate the 2D bounding box intersection over bbox_1
    Args:
        bbox_1: (4,) np.array [x1, y1, x2, y2]
        bbox_2: (4,) np.array [x1, y1, x2, y2]
    '''

    # Calculate the coordinates of the intersection of two bounding boxes
    x1 = max(bbox_1[0], bbox_2[0])
    y1 = max(bbox_1[1], bbox_2[1])
    x2 = min(bbox_1[2], bbox_2[2])
    y2 = min(bbox_1[3], bbox_2[3])

    # Calculate the dimensions of the intersection
    width = max(0, x2 - x1)
    height = max(0, y2 - y1)

    # Calculate the area of the intersection
    area_intersection = width * height

    # Calculate the area of bounding box 1
    area_bbox1 = (bbox_1[2] - bbox_1[0]) * (bbox_1[3] - bbox_1[1])
    
    # Calculate the intersection over bbox_1
    iob1 = area_intersection / area_bbox1 if area_bbox1 != 0 else 0

    return iob1
     
    
##################### Pose utils ######################

def check_if_standing(quat:np.array, **kwargs):
    '''
    Check if the object is standing.
    
    If the object is standing, the z axis of the object's orientation should be close to 1 or -1.
    
    Args:
        quat: (4,) np.array [qx, qy, qz, qw]
    '''
    # project_threshold: the threshold of the dot product of the z axis of the object's orientation and the z axis of the world frame
    project_threshold = kwargs.get('project_threshold', 0.8)
    
    # z axis of the object's orientation
    z_proj = R.from_quat(quat).as_matrix()[:, 2]
    
    # z axis of the world frame
    z_axis_world = np.array([0, 0, 1])
    
    is_standing = abs( np.dot(z_proj, z_axis_world) ) > project_threshold
    
    return is_standing
     
     
    
    