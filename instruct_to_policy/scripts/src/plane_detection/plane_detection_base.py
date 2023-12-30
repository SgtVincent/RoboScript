from typing import Dict, List, Tuple
import open3d as o3d
import numpy as np

class PlaneDetectionBase:
    def __init__(self, model_params: Dict):
        raise NotImplementedError

    def load_model(self):
        raise NotImplementedError

    def detect_planes(self, pcl_o3d: o3d.geometry.PointCloud)-> Tuple[List[np.ndarray], List[o3d.geometry.OrientedBoundingBox]]:
        raise NotImplementedError