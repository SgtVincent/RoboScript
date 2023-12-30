from typing import Dict
import open3d as o3d

class PlaneDetectionBase:
    def __init__(self, model_params: Dict):
        raise NotImplementedError

    def load_model(self):
        raise NotImplementedError

    def detect_planes(self, pcl_o3d: o3d.geometry.PointCloud):
        raise NotImplementedError