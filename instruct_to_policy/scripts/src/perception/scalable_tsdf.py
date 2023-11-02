import numpy as np
import open3d as o3d
from typing import Tuple, List, Dict

from .utils import Transform

class ScalableTSDFVolume(object):
    """Integration of multiple depth images using a TSDF."""

    def __init__(self, voxel_size, color_type=None, trunc_ratio=4.0):
        self.voxel_size = voxel_size
        self.sdf_trunc = trunc_ratio * self.voxel_size

        if color_type is None:
            color = o3d.pipelines.integration.TSDFVolumeColorType.NoColor
        elif color_type == "rgb":
            color = o3d.pipelines.integration.TSDFVolumeColorType.RGB8
        else:
            raise ValueError("Unknown color type: {}".format(color_type))

        self._volume = o3d.pipelines.integration.ScalableTSDFVolume(
            voxel_length=self.voxel_size,
            sdf_trunc=self.sdf_trunc,
            color_type=color,
        )


    def integrate(self, depth_img, intrinsic, extrinsic, rgb_img=None, mask_img=None):
        """
        Args:
            depth_img: The depth image.
            intrinsic: The intrinsic parameters of a pinhole camera model.
            extrinsics: The transform from the TSDF to camera coordinates, T_eye_task.
        """
        if rgb_img is None:
            rgb_img = np.zeros([depth_img.shape[0], depth_img.shape[1], 3], dtype=np.uint8)
        
        # mask out the points out of the region of interest
        if mask_img is not None:
            depth_img[mask_img == 0] = np.inf
        
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
            o3d.geometry.Image(rgb_img),
            o3d.geometry.Image(depth_img),
            depth_scale=1.0,
            depth_trunc=3.0,
            convert_rgb_to_intensity=False,
        )

        intrinsic = o3d.camera.PinholeCameraIntrinsic(
            width=intrinsic.width,
            height=intrinsic.height,
            fx=intrinsic.fx,
            fy=intrinsic.fy,
            cx=intrinsic.cx,
            cy=intrinsic.cy,
        )

        extrinsic = extrinsic.as_matrix()

        self._volume.integrate(rgbd, intrinsic, extrinsic)

    def get_region_of_interest(self, center, size)-> Tuple[o3d.geometry.PointCloud, o3d.geometry.TriangleMesh]:
        """Set the region of interest for the TSDF volume to generate voxel grid as network input.

        Args:
            center: The center of the region of interest.
            size: The size of the region of interest.
        Returns:
            cropped_cloud: The cropped point cloud.
            cropped_mesh: The cropped mesh.
        """

        # create data for cropped region
        cropped_cloud = self.crop_cloud(center, size)
        cropped_mesh = self.crop_mesh(center, size)
        
        return cropped_cloud, cropped_mesh

    def get_mesh(self):
        mesh = self._volume.extract_triangle_mesh()
        return mesh
    
    def crop_mesh(self, crop_center, crop_size):
        mesh = self._volume.extract_triangle_mesh()
        # crop mesh
        bounding_box = o3d.geometry.AxisAlignedBoundingBox(
            min_bound=crop_center - crop_size / 2,
            max_bound=crop_center + crop_size / 2,
        )
        cropped_mesh = mesh.crop(bounding_box)
        return cropped_mesh

    def get_cloud(self):
        point_cloud = self._volume.extract_point_cloud()
        return point_cloud


    def crop_cloud(self, crop_center, crop_size):
        point_cloud = self._volume.extract_point_cloud()
        # crop point cloud 
        bounding_box = o3d.geometry.AxisAlignedBoundingBox(
            min_bound=crop_center - crop_size / 2,
            max_bound=crop_center + crop_size / 2,
        )
        cropped_point_cloud = point_cloud.crop(bounding_box)
        return cropped_point_cloud
