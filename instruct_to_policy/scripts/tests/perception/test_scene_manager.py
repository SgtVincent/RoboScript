import pytest
import os 
import sys
# add catkin_ws/devel/lib into sys.path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../../../../../../devel/lib/python3.9/site-packages'))

from src.perception.scene_manager import SceneManager
from src.perception.utils import (
    CameraIntrinsic,
    Transform,
    load_data_from_multiview_detection
)

DEBUG_VISUALIZE = True

# initialize the scene manager
@pytest.fixture(scope="module")
def scene_manager(): 
    scene_manager = SceneManager()
    return scene_manager

# initialize the test data
@pytest.fixture(scope="module")
def multiview_data():
    world_name = "world_6_mug_to_same_color_plate"
    exclude_keywords = ["cabinet"]
    
    current_dir = os.path.dirname(os.path.realpath(__file__))
    data_dir = os.path.join(current_dir, f"../../../data/benchmark/multiview_detection")

    multiview_data = load_data_from_multiview_detection(
        data_dir=data_dir,
        world_name=world_name,
        exclude_keywords=exclude_keywords
    )
    
    return multiview_data
    
@pytest.mark.dependency()
def test_update_fusion(scene_manager, multiview_data):
    scene_manager.update_fusion(multiview_data)
    
@pytest.mark.dependency(depends=["test_update_fusion"])
def test_update_object_instances(scene_manager, multiview_data):
    scene_manager.update_object_instances(multiview_data)

@pytest.mark.dependency(depends=["test_update_object_instances"])
def test_update_3d_bboxes(scene_manager, multiview_data):
    scene_manager.update_3d_bboxes(multiview_data)
    
    if DEBUG_VISUALIZE:
        scene_manager.visualize_3d_bboxes()
