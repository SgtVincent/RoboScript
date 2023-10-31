import os 
# get package root from current file path
package_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))

from src.prompt.moveit_cap_msgs import *
from src.constants import *


# model_name = 'code-davinci-002' # 'text-davinci-002' # deprecated in 2023
model_name = 'gpt-3.5-turbo-16k' # recommened replacement for 'code-davinci-002'
# model_name = 'gpt-4'

cfg_tabletop = {
  'env': {
      'coords': lmp_tabletop_coords, 
      'frame': 'world',
      'sim': 'gazebo',
      'interface': 'moveit',
      'verbose': False,
      'initial_joint_values': [0.0, -0.7854, 0.0, -2.3562, 0.0, 1.5708, 0.7854],
      'extra_objects': ["cabinet.drawer0", "cabinet.drawer1", "cabinet.drawer2", "cabinet.drawer3"],
      'sensor': {
        'namespace': '', # empty namespace by default 
        'cameras': ['camera_left', 'camera_right', 'camera_top'],
        'gt_point_cloud': False,
      },
      'moveit_config': {
        'debug': True,
        'planning_time': 15,
        'max_velocity': 0.2,
        'max_acceleration': 0.2,
        'goal_position_tolerance': 0.001,
        'goal_orientation_tolerance': 0.02,
        'refenrence_frame': 'world',
        'cartesian_path': True,
      },
      'metadata_files':[
        os.path.join(package_root, 'data', 'ycb', 'metadata.json'),
        os.path.join(package_root, 'data', 'google_scanned_object', 'object_metadata.json'),
        os.path.join(package_root, 'data', 'google_scanned_object', 'container_metadata.json'),
      ]
  },
  'grasp_detection': {
      'method': 'model', # ['heuristic', 'model']
      'model_params': {
        'service_name': '/detect_grasps',
      },
      'visualize': False,
      'verbose': False,
  },
  'lmps': {
    'tabletop_ui': {
      'messages': message_tabletop_ui,
      'model': model_name,
      'max_tokens': 512,
      'temperature': 0,
      'query_prefix': '# ',
      'query_suffix': '.',
      # 'stop': ['#', 'objects = ['],
      'maintain_session': True,
      'debug_mode': False,
      'include_context': True,
      'has_return': False,
      'return_val_name': 'ret_val',
    },
    'parse_obj_name': {
      'messages': message_parse_obj_name,
      'model': model_name,
      'max_tokens': 512,
      'temperature': 0,
      'query_prefix': '# ',
      'query_suffix': '.',
      # 'stop': ['#', 'objects = ['],
      'maintain_session': False,
      'debug_mode': False,
      'include_context': True,
      'has_return': True,
      'return_val_name': 'ret_val',
    },
    # 'parse_position': {
    #   'messages': message_parse_position,
    #   'model': model_name,
    #   'max_tokens': 512,
    #   'temperature': 0,
    #   'query_prefix': '# ',
    #   'query_suffix': '.',
    #   'stop': ['#'],
    #   'maintain_session': False,
    #   'debug_mode': False,
    #   'include_context': True,
    #   'has_return': True,
    #   'return_val_name': 'ret_val',
    # },
    'parse_question': {
      'messages': message_parse_question,
      'model': model_name,
      'max_tokens': 512,
      'temperature': 0,
      'query_prefix': '# ',
      'query_suffix': '.',
      # 'stop': ['#', 'objects = ['],
      'maintain_session': False,
      'debug_mode': False,
      'include_context': True,
      'has_return': True,
      'return_val_name': 'ret_val',
    },
    'transform_shape_pts': {
      'messages': message_transform_shape_pts,
      'model': model_name,
      'max_tokens': 512,
      'temperature': 0,
      'query_prefix': '# ',
      'query_suffix': '.',
      # 'stop': ['#'],
      'maintain_session': False,
      'debug_mode': False,
      'include_context': True,
      'has_return': True,
      'return_val_name': 'new_shape_pts',
    },
    'fgen': {
      'messages': message_fgen,
      'model': model_name,
      'max_tokens': 512,
      'temperature': 0,
      'query_prefix': '# define function: ',
      'query_suffix': '.',
      # 'stop': ['# define', '# example'],
      'maintain_session': False,
      'debug_mode': False,
      'include_context': True,
    }
  }
}

