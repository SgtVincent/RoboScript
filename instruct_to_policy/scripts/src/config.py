import os 
# get package root from current file path
package_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))
from src.constants import *

# from src.prompt.message_tabletop_ui import message_tabletop_ui
from src.prompt.message_tabletop_few_shot import message_tabletop_ui
# from src.prompt.message_tabletop_zero_shot import message_tabletop_ui

# from src.prompt.message_fgen import message_fgen
from src.prompt.message_fgen_few_shot import message_fgen

from src.prompt.message_parse_question import message_parse_question
from src.prompt.message_backup import message_parse_obj_name, message_transform_shape_pts

model_name = 'gpt-3.5-turbo-16k' # recommened replacement for 'code-davinci-002'
# model_name = 'gpt-4'

cfg_tabletop = {
  'env': {
      'coords': lmp_tabletop_coords, 
      'frame': 'world',
      'sim': 'gazebo',
      'interface': 'moveit',
      'verbose': False,
      'extra_objects': ["cabinet.drawer0", "cabinet.drawer1", "cabinet.drawer2", "cabinet.drawer3", 
                        "cabinet.handle_0", "cabinet.handle_1", "cabinet.handle_2", "cabinet.handle_3"],
      'sensor': {
        'namespace': '', # empty namespace by default 
        'cameras': ['camera_left', 'camera_right', 'camera_top'],
        'gt_point_cloud': False,
      },
      'moveit_config': {
        'arm_group_name': 'panda_arm',
        'gripper_group_name': 'panda_hand',
        'manipulator_group_name': 'panda_manipulator',
        'debug': True,
        'planning_time': 15,
        'max_velocity': 0.2,
        'max_acceleration': 0.2,
        'goal_position_tolerance': 0.001,
        'goal_orientation_tolerance': 0.02,
        'refenrence_frame': 'world',
        'cartesian_path': True,
        'initial_joint_values': [0.0, -0.7854, 0.0, -2.3562, 0.0, 1.5708, 0.7854],
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
  'grounding_model': {
    'model_name': 'glip',
    'model_args': {}
  },
  'lmps': {
    'tabletop_ui': {
      'messages': message_tabletop_ui,
      'model': model_name,
      'max_tokens': 2048,
      'temperature': 0,
      'query_prefix': '# ',
      'query_suffix': '.',
      # 'stop': ['#', 'objects = ['],
      'maintain_session': False, # Otherwise model remembers previous history, e.g. defined functions  
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

