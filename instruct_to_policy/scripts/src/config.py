
# from src.prompt.message_definitions import *
# from src.prompt.moveit_message_definitions import *
from src.prompt.moveit_cap_msgs import *
from src.constants import *

# model_name = 'code-davinci-002' # 'text-davinci-002' # deprecated in 2023
model_name = 'gpt-3.5-turbo-16k' # recommened replacement for 'code-davinci-002'

cfg_tabletop = {
  'env': {
      'coords': lmp_tabletop_coords, 
      'frame': 'world',
      'sim': 'gazebo',
      'interface': 'moveit',
      'verbose': False,
      'initial_joint_values': [0.0, -0.7854, 0.0, -2.3562, 0.0, 1.5708, 0.7854],
      'extra_objects': ["cabinet_1076::drawer_0", "cabinet_1076::drawer_1", "cabinet_1076::drawer_2", "cabinet_1076::drawer_3"]
  },
  'lmps': {
    'tabletop_ui': {
      'messages': message_tabletop_ui,
      'model': model_name,
      'max_tokens': 512,
      'temperature': 0,
      'query_prefix': '# ',
      'query_suffix': '.',
      'stop': ['#', 'objects = ['],
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
      'stop': ['#', 'objects = ['],
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
      'stop': ['#', 'objects = ['],
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
      'stop': ['#'],
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
      'stop': ['# define', '# example'],
      'maintain_session': False,
      'debug_mode': False,
      'include_context': True,
    }
  }
}

