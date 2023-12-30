import os 
import yaml
from src.prompt import *

def load_prompt_messages_into_config(config):
    
    lmps_config = config['lmps']
    
    # iterate through all lmps and load prompt messages
    for lmp_name, lmp_conig in lmps_config.items():
        messages_var_name = lmp_conig['messages']
        # load messages from src.prompt module
        messages = globals()[messages_var_name]
        # replace the messages in config
        config['lmps'][lmp_name]['messages'] = messages
        
def dict_update_recursive(base_dict, update_dict):
    '''
    Recursively update base_dict with update_dict. If a key in update_dict
    corresponds to a dictionary in both base_dict and update_dict, do a
    recursive update. Otherwise, update the value in base_dict.

    Args:
        base_dict (dict): The dictionary to update.
        update_dict (dict): The dictionary with updates.
    '''
    for key, value in update_dict.items():
        if key in base_dict and isinstance(base_dict[key], dict) and isinstance(value, dict):
            # If both values are dicts, recurse
            dict_update_recursive(base_dict[key], value)
        else:
            # If they are not both dicts, replace the base value
            base_dict[key] = value
        
def load_config_yaml_recursive(config_file):
    '''
    This function recursively loads the config files and merge it into the base config.
    '''
    # load current config
    config_file_path = os.path.join(os.path.dirname(__file__), config_file)
    with open(config_file_path, 'r') as f:
        config = yaml.safe_load(f) 
    
    # load the base config in the same folder if exists
    if 'base_config' in config:
        base_config_file = config['base_config']
        base_config_file_path = os.path.join(os.path.dirname(__file__), base_config_file)
        base_config = load_config_yaml_recursive(base_config_file_path)
        
        # merge the two configs: overwrite base config with current config
        dict_update_recursive(base_config, config)
        return base_config
    else:
        return config


def load_config(config_file):
    '''
    This function recursively loads the config files and merge it into the base config.
    ''' 
    # load task specific config
    config = load_config_yaml_recursive(config_file)
    
    # process the config with different functions 
    load_prompt_messages_into_config(config)

    return config
    
    
# quick test 
if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--config_file', type=str, default="perception_few_shot_gpt_3.5.yaml")
    args = parser.parse_args()
    config = load_config(args.config_file)
    print(config)

