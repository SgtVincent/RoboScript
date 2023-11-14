#!/usr/bin/env python3
"""
This script is to query chatgpt to generate possible tasks with a robotic arm with gripper in a specific gazebo world. 
"""

import os
import numpy as np
import json
import argparse
from typing import List, Dict, Tuple
import openai
# add parent directory to path
import sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.prompt.message_tabletop_ui import message_tabletop_ui as prompt_messages
from src.openai_api_key import OPENAI_API_KEY


query_base_messages = [
    {
        "role": "system", 
        "content": 
'''You are a creative assistant. You will be provided with a world description and a set of example task queries in other worlds. 
You need to generate plausible task queries based on the world description and example task queries. 
You can generate similar queries as example queries, but you are encouraged to generate more diverse queries. 
All objects on the table are visible and are given in the world description.
There might also be invisible objects in cabinet or drawers, you can also generate queries to explore them.
'''
    }
]

def generate_world_description(json_file):
    """
    Generate a description of the world based on the metadata json file.
    """
    with open(json_file) as f:
        data = json.load(f)

    description = ""

    # get all objects in the world
    objects = [value['name'] for key, value in data.items()]
    description += f"The objects in the world are: objects = [{', '.join(objects)}].\n"
    
    # add description of the robot arm
    description += "The panda robot arm is on the table, near the cabinet.\n"
    
    # describe the location of each object
    for key, value in data.items():
        if 'objects_on' in value and len(value['objects_on']) > 0:
            object_names_on = [data[object_id]['name'] for object_id in value['objects_on']]
            description += f"{', '.join(object_names_on)} are on {value['name']}.\n"
        elif 'objects_in' in value and len(value['objects_in']) > 0:
            object_names_in = [data[object_id]['name'] for object_id in value['objects_in']]
            description += f"{', '.join(object_names_in)} are in {value['name']}.\n"

    return description, objects

def collect_example_queries_from_prompt(prompt)-> List:
    """
    Collect example queries from the prompt.
    The prompt is in message format from openai chat completion API: https://platform.openai.com/docs/guides/gpt/chat-completions-api
    """
    example_queries = []
    for message in prompt:
        if message['role'] == 'user':
            example_queries.append(message['content'].replace('\n', ";"))
    return example_queries


def generate_task_queries(args, world_metadata_file, num_queries=100):
    """
    Generate tasks queries based on the world description.
    """
    world_description, objects = generate_world_description(world_metadata_file)
    example_queries = collect_example_queries_from_prompt(prompt_messages)

    world_description_message = {
        "role": "user",
        "content": world_description
    }

    example_queries_message = {
            "role": "user",
            "content": 
"""
Here are some example task queries, you can generate similar queries or more diverse queries. Each line is an example task query.
'''
""" + 
'\n'.join(example_queries) + 
"""
'''
"""
    }

    objects_string = ', '.join(objects).strip()
    query_message = {
        "role": "user",
        "content": f"Now please generate {num_queries} task queries based on the world description and example task queries." +
                    "You can combine multiple objects or multiple actions in the query." +
                    f"In each line of your response, you should strictly copy string 'objects = [{objects_string}] ;' and concatenate it with your generated query" 
    }


    messages = query_base_messages + [world_description_message, example_queries_message, query_message]
    
    # call openai chat completion API to generate task queries
    response = openai.ChatCompletion.create(
        model=args.model,
        messages=messages,
        temperature=args.temperature,
        max_tokens=args.max_tokens,
    )

    content = response["choices"][0]["message"]["content"].strip()
    
    return content
    


if __name__ == '__main__':

    # parse arguments
    parser = argparse.ArgumentParser()
    parser.add_argument('--model', type=str, default='gpt-3.5-turbo-16k')
    parser.add_argument('--temperature', type=float, default=1.0)
    parser.add_argument('--max_tokens', type=int, default=15000)
    parser.add_argument('--num_queries', type=int, default=100)
    parser.add_argument('--world_metadata_file', type=str, default='data/world_metadata/table_cabinet_0.json')
    parser.add_argument('--output_dir', type=str, default='data/task_queries')
    args = parser.parse_args()
    
    # configure openai api key
    openai.api_key = OPENAI_API_KEY

    # use chatgpt to generate task queries
    queries_gen = generate_task_queries(args, args.world_metadata_file, args.num_queries)

    # save generated task queries to text file
    if not os.path.exists(args.output_dir):
        os.makedirs(args.output_dir)

    output_file = os.path.join(args.output_dir, os.path.basename(args.world_metadata_file).replace('.json', '.txt'))
    with open(output_file, 'w') as f:
        f.write(queries_gen)


    # # test function 
    # json_file = 'data/world_metadata/table_cabinet_0.json'
    # description, _ = generate_world_description(json_file)
    # # print(description)
    # example_queries = collect_example_queries_from_prompt(prompt_messages)
    # print(example_queries)

    