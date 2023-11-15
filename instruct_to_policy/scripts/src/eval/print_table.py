"""
Read the JSON files in the current directory and create a CSV file with the results.
"""
from typing import List, Dict, Tuple
import pandas as pd
import json
import glob
import argparse 
import numpy as np 


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--eval_results_dir', type=str, default='data/benchmark/eval_results')
    parser.add_argument('--world_name', type=str, default='world_1_table_sort')
    parser.add_argument('--code_to_eval_list', nargs='+', type=str, 
                        default=['generated_code_gpt3', 
                                 'generated_code_gpt3_few_shot', 
                                 'generated_code_gpt4'
                                 ])
    # output csv file
    parser.add_argument('--output_csv', type=str, default='results.csv')
    args = parser.parse_args()
    print("Please run this scripts under ./instruct_to_policy")
    return args

if __name__ == '__main__':

    args = parse_args()

    # Get the list of JSON files
    json_files = []
    for code_to_eval in args.code_to_eval_list:
        json_file = f'{args.eval_results_dir}/{code_to_eval}/{args.world_name}.json'
        json_files.append(json_file)

    metrics = ['grammer_correctness', 'finished_steps_ratio', 'finished_whole_task']
    
    # Read queries from eval_items file
    eval_items_dir = args.eval_results_dir.replace('eval_results', 'eval_items')
    eval_items_file = f'{eval_items_dir}/{args.world_name}_eval_items.json'
    with open(eval_items_file, 'r') as f:
        data: List[Dict] = json.load(f)
    queries_dict = {i: query_eval_items['query'] for i, query_eval_items in enumerate(data)}
    
    # Create a single sdf with multi-index 
    # Each row is indexed by code_to_eval and metric name  
    # Each column is indexed by query index ONLY

    # Create a multi-index for rows
    row_index = pd.MultiIndex.from_product([args.code_to_eval_list, metrics], 
                                           names=['code_to_eval', 'metric'])
    # Create a index for columns
    queries_index_list = list(range(len(queries_dict)))
    col_index = pd.Index(queries_index_list, name='query_index')
    
    # Create a DataFrame with the multi-index
    df = pd.DataFrame(index=row_index, columns=col_index)
    print(df)
    
    # Iterate over the list of json files to fill the DataFrame
    for i, json_file in enumerate(json_files):
        # Open the JSON file and load the data
        with open(json_file, 'r') as f:
            data: List[Dict] = json.load(f)

        # Iterate over the query list 
        for query_result in data:
            # Each 'query' becomes a column in the DataFrame.
            query_index = query_result['query_index']
            # TODO: get query index from eval_items file 
            # Aggregate the results of the repeated trials
            grammer_correctness_list = []
            eval_items_results_over_repeat_trials = []
            for j in range(query_result['repeat_times']):
                # if this repeat trial is empty, consider all metrics as fail 
                if str(j) not in query_result:
                    grammer_correctness_list.append(0)
                    eval_items_results_over_repeat_trials.append(0)
                else:
                    grammer_correctness_list.append(query_result[str(j)]['grammer_correctness'])
                    eval_items_results_over_repeat_trials.append(query_result[str(j)]['eval_items_results'])