"""
Read the JSON files in the current directory and create a CSV file with the results.
"""
from typing import List, Dict, Tuple
import pandas as pd
import json
import glob
import argparse 
import numpy as np 

def fill_df_from_json_files(args, df: pd.DataFrame, json_files: List[str], queries_label_list: List[str]):
        # Iterate over the list of json files to fill the DataFrame
    for i, json_file in enumerate(json_files):
        # Open the JSON file and load the data
        with open(json_file, 'r') as f:
            data: List[Dict] = json.load(f)

        # Iterate over the query list 
        for j, query_result in enumerate(data):
            # Each 'query' becomes a column in the DataFrame.
            query_label = queries_label_list[j]
            # TODO: get query index from eval_items file 
            # Aggregate the results of the repeated trials
            grammer_correctness_list = []
            finished_steps_over_repeat_trials = []
            finished_whole_task_over_repeat_trials = []
            
            if 'repeat_times' not in query_result:
                for k in range(args.repeat_times):
                    grammer_correctness_list.append(0)
                    finished_steps_over_repeat_trials.append(0)
                    finished_whole_task_over_repeat_trials.append(0)
            else:
            
                for k in range(query_result['repeat_times']):
                    # if this repeat trial is empty, consider all metrics as fail 
                    if str(k) not in query_result:
                        grammer_correctness_list.append(0)
                        finished_steps_over_repeat_trials.append(0)
                        finished_whole_task_over_repeat_trials.append(0)
                    else:
                        grammer_correctness_list.append(query_result[str(k)]['grammer_correctness'])
                        eval_items_results = query_result[str(k)]['eval_items_results']
                        finished_steps_over_repeat_trials.append(np.mean(eval_items_results))
                        finished_whole_task_over_repeat_trials.append(np.all(eval_items_results).astype(int))
             
            # Fill the DataFrame with the aggregated results
            df.loc[(args.configs_to_eval[i], 'grammer_correctness'), query_label] = np.any(grammer_correctness_list)
            df.loc[(args.configs_to_eval[i], 'semantic_correctness'), query_label] = np.any(finished_whole_task_over_repeat_trials)
            df.loc[(args.configs_to_eval[i], 'finished_steps_ratio'), query_label] = np.mean(finished_steps_over_repeat_trials)
            df.loc[(args.configs_to_eval[i], 'finished_whole_task'), query_label] = np.mean(finished_whole_task_over_repeat_trials)

def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--eval_results_dir', type=str, default='data/benchmark/eval_results')
    parser.add_argument('--repeat_times', type=int, default=5)
    parser.add_argument('--worlds_list', nargs='+', default=[
                                                            # "world_1_table_sort",
                                                            "world_1_table_sort",
                                                            "world_2_pick_and_place",
                                                            "world_3_mug_to_empty_plate",
                                                            "world_4_clear_table_to_basket",
                                                            "world_5_mug_to_cabinet",
                                                            "world_6_mug_to_same_color_plate",
                                                            ])
    parser.add_argument('--configs_to_eval', nargs='+', type=str, 
                        default=[    
                                "text_gpt_3",
                                "text_gpt_4",
                                # "text_codellama",
                                # "text_llama2_chat",
                                "text_gemini",
                                # "text_few_shot_gpt_3",
                                # "text_few_shot_gpt_4",
                                # "text_few_shot_codellama",
                                # "text_few_shot_llama2_chat",
                                # "text_few_shot_gemini",
                                ])
    # output csv file
    parser.add_argument('--output_csv', type=str, default='results.csv')
    args = parser.parse_args()
    print("Please run this scripts under ./instruct_to_policy")
    return args

if __name__ == '__main__':

    args = parse_args()
    metrics = ['grammer_correctness', 'semantic_correctness',  'finished_steps_ratio', 'finished_whole_task']
    
    df_list = []
    for world_name in args.worlds_list:
        # Get the list of JSON files
        json_files = []
        # Read queries and eval_items from eval_items_file
        eval_items_dir = args.eval_results_dir.replace('eval_results', 'eval_items')
        eval_items_file = f'{eval_items_dir}/{world_name}_eval_items.json'
        with open(eval_items_file, 'r') as f:
            query_items_data: List[Dict] = json.load(f)
        queries_dict = {i: query_eval_items['query'] for i, query_eval_items in enumerate(query_items_data)}
        queries_label_list = [query_eval_items['query_label'] for query_eval_items in query_items_data]
        num_eval_items_list = [len(query_eval_items['eval_items']) for query_eval_items in query_items_data]
        
        # Create a multi-index for rows
        row_index = pd.MultiIndex.from_product([args.configs_to_eval, metrics], names=['config_to_eval', 'metric'])
        # Create a index for columns
        col_index = pd.Index(queries_label_list, name='query_index')
        # Create a DataFrame with the multi-index
        world_df = pd.DataFrame(index=row_index, columns=col_index)
        # fill the world_df with nan by default 
        world_df = world_df.fillna(np.nan)
        
        # for each config_to_eval, get the json file with latest time stamp
        for config_to_eval in args.configs_to_eval:
            # for each config_to_eval, get the json file with latest time stamp
            json_file_list = glob.glob(f'{args.eval_results_dir}/{config_to_eval}/{world_name}_*.json')
            json_file_list.sort()
            
            # if no json file found, skip this config_to_eval
            if len(json_file_list) == 0:
                continue
            
            latest_json_file = json_file_list[-1]
            # json_file = f'{args.eval_results_dir}/{config_to_eval}/{world_name}.json'
            json_files.append(latest_json_file)
            
        fill_df_from_json_files(args, world_df, json_files, queries_label_list)
            
        df_list.append(world_df)
    
    # Save the DataFrame to a CSV file
    df = pd.concat(df_list, axis=1)
    
    # Sort the columns from simple_0, simple_1 to hard_0, ... hard_5 exactly
    columns = ['simple_0', 'simple_1', 'hard_0', 'hard_1', 'hard_2', 'hard_3', 'hard_4', 'hard_5']
    df = df[columns]
    
    df.to_csv(args.output_csv)
    print(df)
    print(f"Saved results to {args.output_csv}")
                     
                    