# This bash script runs the code generation python script for 
echo "Running code generation script. Make sure you run this from the root of the instruct_to_policy package."

# define function to run the code generation script for table_cabinet_i.txt
function gen_code {
    config_file=$1
    task_query_file=$2
    output_dir=$3

    echo "Running code generation script for table_cabinet_$1"
    python3 scripts/data/generate_code.py \
            --config_file $config_file \
            --task-queries $task_query_file \
            --output-dir $output_dir
}

config_files=(
    # "perception_few_shot_gpt_3.yaml"  
    # "perception_few_shot_gpt_4.yaml"
    # "perception_few_shot_codellama.yaml"
    # "perception_few_shot_llama2_chat.yaml"
    # "perception_few_shot_gemini.yaml"
    # "text_few_shot_gpt_3.yaml"
    # "text_few_shot_gpt_4.yaml"
    # "text_few_shot_codellama.yaml"
    # "text_few_shot_llama2_chat.yaml"
    # "text_few_shot_gemini.yaml"
    "text_gpt_3.yaml"
    "text_gpt_4.yaml"
    "text_codellama.yaml"
    "text_llama2_chat.yaml"
    "text_gemini.yaml"
)

task_query_files=(
    "data/benchmark/task_queries/world_1_table_sort.txt"
    "data/benchmark/task_queries/world_2_pick_and_place.txt"
    "data/benchmark/task_queries/world_3_mug_to_empty_plate.txt"
    "data/benchmark/task_queries/world_4_clear_table_to_basket.txt"
    "data/benchmark/task_queries/world_5_mug_to_cabinet.txt"
    "data/benchmark/task_queries/world_6_mug_to_same_color_plate.txt"
)

output_dir="data/benchmark/generated_code"

# run the code generation script for each pair of (config_file, task_query_file)
for config_file in "${config_files[@]}"
do
    for task_query_file in "${task_query_files[@]}"
    do
        echo "----------------------------------------------"
        echo "Running command: gen_code $config_file $task_query_file $output_dir"
        
        start_time=$(date +%s.%N)  # get start time
        
        gen_code $config_file $task_query_file $output_dir
        
        end_time=$(date +%s.%N)  # get end time
        duration=$(echo "$end_time - $start_time" | bc)  # calculate duration
        
        echo "Time taken: $duration seconds"
        echo "----------------------------------------------"

    done
done
 
