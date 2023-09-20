# This bash script runs the query generation python script 
echo "Running query generation script. Make sure you run this from the root of the instruct_to_policy package."

# define function to run the query generation script for table_cabinet_i.json
function gen_query {
    echo "Running code generation script for table_cabinet_$1.json"
    python3 scripts/data/generate_queries_from_world.py \
        --world_metadata_file data/world_metadata/table_cabinet_$1.json \
        --output_dir data/task_queries \
        --num_queries 100
}

# run the query generation script for table_cabinet_i.json for i in {0..9}
# for i in {0..99}
# do
#     gen_query $i
# done

