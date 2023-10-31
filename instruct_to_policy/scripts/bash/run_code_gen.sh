# This bash script runs the code generation python script for 
echo "Running code generation script. Make sure you run this from the root of the instruct_to_policy package."

# define function to run the code generation script for table_cabinet_i.txt
function gen_code {
    echo "Running code generation script for table_cabinet_$1"
    python3 scripts/data/generate_code.py \
        --task-queries data/task_queries/table_cabinet_$1.txt \
        --output-dir data/generated_code 
}

# run the qcodeuery generation script for table_cabinet_i.txt for i in {0..99}
# for i in {0..99}
# do
#     gen_code $i
# done

