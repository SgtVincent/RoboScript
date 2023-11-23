# kill previous gazebo instances
killall -9 gzserver gzclient rviz move_group spwaner 

function cleanup {
  PID=$1
  kill -- -$(ps -o pgid= $PID | grep -o [0-9]*)
}

# run the collect_multiview_dataset function for table_cabinet_[0-100]
# for i in {25..30}
# do
#   bash scripts/bash/run_single_multiview_detection.sh table_cabinet_$i
#   sleep 5
# done

world_dir=$(rospack find instruct_to_policy)/data/benchmark/worlds
out_dir=$(rospack find instruct_to_policy)/data/benchmark/multiview_detection

# for all the worlds under data/benchmark/worlds
for world in $(ls $world_dir)
do 
  # get full path to world
  world_path=$(realpath $world_dir/$world)
  bash scripts/bash/run_single_multiview_detection.sh $world_path $out_dir
  sleep 5
done

# clean up any remaining gazebo instances
cleanup $$