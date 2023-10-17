# kill previous gazebo instances
killall -9 gzserver gzclient rviz move_group spwaner 

function cleanup {
  PID=$1
  kill -- -$(ps -o pgid= $PID | grep -o [0-9]*)
}

# run the collect_multiview_dataset function for table_cabinet_[0-100]
for i in {25..30}
do
  bash scripts/bash/run_single_multiview_detection.sh table_cabinet_$i
  sleep 5
done

# clean up any remaining gazebo instances
cleanup $$