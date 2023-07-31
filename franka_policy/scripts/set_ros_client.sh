# Sets up fields for ros client setup.
# Must be run with the source command.
# Assumes the eth0 interface to be present
export ROS_MASTER_URI=http://172.31.154.42:11311
export ROS_IP=$(/sbin/ip -o -4 addr list eno1: | awk '{print $4}' | cut -d/ -f1)
