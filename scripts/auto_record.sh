ROS_DISTRO=humble

ROS_SETUP_PATH=/opt/ros/$ROS_DISTRO/setup.bash
WORKSPACE_SETUP_PATH=$HOME/interbotix_ws/install/setup.bash
RECORD_EPISODES="$HOME/interbotix_ws/src/aloha/scripts/record_episodes.py"

source $ROS_SETUP_PATH || exit 1
source $WORKSPACE_SETUP_PATH || exit 1

print_usage() {
  echo "USAGE:"
  echo "auto_record.sh task num_episodes [-b, --enable_base_torque] [-g, --gravity_compensation]"
}

nargs="$#"

if [ $nargs -lt 2 ]; then
  echo "Passed incorrect number of arguments"
  print_usage
  exit 1
fi

if [ "$2" -lt 0 ]; then
  echo "# of episodes not valid"
  exit 1
fi

echo "Task: $1"
for (( i=0; i<$2; i++ ))
do
  echo "Starting episode $i"
  python3 "$RECORD_EPISODES" --task $1 $3 $4
  if [ $? -ne 0 ]; then
    echo "Failed to execute command. Returning"
    exit 1
  fi
done
