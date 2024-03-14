eval "$(command conda 'shell.bash' 'hook' 2> /dev/null)"

source $HOME/miniconda3/etc/profile.d/conda.sh || exit 1

conda deactivate || exit 1

conda activate aloha || exit 1
source /opt/ros/$ROS_DISTRO/setup.bash || exit 1
source $HOME/interbotix_ws1/devel/setup.bash || exit 1 ## TODO

which pip
pip list | grep dm || exit 1

if [ "$2" -lt 0 ]; then
  echo "# of episodes not valid"
  exit
fi

echo "Task: $1"
for (( i=0; i<$2; i++ ))
do
  echo "Starting episode $i"
  rosrun aloha record_episodes.py --task "$1"
  if [ $? -ne 0 ]; then
    echo "Failed to execute command. Returning"
    exit
  fi
done
