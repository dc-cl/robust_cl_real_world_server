num=${1}
if [ -z "$num" ]; then
    num=3
fi
source devel/setup.bash
roslaunch robust_cl_real_world_server util_real.launch num_robots_arg:=${num}
