num=${1:3}
source devel/setup.bash
roslaunch robust_cl_real_world_server util_real.launch num_robots_arg:=${num}
