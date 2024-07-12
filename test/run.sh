id=${1:3}
source devel/setup.bash
roslaunch robust_cl_real_world_server util_real.launch --ros-args -param num_robots:=${id}
