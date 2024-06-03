# robust_cl_real_world_server
Code about RobustCL for real-world experiment(server part). Still editting...

# How to Deploy
1. Create the workspace
```bash
mkdir -p ~/CL_ws/src && cd ~/CL_ws/src
```
2. Go to workspace and process this repo:
```bash
# Install
git clone https://github.com/dc-cl/robust_cl_real_world_server

# Give execute authority
chmod -R +x ./robust_cl_real_world_server/scripts/

# Install necessary Python package
pip -r install ./robust_cl_real_world_server/scripts/requirements.txt

# Update to environment variable
echo "source ~/CL_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

2. Make
```bash
cd ..
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python
```

# Usage
```bash
rosrlaunch robust_cl_real_world_server util_real.launch
```
