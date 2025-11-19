# CPS Final Project

## Install F1tenth simulator

1. First, create a virtual environment, as this code relies on older versions of common packages (e.g. Numpy)
2. `python3 -m venv ~/.venvs/f1_gym_env`
3. `echo "alias f1_env='source ~/.venvs/f1_gym_env/bin/activate'" >> ~/.bashrc`
4. `source ~/.bashrc`

You should now have a virtual environment that can be activated with the command `f1_env`. Invoke that command now, and look for the parenthetical at the command line that indicates it is active.

With the *virtual environment active*, 
1. git clone https://github.com/f1tenth/f1tenth_gym ~/Downloads/f1tenth_gym/
2. cd ~/Downloads/f1tenth_gym && git checkout dev-dynamics
3. pip3 install -e
4. pip3 install transforms3d

Step 2 ensures we are using a version that is compatible with Humble. Step 4 ensures that this library gets installed (it does not, natively, but it is necessary later).

## Installing F1tenth ROS bridge
1. `cd ~ && mkdir -p f1sim_ws/src`
2. `sudo apt-get update`
3. `sudo apt-get install python3-rosdep`
4. `cd ~/f1sim_ws/src`
5. `git clone https://github.com/burg54/f1tenth_gym_ros.git`
6. `source /opt/ros/humble/setup.bash`
7. `cd ..`
8. `rosdep install -i --from-path src --rosdistro humble -y`

## Building the Workspace
1. `cd ~/f1sim_ws`
2. `colcon build`
3. `source install/local_setup.bash`


## Running and testing the simulator
If everything has been built correctly and sourced, run
``



