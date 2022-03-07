# setup instructions

## Install ROS
These instructions are for Ros Noetic on Ubuntu 20.04.

Setup keys, update apt, and install ros desktop:
```
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    sudo apt install curl # if you haven't already installed curl
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
    sudo apt update
    sudo apt install ros-noetic-desktop-full
    echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
    source ~/.bashrc
```

Now we need to install some additional dependencies, and rosdep.
```
    sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
    sudo rosdep init
    rosdep update
    sudo sh     -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" \
         > /etc/apt/sources.list.d/ros-latest.list'
    wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
    sudo apt-get update
    sudo apt-get install python3-catkin-tools
```

Now we create our ros workspace.
```
    mkdir catkin_ws
    cd catkin_ws/
    catkin init --workspace .
    catkin build
```

There may be some red errors from catkin build which inform us that our directory does not have any packages in it yet.

## Install PX4 Sim

We need more dependencies:
```
    sudo apt install software-properties-common apt-transport-https wget
    sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential -y
    sudo apt-get install python-catkin-tools python-rosinstall-generator -y
    rosinstall_generator --rosdistro kinetic mavlink | tee /tmp/mavros.rosinstall
    rosinstall_generator --upstream mavros | tee -a /tmp/mavros.rosinstall
    sudo apt install ros-noetic-mavros
    sudo apt-get install python3-rosdep
```

The final set of dependencies:
```
   rosdep install --from-paths src --ignore-src -r -y
   sudo apt install geographiclib-tools -y
   install_geo=$(wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh -O -)
   catkin build
   sudo apt-get install protobuf-compiler libeigen3-dev libopencv-dev -y
```

Note that we do not clone the PX4-autopilot in catkin/src as it is not really a ROS package.

Now we can download and install the PX4 firmware simulator:
```
    git clone https://github.com/PX4/PX4-Autopilot.git --recursive
    bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
```

We can now build and run the example:
```
    cd PX4-Autopilot/
    make px4_sitl gazebo
```

```make clean``` May be needed if build fails, this will clean previously built code.

If everything went well we should see Gazebo opening with a UAV on the ground.

If there are problems with connecting to a specific port try this:
```
DONT_RUN=1 make px4_sitl_default gazebo
source ~/catkin_ws/devel/setup.bash    # (optional)
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo
roslaunch px4 mavros_posix_sitl.launch
```

# Mavros
Download mavros from github into ```catkin_ws/src/``` and build workspace with ```catkin build```.

launch mavros using ```roslaunch mavros px4.launch```.

Check everything is ok by subscribing to a topic. Note not all topics are used, but either of the following should be active: ```/mavros/local_position/pose``` or ```/mavros/imu/data_raw```.

# Take off, land, go places

In ```src``` folder there are python scripts for ```fake_mocap.py``` and to control pose.


ARM: ```rosservice call /mavros/cmd/arming "value: true" ```
Publish target pose: ```python3 /home/seb/Documents/uav_workshop/dummy_pose_publisher.py```

```rosservice call /mavros/set_mode "base_mode: 0
custom_mode: 'OFFBOARD'" ```

If you get an error message in the PX4 sim, that looks a bit like: try to set param COM_RCL_EXCEPT to 4 in QGroundControl.


# Realsense

Download github repo:
git@github.com:IntelRealSense/realsense-ros.git

Make sure you use a good quality cable, and some laptops may have issues with supplying enough power through their USB ports.
