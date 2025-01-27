# DARPA Triage Project

## Description
The **DARPA Triage** project is a set of tools for autonomous drone control, including takeoff, navigation, and landing capabilities. It integrates with **MAVROS** for communication with the drone.

## Features
- **Autonomous Takeoff**
- **Navigation to Target Altitudes**
- **Drone State Monitoring**

## Prerequisites
- **Ubuntu 20.04**
- **ROS Noetic**
- **MAVROS**
- **Ardupilot**
- **Python 3.8**

## Installation Instructions

### Create Directory
1. Create a directory for the project.
```bash
mkdir dev
cd dev
mkdir DarpaTriage
cd DarpaTriage
```
### Clone Ardupilot
2. Clone the Ardupilot repository.
```bash
git clone --recurse-submodules [GitHub](https://github.com/aiba4531/ardupilot)
cd ardupilot
```

### Install prerequisites packages and dependencies for Ardupilot
3. Install the prerequisites packages and dependencies for Ardupilot.
```bash
Tools/environment_install/install-prereqs-ubuntu.sh -y
source ~/.profile
echo “source /home/aidanbagley/dev/DarpaTriage/ardupilot/Tools/completion/completion.bash" >> ~/.bashrc
source ~/.bashrc
```

### Build Ardupilot Software in the Loop (SITL)
4. Build Ardupilot.
```bash
./waf configure --board sitl
./waf copter
```

### Run Ardupilot Software in the Loop (SITL)
5. Run Ardupilot with the Gazebo Iris Simulator outputting to a TCP port.
```bash
sim_vehicle.py -v ArduCopter -f gazebo-iris --console --out=tcp:127.0.0.1:5760
```

### Install ROS Noetic
6. Install ROS Noetic.
```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update 
sudo apt install ros-noetic-desktop-full
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Install ROS Dependencies
7. Install ROS dependencies.
```bash
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool 
```
8. Initialize rosdep.
```bash
sudo rosdep init
rosdep update
```

### Install Python Catkin Tools
9. Install Python Catkin Tools.
```bash
sudo apt-get install python3-catkin-pkg python3-catkin-tools
```

### Create a ROS (Catkin) Workspace
10. Create a ROS (Catkin) workspace.
```bash
mkdir catkin_ws
cd catkin_ws
mkdir src
cd src
```
11. Initialize the workspace.
```bash
catkin_init_workspace
```
12. Build the workspace.
```bash
cd catkin_ws
catkin_make
```
12  Source the workspace.
```bash
echo "source /home/aidanbagley/dev/DarpaTriage/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Clone Gazebo ROS Packages
13. Clone the Gazebo ROS packages.
```bash
cd src
git clone https://github.com/ros-simulation/gazebo_ros_pkgs.git -b noetic-devel
rosdep update
```
14. Check for missing dependencies. This should return no errors.
```bash
rosdep check --from-paths . --ignore-src --rosdistro noetic
```

### Remake the Workspace
15. Remake the workspace.
```bash
cd catkin_ws
catkin_make
```

### If you have any issues with the above steps, please refer to the [Ardupilot](https://ardupilot.org/dev/docs/building-setup-linux.html) and [ROS](http://wiki.ros.org/noetic/Installation/Ubuntu) documentation.
### If a CC1 error occurs, try running the following commands to create a swap file for compilation.:
```bash
sudo fallocate -l 8G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
catkin_make
```

### Install MAVRos
16. Install MAVROS.
```bash
sudo apt-get install ros-noetic-mavros ros-noetic-mavros-extras
catkin_make
```
17. Source the workspace again.
```bash
echo "source /home/aidanbagley/dev/DarpaTriage/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Install Geographic Tools
18. Install Geographic Tools.
```bash
sudo apt update
sudo apt install geographiclib-tools
sudo geographiclib-get-geoids egm96-5
```

### Remake the Workspace
19. Remake the workspace.
```bash
cd catkin_ws
catkin_make
```

## Try to run a simple example
20. Terminal 1: Run the Ardupilot SITL.
```bash
sim_vehicle.py -v ArduCopter -f gazebo-iris --console --out=127.0.0.1:14550 --out=127.0.0.1:14551
```
21. Terminal 2: Run Gazebo Iris Arducopter Demo World.
```bash
gazebo --verbose worlds/iris_arducopter_demo.world
```
If you see the Iris drone in the Gazebo world, you have successfully installed the necessary software.
If you see an error like Main Loop Frequnecy too slow, try running the following commands:
```bash
sudo nano /usr/share/gazebo-11/worlds/iris_arducopter_demo
```
Change the following line from:
```bash
<max_step_size>0.0025</max_step_size>
```
To:
```bash
<max_step_size>0.00025</max_step_size>
```
Then rerun the following command:
```bash
gazebo --verbose worlds/iris_arducopter_demo.world
```
22. Terminal 3: Run MAVProxy Ground Station
```bash
mavproxy.py --master=127.0.0.1:14550 --out=udp:127.0.0.1:14552
```
23. Terminal 4: Run MAVROS
```bash
roslaunch mavros apm.launch fcu_url:=udp://127.0.0.1:14551@14551
```
24. Terminal 5: List the ROS Topics
```bash
rostopic list
rostopic echo /mavros/state
```
You should see a list of potential topics and the state of the drone. If it says connected = True, you have successfully connected to the drone.
25. Try running the following command to arm the drone in the SITL terminal:
```bash
mode guided
arm throttle
takeoff 5
```
This should show the drone taking off to 5 meters in gazebo.

## Create a ROS Launch Script to enable autonomous takeoff and landing
26. Create a autonomous control script in the catkin_ws/src directory.
```bash
cd catkin_ws/src
catkin_create_pkg autonomous_drone rospy mavros mavros_msgs geometry_msgs
cd autonomous_drone
mkdir scripts
cd scripts
touch autonomous_control.py
chmod +x autonomous_control.py
```
See the example file inside the autonomous_drone/scripts directory for an example of how to create a simple autonomous control script.
27. Edit CMakeLists.txt to include the Python script by adding this to the end of the file:
```bash
# Install Python scripts
catkin_install_python(PROGRAMS
  scripts/autonomous_control.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
28. Create a launch file to run the script.
```bash
cd autonomous_drone
mkdir launch
cd launch
touch autonomous_control.launch
```
29. Add the following to the launch file:
```bash
<launch>
  <node pkg="autonomous_drone" type="autonomous_control.py" name="autonomous_control" output="screen" />
  <include file="$(find mavros)/launch/apm.launch">
    <arg name="fcu_url" value="udp://127.0.0.1:14551@14551" />
  </include>
</launch>
```
30. Remake the workspace.
```bash
cd catkin_ws
catkin_make
source ~/.bashrc
```

## Run the Autonomous Control Script
31. Terminal 1: Run the Ardupilot SITL.
```bash
sim_vehicle.py -v ArduCopter -f gazebo-iris --console --out=127.0.0.1:14550 --out=127.0.0.1:14551
```
32. Terminal 2: Run Gazebo Iris Arducopter Demo World.
```bash
gazebo --verbose worlds/iris_arducopter_demo.world
```
33. Terminal 3: Run MAVProxy Ground Station
```bash
mavproxy.py --master=127.0.0.1:14550 --out=udp:127.0.0.1:14552
```
34. Terminal 4: Run Autonomous Control Launch Script
```bash
roslaunch autonomous_drone autonomous_control.launch
```
Before running the Launch script, make sure GCS says pre-armed check is good. Afterwards you should see the drone autonomously take off and land!