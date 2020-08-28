# PeTRA Central Control

# List of components

* ROS2 Dashing
    * petra_central_control (1.0.0)
    * petra_core (1.0.0)
    * petra_drivers (1.0.0)
    * petra_services (1.0.0)
    * behaviortree_cpp_v3 (3.1.1)
    * ros1_bridge (0.7.5)
* ROS1 Melodic
    * petra_stop_bridge (1.0.0)
    * neo_simulation (0.1.0)
    * neo_driver (multiple)
    * cob_scan_unifier (0.7.3)
    * Groot (1.0.0)
    * gazebo_ros (2.8.6)
    * rviz (1.13.9)
    * navigation (1.16.6)
    * openslam_gmapping (0.2.1)
    * amcl (1.16.6)


# How to install:

## ROS2 Dashing

1. Install ROS2 via debian packages for Ubuntu Bionic (18.04).

    Instruction in the ROS2 wiki https://index.ros.org/doc/ros2/Installation/Dashing/Linux-Install-Debians/

2. Create a workspace.

    Instructions in the ROS2 wiki https://index.ros.org/doc/ros2/Tutorials/Workspace/Creating-A-Workspace/

3. Clone PeTRA packages to `<ros2-workspace>/src`

    ```bash
    git clone -b AZachariaeSS20 https://www.w.hs-karlsruhe.de/gitlab/robolab/researchprojects/petra/petra_central_control.git
    git clone -b AZachariaeSS20 https://www.w.hs-karlsruhe.de/gitlab/robolab/researchprojects/petra/petra_core.git
    git clone -b AZachariaeSS20 https://www.w.hs-karlsruhe.de/gitlab/robolab/researchprojects/petra/petra_drivers.git
    git clone -b AZachariaeSS20 https://www.w.hs-karlsruhe.de/gitlab/robolab/researchprojects/petra/petra_services.git
    ```

4. Get additional necessary packages

    ```bash
    sudo apt-get install ros-dashing-behaviortree-cpp-v3
    sudo apt-get install ros-dashing-ros1-bridge
    ```

## ROS1 Melodic

1. Install ROS1 for Ubuntu Bionic (18.04).

    Choose the "Desktop-Full Install" for additional gazebo, rviz, rqt, etc.

    Instructions in the ROS wiki http://wiki.ros.org/melodic/Installation/Ubuntu

2. Create a workspace.

    Instructions in the ROS wiki http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment

3. Get Neobotix simulation packages for the MMO-500 robot

    * a) Use the simulation packages as used in the video. There are some changed settings and the own hospital map incuded. (By now Neobotix has a new versions of the packages)

        Unpack the accompanying .tar.xz archives from the USB-stick in the `<ros1-workspace>/src` folder:

        ```
        petra_stop_bridge.tar.xz
        neo_driver.tar.xz
        neo_simulation.tar.xz
        cob_scan_unifier.tar.xz
        ```

        Or clone the metafolder with these packages from the personal GitHub in `<ros1-workspace>/src`:

        ```bash
        git clone https://github.com/AndreasZachariae/Neobotix-Simulation.git
        ```

    * b) Or use the new version directly from neobotix without modifications:

        Follow the instructions at https://docs.neobotix.de/display/ROSSim/Installation+on+Ubuntu

        Clone the packages in `<ros1-workspace>/src`

        ```bash
        git clone https://github.com/neobotix/neo_msgs.git
        git clone https://github.com/neobotix/neo_srvs.git
        git clone https://github.com/neobotix/neo_common.git
        git clone https://github.com/neobotix/neo_kinematics_mecanum.git
        ```

        And unpack the .tar.xz archive from the USB-Stick `petra_stop_bridge.tar.xz` (Or use the package from GitHub).

4. Get additional necessary packages

    ```bash
    sudo apt-get install ros-melodic-ros-controllers
    sudo apt-get install ros-melodic-gazebo-ros-control
    sudo apt-get install ros-melodic-navigation
    sudo apt-get install ros-melodic-neo-local-planner
    sudo apt-get install ros-melodic-openslam-gmapping
    sudo apt-get install ros-melodic-amcl
    sudo apt-get install ros-melodic-joy
    sudo apt-get install ros-melodic-teleop-twist-keyboard
    ```

5. Install Groot from source

    Follow instructions at https://github.com/BehaviorTree/Groot

    in `<ros1-workspace>/src`

    ```bash
    git clone https://github.com/BehaviorTree/Groot.git
    cd Groot
    git submodule update --init --recursive
    mkdir build; cd build
    cmake ..
    make
    ```


# How to launch:

## Run ROS2 PeTRA packages

build the packages in `<ros2-workspace>/src`

```bash
colcon build
```

always source in each shell:

```bash
source /opt/ros/dashing/setup.bash
source ~/<workspace>/install/setup.bash
```

In first shell:

```bash
ros2 launch petra_central_control petra_launch.py
```

or if used with neobotix simulation or real robot:

```bash
ros2 launch petra_central_control petra_simulation_launch.py
```

For user input in second shell:

```bash
ros2 run petra_drivers Keyboard
```

For debugging info from each node:

```bash
rqt
```


### Run single nodes

If you want to run single Nodes in separate shells:

```bash
ros2 run petra_drivers Keyboard
ros2 run petra_drivers Screen
ros2 run petra_drivers RobotDummy
ros2 run petra_services Communication
ros2 run petra_services Navigation2Dummy
ros2 run petra_services PatientMonitoringDummy
ros2 run petra_drivers PairingModuleDummy
ros2 run petra_drivers ManipulatorDummy
ros2 run petra_drivers BatteryDummy
ros2 run petra_central_control PeTRACentralControl
```


### Commands for testing

Write `/Stop` in shell with Keyboard node running to publish a `Stop` message.

To set the Dummies with specific values:

```bash
ros2 topic pub --once /SetCondition std_msgs/msg/String "data: bad"
ros2 topic pub --once /SetBattery std_msgs/msg/Float32 "data: 0.1"
```


## Run the ROS1/2 bridge

```bash
source /opt/ros/melodic/setup.bash
source /opt/ros/dashing/setup.bash 
export ROS_MASTER_URI=http://localhost:11311
ros2 run ros1_bridge dynamic_bridge
```

## Run the ROS1 Neobotix simulation

always source in each shell:

```bash
source /opt/ros/melodic/setup.bash
source ~/<ros1-workspace>/devel/setup.bash
```

launch each in separate shells:

(rosmaster will be started automatically by roslaunch)

```bash
roslaunch neo_simulation simulation.launch
```

For localization the previously scanned map `hska_hospital` should be loaded:

```bash
export MAP_NAME='hska_hospital'
roslaunch neo_simulation mmo_500_amcl.launch
```

For the navigation:

```bash
roslaunch neo_simulation mmo_500_move_base.launch
```

Necessary to convert `Stop` message to a navigation cancel command:

```bash
rosrun petra_stop_bridge StopBridge
```


## Run Groot in ROS1

```bash
rosrun groot Groot
```

To see the MainTree, navigate in the GUI to `<ros1-workspace>/src/petra_central_control/behaviors` and open `MainTreeForGroot.xml`


# Documentation

The documentation for the ROS2 PeTRA Code is the folder `documentation` on the accompanying USB-Stick.

Open the `documentation.html` or the `documentation.pdf` file.