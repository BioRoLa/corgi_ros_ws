# Corgi Robot ROS Workspace

![ROS Version](https://img.shields.io/badge/ROS-Noetic-blue)
![Platform](https://img.shields.io/badge/Platform-Ubuntu%2020.04-orange)
![Language](https://img.shields.io/badge/Language-C%2B%2B%20%7C%20Python-yellowgreen)

This is the central ROS workspace for the Corgi quadruped robot, developed at the Bio-Inspired Robotic Laboratory (BioRoLa), NTU. It contains all necessary packages for control, simulation, and hardware interfacing.

<!-- <p align="center">
  <img src="[INSERT_IMAGE_OR_GIF_URL_HERE]" alt="Corgi Robot Demo" width="600"/>
</p> -->


## Table of Contents

- [***System Architecture***](#system-architecture)
- [***System Requirements & Dependencies***](#system-requirements--dependencies)
  - [*Hardware*](#hardware)
  - [*Software*](#software)
- [***Installation and Build Instructions***](#installation-and-build-instructions)
- [***Usage Guide***](#usage-guide)
  - [*Running the Simulation*](#running-the-simulation)
  - [*Running on the Real Robot*](#running-on-the-real-robot)
- [***Workspace Structure & Key Packages***](#workspace-structure--key-packages)
- [***Notes & Contact***](#notes--contact)


## System Architecture

The system uses ROS on a high-level computer (PC/Jetson) to communicate with a low-level FPGA driver (NI sbRIO) via gRPC.


## System Requirements & Dependencies

### Hardware
* **Main Controller:**
    * Real Robot: NVIDIA Jetson AGX Orin
    * Panel/Simulation/Dev: PC with Ubuntu 20.04
* **Low-Level Controller:**
    * NI sbRIO-9629

### Software

* **NI sbRIO-9629**
    * [**fpga_driver**](https://github.com/BioRoLa/fpga_driver)
    * [**grpc_core**](https://github.com/BioRoLa/grpc_core)

* **PC / Nvidia Jetson**
    * **OS**: Ubuntu 20.04
    * **ROS**: [**ROS Noetic**](http://wiki.ros.org/noetic/Installation/Ubuntu) (with `catkin_tools`)
    * **Simulator**: [**Webots**](https://cyberbotics.com/) (for simulation only)
    * **Apt Dependencies**:
      * **ros-noetic-joy** `sudo apt install ros-noetic-joy`
      * **doxygen** `sudo apt-get install doxygen`
    * **Python Dependencies**:
        * `pip install numpy`
        * `pip install PyQt5`
        * `pip install Jetson.GPIO`
    * **C++ Dependencies**:
        * [**Eigen 3.4.0**](https://eigen.tuxfamily.org/index.php?title=Main_Page)
        * [**yaml-cpp**](https://github.com/jbeder/yaml-cpp)
        * [**mip_sdk (v2.0.0)**](https://github.com/LORD-MicroStrain/mip_sdk/tree/v2.0.0) (***Important:*** You must modify its `CMakeLists.txt` at line 241, changing `src` to `include`.)
        * [**osqp (v0.6.3)**](https://github.com/osqp/osqp/tree/v0.6.3)
        * [**osqp-eigen**](https://github.com/robotology/osqp-eigen)
        * [**grpc_core**](https://github.com/BioRoLa/grpc_core)
    

> ***CRITICAL INSTALLATION NOTE***
> 
> To avoid build errors, it is **strongly recommended** to install all C++ dependencies listed above into the `~/corgi_ws/install/` directory. If you install them elsewhere, you **must** manually update the compile commands or the paths in the `CMakeLists.txt` files.


## Installation and Build Instructions

1.  **Create the Workspace Directory**

    Use this structure to keep all related projects organized.

    ```bash
    mkdir ~/corgi_ws/
    ```

2.  **Clone the Repository**

    ```bash
    cd ~/corgi_ws/
    git clone https://github.com/BioRoLa/corgi_ros_ws.git
    ```

3.  **Install All Dependencies**

    Follow the list in the [***Software***](#software) section to install all required libraries.
    ```bash
    cd ~/corgi_ws/
    mkdir install/
    ```

    1. **yaml-cpp**
    Clone the `yaml-cpp` repository and follow the installation instructions:
      ```bash
      cd ~/corgi_ws/install
      git clone https://github.com/jbeder/yaml-cpp.git
      cd yaml-cpp
      mkdir build && cd build
      cmake .. -DCMAKE_PREFIX_PATH=$HOME/corgi_ws/install -DCMAKE_INSTALL_PREFIX=$HOME/corgi_ws/install
      make -j16
      sudo make install
      ```
    2. **Eigen**
    Clone the `Eigen` repository and follow the installation instructions:
      ```bash
      cd ~/corgi_ws/install
      git clone https://gitlab.com/libeigen/eigen.git
      cd eigen
      mkdir build && cd build
      cmake .. -DCMAKE_PREFIX_PATH=$HOME/corgi_ws/install -DCMAKE_INSTALL_PREFIX=$HOME/corgi_ws/install
      make -j16
      sudo make install
      ```
    3. **grpc_core**
    Install gRPC & grpc_core as follows: [**grpc_core**](https://github.com/BioRoLa/grpc_core)

    4. **osqp_eigen**
    Install osqp_eigen as follows: [**osqp_eigen**](https://github.com/robotology/osqp-eigen)

    5. **MIP_SDK**
    Clone the `mip_sdk` repository and follow the installation instructions:
      ```bash
       cd ~/corgi_ws/install
       git clone --branch fix-install-interface --single-branch https://github.com/hiho817/mip_sdk.git
       cd mip_sdk
       git checkout v2.0.0
       mkdir build
       cd build
       cmake .. -DMIP_USE_SERIAL=ON -DCMAKE_PREFIX_PATH=$HOME/corgi_ws/install -DCMAKE_INSTALL_PREFIX=$HOME/corgi_ws/install
       make -j16
       sudo make install
      ```


4.  **Build the ROS Workspace**

    If you installed C++ dependencies to the recommended path, use the following command.

    ```bash
    cd ~/corgi_ws/corgi_ros_ws/
    catkin build -DLOCAL_PACKAGE_PATH=${HOME}/corgi_ws/install
    source devel/setup.bash
    ```

5.  **Source the Environment**

    Add the following command to your `~/.bashrc` to source it automatically in new terminals.

    ```bash
    echo "source ~/corgi_ws/corgi_ros_ws/devel/setup.bash" >> ~/.bashrc
    source ~/.bashrc
    ```


## Usage Guide

### Running the Simulation
To launch the Corgi robot in the Webots simulator:

```bash
roslaunch corgi_sim run_simulation.launch
```

After launching the simulation, you can run other nodes to interact with the simulated robot. Press ***Enter*** to start the simulation, and the data will be recorded in `output_data/` if the output file name are not empty.


### Running on the Real Robot

<!-- **Safety First:** Always have the robot on a stand before powering the motors. -->

1.  **Start Low-Level Driver**

    Before anything else, ensure the **`fpga_driver`** is running on the NI sbRIO.

2.  **Launch ROS Control Panel**

    On the Jetson/PC, run the main launch file. This will start the ***panel***, ***data recorder***, ***force estimation***, and ***IMU*** nodes.

    ```bash
    roslaunch corgi_panel corgi_control_panel.launch
    ```

3.  **Control Panel Operation Sequence**

    Follow these steps **in order** on the GUI to safely start the robot:
    1.  **Power On**: Click `Digital` -> `Signal` -> `Power`.
    2.  **Set Zero Position**: Press `Set Zero` to define the motor's home position.
    3.  **Calibrate & Enable**:
        * If legs are fully extended, press `Hall Calibration`.
        * If legs are already in wheeled mode, press `Motor Mode` to enable motors.
    4.  **Steer Calibration (Optional)**: If you will use wheeled steering, press `Steer Calibration`.
    5.  **Select Control Mode**: After entering `Motor Mode`, PID gains will be zeroed. Choose either `RT` (Real-Time) or `CSV` (from `input_csv/`) mode.
    6.  **Trigger Motion & Recording**:
        * Fill in a filename in the `Output File Name` field to enable data logging, or data will not be recorded.
        * Press `Trigger` to start the motion trajectory, and data will be saved to `output_data/`.
    7.  **Powery Reset**: Pressing `Reset` will **cut all power** to the motors. Ensure the robot is secure before using this.


## Workspace Structure & Key Packages

### Key Packages in `src/`

For more details on a specific package, please see its respective `README.md` file.

* **Core**
    * [`corgi_msgs`](src/corgi_msgs): Defines all custom ROS messages used in the project.
    * [`corgi_ros_bridge`](src/corgi_ros_bridge): The gRPC client/server that connects ROS to the FPGA driver.
    * [`corgi_virtual_agent`](src/corgi_virtual_agent): A mock FPGA driver for testing the `corgi_ros_bridge` without hardware.
    * [`corgi_data_recorder`](src/corgi_data_recorder): A flexible node to subscribe to topics and log data to CSV.
    * [`corgi_panel`](src/corgi_panel): The PyQt5-based GUI for robot control and monitoring.
    * [`*corgi_utils`](src/corgi_utils): Shared utility functions, constants, and helper classes.

* **Simulation**
    * [`corgi_sim`](src/corgi_sim): Contains Webots models, worlds, and controllers for simulation.

* **Sensing & Estimation**
    * [`*corgi_imu`](src/corgi_imu): Driver and interface for the LORD MicroStrain IMU.
    * [`corgi_force_estimation`](src/corgi_force_estimation): Estimates contact forces from motor currents.
    * [`*corgi_odometry`](src/corgi_odometry): Robot odometry estimation.
    * [`*corgi_camera`](src/corgi_camera): Integrates ZED camera for visual perception.

* **Control & Planning**
    * [`corgi_csv_control`](src/corgi_csv_control): Publishes motor commands from a pre-defined CSV file.
    * [`corgi_rt_control`](src/corgi_rt_control): Handles real-time trajectory commands.
    * [`*corgi_walk`](src/corgi_walk): General legged locomotion and gait planning.
    * [`*corgi_stair`](src/corgi_stair): Algorithms specifically for stair climbing locomotion.
    * [`corgi_force_control`](src/corgi_force_control): Foot contact force control algorithms.
    * [`corgi_mpc`](src/corgi_mpc): Model Predictive Control implementation.
    * [`*corgi_gait_generate`](src/corgi_gait_generate): Procedural gait pattern generation.
    * [`*corgi_gait_selector`](src/corgi_gait_selector): Selects the appropriate gait based on robot state or command.
    * [`*corgi_algo`](src/corgi_algo): Contains various core algorithms for control.


## Notes

