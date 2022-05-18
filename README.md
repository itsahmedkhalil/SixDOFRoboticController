# 6 DOF Robot Arm Controller using Intertial Measurement Unit (IMU)

## Project Overview

### Description of Project

This project's purpose is to control a 6 DOF robot arm using an IMU for orientation, a joystick for 2D/planar control, and two push buttons for the third degree of freedom.

### Equipment Setup

- IMU: [**LPMS-B2 Series**](https://lp-research.com/9-axis-bluetooth-imu-lpmsb2-series/)
- Joystick: [**HiLetgo Joystick**](https://www.amazon.com/HiLetgo-Controller-JoyStick-Breakout-Arduino/dp/B00P7QBGD2/ref=sr_1_4?keywords=arduino+joystick&qid=1652826594&sr=8-4)
- Buttons: Push buttons from [UW Madison Makerspace](https://making.engr.wisc.edu/mini-mart/#electronics)
- Arduino Nano: [**Arduino Nano with ATMega328**](http://store.arduino.cc/products/arduino-nano)

The controller's frame, button fixture, and joystick fixture, were all 3D printed using PLA material on a Ultimaker 3 3D printer. The IMU's fixture was 3D printed using Flexible resin on a Formslab 3D printer. While all of the Solidwork models are available for download, many of them require small design changes to prevent the user from needing to do any extra machining. Below is a photo of the final controller. Click on the photo to be redirected to a video of the controller working on a 6 DOF 'mini' robotic arm.

**Note:** The reflective balls attached to the controller's frame are used to track the controller using an optical mocap tracking system. We only used this tracking system to test the IMU's orientation values by comparing it to 'absolute' values and it was not used when controlling the robot.

[![IMAGE_ALT](/images/controller.jpeg)](https://youtu.be/g5Sf1ONjNlU)

### Collaborators

This project was created by [Ahmed Khalil](https://itsahmedkhalil.github.io/) and [Mohamed Safwat](https://mohsafwat23.github.io/) as part of the NASA's ULI Project with [Mike Hagenow](https://www.hageneaux.com/).

## Setting up Code

### Clone this repository

`$ git clone https://github.com/mohsafwat23/Controller_Research.git`

### Build the workspace

`$ cd Controller_Research && catkin build #builds all of the packages in the workspace `

## Setting up the Arduino

The Arduino used in this project is the [**Arduino Nano with ATMega328**](http://store.arduino.cc/products/arduino-nano). It runs using [**rosserial**](http://wiki.ros.org/rosserial_arduino/Tutorials) which allowes devides like the Arduino to interface with ROS by creating ROS nodes.

### Install the rosserial package

#### Source Workspace

`$ source devel/setup.bash`

#### Clone the rosserial package

`$ cd src && git clone https://github.com/ros-drivers/rosserial.git`

#### Build the project

`$ cd .. && catkin build`

### Upload code to the Arduino board via Arduino IDE

#### **Note**: When you uploade the code, the Arduino will not print any messages through the serial monitor.

#### To check the messages being published by the Arduino, you can use the following command:

`$ rosrun rosserial_python serial_node.py`

#### Check that the **'/joy'** rostopic exists:

`$ rostopic list`

#### Echo the **'/joy'** messages to the terminal:

`$ rostopic echo /joy`

Make sure that the buttons are working properly by observing changes in their values when the buttons are pressed.

#### **For more info on rosserial, see [**rosserial**](http://wiki.ros.org/rosserial)**

### Reasons why the code might not be uploaded to the Arduino board:

- Not enough current is being supplied to the board from the USB port.
  - Try changing the USB cable.
- The USB cable is broken and needs to be replaced.
  - Open the controller and replace the old one with a newer one.
- Selected the wrong board from the Arduino IDE's menu.
  - Go to Tools>Board>Boards>Arduino AVR Boards> Arduino Nano.
- Selected the wrong programmer from the Arduino IDE's menu.
  - Go to Tools>Programmer>.
- None of the above?
  - Try resetting the Arduino board using this [method](https://stackoverflow.com/questions/5290428/how-can-i-reset-an-arduino-board)
  - Get a new Arduino Nano

## Setting up the IMU

The IMU used in this project is the [**LPMS-B2 Series**](https://lp-research.com/9-axis-bluetooth-imu-lpmsb2-series/) by lp-research. It determines the orientation of the controller by fusing three different MEMS sensors (3-axis gyroscope, 3-axis accelerometer, and 3-axis magnetometer). To use the IMU you need to download [**OpenZen**](https://lpresearch.bitbucket.io/openzen/latest/index.html) which is a high performance sensor data streaming and processing software. The IMU communicates with the host computer via Bluetooth. The bluetooth dongle that worked for us was the [**TP-Link USB Adapter**](https://www.amazon.com/gp/product/B07V1SZCY6/ref=ppx_yo_dt_b_asin_title_o01_s00?ie=UTF8&psc=1).

### Download the pre-compiled OpenZen Software

#### Install gcc7 or higher (requires C++17 support):

`$ sudo apt-get install gcc-7`

#### Install CMake, git:

`$ sudo apt-get install git cmake`

#### Install the Bluetooth libraries for Bluetooth support:

`$ sudo apt-get install libbluetooth-dev`

#### Install Qt (5.11.2 or higher) for Bluetooth Low Energy support:

`$ sudo apt-get install qtbase5-dev qtconnectivity5-dev`

#### Clone the repository:

`$ cd src && git clone --recurse-submodules https://bitbucket org/lpresearch/openzen.git`

#### Create a build folder and run cmake:

`$ cd openzen && mkdir build && cd build`

#### Build folder and run cmake:

`$ cmake .. && make -j4`

#### Customize the OpenZen build to enable Bluetooth, BLE, and Python

`$ cmake -DZEN_BLUETOOTH=ON -DZEN_PYTHON=ON -ZEN_BLUETOOTH_BLE=ON .. && make -j4`

**Note**: To make sure that the build was successful, an **"openzen.so"** file needs to be present in the /openzen/build directory.

## Using the controller

- Switch on the IMU such that IMU's blue LED lights up at a regular rate.
- Plug the Arduino into your a USB port

### Launch the controller in rviz as a marker:

`$ roslaunch data_to_pose joy_rviz.launch`

### Code structure:

- #### joystick_final.ino

  - Arduino file that reads the joystick's x, y, and button values. The file also reads the two push buttons' values.
  - The joystick's x and y values are used to control the robot's planar motion, and they mapped to -512 to 512.
  - The joystick's button generates a digital reading and is used to trigger the robot's clutch control.
  - The top push button controls positive translation while the bottom push button controls negative translation, perpindicular to the robot's planar motion using the right-hand rule. The push buttons are digital values and not mapped to anything but 0 and 1.
  - The message sent is a 16IntMultiArray message. The contents are x, y, z, and clutch have indexes of 0, 1, 2, and 3 respectively.
  - The Arduino publishes the data to the ROS node at a rate of 50 Hz.

- #### data_to_pose_rviz.py

  - Python file that reads sensor data and commands pose to the robot in rviz for visualization.
  - The file subscribes to /gyr_data

- #### imudata.py

  - Python file that reads the LPMS sensor data and publishes the data to the ROS node.
  - **IMPORTANT**: Make sure to change this line sys.path.append('/home/ahmed/build/openzen') to your path to the OpenZen build folder. Also make sure that you 'import openzen' after you had appended the openzen path to the sys, otherwise Python will not be able to find the openzen module.

  - The file subscribes to /gyr_data

# To do:

- Circuit design
