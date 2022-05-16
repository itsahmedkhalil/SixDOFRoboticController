# 6 DOF Robot Arm Controller using Intertial Measurement Unit (IMU)

## Setting up Code
### Clone this repository
`$ git clone https://github.com/mohsafwat23/Controller_Research.git`

### Build the workspace
`$ cd Controller_Research && catkin build #builds all of the packages in the workspace
`

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

#### The IMU used in this project is the [**LPMS-B2 Series**](https://lp-research.com/9-axis-bluetooth-imu-lpmsb2-series/) by lp-research. It determines the orientation of the controller by fusing three different MEMS sensors (3-axis gyroscope, 3-axis accelerometer, and 3-axis magnetometer). To use the IMU you need to download [**OpenZen**](https://lpresearch.bitbucket.io/openzen/latest/index.html) which is a high performance sensor data streaming and processing software. The IMU communicates with the host computer via Bluetooth. The bluetooth dongle that worked for us was the [**TP-Link USB Adapter**](https://www.amazon.com/gp/product/B07V1SZCY6/ref=ppx_yo_dt_b_asin_title_o01_s00?ie=UTF8&psc=1). 

### Download the pre-compiled OpenZen Software
#### Install gcc7 or higher (requires C++17 support): 
`$ sudo apt-get install gcc-7`

#### Install CMake, git: 
`$ sudo apt-get install git cmake`

#### Install the Bluetooth libraries if you need Bluetooth support: 
`$ sudo apt-get install libbluetooth-dev`

#### Install Qt (5.11.2 or higher) if you need Bluetooth Low Energy support: 
`$ sudo apt-get install qtbase5-dev qtconnectivity5-dev`

#### Clone the repository:
`$ cd src && git clone --recurse-submodules https://bitbucket org/lpresearch/openzen.git`

#### Create a build folder and run cmake:
`$ cd openzen && mkdir build && cd build`

#### Build folder and run cmake:
`$ cmake .. && make -j4`

#### Customize your OpenZen build enable Bluetooth and Python
`$ cmake -DZEN_BLUETOOTH=ON -DZEN_PYTHON=ON -ZEN_BLUETOOTH_BLE=ON .. && make -j4`

#### **Note**: To make sure that the build was successful, an **"openzen.so"** file needs to be present in the /openzen/build directory.

# To do:
## - How to run the code (roslaunch)
## - How to debug the code 


