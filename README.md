# Purpose of the Repository
An ROS implementation for driving a lego car with two engines (left and right engine). The engines (the whole car) is controlled with a Raspberry PI3b. In PinBelegung.txt are the used GPOIOs listed. The car includes two engines, a Ultrasonic Sensor and a Camera. ROS is used to be able to run the driving-Programs on a Laptop or Desktop and the Software for the sensors/actors on the Raspberry. 

# Install Raspberry
The ROS installation is based on https://roboticsbackend.com/install-ros-on-raspberry-pi-3/.
- Prepare sd-Card with Ubuntu 20.04 (!not Raspbian)
- Enable Repository sources
```shell
    sudo add-apt-repository universe
    sudo add-apt-repository restricted
    sudo add-apt-repository multiverse
```
- Setup Sources
```shell
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
```
```shell
    sudo apt update
    sudo apt-get update
    sudo apt-get upgrade
```
Install ROS, we do not install the full version. All the Vision based things run on the PC
```shell
    sudo apt install ros-noetic-ros-base
```



# Install Host