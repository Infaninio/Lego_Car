# Purpose of the Repository
An ROS implementation for driving a lego car with two engines (left and right engine). The engines (the whole car) is controlled with a Raspberry PI3b. In PinBelegung.txt are the used GPOIOs listed. The car includes two engines, a Ultrasonic Sensor and a Camera. ROS is used to be able to run the driving-Programs on a Laptop or Desktop and the Software for the sensors/actors on the Raspberry. 

# Install Raspberry
The ROS installation is based on https://roboticsbackend.com/install-ros-on-raspberry-pi-3/. You have to use ROS noetic instead of melodic due to the Ubuntu Version.
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
Add all Systems, which want to communicat via ROS, with their IP-Adress and Hostname to assure that Messages will be received/sent correctly.
```shell
    sudo nano /etc/hosts
```
Add the Ip Adress of the Systems and the Name them e.g "10.0.0.1 Ros-Server". 
Set the ROS_MASTER_URI on all PCs that are not running the roscore. Set the ROS_MASTER_URI to the System which will run the roscore process
```shell
    export ROS_MASTER_URI=http://Ros-Server:11311
```

Setup the Workspace see [Workspace Setup](#Setup the Workspace same for Host and Raspberry)



## Error-Handling

Raspberry does not receive any Messages 



# Install Host
Use Ubuntu 20.04

Installe ROS as Described here: wiki.ros.org/noetic/Installation/Ubuntu




# Setup the Workspace same for Host and Rasperry
Create a Folder for the catkin Workspace the. Create a src folder inside the new folder. Afterwards create the catkin Workspace. 
```shell
    mkdir lego_car
    cd lego_car
    mkdir src
    catkin_make
```

If the System does not find catkin, you forgot to source ROS 
```shell
    source /opt/ros/noetic/setup.bash
```

Clone this Repository insto the src directory.

```shell
    cd src
    git clone https://github.com/Infaninio/Lego_Car.git
```

Build the Packages by using catkin_make again. You have to be in the top directory of the catkin Workspace. Source the Workspace to be able to run the rosnodes
```shell 
    cd ..
    catkin_make install
    source devel/setup.bash
```
If you don't want to source and export the Names in every new Terminal add the Lines to ~/.bashrc