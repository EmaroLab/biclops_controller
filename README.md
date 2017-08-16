# Biclops controller

This package is a fork of https://github.com/Yeshasvitvs/Biclops_Kinect_CMT and 
https://gitlab.com/SofarProject/Biclops. it has been modified to act as a generic
controller for the Biclops Robotic Camera Positioning Mechanism.

## Installation

Clone the repo:

    git clone git@github.com:EmaroLab/biclops_controller.git

Build it:

`catkin_make`

To connect to the Biclops add yourself to the `dialout` group:

- `whoami` (returns your `USERNAME`)
- Connect the biclops
- `sudo usermod -a -G dialout USERNAME`
- Restart

If this does not work, you can try to add an *udev* rule:

    cd YOUR_WS/src/biclopscontroller
    sudo cp 50-rs485.rules /etc/udev/rules.d/50-rs485.rules
    sudo udevadm control --reload

## Launching

There is a sample launch file in the launch dolder you can launch and modify

    roslaunch biclops_controller biclops.launch


## ROS topics

Send commands (Pan/Tilt) to:

`/biclops/tip_orientation`, message type:`biclops_controller::biclops_joint_states`

Read current state (Pan/Tilt) from:

`/biclops/joint_states`, message type:`biclops_controller::biclops_joint_states`

Angles are in degrees.

## ROS parameters

Where to load the configuration file from: 

`biclops/config_file_path`, default: `/src/biclops_controller/BiclopsDefault.cfg`

Whether publish a `tf` transform to the Baxter robot head or not:

`biclops/baxter_tf`, default: `true`
    
## Mantainer

[alessio.capitanelli@gmail.com](alessio.capitanelli@gmail.com)