INTRODUCTION
------------

This package is a fork of the package https://github.com/Yeshasvitvs/Biclops_Kinect_CMT. 
Please refer to the project homepage for further informations.


ATTENTION
-----------
Please note that the original package doen't compile when following 
the install instructions. This is due to a inconstistency in the CMakeLists. 
In our modified package this issue is resolved and should work out of the box.


INSTALLATION
-----------
* clone the package to your workspace
* adjust the path in src/Biclops_node.cpp line 33 char *config_path="your_path/src/Biclops/BiclopsDefault.cfg";
* make sure that the current user on your linux machine has the rights to use usb: (you could also set udev rules instead checkout here http://ask.xmodulo.com/change-usb-device-permission-linux.html)
- whoami      ->outputs the username
- connect the biclops
- check out which group
- ls -l /dev/ttyUSB*				-> shoud be sth like root:dialout
- then add your user to the dialout group
- sudo usermod -a -G dialout yourusername
- restart/logout
- done


ROS TOPICS
-----------
subscribes to:
	/oculus/orientation

