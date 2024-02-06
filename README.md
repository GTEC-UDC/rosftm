# README

This repository includes several tools and ROS nodes to process FTM ranging values.

The nodes included in the repository are:

* **ESP32S2FTMReader** This node reads the frames from a FTM communication between a ESP32 S2 and another FTM device. 

## Dependencies

GTEC ROS TOA package has the next dependencies:

* **gtec_msgs** Another GTEC ROS project with a set o custom messages definitions. Available [https://github.com/GTEC-UDC/rosmsgs](https://github.com/GTEC-UDC/rosmsgs)

Additionally, the ESP32 S2 must be loaded with the code available here: [https://github.com/GTEC-UDC/esp32s2-ftm-tag](https://github.com/GTEC-UDC/esp32s2-ftm-tag)

## Building the nodes

To build the nodes, source code must be cloned inside a catkin workspace on a ROS installation (see [Creating a workspace for catkin](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)). If the catkin work space is located at ```~/catkin_ws``` then:

```bash
$ cd ~/catkin_ws/src
$ git clone https://github.com/GTEC-UDC/rosftm.git
```

Then ```catkin_make``` must be used to build the nodes:

```bash
$ cd ~/catkin_ws
$ catkin_make
```

## Launching the nodes

There are several *.launch* files in the project to launch the nodes using the ```roslaunch``` command. 

Launching **ESP32S2FTMReader** node:

```bash
$ roslaunch gtec_ftm esp32s2ftmtagreader.launch
```

The launcher accepts the parameter *serial*, that points to the port where the ESP32 is connected. By default: */dev/ttyUSB0*.

## Cite

The code in this repository is related to the following work:

*V. Barral Vales, O. C. Fernández, T. Domínguez-Bolaño, C. J. Escudero and J. A. García-Naya, "Fine Time Measurement for the Internet of Things: A Practical Approach Using ESP32," in IEEE Internet of Things Journal, vol. 9, no. 19, pp. 18305-18318, 1 Oct.1, 2022, doi: 10.1109/JIOT.2022.3158701.* 

If you make use of this code, a citation is appreciated.

