.. _ros2_control_demos:

Demos
-----

This repository provides templates for the development of ros2_control-enabled robots and a simple simulations to demonstrate and prove ros2_control concepts.

Repository organization
^^^^^^^^^^^^^^^^^^^^^^^

**Note:** the following is a comprehensive list.

The repository is organized in the following packages:

  - ``ros2_control_demo_hardware`` - implementation of example hardware interfaces,
  - ``ros2_control_demo_description`` - descriptions of the robot models,    
  - ``ros2_control_demo_bringup`` - nodes starting hardware interfaces, controllers and GUIs.
  - ``ros2_control_test_node`` - nodes for testing ros2_control-enabled robots and their integration in the framework.


Goals
=====

The repository has three goals:
1. Implements the example configuration described in the ``ros-controls/roadmap`` repository file [components_architecture_and_urdf_examples](https://github.com/ros-controls/roadmap/blob/master/design_drafts/components_architecture_and_urdf_examples.md).
2. It provides templates for faster implementation of custom hardware and controllers;
3. The repository is a validation environment for ``ros2_control`` concepts, which can only be tested during run-time (e.g., execution of controllers by the controller manager, communication between robot hardware and controllers).


Description
===========

The repository is inspired by the [ros_control_boilerplate](https://github.com/PickNikRobotics/ros_control_boilerplate) repository from Dave Coleman.
The simulation has three parts/packages:
1. The first package, ``ros2_control_demo_bringup``, holds launch files and runtime configurations for demo robots.
2. The second package, ``rrbot_description``, stored URDF-description files, rviz configurations and meshes for the demo robots.
3. The third package, ``ros2_control_demo_hardware``, implements the hardware interfaces described in the roadmap.
The examples simulate a simple *RRbot* internally to provide sufficient test and demonstration data and reduce external dependencies.
This package does not have any dependencies except ``ros2`` core packages and can, therefore, be used on SoC-hardware of headless systems.

This repository demonstrates the following ``ros2_control`` concepts:

* Creating a ``*HardwareInterface`` for a System, Sensor, and Actuator.
* Creating a robot description in the form of URDF files.
* Loading the configuration and starting a robot using launch files.
* Control of two joints of *RRBot*.
* Using simulated robots and starting ``ros2_control`` with Gazebo simulator.
* Implementing a controller switching strategy for a robot.
* Using joint limits and transmission concepts in ``ros2_control``.

Quick Hints
===========

These are some quick hints, especially for those coming from a ROS1 control background:

* There are now three categories of hardware components: *Sensor*, *Actuator*, and *System*.
  *Sensor* is for individual sensors; *Actuator* is for individual actuators; *System* is for any combination of multiple sensors/actuators.
  You could think of a Sensor as read-only.
  All components are used as plugins and therefore exported using `PLUGINLIB_EXPORT_CLASS` macro.
* *ros(1)_control* only allowed three hardware interface types: position, velocity, and effort.
  *ros2_control* allows you to create any interface type by defining a custom string. For example, you might define a `position_in_degrees` or a `temperature` interface.
  The most common (position, velocity, acceleration, effort) are already defined as constants in hardware_interface/types/hardware_interface_type_values.hpp.
* Joint names in <ros2_control> tags in the URDF must be compatible with the controller's configuration.
* In ros2_control, all parameters for the driver are specified in the URDF.
  The ros2_control framework uses the **<ros2_control>** tag in the URDF.
* Joint names in <ros2_control> tags in the URDF must be compatible with the controller's configuration.
