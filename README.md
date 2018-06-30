# CS::APEX (Algorithm Prototyper and EXperimentor for Cognitive Systems)

CS::APEX is a framework based on synchronous dataflow and event-based message passing that
aims to speed up prototyping of new robotic algorithms using visual programming aspects.

Example workflow: (click for better quality)

[![Graph Construction Video](https://media.giphy.com/media/aFtp2izVp2xMzLAqC2/giphy.gif)](https://youtu.be/sNkHnQhNXuU)


## Dataflow Graph and Core Features

Calculations are represented by a nodes in a directed graph with data flowing on
the directed edges.This execution graph is manipulated using a simple graphical user
interface that allows spawning and deleting nodes, adding and removing edges and
visualizating data in the stream. To speed up the prototyping process,
other features like undo/redo mechanisms and profiling are implemented as well.
Finally there exists an easy to use parameter system that generates UI controls
for each block and allows parameter tuning and optimization.


## Functionality via Plug-ins

The framework itself does not provide any predefined computation nodes and does
not depend on specific message definitions or node types.
These details are instead implemented in plug-in libraries that extend
the functionality of the whole system.


## Application to Robotics

The framework is targeted toward use in robotics and is fully compatible with
[ROS](http://wiki.ros.org/). Configurations generated using the GUI can
directly be deployed on any ROS-based robotic system.


## Tutorials and more Information

For more information, please refer to the Wiki at
    https://github.com/cogsys-tuebingen/csapex/wiki

the official website at
    http://www.ra.cs.uni-tuebingen.de/forschung/apex/

or contact the maintainers via email.


## Dependencies

We rely on the [catkin](http://wiki.ros.org/catkin) build system
developed in the [ROS](http://wiki.ros.org/) ecosystem. The core framework
is independent of ROS, however.

Required for a build are:
- Linux system (tested with Ubuntu)
- C++11 compatible compiler (g++, clang++)
- Qt5 (on Ubuntu: qt5-default libqt5svg5-dev)
- libraries:
    - boost (program_options, filesystem, system, regex)
    - [classloader](https://github.com/ros/class_loader)
    - TinyXML (on Ubuntu: libtinyxml-dev)
    - yaml-cpp (on Ubuntu: libyaml-cpp-dev)


These dependencies can be installed via rosdep (see below.)


## Basic Installation

To get the cs::APEX framework and a set of core plugins, perform the following:

    cd ~
    mkdir -p ws/csapex/src
    cd ws/csapex/src

    git clone https://github.com/cogsys-tuebingen/csapex.git

    cd csapex/plugins
    git clone https://github.com/cogsys-tuebingen/csapex_core_plugins.git

    cd ../..
    mkdir libs
    cd libs

    git clone https://github.com/cogsys-tuebingen/cslibs_vision.git
    git clone https://github.com/cogsys-tuebingen/cslibs_laser_processing.git
    git clone https://github.com/cogsys-tuebingen/cslibs_arff.git
    git clone https://github.com/cogsys-tuebingen/cslibs_indexed_storage.git

    cd ../..
    rosdep install -y -r -i --from-paths src
    catkin_make


## Creating Documenation

To create the documentation, run

    doxygen doc/Doxyfile

This will generate the documentation at

    doc/html/index.html


## Contributions

All contributions are welcome, please refer to the CONTRIBUTING.md file.
