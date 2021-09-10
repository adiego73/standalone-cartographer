# Standalone implementation of Google Cartographer

_I have created this project in order to have a guide if I want to use Google Cartographer library as standalone
library. I have done this because I didn't find a tutorial or guide about how to use it without ROS framework. In
addition, this implementation will not work at its own,since it depends on a 2d robotics simulator I am implementing for
the company I work for._

### Overview

Basically this implementation works along with a simulator that provides an odometry estimation and LiDAR sensor data.
All this information is provided through Protobuf messages in a publisher/subscriber way. So, the mapping software
subscribes to the odometry and lidar data from the simulator, and it performs the mapping of the unknown environment.

### Compile

Firts, you will need to install the dependencies:
Boost, Google Protobuffers, ZeroMQ, Eigen3 and yaml-cpp.

Then, just run ``cmake --build build --target all``

