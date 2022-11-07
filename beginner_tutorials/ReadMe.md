# Begineer Tutorials - Publisher-Subscriber

## Overview
This repository contains tutorials from <a href="http://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries.html">ROS2 Beginner: Client libraries</a>. The tutorials cover publishers, subscribers, services, logging, and launch files with respect to the ROS2 Humble Hawksbill.

# Dependencies
<ul>
  <li>Ubuntu 20.04+</li>
  <li>ROS2 Humble Hawksbill</li>
</ul>

# Compile and run instructions
```
    cd "your_colcon_workspace"/src
    git clone https://github.com/tanujthakkar/beginner_tutorials.git
    cd ..
    colcon build
    source "your_colcon_workspace"/install/setup.bash
```

After you've compiler and sourced your workspace

## Running publisher
```
    ros2 run beginner_tutorials talker
```

## Running publisher
```
    ros2 run beginner_tutorials listener
```