# Begineer Tutorials - Publisher-Subscriber

## Tutorials
- [Publisher - Subscriber](#publisher---subscriber)
- [Service - Logging - Launch Files](#service---logging---launch-files)
- [tf2 - Unit Testing - Bags](#tf2---unit-testing---bags)

## Overview
This repository contains tutorials from <a href="http://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries.html">ROS2 Beginner: Client libraries</a>. The tutorials cover publishers, subscribers, services, logging, and launch files with respect to the ROS2 Humble Hawksbill.

## Dependencies
<ul>
  <li>Ubuntu 20.04+</li>
  <li>ROS2 Humble Hawksbill</li>
</ul>

## Compile and run instructions
```
    cd "your_colcon_workspace"/src
    git clone https://github.com/tanujthakkar/beginner_tutorials.git
    cd ..
    colcon build
    source "your_colcon_workspace"/install/setup.bash
```

## Publisher - Subscriber

After you've compiled and sourced your workspace

### Running publisher
```
    ros2 run beginner_tutorials talker
```

### Running Subscriber
```
    ros2 run beginner_tutorials listener
```

## Service - Logging - Launch Files

### Compile and run instructions
```
  git checkout Week10_HW_Release
```
Follow same compile and build instructions as above.

### Using the launch file with argument
```
  ros2 launch beginner_tutorials beginner_tutorials.yaml count:=10
```

### Results

### Service and Logging
![image](https://github.com/tanujthakkar/beginner_tutorials/blob/Week10_HW/beginner_tutorials/results/service_and_logging.png)

### rqt_console
![image](https://github.com/tanujthakkar/beginner_tutorials/blob/Week10_HW/beginner_tutorials/results/rqt_console_output.png)


## tf2 - Unit Testing - Bags

### Compile and run instructions
```
  git checkout Week11_HW_Release
```
Follow same compile and build instructions as above.

### Running tf
```
  ros2 run beginner_tutorials talker
```

### Echoing transformations
```
  ros2 run tf2_ros tf2_echo world talk
```
![image](https://github.com/tanujthakkar/beginner_tutorials/blob/Week11_HW/beginner_tutorials/results/tf2_echo.png)

### Exporting tf frames
```
  ros2 run tf2_tools view_frames
```
![Screenshot from 2022-12-05 16-58-25](https://user-images.githubusercontent.com/32800090/205751163-9b616d4a-8199-4e7c-b935-9edec7b8f91d.png)

### Unit Testing
```
  colcon test --event-handlers console_direct+ --packages-select beginner_tutorials
```

### gtest only
```
  ros2 run beginner_tutorials beginner_tutorials_test
```
![image](https://github.com/tanujthakkar/beginner_tutorials/blob/Week11_HW/beginner_tutorials/results/testing.png)

### Bags

Record a bag file using the following command,
```
  ros2 launch beginner_tutorials beginner_tutorials.py record:=True
```

To verify the bag recording, run the listener using the following command,
```
  ros2 run beginner_tutorials listener
```
Play the bag file using the following command,
```
  ros2 bag play path_to_bag_directory
```
![image](https://github.com/tanujthakkar/beginner_tutorials/blob/Week11_HW/beginner_tutorials/results/rosbag_playback.png)
