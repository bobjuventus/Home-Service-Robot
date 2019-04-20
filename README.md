# GazeboWorld

[![Udacity - Robotics NanoDegree Program](https://s3-us-west-1.amazonaws.com/udacity-robotics/Extra+Images/RoboND_flag.png)](https://www.udacity.com/robotics)

# RoboND-Home Service Robot!
The **Home Service Robot** project is to combine multiple packages (`slam_gmapping` for SLAM,, `turtlebot` for turtlebot description and teleoperation, `turtlebot_interactions` for rviz config and markers, `turtlebot_gazebo` for launching the world and using amcl for localization, `turtlebot_navigation` for navigation).

Several script files were also created to test sub-tasks. Please see the directory structure below for more information.

### Directory Structure
```
    .Project                           # Home Service Robot Project
    ├──                                # Official ROS packages
    |
    ├── slam_gmapping                  # gmapping_demo.launch file                   
    │   ├── gmapping
    │   ├── ...
    ├── turtlebot                      # keyboard_teleop.launch file
    │   ├── turtlebot_teleop
    │   ├── ...
    ├── turtlebot_interactions         # view_navigation.launch file      
    │   ├── turtlebot_rviz_launchers
    │   ├── ...
    ├── turtlebot_simulator            # turtlebot_world.launch file 
    │   ├── turtlebot_gazebo
    │   ├── ...
    ├── turtlebot_navigation           # system installed navigation package for turtlebot 
    │   ├── param
    │       ├── dwa_local_planner_params.yaml      # this file needs to be modified to have proper control and Goal Tolerance Parameters
    │       ├── ...
    ├──                                # Your packages and direcotries
    |
    ├── map                            # map files
    │   ├── ...
    ├── scripts                        # shell scripts files
    │   ├── add_marker.sh
    │   ├── home_service.sh
    │   ├── pick_objects.sh
    │   ├── test_navigation.sh
    │   ├── test_slam.sh
    │   ├── ...
    ├──rvizConfig                      # rviz configuration files
    │   ├──                            # Notice this rviz file is also saved under turtlebot_interactions/turtlebot_rviz_launchers/rviz/ which is the one being called.
    ├──pick_objects                    # pick_objects C++ node
    │   ├── src/pick_objects.cpp
    │   ├── ...
    ├──add_markers                     # add_marker C++ node
    │   ├── src/add_markers.cpp
    │   ├── ...
    ├──home_service                    # home_service C++ node
    │   ├── src/home_service.cpp
    │   ├── ...
    └──                   
                             
```

### Using `RTAB-Map` package to perform SLAM

#### Steps:
* After launching the world (which in this case is the kitchen model), teleop_keyboard and rtabmap package, we drive robot around to generate the map. Notice that the `Grid/3D` and `Grid/FromDepth` parameters in the `rtabmap` package have to be true. Otherwise the map is not updated. (Maybe only one of these two parameters needs to be true)
![alt text](images/overall.png)
* While driving robot around, terminal will display time to time that it rejects a loop closure because the number of matching features has not reached threshold (15). Keep driving robot until the whole map is filled.
* After finishing driving robot around, close the rtabmap terminal and a `.db` file should be saved under `/root/.ros` folder. Run this command to view the file: `rtabmap-databaseViewer ~/.ros/rtabmap.db`. My db file size is around 225 MB, and can be downloaded from [here](https://www.amazon.com/clouddrive/share/JEVrLQkPEMqEXpeWf4og44LQVGOPVH3Uja91RRApwdv).
* We can see that there are 502 frames and 38 global loop closures. Below are three examples of global loop closures which we will look at the features closely soon.
![alt text](images/lc1.png)

![alt text](images/lc2.png)

![alt text](images/lc3.png)

* From the images we can see that the features are usually at corners, places with color changes or contour of specific shapes. Larger the purple circle is, higher confidence there is a match.
* Finally, we can display everything flat to look from top down.
![alt text](images/occupancygrid.png)

### Future Steps

* Not sure if the terminal outputs when there is a loop closure detected. Right now have to blindly drive around and hope there are enough loop closures.
* The map generated still has some noise after roaming robot around a few times. Is this related to the depth sensor resolution is not high enough?