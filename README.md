# aepl_task
AEPL VRX autonomous surface vehicle task

## Deliverables

* ~~VRX simulation setup~~
* ~~teleoperate~~
* ~~Use ground truth or GPS for state estimation~~
* ~~Retrieve LIDAR data~~
* ~~SLAM with 3D-LIDAR data~~
* autonomously control a surface vehicle(non-functional)
* Initiate and cancel mission using ros action-client(pending)

## Notes:

### Update: 2

* Slam toolbox has been integrated and mapping works.
* Navigation2 stack has been integrated but it is non-functional and moves the vehicle in opposite direction and then rotates along Z. Probably due to some TF issue.
* Starting and stopping the mission is also pending but is a minor addition once navigation2 is functional.



### Update: 1

* Here I am taking ground truth from gazebo using P3D plugin and then using my vrx_odom node to get relative odometry and updated TF to show the vehicle at the map center. Slow TF updates causes erratic vehicle updates.
* I have created a botched vrx_controller node that converts cmd_vel to individual thrust and pos for either aft thrusters.
* I am using Xbox controller and with some remapping in usv_joy_teleop node from VRX simulation environment for teleoperation.
* slam_toolbox for async_online mapping.
* Navigation2 for autonomous navigation.
* Nav2 collision monitor for stopping before collision.

Screen recording to demonstrate VVRX simulation setup with funtional teleop, and mapping. Autonomous navigation is still under works.

[demo1.webm](https://github.com/carpit680/aepl_task/assets/43350102/49e1cf6c-85f0-43fc-9932-1e7aad77453b)
