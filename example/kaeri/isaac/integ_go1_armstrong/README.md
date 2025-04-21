# message
Publications: 
 * /armstrong/d455/camera_info [sensor_msgs/CameraInfo]
 * /armstrong/d455/depth [sensor_msgs/Image]
 * /armstrong/d455/depth_pcl [sensor_msgs/PointCloud2]
 * /armstrong/d455/rgb [sensor_msgs/Image]
 * /go1/d455_front/camera_info [sensor_msgs/CameraInfo]
 * /go1/d455_front/depth [sensor_msgs/Image]
 * /go1/d455_front/depth_pcl [sensor_msgs/PointCloud2]
 * /go1/d455_front/rgb [sensor_msgs/Image]
 * /go1/d455_left/camera_info [sensor_msgs/CameraInfo]
 * /go1/d455_left/depth [sensor_msgs/Image]
 * /go1/d455_left/depth_pcl [sensor_msgs/PointCloud2]
 * /go1/d455_left/rgb [sensor_msgs/Image]
 * /go1/d455_rear/camera_info [sensor_msgs/CameraInfo]
 * /go1/d455_rear/depth [sensor_msgs/Image]
 * /go1/d455_rear/depth_pcl [sensor_msgs/PointCloud2]
 * /go1/d455_rear/rgb [sensor_msgs/Image]
 * /go1/d455_right/camera_info [sensor_msgs/CameraInfo]
 * /go1/d455_right/depth [sensor_msgs/Image]
 * /go1/d455_right/depth_pcl [sensor_msgs/PointCloud2]
 * /go1/d455_right/rgb [sensor_msgs/Image]
 * /go1/imu [sensor_msgs/Imu]
 * /go1/odom_sim [nav_msgs/Odometry]
 * /go1/point_cloud [sensor_msgs/PointCloud2]
 * /tf [tf2_msgs/TFMessage]

Subscriptions: 
 * /armstrong/cmd_vel [geometry_msgs/Twist]
 * /go1/cmd_vel [geometry_msgs/Twist]

# how to move robot
```
cd <catkin_ws>/src
git clone http://183.107.37.79:13003/kaeri_ert/navigation/teleop.git
cd <catkin_ws>
catkin_make
rosrun teleop teleop.py _topic:=/armstrong/cmd_vel
```