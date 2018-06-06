![Screenshot](/kitti.jpg)

Sample map built from [Kitti Raw Data Sequence 05](http://kitti.is.tue.mpg.de/kitti/raw_data/2011_09_30_drive_0018/2011_09_30_drive_0018_sync.zip)

:white_check_mark: Tested with ROS Kinetic Running on Ubuntu 16.04, Velodyne VLP16.

All sources were taken from [ROS documentation](http://docs.ros.org/indigo/api/loam_velodyne/html/files.html)

This is a reimplementation of the original paper [LOAM](http://www.roboticsproceedings.org/rss10/p07.pdf)

[This](https://github.com/laboshinl/loam_velodyne) is the main reference of this project. The project is for study purpose only.

How to build with catkin:

```
$ cd ~/catkin_ws/src/
$ git clone https://github.com/umrichen/loam_velodyne.git
$ cd ~/catkin_ws
$ catkin_make -DCMAKE_BUILD_TYPE=Release 
$ source ~/catkin_ws/devel/setup.bash
```

Running:
```
roslaunch loam_velodyne loam_velodyne.launch
```

Then choose one of the following:

In second terminal play sample velodyne data from rosbag, for example [VLP16 rosbag](https://db.tt/t2r39mjZ):
```
rosbag play /path/to/velodyne.bag 
```

Or read from velodyne .pcap file, for example [VLP16 sample pcap](https://midas3.kitware.com/midas/folder/12979):
```
roslaunch velodyne_pointcloud VLP16_points.launch pcap:="/full/path/to/velodyne.pcap"
```
Note: using full path for pcap file is required.

---

Note on converting Kitti dataset: in kitti2bag/bin/kitti2bag, remove save_camera_data() function call to avoid error in saving camera frames.
