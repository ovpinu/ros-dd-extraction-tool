# ros-dd-extraction-tool
The following tool extracts driving data stored in rosbag files.

Initial ROS package setup:
```
$ cd driving_data
$ catkin_make
$ source devel/setup.bash
```
----------------

rosbag_data_extract_unsync is a python script that subscribes to the following topics:

/camera0/image_raw	: sensor_msgs/Image      
/camera1/image_raw	: sensor_msgs/Image      
/camera2/image_raw	: sensor_msgs/Image      
/can_raw		        : kvaser/CANPacket       
/nmea_sentence 		  : nmea_msgs/Sentence     
/points_raw		      : sensor_msgs/PointCloud2

The script stores image files as png, can data as binary files, nmea_sentence in a csv file and points_raw as pcd files. Data is not synchronized. Each message has its own timestamp.

###Example of usage:
```
$ python driving_data/src/driving_data_tools/scripts/rosbag_data_extract_unsync.py output_directory/
```
The parameter specifies the path into which data will be saved.

Requirements:

- ROS
- Python 2.7
- pcl 1.7
- Cython 0.21
- NumPy 1.8

----------------

rosbag_data_extract is a python script that subscribes to the following topics:

/camera0/image_raw	: sensor_msgs/Image      
/camera1/image_raw	: sensor_msgs/Image      
/camera2/image_raw	: sensor_msgs/Image      
/can_raw		        : kvaser/CANPacket       
/nmea_sentence 		  : nmea_msgs/Sentence     
/points_raw	      	: sensor_msgs/PointCloud2

The script stores image files as png, can data as binary files, nmea_sentence in a csv file and points_raw as pcd files. All data is synchronized, so messages containing important data detail might be lost.

###Example of usage:
```
$ python driving_data/src/driving_data_tools/scripts/rosbag_data_extract.py output_directory/
```
The parameter specifies the path into which data will be saved.

Requirements:

- ROS
- Python 2.7
- pcl 1.7
- Cython 0.21
- NumPy 1.8
