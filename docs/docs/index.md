# nmea gps driver

ROS2 driver for nmea gps 

![ROS2-Foxy](https://github.com/OUXT-Polaris/nmea_gps_driver/workflows/ROS2-Foxy/badge.svg)

![Developed By OUXT Polaris](image/logo.png "Logo")

## Checked device

### F9P Evaluation Kit from CQ Publisher
[URL](https://shop.cqpub.co.jp/hanbai/books/I/I000316.htm)


## ROS2 Components  

### nmea_gps_driver::NmeaGpsDriverComponent  

this driver component reads serial nmea sentense data from GPS and publish them as a "/nmea_sentence" topic. (nmea_msgs/Sentence)

| topic | type | input/output | description |
| ---- | ---- | ---- | ---- |
| /nmea_sentense | [nmea_msgs/msg/Sentense](https://github.com/ros-drivers/nmea_msgs/blob/ros2/msg/Sentence.msg) | output | nmea sentense from GPS devices |

| parameter | type | default | description | dynamic_reconfigure |
| ---- | ---- | ---- | ---- | ---- |
| device_file | string | /dev/ttyACM0 | device file of the GPS | NO |
| baud_rate | int | 9600 | baud rate of the serial communication | NO |

## ROS2 Nodes  

### nmea_gps_driver_node  

this node reads serial nmea sentense data from GPS and publish them as a "nmea_sentence" topic. (nmea_msgs/Sentence)

| topic | type | input/output | description |
| ---- | ---- | ---- | ---- |
| /nmea_sentense | [nmea_msgs/msg/Sentense](https://github.com/ros-drivers/nmea_msgs/blob/ros2/msg/Sentence.msg) | output | nmea sentense from GPS devices |

| parameter | type | default | description | dynamic_reconfigure |
| ---- | ---- | ---- | ---- | ---- |
| device_file | string | /dev/ttyACM0 | device file of the GPS | NO |
| baud_rate | int | 9600 | baud rate of the serial communication | NO |
