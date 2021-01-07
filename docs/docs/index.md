# nmea gps driver

ROS2 driver for nmea gps ![ROS2-Foxy](https://github.com/OUXT-Polaris/nmea_gps_driver/workflows/ROS2-Foxy/badge.svg)

![Developed By OUXT Polaris](image/logo.png "Logo")

## components
1. nmea_gps_driver::NmeaGpsDriverComponent  
this driver component reads serial nmea sentense data from GPS and publish them as a nmea_sentence topic. (nmea_msgs/Sentence)

## nodes
1. nmea_gps_driver_node
this node reads serial nmea sentense data from GPS and publish them as a nmea_sentence topic. (nmea_msgs/Sentence)