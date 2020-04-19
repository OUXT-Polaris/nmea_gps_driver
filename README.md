# nmea gps driver

ROS2 driver for nmea gps ![ROS2-Eloquent](https://github.com/OUXT-Polaris/nmea_gps_driver/workflows/ROS2-Eloquent/badge.svg)

![Developed By OUXT Polaris](img/logo.png "Logo")

## components
1. nmea_gps_driver::NmeaGpsDriverComponent  
this driver component reads serial nmea sentense data from GPS and publish them as a nmea_sentence topic. (nmea_msgs/Sentence)

## nodes
1. nmea_gps_driver_node
this node reads serial nmea sentense data from GPS and publish them as a nmea_sentence topic. (nmea_msgs/Sentence)