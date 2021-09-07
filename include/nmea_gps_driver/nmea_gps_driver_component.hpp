// Copyright (c) 2020 OUXT Polaris
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NMEA_GPS_DRIVER__NMEA_GPS_DRIVER_COMPONENT_HPP_
#define NMEA_GPS_DRIVER__NMEA_GPS_DRIVER_COMPONENT_HPP_

#if __cplusplus
extern "C" {
#endif

// The below macros are taken from https://gcc.gnu.org/wiki/Visibility and from
// demos/composition/include/composition/visibility_control.h at https://github.com/ros2/demos
#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define NMEA_GPS_DRIVER_NMEA_GPS_DRIVER_COMPONENT_EXPORT __attribute__((dllexport))
#define NMEA_GPS_DRIVER_NMEA_GPS_DRIVER_COMPONENT_IMPORT __attribute__((dllimport))
#else
#define NMEA_GPS_DRIVER_NMEA_GPS_DRIVER_COMPONENT_EXPORT __declspec(dllexport)
#define NMEA_GPS_DRIVER_NMEA_GPS_DRIVER_COMPONENT_IMPORT __declspec(dllimport)
#endif
#ifdef NMEA_GPS_DRIVER_NMEA_GPS_DRIVER_COMPONENT_BUILDING_DLL
#define NMEA_GPS_DRIVER_NMEA_GPS_DRIVER_COMPONENT_PUBLIC \
  NMEA_GPS_DRIVER_NMEA_GPS_DRIVER_COMPONENT_EXPORT
#else
#define NMEA_GPS_DRIVER_NMEA_GPS_DRIVER_COMPONENT_PUBLIC \
  NMEA_GPS_DRIVER_NMEA_GPS_DRIVER_COMPONENT_IMPORT
#endif
#define NMEA_GPS_DRIVER_NMEA_GPS_DRIVER_COMPONENT_PUBLIC_TYPE \
  NMEA_GPS_DRIVER_NMEA_GPS_DRIVER_COMPONENT_PUBLIC
#define NMEA_GPS_DRIVER_NMEA_GPS_DRIVER_COMPONENT_LOCAL
#else
#define NMEA_GPS_DRIVER_NMEA_GPS_DRIVER_COMPONENT_EXPORT __attribute__((visibility("default")))
#define NMEA_GPS_DRIVER_NMEA_GPS_DRIVER_COMPONENT_IMPORT
#if __GNUC__ >= 4
#define NMEA_GPS_DRIVER_NMEA_GPS_DRIVER_COMPONENT_PUBLIC __attribute__((visibility("default")))
#define NMEA_GPS_DRIVER_NMEA_GPS_DRIVER_COMPONENT_LOCAL __attribute__((visibility("hidden")))
#else
#define NMEA_GPS_DRIVER_NMEA_GPS_DRIVER_COMPONENT_PUBLIC
#define NMEA_GPS_DRIVER_NMEA_GPS_DRIVER_COMPONENT_LOCAL
#endif
#define NMEA_GPS_DRIVER_NMEA_GPS_DRIVER_COMPONENT_PUBLIC_TYPE
#endif

#if __cplusplus
}  // extern "C"
#endif

#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/optional.hpp>
#include <boost/thread.hpp>
#include <memory>
#include <nmea_msgs/msg/sentence.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

namespace nmea_gps_driver
{
class NmeaGpsDriverComponent : public rclcpp::Node
{
public:
  NMEA_GPS_DRIVER_NMEA_GPS_DRIVER_COMPONENT_PUBLIC
  explicit NmeaGpsDriverComponent(const rclcpp::NodeOptions & options);
  ~NmeaGpsDriverComponent();

private:
  rclcpp::Publisher<nmea_msgs::msg::Sentence>::SharedPtr publisher_;
  std::string device_file_;
  int baud_rate_;
  std::string frame_id_;
  boost::asio::io_service io_;
  std::shared_ptr<boost::asio::serial_port> port_ptr_;
  boost::thread io_thread_;
  void readSentence();
  boost::array<char, 256> buf_;
  std::vector<std::string> split(std::string s, char delim);
  void connectSerialPort();
  bool connected_ = false;
  void timerCallback();
  rclcpp::TimerBase::SharedPtr timer_;
  boost::optional<std::string> validate(std::string sentence);
  bool validatecheckSum(std::string sentence);
  std::string getHexString(uint8_t value);
};
}  // namespace nmea_gps_driver

#endif  // NMEA_GPS_DRIVER__NMEA_GPS_DRIVER_COMPONENT_HPP_
