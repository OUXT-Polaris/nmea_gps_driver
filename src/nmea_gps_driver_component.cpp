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


#include <nmea_gps_driver/nmea_gps_driver_component.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <chrono>
#include <vector>
#include <string>
#include <sstream>
#include <memory>

namespace nmea_gps_driver
{
NmeaGpsDriverComponent::NmeaGpsDriverComponent(const rclcpp::NodeOptions & options)
: Node("nmea_gps_driver", options)
{
  declare_parameter("device_file", "/dev/ttyACM0");
  get_parameter("device_file", device_file_);
  declare_parameter("baud_rate", 9600);
  get_parameter("baud_rate", baud_rate_);
  publisher_ = this->create_publisher<nmea_msgs::msg::Sentence>("nmea_sentence", 1);
  port_ptr_ = std::make_shared<boost::asio::serial_port>(io_, device_file_);
  port_ptr_->set_option(boost::asio::serial_port_base::baud_rate(baud_rate_));
  port_ptr_->set_option(boost::asio::serial_port_base::character_size(8));
  port_ptr_->set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::
    flow_control::none));
  port_ptr_->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity
    ::none));
  port_ptr_->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::
    stop_bits::one));
  io_thread_ = boost::thread(boost::bind(&NmeaGpsDriverComponent::readSentence, this));
}

NmeaGpsDriverComponent::~NmeaGpsDriverComponent()
{
  io_thread_.join();
}

std::vector<std::string> NmeaGpsDriverComponent::split(std::string s, char delim)
{
  std::vector<std::string> elems;
  std::stringstream ss(s);
  std::string item;
  while (getline(ss, item, delim)) {
    if (!item.empty()) {
      elems.push_back(item);
    }
  }
  return elems;
}

void NmeaGpsDriverComponent::readSentence()
{
  while (rclcpp::ok()) {
    buf_ = boost::array<char, 256>();
    port_ptr_->read_some(boost::asio::buffer(buf_));
    std::string data(buf_.begin(), buf_.end());
    nmea_msgs::msg::Sentence sentence;
    sentence.sentence = data;
    publisher_->publish(sentence);
  }
}
}  // namespace nmea_gps_driver

RCLCPP_COMPONENTS_REGISTER_NODE(nmea_gps_driver::NmeaGpsDriverComponent)
