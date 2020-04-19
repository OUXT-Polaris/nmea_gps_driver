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
  declare_parameter("frame_id", "gps");
  get_parameter("frame_id", frame_id_);
  publisher_ = this->create_publisher<nmea_msgs::msg::Sentence>("nmea_sentence", 1);
  connectSerialPort();
  using namespace std::chrono_literals;
  timer_ = create_wall_timer(1000ms, std::bind(&NmeaGpsDriverComponent::timerCallback, this));
}

NmeaGpsDriverComponent::~NmeaGpsDriverComponent()
{
  io_thread_.join();
}

bool NmeaGpsDriverComponent::validatecheckSum(std::string sentence)
{
  auto splited_sentence = split(sentence, '*');
  if (splited_sentence.size() != 2) {
    return false;
  }
  uint8_t checksum = 0;
  std::string xor_target_str = splited_sentence[0].substr(1, splited_sentence[0].size() - 1);
  for (int i = 0; i < static_cast<int>(xor_target_str.size()); i++) {
    int c = xor_target_str[i];
    checksum ^= c;
  }
  uint8_t rest = checksum % 16;
  uint8_t quotient = (checksum - rest) / 16;
  std::string ret = getHexString(quotient) + getHexString(rest);
  if (ret == splited_sentence[1]) {
    return true;
  }
  std::string message = "checksum does not match in calculating sentence :" + sentence +
    " calculated checksum is " + ret;
  RCLCPP_DEBUG(get_logger(), message);
  return false;
}

std::string NmeaGpsDriverComponent::getHexString(uint8_t value)
{
  std::string ret;
  if (value == 10) {
    ret = "A";
  } else if (value == 11) {
    ret = "B";
  } else if (value == 12) {
    ret = "C";
  } else if (value == 13) {
    ret = "D";
  } else if (value == 14) {
    ret = "E";
  } else if (value == 15) {
    ret = "F";
  } else {
    ret = std::to_string(value);
  }
  return ret;
}

boost::optional<std::string> NmeaGpsDriverComponent::validate(std::string sentence)
{
  try {
    sentence = "$" + sentence;
    std::stringstream ss1{sentence};
    if (std::getline(ss1, sentence)) {
      std::stringstream ss2{sentence};
      if (std::getline(ss2, sentence, '\r')) {
        if (validatecheckSum(sentence)) {
          return sentence;
        }
        return boost::none;
      }
    }
  } catch (const std::exception & e) {
    std::string message = "while processing : " + sentence + " : " + e.what();
    RCLCPP_WARN(get_logger(), message);
    return boost::none;
  }
  return sentence;
}

void NmeaGpsDriverComponent::connectSerialPort()
{
  try {
    port_ptr_ = std::make_shared<boost::asio::serial_port>(io_, device_file_);
    port_ptr_->set_option(boost::asio::serial_port_base::baud_rate(baud_rate_));
    port_ptr_->set_option(boost::asio::serial_port_base::character_size(8));
    port_ptr_->set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base
      ::flow_control::none));
    port_ptr_->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::
      parity::none));
    port_ptr_->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::
      stop_bits::one));
    io_thread_ = boost::thread(boost::bind(&NmeaGpsDriverComponent::readSentence, this));
    connected_ = true;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), e.what());
    connected_ = false;
  }
}

void NmeaGpsDriverComponent::timerCallback()
{
  if (!connected_) {
    connectSerialPort();
  }
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
    try {
      port_ptr_->read_some(boost::asio::buffer(buf_));
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), e.what());
      connected_ = false;
      return;
    }
    std::string data(buf_.begin(), buf_.end());
    std::vector<std::string> splited_sentence = split(data, '$');
    rclcpp::Time time = get_clock()->now();
    for (auto itr = splited_sentence.begin(); itr != splited_sentence.end(); itr++) {
      auto line = validate(*itr);
      if (line) {
        nmea_msgs::msg::Sentence sentence;
        sentence.header.frame_id = frame_id_;
        sentence.header.stamp = time;
        sentence.sentence = line.get();
        publisher_->publish(sentence);
      }
    }
  }
}
}  // namespace nmea_gps_driver

RCLCPP_COMPONENTS_REGISTER_NODE(nmea_gps_driver::NmeaGpsDriverComponent)
