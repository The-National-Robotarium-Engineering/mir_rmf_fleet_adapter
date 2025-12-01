// Copyright 2024 Open Source Robotics Foundation, Inc.
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

#include <rmf_dispenser_msgs/msg/dispenser_request.hpp>
#include <rmf_dispenser_msgs/msg/dispenser_result.hpp>
#include <rmf_dispenser_msgs/msg/dispenser_state.hpp>

#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <unordered_set>
#include <curl/curl.h>
#include <nlohmann/json.hpp>

using namespace std::chrono_literals;
using json = nlohmann::json;

using rmf_dispenser_msgs::msg::DispenserRequest;
using rmf_dispenser_msgs::msg::DispenserResult;
using rmf_dispenser_msgs::msg::DispenserState;

/* PRE_GRASP
 * -11
 * 17
 * -2
 *  -114
 *  1
 *  130
 *  0
*/


/* GRASP
 * -11
 * 30
 * 0
 *  -116
 *  0
 *  145
 *  0
*/

/* PRE_DROP
 * 56
 * 61
 * 24
 * -102
 * -48
 * 152
 *  0
*/

/* DROP
 * 59
 * 72
 * 20
 * -92
 *  -50
 *  155
 *  0
*/
static size_t WriteCallback(void *contents, size_t size, size_t nmemb, void *userp)
{
    ((std::string*)userp)->append((char*)contents, size * nmemb);
    return size * nmemb;
}

class MoveItFollowTarget : public rclcpp::Node
{
public:
  /// Constructor
  MoveItFollowTarget();

private:
  void dispenser_request_callback(const DispenserRequest::ConstSharedPtr msg);

  void timer_callback();
  void publish_dispenser_state(bool busy);
  void publish_dispenser_result(uint8_t result);

  /// RMF dispenser interfaces
  rclcpp::Subscription<DispenserRequest>::SharedPtr dispenser_request_sub_;
  rclcpp::Publisher<DispenserResult>::SharedPtr dispenser_result_pub_;
  rclcpp::Publisher<DispenserState>::SharedPtr dispenser_state_pub_;

  rclcpp::TimerBase::SharedPtr timer_;
  CURL *curl_;
  CURLcode res_;
  std::unordered_set<std::string> past_request_guids_;
  std::string last_request_guid_;
};

MoveItFollowTarget::MoveItFollowTarget() : Node("moveit_dispenser")
{
  // Use upper joint velocity and acceleration limits
  // TODO(luca) Add the poses to the robot here instead of the srdf

  dispenser_state_pub_ = this->create_publisher<DispenserState>("/dispenser_states", rclcpp::QoS(10));
  dispenser_result_pub_ = this->create_publisher<DispenserResult>("/dispenser_results", rclcpp::QoS(10));
  dispenser_request_sub_ = this->create_subscription<DispenserRequest>("/dispenser_requests", rclcpp::QoS(1), std::bind(&MoveItFollowTarget::dispenser_request_callback, this, std::placeholders::_1));
  timer_ = this->create_wall_timer(500ms, std::bind(&MoveItFollowTarget::timer_callback, this));
  curl_ = curl_easy_init();
  RCLCPP_INFO(this->get_logger(), "Initialization successful.");
}

void MoveItFollowTarget::timer_callback()
{
  // Publish a dispenser state
  this->publish_dispenser_state(false);
}

void MoveItFollowTarget::publish_dispenser_state(bool busy)
{
  DispenserState msg;
  msg.time = this->get_clock()->now();
  msg.guid = "moveit_dispenser";
  if (busy)
  {
    msg.mode = msg.BUSY;
    msg.request_guid_queue = {this->last_request_guid_};
  }
  else
  {
    msg.mode = msg.IDLE;
  }
  dispenser_state_pub_->publish(msg);
}

void MoveItFollowTarget::publish_dispenser_result(uint8_t result)
{
  DispenserResult msg;
  msg.time = this->get_clock()->now();
  msg.source_guid = "moveit_dispenser";
  msg.request_guid = this->last_request_guid_;
  msg.status = result;
  dispenser_result_pub_->publish(msg);
}

void MoveItFollowTarget::dispenser_request_callback(const DispenserRequest::ConstSharedPtr msg)
{
  if (msg->target_guid != "moveit_dispenser")
    return;
  if (past_request_guids_.find(msg->request_guid) != past_request_guids_.end())
    return;
  this->last_request_guid_ = msg->request_guid;
  this->past_request_guids_.insert(msg->request_guid);
  this->publish_dispenser_result(DispenserResult::ACKNOWLEDGED);
  this->publish_dispenser_state(true);
  RCLCPP_INFO(this->get_logger(), "Received dispenser request");
  std::string readBuffer;
  curl_easy_setopt(curl_, CURLOPT_URL, "http://localhost:5000/load");
  // curl_easy_setopt(curl_, CURLOPT_URL, "http://localhost:5000/unload");
  curl_easy_setopt(curl_, CURLOPT_WRITEFUNCTION, WriteCallback);
  curl_easy_setopt(curl_, CURLOPT_WRITEDATA, &readBuffer);
  res_ = curl_easy_perform(curl_);
  curl_easy_cleanup(curl_);
  json parsed_json = json::parse(readBuffer);
  std::cout << parsed_json["msg"] << std::endl; //TODO: move to dispenser callback and test
  this->publish_dispenser_state(false);
  if (parsed_json["success"] == true)
  {
    RCLCPP_INFO(this->get_logger(), "Dispenser task succeeded.");
    this->publish_dispenser_result(DispenserResult::SUCCESS);
  }
  else
  {
    RCLCPP_INFO(this->get_logger(), "Dispenser task failed.");
    this->publish_dispenser_result(DispenserResult::FAILED);
  }
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  auto target_follower = std::make_shared<MoveItFollowTarget>();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(target_follower);
  executor.spin();

  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
