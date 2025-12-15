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

#include <rmf_ingestor_msgs/msg/ingestor_request.hpp>
#include <rmf_ingestor_msgs/msg/ingestor_result.hpp>
#include <rmf_ingestor_msgs/msg/ingestor_state.hpp>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <unordered_set>
#include <curl/curl.h>
#include <nlohmann/json.hpp>

using namespace std::chrono_literals;
using json = nlohmann::json;

using rmf_ingestor_msgs::msg::IngestorRequest;
using rmf_ingestor_msgs::msg::IngestorResult;
using rmf_ingestor_msgs::msg::IngestorState;

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

class MoveItFollowTarget : public rclcpp::Node
{
public:
  /// Constructor
  MoveItFollowTarget();

private:
  void ingestor_request_callback(const IngestorRequest::ConstSharedPtr msg);

  void timer_callback();

  void publish_ingestor_state(bool busy);
  void publish_ingestor_result(uint8_t result);

  // RMF ingestor interfaces
  rclcpp::Subscription<IngestorRequest>::SharedPtr ingestor_request_sub_;
  rclcpp::Publisher<IngestorResult>::SharedPtr ingestor_result_pub_;
  rclcpp::Publisher<IngestorState>::SharedPtr ingestor_state_pub_;

  rclcpp::TimerBase::SharedPtr timer_;
  CURL *curl_;
  CURLcode res_;
  std::unordered_set<std::string> past_request_guids_;
  std::string last_request_guid_;
};

MoveItFollowTarget::MoveItFollowTarget() : Node("moveit_ingestor_dummy")
{
  // Use upper joint velocity and acceleration limits
  // TODO(luca) Add the poses to the robot here instead of the srdf

  ingestor_state_pub_ = this->create_publisher<IngestorState>("/ingestor_states", rclcpp::QoS(10));
  ingestor_result_pub_ = this->create_publisher<IngestorResult>("/ingestor_results", rclcpp::QoS(10));
  ingestor_request_sub_ = this->create_subscription<IngestorRequest>("/ingestor_requests", rclcpp::QoS(1), std::bind(&MoveItFollowTarget::ingestor_request_callback, this, std::placeholders::_1));
  timer_ = this->create_wall_timer(500ms, std::bind(&MoveItFollowTarget::timer_callback, this));
  curl_ = curl_easy_init();
  RCLCPP_INFO(this->get_logger(), "Initialization successful.");
}

void MoveItFollowTarget::timer_callback()
{
  // Publish a dispenser state
  this->publish_ingestor_state(false);
}

void MoveItFollowTarget::publish_ingestor_state(bool busy)
{
  IngestorState msg;
  msg.time = this->get_clock()->now();
  msg.guid = "moveit_ingestor_dummy";
  if (busy)
  {
    msg.mode = msg.BUSY;
    msg.request_guid_queue = {this->last_request_guid_};
  }
  else
  {
    msg.mode = msg.IDLE;
  }
  ingestor_state_pub_->publish(msg);
}

void MoveItFollowTarget::publish_ingestor_result(uint8_t result)
{
  IngestorResult msg;
  msg.time = this->get_clock()->now();
  msg.source_guid = "moveit_ingestor_dummy";
  msg.request_guid = this->last_request_guid_;
  msg.status = result;
  ingestor_result_pub_->publish(msg);
}

void MoveItFollowTarget::ingestor_request_callback(const IngestorRequest::ConstSharedPtr msg)
{
  if (msg->target_guid != "moveit_ingestor_dummy")
    return;
  if (past_request_guids_.find(msg->request_guid) != past_request_guids_.end())
    return;
  this->last_request_guid_ = msg->request_guid;
  this->past_request_guids_.insert(msg->request_guid);
  this->publish_ingestor_result(IngestorResult::ACKNOWLEDGED);
  this->publish_ingestor_state(true);
  RCLCPP_INFO(this->get_logger(), "Received ingestor request");
  RCLCPP_INFO(this->get_logger(), "ingestor task succeeded.");
  this->publish_ingestor_result(IngestorResult::SUCCESS);

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
