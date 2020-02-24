// Copyright 2014 Open Source Robotics Foundation, Inc.
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

#include <chrono>
#include <cstdio>
#include <memory>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"
#include "pub_msgs/msg/pub_test.hpp"

using namespace std::chrono_literals;

void print_usage()
{
  printf("Usage for talker app:\n");
  printf("talker [-t topic_name] [-h]\n");
  printf("options:\n");
  printf("-h : Print this help function.\n");
  printf("-t topic_name : Specify the topic on which to publish. Defaults to chatter.\n");
}

// Create a Talker class that subclasses the generic rclcpp::Node base class.
// The main function below will instantiate the class as a ROS node.
class Talker : public rclcpp::Node
{
public:
  explicit Talker(const std::string & topic_name, int32_t msgsize, int32_t interval)
  : Node("talker")
  {
    msg_size_ = msgsize;
    interval_ = interval;

    // Create a publisher with a custom Quality of Service profile.
    // rclcpp::QoS qos(rclcpp::KeepLast(7));
    pub_ = this->create_publisher<pub_msgs::msg::PubTest>(topic_name, rmw_qos_profile_default);
    std::weak_ptr<std::remove_pointer<decltype(pub_.get())>::type> captured_pub = pub_;

    // Create a function for when messages are to be sent.
    auto publish_message =[this, captured_pub]() -> void {
        auto sptr = captured_pub.lock();
        if (!sptr) return;

        this->msg_ = std::make_unique<pub_msgs::msg::PubTest>();
        this->msg_->data.resize(msg_size_ - 8);
        memset(&(this->msg_->data[0]), (uint8_t)this->count_, msg_size_ - 8);
        auto now = std::chrono::high_resolution_clock::now();
        auto nano_time_point = std::chrono::time_point_cast<std::chrono::nanoseconds>(now);
        auto epoch = nano_time_point.time_since_epoch();
        uint64_t now_nano = std::chrono::duration_cast<std::chrono::nanoseconds>(epoch).count();
        this->msg_->timestamp = now_nano;

        // msg_->data = "Hello World: " + std::to_string(count_++);
        // RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", msg_->data.c_str());

        // Put the message into a queue to be processed by the middleware.
        // This call is non-blocking.
        sptr->publish(std::move(msg_));

        printf("pub msg seq: %d\n", this->count_);
        this->count_++;

        if (this->count_ == 128) {
          this->timer_->cancel();
          printf("send complete\n");
        }
      };

    // Use a timer to schedule periodic message publishing.
    timer_ = this->create_wall_timer(std::chrono::milliseconds(interval_), publish_message);
  }

private:
  size_t count_ = 0;
  std::unique_ptr<pub_msgs::msg::PubTest> msg_;
  rclcpp::Publisher<pub_msgs::msg::PubTest>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  int32_t msg_size_;
  int32_t interval_;
};

int main(int argc, char * argv[])
{
  // Force flush of the stdout buffer.
  // This ensures a correct sync of all prints
  // even when executed simultaneously within the launch file.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  // if (rcutils_cli_option_exist(argv, argv + argc, "-h")) {
  //   print_usage();
  //   return 0;
  // }

  // Initialize any global resources needed by the middleware and the client library.
  // You must call this before using any other part of the ROS system.
  // This should be called once per process.
  rclcpp::init(argc, argv);

  // Parse the command line options.
  // auto topic = std::string("chatter");
  // char * cli_option = rcutils_cli_get_option(argv, argv + argc, "-t");
  // if (nullptr != cli_option) {
  //   topic = std::string(cli_option);
  // }

  if (argc < 3) {
    printf("invalid cmd param!\n");
    return -1;
  }

  int32_t msg_size = std::stoi(argv[1]);
  int32_t interval = std::stoi(argv[2]);
  if (msg_size <= 8) {
    printf("invalid msg size param!\n");
    return -1;
  }

  // Create a node.
  auto node = std::make_shared<Talker>("chatter", msg_size, interval);

  // spin will block until work comes in, execute work as it becomes available, and keep blocking.
  // It will only be interrupted by Ctrl-C.
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
