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

#include <cstdio>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"

#include "pub_msgs/msg/pub_test.hpp"

void print_usage()
{
  printf("Usage for listener app:\n");
  printf("listener [-t topic_name] [-h]\n");
  printf("options:\n");
  printf("-h : Print this help function.\n");
  printf("-t topic_name : Specify the topic on which to subscribe. Defaults to chatter.\n");
}

// Create a Listener class that subclasses the generic rclcpp::Node base class.
// The main function below will instantiate the class as a ROS node.
class Listener : public rclcpp::Node
{
public:
  explicit Listener(const std::string & topic_name)
  : Node("listener")
  {
    // Create a callback function for when messages are received.
    // Variations of this function also exist using, for example UniquePtr for zero-copy transport.
    auto callback = [this](const pub_msgs::msg::PubTest::UniquePtr msg) -> void {
        // RCLCPP_INFO(this->get_logger(), "I heard: [%s]", msg->data.c_str());
        auto now = std::chrono::high_resolution_clock::now();
        auto nano_time_point = std::chrono::time_point_cast<std::chrono::nanoseconds>(now);
        auto epoch = nano_time_point.time_since_epoch();
        uint64_t now_nano = std::chrono::duration_cast<std::chrono::nanoseconds>(epoch).count();
        uint64_t delta = now_nano - msg->timestamp;
        time_sum += delta;
        printf("recv msg: %d, delta-time: %f\n", (int32_t)msg->data[0], (double)delta / 1000000.0f);
        if ((int32_t)msg->data[0] == 127) {
          printf("recv complete, average time: %f \d", time_sum / 128000000.0f);
        }
      };

    // Create a subscription to the topic which can be matched with one or more compatible ROS
    // publishers.
    // Note that not all publishers on the same topic with the same type will be compatible:
    // they must have compatible Quality of Service policies.
    sub_ = create_subscription<pub_msgs::msg::PubTest>(topic_name, 10, callback);
  }

private:
  rclcpp::Subscription<pub_msgs::msg::PubTest>::SharedPtr sub_;
  double time_sum = 0;
};

int main(int argc, char * argv[])
{
  // Force flush of the stdout buffer.
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

  // Create a node.
  auto node = std::make_shared<Listener>("chatter");

  // spin will block until work comes in, execute work as it becomes available, and keep blocking.
  // It will only be interrupted by Ctrl-C.
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
