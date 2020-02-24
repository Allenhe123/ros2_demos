#include <iostream>
#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/byte_multi_array.hpp"

using namespace std;
using namespace std::chrono_literals;

uint32_t num = 0;
double g_time_sum = 0;

std::chrono::time_point<std::chrono::high_resolution_clock> g_send_time;

struct SendNode : public rclcpp::Node
{
  SendNode(const std::string & name, const std::string & topic, int len, int interval): Node(name, rclcpp::NodeOptions().use_intra_process_comms(true))
  {
    msg_size_ = len;
    interval_ = interval;
    // Create a publisher on the output topic.
    pub_ = this->create_publisher<std_msgs::msg::ByteMultiArray>(topic, rmw_qos_profile_default);
    std::weak_ptr<std::remove_pointer<decltype(pub_.get())>::type> captured_pub = pub_;

    auto doit = [captured_pub, this] () -> void {
        auto sptr = captured_pub.lock();
        if (!sptr) return;

        std_msgs::msg::ByteMultiArray::UniquePtr msg(new std_msgs::msg::ByteMultiArray());
        msg->data.resize(this->msg_size_);
        // memcpy(&(msg->data[0]), (unsigned char)num, this->msg_size_);
        for (uint32_t i=0; i<this->msg_size_; i++)
            msg->data[i] = (unsigned char)num;

        g_send_time = chrono::high_resolution_clock::now();
        sptr->publish(std::move(msg));

        cout << "send msg: " << num << endl;
        num++;

        if (num == 256) {
            cout << "###have sent 256 msgs, stop" << endl;
            this->timer_->cancel();
        }
    };

    timer_ = this->create_wall_timer(std::chrono::milliseconds(interval_), doit);
  }

  rclcpp::Publisher<std_msgs::msg::ByteMultiArray>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  int msg_size_;
  int interval_;
};

struct RecvNode : public rclcpp::Node
{
  RecvNode(const std::string & name, const std::string & topic, int len): Node(name, rclcpp::NodeOptions().use_intra_process_comms(true))
  {
      msg_size_ = len;
    // Create a subscription on the input topic.
    sub_ = this->create_subscription<std_msgs::msg::ByteMultiArray>( topic,
        [this](std_msgs::msg::ByteMultiArray::UniquePtr msg) {
            cout << "recv msg: " << (uint32_t)msg->data[this->msg_size_ - 1]<< endl;

            auto recv_time = chrono::high_resolution_clock::now();
            // std::chrono::duration<double> elapsed_seconds = recv_time - g_send_time;
            auto delta = std::chrono::duration_cast<std::chrono::microseconds>(recv_time - g_send_time);
            g_time_sum += delta.count();
            cout <<" delta time: " << delta.count() / 1000.0 << " ms" << endl;

            if ((uint32_t)msg->data[0] == 255)
                cout << "###latency: " << g_time_sum / 256000.0 << " ms" << endl;
        
        }, rmw_qos_profile_default);
  }

  rclcpp::Subscription<std_msgs::msg::ByteMultiArray>::SharedPtr sub_;
  int32_t msg_size_;
};

int main(int argc, char * argv[])
{
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor executor;

    cout << "msg size:" << argv[1] << endl;

    auto sender = std::make_shared<SendNode>("send_node", "TOPIC", std::stoi(argv[1]), std::stoi(argv[2]));
    auto recver = std::make_shared<RecvNode>("recv_node", "TOPIC", std::stoi(argv[1]));

    executor.add_node(sender);
    executor.add_node(recver);
    executor.spin();

    rclcpp::shutdown();

    return 0;
}