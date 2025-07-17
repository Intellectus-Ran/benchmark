#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <chrono>
#include <vector>
#include <string>
#include <cstdlib>  // for atoi

class BytePublisher : public rclcpp::Node
{
public:
    BytePublisher(int interval_ms, size_t message_size, int sample_count)
    : Node("byte_publisher"), count_(0),
      interval_ms_(interval_ms), message_size_(message_size), sample_count_(sample_count)
    {
        publisher_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>("ros_benchmark", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(interval_ms_),
            std::bind(&BytePublisher::publish_bytes, this));
    }

private:
    void publish_bytes()
    {
        std_msgs::msg::UInt8MultiArray msg;
        
        if (count_ < sample_count_) {
            msg.data.resize(message_size_);
            
            publisher_->publish(msg);
            count_++;
            RCLCPP_INFO(this->get_logger(), "Published message %d with size: %zu", count_, msg.data.size());
        } 
        else if (count_ == sample_count_) {
            // 빈 메시지 전송 (샘플 카운트 도달 시)
            msg.data.clear();
            publisher_->publish(msg);
            count_++;
            RCLCPP_INFO(this->get_logger(), "Finished sending %d messages. Sent empty message.", sample_count_);
        }
        else {
            RCLCPP_INFO(this->get_logger(), "Shutting down...");
            rclcpp::shutdown();
        }
    }

    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    int count_;
    int interval_ms_;
    size_t message_size_;
    int sample_count_;
};

// 명령줄 인자 파싱 함수
void parse_args(int argc, char* argv[], int &interval_ms, size_t &message_size, int &sample_count)
{
    for (int i = 1; i < argc; ++i) {
        std::string arg(argv[i]);
        if (arg == "-i" && i + 1 < argc) {
            interval_ms = std::atoi(argv[++i]);
        } else if (arg == "-m" && i + 1 < argc) {
            message_size = static_cast<size_t>(std::atoi(argv[++i]));
        } else if (arg == "-s" && i + 1 < argc) {
            sample_count = std::atoi(argv[++i]);
        }
    }
}

int main(int argc, char * argv[])
{
    // 기본값
    int interval_ms = 33;
    size_t message_size = 1048576;
    int sample_count = 1000;

    // ROS 2 초기화 전에 인자 파싱
    parse_args(argc, argv, interval_ms, message_size, sample_count);

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BytePublisher>(interval_ms, message_size, sample_count));
    return 0;
}