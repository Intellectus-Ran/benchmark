#include <memory>
#include <fstream>
#include <chrono>
#include <ctime>
#include <iomanip>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>

class MinimalSubscriber : public rclcpp::Node
{
public:
    MinimalSubscriber(size_t total_expected_samples)
    : Node("byte_subscriber"), received_samples_(0), total_bytes_(0), total_expected_samples_(total_expected_samples)
    {
        subscription_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
            "ros_benchmark", 10,
            std::bind(&MinimalSubscriber::topic_callback, this, std::placeholders::_1));

        csv_file_.open("ros2_timestamp.csv");
        csv_file_ << "Timestamp\n";
    }

    ~MinimalSubscriber()
    {
        if (csv_file_.is_open()) {
            csv_file_.close();
        }
    }

private:
    void topic_callback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg)
    {
        using namespace std::chrono;

        if (received_samples_ == 0) {
            start_time_ = system_clock::now();
        }

        // 현재 시각을 CSV에 기록
        std::string timestamp = get_timestamp_with_ms();
        csv_file_ << timestamp << "\n";

        // 빈 메시지 체크 (메시지 크기가 0이면 종료 신호로 간주)
        if (msg->data.size() == 0) {
            RCLCPP_INFO(this->get_logger(), "Received empty message - publisher finished");

            end_time_ = system_clock::now();
            auto duration_us = duration_cast<microseconds>(end_time_ - start_time_).count();
            double duration_sec = duration_us / 1'000'000.0;
            double throughput = static_cast<double>(total_bytes_) / duration_sec;
            double loss_rate = static_cast<double>(total_expected_samples_ - received_samples_) / total_expected_samples_;

            RCLCPP_INFO(this->get_logger(), "Total received: %zu", received_samples_);
            RCLCPP_INFO(this->get_logger(), "Total time (us): %ld", duration_us);
            RCLCPP_INFO(this->get_logger(), "Throughput per sec: %.2f", throughput);
            RCLCPP_INFO(this->get_logger(), "Loss rate: %.4f", loss_rate);
            rclcpp::shutdown();
            return;
        }

        // 바이트 수 누적
        total_bytes_ += msg->data.size();
        received_samples_++;

        RCLCPP_INFO(this->get_logger(), "Received message [%zu bytes] (%zu / %zu)", 
                   msg->data.size(), received_samples_, total_expected_samples_);
    }

    std::string get_timestamp_with_ms() {
        using namespace std::chrono;

        auto now = system_clock::now();
        auto ms = duration_cast<milliseconds>(now.time_since_epoch()) % 1000;

        auto time_t_now = system_clock::to_time_t(now);
        std::tm local_tm = *std::localtime(&time_t_now);

        std::ostringstream oss;
        oss << std::put_time(&local_tm, "%Y-%m-%d %H:%M:%S");
        oss << '.' << std::setfill('0') << std::setw(3) << ms.count();

        return oss.str();
    }

    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr subscription_;
    std::ofstream csv_file_;

    std::chrono::time_point<std::chrono::system_clock> start_time_;
    std::chrono::time_point<std::chrono::system_clock> end_time_;

    size_t received_samples_;
    size_t total_bytes_;
    const size_t total_expected_samples_;
};

int main(int argc, char * argv[])
{
    // 기본 기대 수신 수
    size_t expected_samples = 1000;

    // 인자 파싱: -s <expected_samples>
    for (int i = 1; i < argc - 1; ++i) {
        std::string arg(argv[i]);
        if (arg == "-s") {
            expected_samples = static_cast<size_t>(std::stoi(argv[i + 1]));
        }
    }

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalSubscriber>(expected_samples));
    rclcpp::shutdown();
    return 0;
}
