#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <atomic>
#include <cstdio>
#include <chrono>

class RateSubscriber : public rclcpp::Node
{
public:
    RateSubscriber()
        : Node("rate_subscriber"), print_flag_(true), duration_sum_(0.0), sample_count_(0)
    {
        auto qos = rclcpp::QoS(1).best_effort();
        subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/ap/imu/experimental/data", qos,
            std::bind(&RateSubscriber::topic_callback, this, std::placeholders::_1));
    }

private:
    void topic_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        if (print_flag_.exchange(false))
        {
            auto start = std::chrono::high_resolution_clock::now();

            printf("%f\n", msg->linear_acceleration.z);
            fflush(stdout);

            auto end = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double, std::micro> print_duration = end - start;

            // Accumulate the print duration and increment count
            duration_sum_ += print_duration.count();
            ++sample_count_;

            // Print average duration every 1000 samples to avoid extra print overhead
            if (sample_count_ % 1000 == 0)
            {
                double average_duration = duration_sum_ / sample_count_;
                printf("Average print duration over %d samples: %f microseconds\n", sample_count_, average_duration);
                fflush(stdout);
                // Reset accumulation
                duration_sum_ = 0.0;
                sample_count_ = 0;
            }

            print_flag_ = true;
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
    std::atomic<bool> print_flag_;
    double duration_sum_;
    int sample_count_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RateSubscriber>());
    rclcpp::shutdown();
    return 0;
}

