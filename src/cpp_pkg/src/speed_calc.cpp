#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include <chrono>
#include <functional>
#include <math.h>

const float WHEEL_RADIUS = 12.5 /100; // centimeters to meters 

class SpeedCalcSubNode : public rclcpp::Node
{
  public:
    SpeedCalcSubNode(): Node("speed_calc_sub_node") 
    {
        rpm_subscription_ = this->create_subscription<std_msgs::msg::Float64>(
            "rpm",
            10,
            std::bind(&SpeedCalcSubNode::calculate_and_pub_speed, this, std::placeholders::_1)
        );

        speed_publisher_ = this->create_publisher<std_msgs::msg::Float64>(
          "speed",
          10
        );

        std::cout << "Speed Calc Node Is Running..." << std::endl;
    }

  private:
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr rpm_subscription_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr speed_publisher_;

    void calculate_and_pub_speed(const std_msgs::msg::Float64 &rpm_msg) const
    {
        auto speed_msg = std_msgs::msg::Float64();

        // Speed m/s = RPM [rev/min] * wheel circumference [meters/rev]/ 60 [seconds/min]
        speed_msg.data = rpm_msg.data * (2 * WHEEL_RADIUS * M_PI) / 60;

        speed_publisher_->publish(speed_msg);

    }
};


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SpeedCalcSubNode>());
  rclcpp::shutdown();
  
  return 0;
}