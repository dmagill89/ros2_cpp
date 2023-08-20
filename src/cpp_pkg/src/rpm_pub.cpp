#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include <chrono>
#include <functional>

const double RPM_DEFAULT_VALUE = 100.0;

class RpmPubNode : public rclcpp::Node
{
  public:
    RpmPubNode() : Node("rpm_pub_node")
    {
      this->declare_parameter<double>("rpm_value", RPM_DEFAULT_VALUE);
      publisher_ = this->create_publisher<std_msgs::msg::Float64>("rpm", 10);
      timer_ = this->create_wall_timer(
          std::chrono::seconds(1),
          std::bind(&RpmPubNode::publish_rpm, this));

          std::cout << "RPM Publisher Node Is Running" << std::endl;
    }

  private:
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    double rpm_val_param_ = RPM_DEFAULT_VALUE;

    void publish_rpm()
    {
      auto message = std_msgs::msg::Float64();
      this->get_parameter("rpm_val", rpm_val_param_);
      message.data = rpm_val_param_;

      publisher_->publish(message);
    }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RpmPubNode>());
  rclcpp::shutdown();

  return 0;
}