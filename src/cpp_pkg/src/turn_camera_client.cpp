#include "rclcpp/rclcpp.hpp"
#include "cpp_pkg/srv/turn_camera.hpp"
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>

typedef cpp_pkg::srv::TurnCamera TurnCameraSrv;

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  auto service_client_node = rclcpp::Node::make_shared("turn_camera_client_node");
  auto client = service_client_node->create_client<TurnCameraSrv>("turn_camera");
  auto request = std::make_shared<TurnCameraSrv::Request>();

  std::cout << "Enter the position in degress you want to turn the robot's camera to:";
  std::cin >> request->degree_turn;

  client->wait_for_service();
  auto result = client->async_send_request(request);

  if (rclcpp::spin_until_future_complete(service_client_node, result) == rclcpp::FutureReturnCode::SUCCESS)
  {
    auto cv_ptr = cv_bridge::toCvCopy(result.get()->camera_image, "bgr8");
    cv::imshow("Robot Camera Image", cv_ptr->image);
    cv::waitKey(0);
  }
  else
  {
    std::cout << "there was an error processing the request..." << std::endl;
  }

  rclcpp::shutdown();
  return 0;
}