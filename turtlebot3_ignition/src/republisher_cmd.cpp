#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using std::placeholders::_1;

class CMDSubscriber : public rclcpp::Node
{
  public:
    CMDSubscriber()
    : Node("cmd_subscriber")
    {
      subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel",
        10,
        std::bind(&CMDSubscriber::topic_callback, this, _1));
      publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/diff_drive_base_controller/cmd_vel_unstamped", 10);
    }

  private:
    void topic_callback(const geometry_msgs::msg::Twist::SharedPtr msg) const
    {
      geometry_msgs::msg::Twist republished;
      republished.linear.x = msg->linear.x;
      republished.linear.y = msg->linear.y;
      republished.linear.z = msg->linear.z;

      republished.angular.x = msg->angular.x;
      republished.angular.y = msg->angular.y;
      republished.angular.z = msg->angular.z;

      publisher_->publish(republished);
    }
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CMDSubscriber>());
  rclcpp::shutdown();
  return 0;
}
