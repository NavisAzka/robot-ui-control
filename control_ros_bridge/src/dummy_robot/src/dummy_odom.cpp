#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <termios.h>
#include <unistd.h>
#include <thread>
#include <cmath>

class DummyOdomNode : public rclcpp::Node
{
public:
  DummyOdomNode() : Node("dummy_odom_node")
  {
    pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

    service_ = this->create_service<std_srvs::srv::SetBool>(
        "/toggle_robot",
        std::bind(&DummyOdomNode::callbackService, this,
                  std::placeholders::_1,
                  std::placeholders::_2));

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&DummyOdomNode::publishOdom, this));

    input_thread_ = std::thread(&DummyOdomNode::keyboardLoop, this);

    RCLCPP_INFO(this->get_logger(), "Dummy Odom Node started (WASD control)");
  }

private:
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;

  std::thread input_thread_;

  float x_ = 0.0f;
  float y_ = 0.0f;
  float yaw_ = 0.0f;

  float linear_speed_ = 0.1f;
  float angular_speed_ = 0.1f;

  void publishOdom()
  {
    nav_msgs::msg::Odometry msg;

    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = "odom";
    msg.child_frame_id = "base_link";

    msg.pose.pose.position.x = x_;
    msg.pose.pose.position.y = y_;
    msg.pose.pose.position.z = 0.0;

    geometry_msgs::msg::Quaternion q;
    q.w = cos(yaw_ * 0.5);
    q.x = 0.0;
    q.y = 0.0;
    q.z = sin(yaw_ * 0.5);

    msg.pose.pose.orientation = q;

    pub_->publish(msg);
  }

  void callbackService(
      const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
      std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
      if (request->data)
          RCLCPP_INFO(this->get_logger(), "Robot ON");
      else
          RCLCPP_INFO(this->get_logger(), "Robot OFF");

      response->success = true;
      response->message = "OK";
  }

  void keyboardLoop()
  {
    struct termios oldt, newt;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);

    while (rclcpp::ok())
    {
      char c = getchar();

      if (c == 'w')
      {
        x_ += linear_speed_ * cos(yaw_);
        y_ += linear_speed_ * sin(yaw_);
      }
      if (c == 's')
      {
        x_ -= linear_speed_ * cos(yaw_);
        y_ -= linear_speed_ * sin(yaw_);
      }
      if (c == 'a')
      {
        yaw_ += angular_speed_;
      }
      if (c == 'd')
      {
        yaw_ -= angular_speed_;
      }
    }

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DummyOdomNode>());
  rclcpp::shutdown();
  return 0;
}
