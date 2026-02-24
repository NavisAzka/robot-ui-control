// ROS Include files
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_srvs/srv/set_bool.hpp>

// WebSocket++ Include files
#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>

// Standard library includes
#include <thread>
#include <vector>
#include <cstring>
#include <cmath>


typedef websocketpp::server<websocketpp::config::asio> server;

class BridgeNode : public rclcpp::Node
{
public:
  // Subscriber ROS
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_point_cloud;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom;

  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr toggle_client_;

  // WebSocket server and connection handle
  server ws_server_;
  websocketpp::connection_hdl connection_;

  BridgeNode() : Node("slam_web_bridge")
  {

    sub_point_cloud = this->create_subscription<sensor_msgs::msg::PointCloud2>("/filtered_points", 10, std::bind(&BridgeNode::callbackPointCloud, this, std::placeholders::_1));
    sub_odom = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&BridgeNode::callbackOdom, this, std::placeholders::_1));

    toggle_client_ = this->create_client<std_srvs::srv::SetBool>("/toggle_robot");

    ws_server_.init_asio();

    ws_server_.set_open_handler([this](websocketpp::connection_hdl hdl)
                                {
      connection_ = hdl;
      RCLCPP_INFO(this->get_logger(), "WebSocket client connected"); });

    std::thread([this]()
                {
      ws_server_.listen(9002);
      ws_server_.start_accept();
      ws_server_.run(); })
        .detach();

    ws_server_.set_message_handler(
        [this](websocketpp::connection_hdl hdl, server::message_ptr msg)
        {
          auto payload = msg->get_payload();
          if (payload.size() < 2)
            return;

          uint8_t type = payload[0];

          if (type == 3) // service command
          {
            bool value = payload[1];

            auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
            request->data = value;

            toggle_client_->async_send_request(request);

            RCLCPP_INFO(this->get_logger(), "Service request sent");
          }
        });

    RCLCPP_INFO(this->get_logger(), "Bridge node started");
  }

void callbackOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  if (!connection_.lock())
    return;

  float x = msg->pose.pose.position.x;
  float y = msg->pose.pose.position.y;

  auto q = msg->pose.pose.orientation;

  float siny = 2.0f * (q.w * q.z + q.x * q.y);
  float cosy = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
  float yaw = std::atan2(siny, cosy);

  float pose_data[3] = {x, y, yaw};

  uint8_t packet[1 + sizeof(pose_data)];
  packet[0] = 2; // type 2 = odom

  memcpy(packet + 1, pose_data, sizeof(pose_data));

  ws_server_.send(
      connection_,
      packet,
      sizeof(packet),
      websocketpp::frame::opcode::binary);
}

void callbackPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  if (!connection_.lock())
    return;

  std::vector<float> buffer;
  buffer.reserve(msg->width * 3);

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

  int stride = 5;
  int count = 0;

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
  {
    if (count % stride == 0)
    {
      buffer.push_back(*iter_x);
      buffer.push_back(*iter_y);
      buffer.push_back(*iter_z);
    }
    count++;
  }

  // Buat packet dengan header type
  std::vector<uint8_t> packet;
  packet.push_back(1); // type 1 = cloud

  uint8_t* raw = reinterpret_cast<uint8_t*>(buffer.data());
  packet.insert(packet.end(), raw, raw + buffer.size() * sizeof(float));

  ws_server_.send(
      connection_,
      packet.data(),
      packet.size(),
      websocketpp::frame::opcode::binary);
}
};


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BridgeNode>());
  rclcpp::shutdown();
  return 0;
}