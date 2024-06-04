#include "rtd_node/rtd_node.hpp"

using namespace putm_vcl_interfaces;
using namespace std::chrono_literals;
using std::placeholders::_1;

RtdNode::RtdNode()
    : Node("rtd_node"),
      rtd_timer(this->create_wall_timer(100ms, std::bind(&RtdNode::rtd_callback, this))),
      rtd_publisher(this->create_publisher<msg::Rtd>("putm_vcl/rtd", 1)),
      frontbox_driver_input_subscriber(this->create_subscription<msg::FrontboxDriverInput>("putm_vcl/frontbox_driver_input", 1, std::bind(&RtdNode::frontbox_driver_input_callback, this, _1))),
      dashboard_subscriber(this->create_subscription<msg::Dashboard>("putm_vcl/dashboard", 1, std::bind(&RtdNode::dashboard_callback, this, _1))) {}

void RtdNode::rtd_callback() {
  /* Entry condition */
  if ((frontbox_driver_input.brake_pressure_front >= 100.0) or (frontbox_driver_input.brake_pressure_rear >= 100.0)) {
    RCLCPP_INFO_ONCE(this->get_logger(), "Braking...");
    if ((dashboard.rtd_button) and not(rtd.state)) {
      RCLCPP_INFO(this->get_logger(), "RTD ON");
      rtd.state = true;
      rtd_publisher->publish(rtd);
    }
  } else if (rtd.state & dashboard.rtd_button) {
    RCLCPP_INFO(this->get_logger(), "RTD OFF");
    rtd_publisher->publish(rtd);
  }
  /* Exit conditions */
  // if ((rtd_state) and (rtd_button))
  // {
  //   rtd_state = false;
  //   RCLCPP_INFO(this->get_logger(), "RTD OFF");
  // }
}

void RtdNode::frontbox_driver_input_callback(const msg::FrontboxDriverInput::SharedPtr msg) { frontbox_driver_input = *msg; }

void RtdNode::dashboard_callback(const msg::Dashboard::SharedPtr msg) { dashboard = *msg; }

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RtdNode>());
  rclcpp::shutdown();
}
