#pragma once

#include "PUTM_DV_CAN_LIBRARY_RAII_2024/include/can_rx.hpp"
#include "putm_vcl_interfaces/msg/amk_data.hpp"
#include "putm_vcl_interfaces/msg/amk_status.hpp"
#include "putm_vcl_interfaces/msg/dashboard.hpp"
#include "putm_vcl_interfaces/msg/frontbox_data.hpp"
#include "putm_vcl_interfaces/msg/frontbox_driver_input.hpp"
#include "rclcpp/rclcpp.hpp"

class CanRxNode : public rclcpp::Node {
 public:
  CanRxNode();

 private:
  PUTM_CAN::CanRx can_rx_amk;
  PUTM_CAN::CanRx can_rx_common;

  putm_vcl_interfaces::msg::AmkStatus amk_status;
  putm_vcl_interfaces::msg::AmkData amk_data;

  rclcpp::TimerBase::SharedPtr can_rx_amk_timer;
  rclcpp::TimerBase::SharedPtr can_rx_common_timer;

  rclcpp::Publisher<putm_vcl_interfaces::msg::FrontboxDriverInput>::SharedPtr frontbox_driver_input_publisher;
  rclcpp::Publisher<putm_vcl_interfaces::msg::FrontboxData>::SharedPtr frontbox_data_publisher;
  rclcpp::Publisher<putm_vcl_interfaces::msg::Dashboard>::SharedPtr dashboard_publisher;
  rclcpp::Publisher<putm_vcl_interfaces::msg::AmkStatus>::SharedPtr amk_status_publisher;
  rclcpp::Publisher<putm_vcl_interfaces::msg::AmkData>::SharedPtr amk_data_publisher;

  void can_rx_amk_callback();
  void can_rx_common_callback();
  inline float calculate_actual_current(int16_t raw_current) { return ((raw_current * 107.2)/ 16384);}
  inline uint8_t calculate_actual_temperature(int16_t raw_temperature) { return raw_temperature / 10;}
};
