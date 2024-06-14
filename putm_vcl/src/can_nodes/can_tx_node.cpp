#include "can_nodes/can_tx_node.hpp"
#include "putm_vcl/putm_vcl.hpp"

using namespace PUTM_CAN;
using namespace putm_vcl;
using namespace putm_vcl_interfaces;
using namespace std::chrono_literals;
using std::placeholders::_1;

CanTxNode::CanTxNode()
    : Node("can_tx_node"),
      can_tx_amk(can_interface_amk),
      can_tx_common(can_interface_common),
      can_tx_amk_timer(this->create_wall_timer(10ms, std::bind(&CanTxNode::can_tx_amk_callback, this))),
      can_tx_common_timer(this->create_wall_timer(10ms, std::bind(&CanTxNode::can_tx_common_callback, this))),
      amk_control_subscription(this->create_subscription<msg::AmkControl>("putm_vcl/amk_control", 1, std::bind(&CanTxNode::amk_control_callback, this, _1))),
      rtd_subscription(this->create_subscription<msg::Rtd>("putm_vcl/rtd", 1, std::bind(&CanTxNode::rtd_callback, this, _1))) ,
      amk_data_subscription(this->create_subscription<msg::AmkData>("putm_vcl/amk_data", 1, std::bind(&CanTxNode::amk_data_callback, this, _1))),
      amk_status_subscription(this->create_subscription<msg::AmkStatus>("putm_vcl/amk_status", 1, std::bind(&CanTxNode::amk_status_callback, this, _1))) {}
      


void CanTxNode::amk_control_callback(const msg::AmkControl msg) {
  amk_front_left_setpoints.AMK_Control.AMK_bInverterOn = msg.amk_control_binverter_on[Inverters::FRONT_LEFT];
  amk_front_left_setpoints.AMK_Control.AMK_bDcOn = msg.amk_control_bdc_on[Inverters::FRONT_LEFT];
  amk_front_left_setpoints.AMK_Control.AMK_bEnable = msg.amk_control_benable[Inverters::FRONT_LEFT];
  amk_front_left_setpoints.AMK_Control.AMK_bErrorReset = msg.amk_control_amkb_error_reset[Inverters::FRONT_LEFT];
  amk_front_left_setpoints.AMK_TargetVelocity = msg.amk_target_torque[Inverters::FRONT_LEFT];
  amk_front_left_setpoints.AMK_TorqueLimitPositiv = msg.amk_torque_positive_limit[Inverters::FRONT_LEFT];
  amk_front_left_setpoints.AMK_TorqueLimitNegativ = msg.amk_torque_negative_limit[Inverters::FRONT_LEFT];

  amk_front_right_setpoints.AMK_Control.AMK_bInverterOn = msg.amk_control_binverter_on[Inverters::FRONT_RIGHT];
  amk_front_right_setpoints.AMK_Control.AMK_bDcOn = msg.amk_control_bdc_on[Inverters::FRONT_RIGHT];
  amk_front_right_setpoints.AMK_Control.AMK_bEnable = msg.amk_control_benable[Inverters::FRONT_RIGHT];
  amk_front_right_setpoints.AMK_Control.AMK_bErrorReset = msg.amk_control_amkb_error_reset[Inverters::FRONT_RIGHT];
  amk_front_right_setpoints.AMK_TargetVelocity = msg.amk_target_torque[Inverters::FRONT_RIGHT];
  amk_front_right_setpoints.AMK_TorqueLimitPositiv = msg.amk_torque_positive_limit[Inverters::FRONT_RIGHT];
  amk_front_right_setpoints.AMK_TorqueLimitNegativ = msg.amk_torque_negative_limit[Inverters::FRONT_RIGHT];

  amk_rear_left_setpoints.AMK_Control.AMK_bInverterOn = msg.amk_control_binverter_on[Inverters::REAR_LEFT];
  amk_rear_left_setpoints.AMK_Control.AMK_bDcOn = msg.amk_control_bdc_on[Inverters::REAR_LEFT];
  amk_rear_left_setpoints.AMK_Control.AMK_bEnable = msg.amk_control_benable[Inverters::REAR_LEFT];
  amk_rear_left_setpoints.AMK_Control.AMK_bErrorReset = msg.amk_control_amkb_error_reset[Inverters::REAR_LEFT];
  amk_rear_left_setpoints.AMK_TargetVelocity = msg.amk_target_torque[Inverters::REAR_LEFT];
  amk_rear_left_setpoints.AMK_TorqueLimitPositiv = msg.amk_torque_positive_limit[Inverters::REAR_LEFT];
  amk_rear_left_setpoints.AMK_TorqueLimitNegativ = msg.amk_torque_negative_limit[Inverters::REAR_LEFT];

  amk_rear_right_setpoints.AMK_Control.AMK_bInverterOn = msg.amk_control_binverter_on[Inverters::REAR_RIGHT];
  amk_rear_right_setpoints.AMK_Control.AMK_bDcOn = msg.amk_control_bdc_on[Inverters::REAR_RIGHT];
  amk_rear_right_setpoints.AMK_Control.AMK_bEnable = msg.amk_control_benable[Inverters::REAR_RIGHT];
  amk_rear_right_setpoints.AMK_Control.AMK_bErrorReset = msg.amk_control_amkb_error_reset[Inverters::REAR_RIGHT];
  amk_rear_right_setpoints.AMK_TargetVelocity = msg.amk_target_torque[Inverters::REAR_RIGHT];
  amk_rear_right_setpoints.AMK_TorqueLimitPositiv = msg.amk_torque_positive_limit[Inverters::REAR_RIGHT];
  amk_rear_right_setpoints.AMK_TorqueLimitNegativ = msg.amk_torque_negative_limit[Inverters::REAR_RIGHT];
}

void CanTxNode::amk_data_callback(const putm_vcl_interfaces::msg::AmkData msg)
{
  uint32_t meanRph = ((msg.amk_actual_velocity[0]) * 60);
  float speed = (meanRph * 2 * 3.14 * 0.000205 / 14.25);
  pcMainData.vehicleSpeed = static_cast<uint8_t>(speed);

  pcMainData.frontLeftInverterTemperature = msg.amk_temp_inverter[Inverters::FRONT_LEFT];
  pcMainData.frontRightInverterTemperature = msg.amk_temp_inverter[Inverters::FRONT_RIGHT];
  pcMainData.rearLeftIverterTemperature = msg.amk_temp_inverter[Inverters::REAR_LEFT];
  pcMainData.rearRightIverterTemperature = msg.amk_temp_inverter[Inverters::REAR_RIGHT];

  pcMainData.frontLeftMotorTemperature = msg.amk_temp_motor[Inverters::FRONT_LEFT];
  pcMainData.frontRightMotorTemperature = msg.amk_temp_motor[Inverters::FRONT_RIGHT];
  pcMainData.rearLeftMotorTemperature = msg.amk_temp_motor[Inverters::REAR_LEFT];
  pcMainData.rearRightMotorTemperature = msg.amk_temp_motor[Inverters::REAR_RIGHT];
}

void CanTxNode::amk_status_callback(const putm_vcl_interfaces::msg::AmkStatus msg)
{
  if (msg.amk_status_binverter_on[Inverters::FRONT_LEFT] & msg.amk_status_binverter_on[Inverters::FRONT_RIGHT] & msg.amk_status_binverter_on[Inverters::REAR_LEFT] & msg.amk_status_binverter_on[Inverters::REAR_RIGHT])
  {
    pcMainData.inverters_ready = true;
  }
  else
  {
    pcMainData.inverters_ready = false;
  }

}

void CanTxNode::rtd_callback(const putm_vcl_interfaces::msg::Rtd msg)
{
  pcMainData.rtd = msg.state;
}

void CanTxNode::can_tx_amk_callback() {
  can_tx_amk.transmit(amk_front_left_setpoints);
  can_tx_amk.transmit(amk_front_right_setpoints);
  can_tx_amk.transmit(amk_rear_left_setpoints);
  can_tx_amk.transmit(amk_rear_right_setpoints);
}

void CanTxNode::can_tx_common_callback() {
  // Add common tx in here if needed
  // Example:
  // can_tx_common.transmit(something);
  can_tx_common.transmit(pcMainData);
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CanTxNode>());
  rclcpp::shutdown();
}
