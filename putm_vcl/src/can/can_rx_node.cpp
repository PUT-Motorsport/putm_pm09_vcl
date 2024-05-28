#include "PUTM_DV_CAN_LIBRARY_RAII_2024/include/can_headers/PM09-CANBUS-FRONTBOX.hpp"
#include "PUTM_DV_CAN_LIBRARY_RAII_2024/include/can_rx.hpp"
#include "putm_vcl_interfaces/msg/detail/frontbox__struct.hpp"
#include "putm_vcl_interfaces/msg/frontbox.hpp"
#include "putm_vcl_interfaces/msg/amk_status.hpp"
#include "putm_vcl_interfaces/msg/amk_data.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/utilities.hpp"

using namespace PUTM_CAN;
using namespace putm_vcl_interfaces;
using namespace std::chrono_literals;

class CanRxNode : public rclcpp::Node {
 public:
  CanRxNode();

 private:
  enum Inverters { FRONT_LEFT = 0, FRONT_RIGHT = 1, REAR_LEFT = 2, REAR_RIGHT = 3 };

  CanRx can_rx_amk;
  CanRx can_rx_common;

  msg::AmkStatus amk_status;
  msg::AmkData amk_data;

  rclcpp::TimerBase::SharedPtr can_rx_amk_timer;
  rclcpp::TimerBase::SharedPtr can_rx_common_timer;

  rclcpp::Publisher<msg::Frontbox>::SharedPtr frontbox_publisher;
  rclcpp::Publisher<msg::AmkStatus>::SharedPtr amk_status_publisher;
  rclcpp::Publisher<msg::AmkData>::SharedPtr amk_data_publisher;

  void can_rx_amk_callback();
  void can_rx_common_callback();
};

CanRxNode::CanRxNode()
    : Node("can_rx_node"),
      can_rx_amk("can0", NO_TIMEOUT),
      can_rx_common("can1", NO_TIMEOUT),
      can_rx_amk_timer(this->create_wall_timer(1ms, std::bind(&CanRxNode::can_rx_amk_callback, this))),
      can_rx_common_timer(this->create_wall_timer(1ms, std::bind(&CanRxNode::can_rx_common_callback, this))),
      frontbox_publisher(this->create_publisher<msg::Frontbox>("putm_vcl/frontbox", 1)),
      amk_status_publisher(this->create_publisher<msg::AmkStatus>("putm_vcl/amk_status", 1)),
      amk_data_publisher(this->create_publisher<msg::AmkData>("putm_vcl/amk_data", 1)) {}

void CanRxNode::can_rx_common_callback() {
  can_frame frame = can_rx_common.receive();
  switch (frame.can_id) {
    case can_id<Frontbox_main>: {
      auto can_frontbox = convert<Frontbox_main>(frame);
      msg::Frontbox frontbox;
      frontbox.pedal_position = (((can_frontbox.pedal_position) / 500.0) * 100.0);
      frontbox_publisher->publish(frontbox);
    }
  }
}

void CanRxNode::can_rx_amk_callback() {
  can_frame frame = can_rx_amk.receive();
  switch (frame.can_id) {
    case FRONT_LEFT_AMK_ACTUAL_VALUES_1_CAN_ID: {
      auto frontLeftAmkActualValues1 = convert<AmkFrontLeftActualValues1>(frame);
      amk_status.amk_status_bsystem_ready[Inverters::FRONT_LEFT] = frontLeftAmkActualValues1.AMK_Status.AMK_bSystemReady;
      amk_status.amk_status_berror[Inverters::FRONT_LEFT] = frontLeftAmkActualValues1.AMK_Status.AMK_bError;
      amk_status.amk_status_bwarn[Inverters::FRONT_LEFT] = frontLeftAmkActualValues1.AMK_Status.AMK_bWarn;
      amk_status.amk_status_bquit_dc_on[Inverters::FRONT_LEFT] = frontLeftAmkActualValues1.AMK_Status.AMK_bQuitDcOn;
      amk_status.amk_status_bdc_on[Inverters::FRONT_LEFT] = frontLeftAmkActualValues1.AMK_Status.AMK_bDcOn;
      amk_status.amk_status_bquit_inverter_on[Inverters::FRONT_LEFT] = frontLeftAmkActualValues1.AMK_Status.AMK_bQuitInverterOn;
      amk_status.amk_status_binverter_on[Inverters::FRONT_LEFT] = frontLeftAmkActualValues1.AMK_Status.AMK_bInverterOn;
      amk_status.amk_status_bderating[Inverters::FRONT_LEFT] = frontLeftAmkActualValues1.AMK_Status.AMK_bDerating;

      amk_data.amk_actual_velocity[Inverters::FRONT_LEFT] = frontLeftAmkActualValues1.AMK_ActualVelocity;
      amk_data.amk_torque_current[Inverters::FRONT_LEFT] = frontLeftAmkActualValues1.AMK_TorqueCurrent;
    } break;

    case FRONT_RIGHT_AMK_ACTUAL_VALUES_1_CAN_ID: {
      auto frontRightAmkActualValues1 = convert<AmkFrontRightActualValues1>(frame);
      amk_status.amk_status_bsystem_ready[Inverters::FRONT_RIGHT] = frontRightAmkActualValues1.AMK_Status.AMK_bSystemReady;
      amk_status.amk_status_berror[Inverters::FRONT_RIGHT] = frontRightAmkActualValues1.AMK_Status.AMK_bError;
      amk_status.amk_status_bwarn[Inverters::FRONT_RIGHT] = frontRightAmkActualValues1.AMK_Status.AMK_bWarn;
      amk_status.amk_status_bquit_dc_on[Inverters::FRONT_RIGHT] = frontRightAmkActualValues1.AMK_Status.AMK_bQuitDcOn;
      amk_status.amk_status_bdc_on[Inverters::FRONT_RIGHT] = frontRightAmkActualValues1.AMK_Status.AMK_bDcOn;
      amk_status.amk_status_bquit_inverter_on[Inverters::FRONT_RIGHT] = frontRightAmkActualValues1.AMK_Status.AMK_bQuitInverterOn;
      amk_status.amk_status_binverter_on[Inverters::FRONT_RIGHT] = frontRightAmkActualValues1.AMK_Status.AMK_bInverterOn;
      amk_status.amk_status_bderating[Inverters::FRONT_RIGHT] = frontRightAmkActualValues1.AMK_Status.AMK_bDerating;
    } break;

    case REAR_LEFT_AMK_ACTUAL_VALUES_1_CAN_ID: {
      auto rearLeftAmkActualValues1 = convert<AmkRearLeftActualValues1>(frame);
      amk_status.amk_status_bsystem_ready[Inverters::REAR_LEFT] = rearLeftAmkActualValues1.AMK_Status.AMK_bSystemReady;
      amk_status.amk_status_berror[Inverters::REAR_LEFT] = rearLeftAmkActualValues1.AMK_Status.AMK_bError;
      amk_status.amk_status_bwarn[Inverters::REAR_LEFT] = rearLeftAmkActualValues1.AMK_Status.AMK_bWarn;
      amk_status.amk_status_bquit_dc_on[Inverters::REAR_LEFT] = rearLeftAmkActualValues1.AMK_Status.AMK_bQuitDcOn;
      amk_status.amk_status_bdc_on[Inverters::REAR_LEFT] = rearLeftAmkActualValues1.AMK_Status.AMK_bDcOn;
      amk_status.amk_status_bquit_inverter_on[Inverters::REAR_LEFT] = rearLeftAmkActualValues1.AMK_Status.AMK_bQuitInverterOn;
      amk_status.amk_status_binverter_on[Inverters::REAR_LEFT] = rearLeftAmkActualValues1.AMK_Status.AMK_bInverterOn;
      amk_status.amk_status_bderating[Inverters::REAR_LEFT] = rearLeftAmkActualValues1.AMK_Status.AMK_bDerating;
    } break;

    case REAR_RIGHT_AMK_ACTUAL_VALUES_1_CAN_ID: {
      auto rearRightAmkActualValues1 = convert<AmkRearRightActualValues1>(frame);
      amk_status.amk_status_bsystem_ready[Inverters::REAR_RIGHT] = rearRightAmkActualValues1.AMK_Status.AMK_bSystemReady;
      amk_status.amk_status_berror[Inverters::REAR_RIGHT] = rearRightAmkActualValues1.AMK_Status.AMK_bError;
      amk_status.amk_status_bwarn[Inverters::REAR_RIGHT] = rearRightAmkActualValues1.AMK_Status.AMK_bWarn;
      amk_status.amk_status_bquit_dc_on[Inverters::REAR_RIGHT] = rearRightAmkActualValues1.AMK_Status.AMK_bQuitDcOn;
      amk_status.amk_status_bdc_on[Inverters::REAR_RIGHT] = rearRightAmkActualValues1.AMK_Status.AMK_bDcOn;
      amk_status.amk_status_bquit_inverter_on[Inverters::REAR_RIGHT] = rearRightAmkActualValues1.AMK_Status.AMK_bQuitInverterOn;
      amk_status.amk_status_binverter_on[Inverters::REAR_RIGHT] = rearRightAmkActualValues1.AMK_Status.AMK_bInverterOn;
      amk_status.amk_status_bderating[Inverters::REAR_RIGHT] = rearRightAmkActualValues1.AMK_Status.AMK_bDerating;
    } break;
  }
  amk_status_publisher->publish(amk_status);
  amk_data_publisher->publish(amk_data);
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CanRxNode>());
  rclcpp::shutdown();
}