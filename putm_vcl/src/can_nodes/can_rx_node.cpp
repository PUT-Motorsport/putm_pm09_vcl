#include "can_nodes/can_rx_node.hpp"

#include "putm_vcl/putm_vcl.hpp"

using namespace PUTM_CAN;
using namespace putm_vcl;
using namespace putm_vcl_interfaces;
using namespace std::chrono_literals;

CanRxNode::CanRxNode()
    : Node("can_rx_node"),
      can_rx_amk(can_interface_amk, NO_TIMEOUT),
      can_rx_common(can_interface_common, NO_TIMEOUT),
      can_rx_amk_timer(this->create_wall_timer(1ms, std::bind(&CanRxNode::can_rx_amk_callback, this))),
      can_rx_common_timer(this->create_wall_timer(1ms, std::bind(&CanRxNode::can_rx_common_callback, this))),
      frontbox_driver_input_publisher(this->create_publisher<msg::FrontboxDriverInput>("putm_vcl/frontbox_driver_input", 1)),
      frontbox_data_publisher(this->create_publisher<msg::FrontboxData>("putm_vcl/frontbox_data", 1)),
      dashboard_publisher(this->create_publisher<msg::Dashboard>("putm_vcl/dashboard", 1)),
      amk_status_publisher(this->create_publisher<msg::AmkStatus>("putm_vcl/amk_status", 1)),
      amk_data_publisher(this->create_publisher<msg::AmkData>("putm_vcl/amk_data", 1)) {}

void CanRxNode::can_rx_common_callback() {
  can_frame frame = can_rx_common.receive();
  switch (frame.can_id) {
    case can_id<FrontboxDriverInput>: {
      auto can_frontbox_driver_input = convert<FrontboxDriverInput>(frame);
      msg::FrontboxDriverInput frontbox_driver_input;
      frontbox_driver_input.pedal_position = can_frontbox_driver_input.pedal_position;
      frontbox_driver_input.brake_pressure_front = can_frontbox_driver_input.brake_pressure_front;
      frontbox_driver_input.brake_pressure_rear = can_frontbox_driver_input.brake_pressure_rear;
      frontbox_driver_input.steering_wheel_position = can_frontbox_driver_input.steering_wheel_position;
      frontbox_driver_input_publisher->publish(frontbox_driver_input);
      break;
    }

    case can_id<FrontboxData>: {
      auto can_frontbox_data = convert<FrontboxData>(frame);
      msg::FrontboxData frontbox_data;
      frontbox_data.sense_left_kill = can_frontbox_data.sense_left_kill;
      frontbox_data.sense_right_kill = can_frontbox_data.sense_right_kill;
      frontbox_data.sense_driver_kill = can_frontbox_data.sense_driver_kill;
      frontbox_data.sense_inertia = can_frontbox_data.sense_inertia;
      frontbox_data.sense_bspd = can_frontbox_data.sense_bspd;
      frontbox_data.sense_overtravel = can_frontbox_data.sense_overtravel;
      frontbox_data.sense_right_wheel = can_frontbox_data.sense_right_wheel;
      frontbox_data.sc_state = can_frontbox_data.sc_state;
      frontbox_data.front_left_suspension = can_frontbox_data.front_left_suspension;
      frontbox_data.front_right_suspension = can_frontbox_data.front_right_suspension;
      frontbox_data.front_left_hub_temperature = can_frontbox_data.front_left_hub_temperature;
      frontbox_data.front_right_hub_temperature = can_frontbox_data.front_right_hub_temperature;
      frontbox_data_publisher->publish(frontbox_data);
      break;
    }

    case can_id<Dashboard>: {
      auto can_dashboard = convert<Dashboard>(frame);
      msg::Dashboard dashboard;
      dashboard.rtd_button = can_dashboard.rtd_button;
      dashboard.ts_activate_button = can_dashboard.ts_activate_button;
      dashboard.rfu_button = can_dashboard.rfu_button;
      dashboard_publisher->publish(dashboard);
      break;
    }
  }
}

void CanRxNode::can_rx_amk_callback() {
  can_frame frame = can_rx_amk.receive();
  switch (frame.can_id) {
    case can_id<AmkFrontLeftActualValues1>: {
      auto can_amk = convert<AmkFrontLeftActualValues1>(frame);
      amk_status.amk_status_bsystem_ready[Inverters::FRONT_LEFT] = can_amk.AMK_Status.AMK_bSystemReady;
      amk_status.amk_status_berror[Inverters::FRONT_LEFT] = can_amk.AMK_Status.AMK_bError;
      amk_status.amk_status_bwarn[Inverters::FRONT_LEFT] = can_amk.AMK_Status.AMK_bWarn;
      amk_status.amk_status_bquit_dc_on[Inverters::FRONT_LEFT] = can_amk.AMK_Status.AMK_bQuitDcOn;
      amk_status.amk_status_bdc_on[Inverters::FRONT_LEFT] = can_amk.AMK_Status.AMK_bDcOn;
      amk_status.amk_status_bquit_inverter_on[Inverters::FRONT_LEFT] = can_amk.AMK_Status.AMK_bQuitInverterOn;
      amk_status.amk_status_binverter_on[Inverters::FRONT_LEFT] = can_amk.AMK_Status.AMK_bInverterOn;
      amk_status.amk_status_bderating[Inverters::FRONT_LEFT] = can_amk.AMK_Status.AMK_bDerating;

      // TODO: Determine why only there
      amk_data.amk_actual_velocity[Inverters::FRONT_LEFT] = can_amk.AMK_ActualVelocity;
      amk_data.amk_torque_current[Inverters::FRONT_LEFT]       = calculate_actual_current(can_amk.AMK_TorqueCurrent);
      amk_data.amk_magnetizing_current[Inverters::FRONT_LEFT]  = calculate_actual_current(can_amk.AMK_MagnetizingCurrent);

      
      break;
    }

    case can_id<AmkFrontLeftActualValues2>: {
      auto can_amk = convert<AmkFrontLeftActualValues2>(frame);
      amk_data.amk_temp_motor[Inverters::FRONT_LEFT]    = calculate_actual_temperature(can_amk.AMK_TempMotor);
      amk_data.amk_temp_inverter[Inverters::FRONT_LEFT] = calculate_actual_temperature(can_amk.AMK_TempInverter);
      amk_data.amk_error_info[Inverters::FRONT_LEFT]    = can_amk.AMK_ErrorInfo;
      amk_data.amk_temp_igbt[Inverters::FRONT_LEFT]     = calculate_actual_temperature(can_amk.AMK_TempIGBT);
      break;
    }


    case can_id<AmkFrontRightActualValues1>: {
      auto can_amk = convert<AmkFrontRightActualValues1>(frame);
      amk_status.amk_status_bsystem_ready[Inverters::FRONT_RIGHT] = can_amk.AMK_Status.AMK_bSystemReady;
      amk_status.amk_status_berror[Inverters::FRONT_RIGHT] = can_amk.AMK_Status.AMK_bError;
      amk_status.amk_status_bwarn[Inverters::FRONT_RIGHT] = can_amk.AMK_Status.AMK_bWarn;
      amk_status.amk_status_bquit_dc_on[Inverters::FRONT_RIGHT] = can_amk.AMK_Status.AMK_bQuitDcOn;
      amk_status.amk_status_bdc_on[Inverters::FRONT_RIGHT] = can_amk.AMK_Status.AMK_bDcOn;
      amk_status.amk_status_bquit_inverter_on[Inverters::FRONT_RIGHT] = can_amk.AMK_Status.AMK_bQuitInverterOn;
      amk_status.amk_status_binverter_on[Inverters::FRONT_RIGHT] = can_amk.AMK_Status.AMK_bInverterOn;
      amk_status.amk_status_bderating[Inverters::FRONT_RIGHT] = can_amk.AMK_Status.AMK_bDerating;

      amk_data.amk_actual_velocity[Inverters::FRONT_RIGHT] = can_amk.AMK_ActualVelocity;
      amk_data.amk_torque_current[Inverters::FRONT_RIGHT]      = calculate_actual_current(can_amk.AMK_TorqueCurrent);
      amk_data.amk_magnetizing_current[Inverters::FRONT_RIGHT] = calculate_actual_current(can_amk.AMK_MagnetizingCurrent);

      break;
    }

    case can_id<AmkFrontRightActualValues2>: {
      auto can_amk = convert<AmkFrontRightActualValues2>(frame);
      amk_data.amk_temp_motor[Inverters::FRONT_RIGHT]                 = calculate_actual_temperature(can_amk.AMK_TempMotor);
      amk_data.amk_temp_inverter[Inverters::FRONT_RIGHT]              = calculate_actual_temperature(can_amk.AMK_TempInverter);
      amk_data.amk_error_info[Inverters::FRONT_RIGHT]                 = can_amk.AMK_ErrorInfo;
      amk_data.amk_temp_igbt[Inverters::FRONT_RIGHT]                  = calculate_actual_temperature(can_amk.AMK_TempIGBT);
      break;
    }

    case can_id<AmkRearLeftActualValues1>: {
      auto can_amk = convert<AmkRearLeftActualValues1>(frame);
      amk_status.amk_status_bsystem_ready[Inverters::REAR_LEFT] = can_amk.AMK_Status.AMK_bSystemReady;
      amk_status.amk_status_berror[Inverters::REAR_LEFT] = can_amk.AMK_Status.AMK_bError;
      amk_status.amk_status_bwarn[Inverters::REAR_LEFT] = can_amk.AMK_Status.AMK_bWarn;
      amk_status.amk_status_bquit_dc_on[Inverters::REAR_LEFT] = can_amk.AMK_Status.AMK_bQuitDcOn;
      amk_status.amk_status_bdc_on[Inverters::REAR_LEFT] = can_amk.AMK_Status.AMK_bDcOn;
      amk_status.amk_status_bquit_inverter_on[Inverters::REAR_LEFT] = can_amk.AMK_Status.AMK_bQuitInverterOn;
      amk_status.amk_status_binverter_on[Inverters::REAR_LEFT] = can_amk.AMK_Status.AMK_bInverterOn;
      amk_status.amk_status_bderating[Inverters::REAR_LEFT] = can_amk.AMK_Status.AMK_bDerating;

      amk_data.amk_actual_velocity[Inverters::REAR_LEFT] = can_amk.AMK_ActualVelocity;
      amk_data.amk_torque_current[Inverters::REAR_LEFT]      = calculate_actual_current(can_amk.AMK_TorqueCurrent);
      amk_data.amk_magnetizing_current[Inverters::REAR_LEFT] = calculate_actual_current(can_amk.AMK_MagnetizingCurrent);
      break;
    }

    case can_id<AmkRearLeftActualValues2>: {
      auto can_amk = convert<AmkRearLeftActualValues2>(frame);
      amk_data.amk_temp_motor[Inverters::REAR_LEFT]           = calculate_actual_temperature(can_amk.AMK_TempMotor);
      amk_data.amk_temp_inverter[Inverters::REAR_LEFT]        = calculate_actual_temperature(can_amk.AMK_TempInverter);
      amk_data.amk_error_info[Inverters::REAR_LEFT]           = can_amk.AMK_ErrorInfo;
      amk_data.amk_temp_igbt[Inverters::REAR_LEFT]            = calculate_actual_temperature(can_amk.AMK_TempIGBT);
      break;
    }

    case can_id<AmkRearRightActualValues1>: {
      auto can_amk = convert<AmkRearRightActualValues1>(frame);
      amk_status.amk_status_bsystem_ready[Inverters::REAR_RIGHT] = can_amk.AMK_Status.AMK_bSystemReady;
      amk_status.amk_status_berror[Inverters::REAR_RIGHT] = can_amk.AMK_Status.AMK_bError;
      amk_status.amk_status_bwarn[Inverters::REAR_RIGHT] = can_amk.AMK_Status.AMK_bWarn;
      amk_status.amk_status_bquit_dc_on[Inverters::REAR_RIGHT] = can_amk.AMK_Status.AMK_bQuitDcOn;
      amk_status.amk_status_bdc_on[Inverters::REAR_RIGHT] = can_amk.AMK_Status.AMK_bDcOn;
      amk_status.amk_status_bquit_inverter_on[Inverters::REAR_RIGHT] = can_amk.AMK_Status.AMK_bQuitInverterOn;
      amk_status.amk_status_binverter_on[Inverters::REAR_RIGHT] = can_amk.AMK_Status.AMK_bInverterOn;
      amk_status.amk_status_bderating[Inverters::REAR_RIGHT] = can_amk.AMK_Status.AMK_bDerating;

      amk_data.amk_actual_velocity[Inverters::REAR_RIGHT] = can_amk.AMK_ActualVelocity;
      amk_data.amk_torque_current[Inverters::REAR_RIGHT]      = calculate_actual_current(can_amk.AMK_TorqueCurrent);
      amk_data.amk_magnetizing_current[Inverters::REAR_RIGHT] = calculate_actual_current(can_amk.AMK_MagnetizingCurrent);

      break;
    }

    case can_id<AmkRearRightActualValues2>: {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,  "AMkRearrightAcValues2");
      auto can_amk = convert<AmkRearRightActualValues2>(frame);
      amk_data.amk_temp_motor[Inverters::REAR_RIGHT] = calculate_actual_temperature(can_amk.AMK_TempMotor);
      amk_data.amk_temp_inverter[Inverters::REAR_RIGHT] = calculate_actual_temperature(can_amk.AMK_TempInverter);
      amk_data.amk_error_info[Inverters::REAR_RIGHT] = can_amk.AMK_ErrorInfo;
      amk_data.amk_temp_igbt[Inverters::REAR_RIGHT] = calculate_actual_temperature(can_amk.AMK_TempIGBT);
      break;
    }
  }
  amk_status_publisher->publish(amk_status);
  amk_data_publisher->publish(amk_data);
}

inline float CanRxNode::calculate_actual_current(int16_t raw_current) 
{
  return ((raw_current * 107.2)/ 16384);
}

inline uint8_t CanRxNode::calculate_actual_temperature(int16_t raw_temperature) 
{
  return raw_temperature / 10;
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CanRxNode>());
  rclcpp::shutdown();
}
