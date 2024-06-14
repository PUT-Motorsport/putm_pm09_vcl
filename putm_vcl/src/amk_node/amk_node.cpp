#include "amk_node/amk_node.hpp"
#include "putm_vcl/putm_vcl.hpp"

using namespace putm_vcl;
using namespace putm_vcl_interfaces;
using namespace std::chrono_literals;
using std::placeholders::_1;

AmkNode::AmkNode()
    : Node("amk_node"),
      state(StateMachine::UNDEFINED),
      publisher_amk_control(this->create_publisher<msg::AmkControl>("putm_vcl/amk_control", 1)),
      subscription_rtd(this->create_subscription<msg::Rtd>("putm_vcl/rtd", 1, std::bind(&AmkNode::rtd_callback, this, _1))),
      subscription_setpoints(this->create_subscription<msg::Setpoints>("putm_vcl/setpoints", 1, std::bind(&AmkNode::setpoints_callback, this, _1))),
      subscription_amk_status(this->create_subscription<msg::AmkStatus>("putm_vcl/amk_status", 1, std::bind(&AmkNode::amk_status_callback, this, _1))),
      amk_state_machine_timer(this->create_wall_timer(2ms, std::bind(&AmkNode::amk_state_machine_callback, this))),
      amk_control_timer(this->create_wall_timer(2ms, std::bind(&AmkNode::amk_control_callback, this))),
      setpoints_watchdog(this->create_wall_timer(500ms, std::bind(&AmkNode::setpoints_watchdog_callback, this))),
      amk_state_machine_watchdog(this->create_wall_timer(5000ms, std::bind(&AmkNode::amk_state_machine_watchdog_callback, this))) {
  setpoints_watchdog->cancel();
  amk_state_machine_watchdog->cancel();
}

// Subscriber callbacks
void AmkNode::rtd_callback(const msg::Rtd::SharedPtr msg) { rtd = *msg; }
void AmkNode::setpoints_callback(const msg::Setpoints::SharedPtr msg) {
  setpoints_watchdog->cancel();
  setpoints = *msg;
  setpoints_watchdog->reset();
}
void AmkNode::amk_status_callback(const msg::AmkStatus::SharedPtr msg) { amk_status = *msg; }

// Watchdog callbacks
void AmkNode::setpoints_watchdog_callback() {
  RCLCPP_WARN(this->get_logger(), "Setpoints watchdog triggered");
  setpoints.torques.fill(0);
  setpoints_watchdog->cancel();
}
void AmkNode::amk_state_machine_watchdog_callback() {
  RCLCPP_WARN(this->get_logger(), "State machine watchdog triggered");
  state = StateMachine::UNDEFINED;
  amk_state_machine_watchdog->cancel();
}

// Timer callbacks
void AmkNode::amk_control_callback() { publisher_amk_control->publish(amk_control); }
void AmkNode::amk_state_machine_callback() {
  switch (state) {
    case StateMachine::UNDEFINED: {
      if (amk_status.amk_status_berror[Inverters::FRONT_LEFT] || amk_status.amk_status_berror[Inverters::FRONT_RIGHT] ||
          amk_status.amk_status_berror[Inverters::REAR_LEFT] || amk_status.amk_status_berror[Inverters::REAR_RIGHT]) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,  "Detected inverter error");
        state = StateMachine::ERROR_RESET;
      }
      if (amk_status.amk_status_bsystem_ready[Inverters::FRONT_LEFT] && amk_status.amk_status_bsystem_ready[Inverters::FRONT_RIGHT] &&
          amk_status.amk_status_bsystem_ready[Inverters::REAR_LEFT] && amk_status.amk_status_bsystem_ready[Inverters::REAR_RIGHT]) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,  "Inverters online, going to idle");
        state = StateMachine::IDLING;
      }
    } break;
    case StateMachine::ERROR_RESET: {
      if (amk_status.amk_status_bsystem_ready[Inverters::FRONT_LEFT] && amk_status.amk_status_bsystem_ready[Inverters::FRONT_RIGHT] &&
          amk_status.amk_status_bsystem_ready[Inverters::REAR_LEFT] && amk_status.amk_status_bsystem_ready[Inverters::REAR_RIGHT]) {
        amk_control.amk_control_amkb_error_reset.fill(false);
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,  "Error recovered");
        state = StateMachine::IDLING;
      } else {
        amk_control.amk_control_amkb_error_reset.fill(true);
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Waiting for error reset");
        rclcpp::sleep_for(10ms);
      }
    } break;
    case StateMachine::IDLING: {
      /* Check for RTD*/
      if (rtd.state == true) {
        state = StateMachine::STARTUP;
        amk_state_machine_watchdog->reset();
      }
    } break;
    case StateMachine::STARTUP: {
      amk_control.amk_control_bdc_on.fill(true);
      amk_control.amk_torque_positive_limit.fill(0);
      amk_control.amk_torque_negative_limit.fill(0);
      amk_control.amk_target_torque.fill(0);

      if (!amk_status.amk_status_bdc_on[Inverters::FRONT_LEFT] && !amk_status.amk_status_bdc_on[Inverters::FRONT_RIGHT] &&
          !amk_status.amk_status_bdc_on[Inverters::REAR_LEFT] && !amk_status.amk_status_bdc_on[Inverters::REAR_RIGHT]) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Waiting for bdc_on");
        rclcpp::sleep_for(10ms);
        break;
      }
      amk_control.amk_control_binverter_on.fill(true);
      amk_control.amk_control_benable.fill(true);
      if (!amk_status.amk_status_binverter_on[Inverters::FRONT_LEFT] && !amk_status.amk_status_binverter_on[Inverters::FRONT_RIGHT] &&
          !amk_status.amk_status_binverter_on[Inverters::REAR_LEFT] && !amk_status.amk_status_binverter_on[Inverters::REAR_RIGHT]) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,  "Waiting for inverter enable");
        rclcpp::sleep_for(10ms);
        break;
      }
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "inverters enabled");
      amk_control.amk_control_benable.fill(true);
      if (!(amk_status.amk_status_bquit_inverter_on[Inverters::FRONT_LEFT] && amk_status.amk_status_bquit_inverter_on[Inverters::FRONT_RIGHT] &&
            amk_status.amk_status_bquit_inverter_on[Inverters::REAR_LEFT] && amk_status.amk_status_bquit_inverter_on[Inverters::REAR_RIGHT])) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Waiting for bquit inverter on");
        rclcpp::sleep_for(10ms);
        break;
      } else {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,  "Inverters On");
        rclcpp::sleep_for(10ms);
        amk_state_machine_watchdog->cancel();
        state = StateMachine::WAIT_FOR_ZERO_INPUT;
      }
    } break;
    case StateMachine::WAIT_FOR_ZERO_INPUT:{
      if (setpoints.torques[Inverters::FRONT_LEFT] == 0 and setpoints.torques[Inverters::FRONT_RIGHT] == 0 and setpoints.torques[Inverters::REAR_LEFT] == 0 and setpoints.torques[Inverters::REAR_RIGHT] == 0)
      {
        state = StateMachine::TORQUE_CONTROL;
      }
      else
      {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Waiting for pedal zero position");
      }
    } break;
    case StateMachine::TORQUE_CONTROL: {
      /* Check some stop conditions*/
      if (!(amk_status.amk_status_binverter_on[Inverters::FRONT_LEFT] && amk_status.amk_status_binverter_on[Inverters::FRONT_RIGHT] &&
            amk_status.amk_status_binverter_on[Inverters::REAR_LEFT] && amk_status.amk_status_binverter_on[Inverters::REAR_RIGHT])) {
        state = StateMachine::SWITCH_OFF;
      }
      amk_control.amk_torque_positive_limit.fill(2000);
      amk_control.amk_target_torque[Inverters::FRONT_LEFT] = setpoints.torques[Inverters::FRONT_LEFT];
      amk_control.amk_target_torque[Inverters::FRONT_RIGHT] = setpoints.torques[Inverters::FRONT_RIGHT];
      amk_control.amk_target_torque[Inverters::REAR_LEFT] = setpoints.torques[Inverters::REAR_LEFT];
      amk_control.amk_target_torque[Inverters::REAR_RIGHT] = setpoints.torques[Inverters::REAR_RIGHT];
      if (rtd.state == false) {
        state = StateMachine::SWITCH_OFF;
      }
      rclcpp::sleep_for(5ms);

    } break;
    case StateMachine::SWITCH_OFF: {
      amk_control.amk_control_binverter_on.fill(false);
      amk_control.amk_control_benable.fill(false);
      amk_control.amk_control_bdc_on.fill(false);
      amk_control.amk_torque_positive_limit.fill(0);
      amk_control.amk_torque_negative_limit.fill(0);
      amk_control.amk_target_torque.fill(0);
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,  "Inverters OFF");
      /* Wait until inverter 0 is switched-off.*/
      if (amk_status.amk_status_binverter_on[Inverters::FRONT_LEFT] || amk_status.amk_status_binverter_on[Inverters::FRONT_RIGHT] ||
          amk_status.amk_status_binverter_on[Inverters::REAR_LEFT] || amk_status.amk_status_binverter_on[Inverters::REAR_RIGHT]) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Waiting for inverter switch-off");
        rclcpp::sleep_for(5ms);
        break;
      }
      state = StateMachine::IDLING;
    } break;
    case StateMachine::ERROR_HANDLER: {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Error handler");
      rclcpp::sleep_for(1000ms);
    } break;
    default:
      break;
  }
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AmkNode>());
  rclcpp::shutdown();
}
