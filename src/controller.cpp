#include "putm_vcl_interfaces/msg/frontbox_driver_input.hpp"
#include "putm_vcl_interfaces/msg/setpoints.hpp"
#include "rclcpp/rclcpp.hpp"
#include <putm_vcl_interfaces/msg/detail/frontbox_data__struct.hpp>

using namespace std::chrono_literals;
using namespace putm_vcl_interfaces::msg;

using std::placeholders::_1;

class Controller : public rclcpp::Node {
 public:
  Controller();

 private:

  static constexpr float INVERTER_MAX_POSITIVE_TORQUE = 9.8;
  static constexpr float INVERTER_MAX_NEGATIVE_TORQUE = -9.8;

  FrontboxDriverInput fbox_driver_input;

  rclcpp::Subscription<FrontboxDriverInput>::SharedPtr frontbox_driver_input_subscriber;
  rclcpp::Publisher<Setpoints>::SharedPtr setpoints_publisher;
  rclcpp::TimerBase::SharedPtr control_loop_timer;


  void frontbox_driver_input_topic_callback(const FrontboxDriverInput msg);
  void control_loop();
};

Controller::Controller() : Node("controller"){
  setpoints_publisher = this->create_publisher<Setpoints>("putm_vcl/setpoints", 1);
  frontbox_driver_input_subscriber = this->create_subscription<FrontboxDriverInput>("putm_vcl/frontbox_driver_input", 1, std::bind(&Controller::frontbox_driver_input_topic_callback, this, _1));
  control_loop_timer = this->create_wall_timer(10ms, std::bind(&Controller::control_loop, this));
}

void Controller::control_loop() {

  float scaled_pedal_position = ((float)(fbox_driver_input.pedal_position) / 500.0);
  float steering_wheel_position = fbox_driver_input.steering_wheel_position;
  float bp_front = fbox_driver_input.brake_pressure_front;
  float bp_rear = fbox_driver_input.brake_pressure_rear;

  (void)bp_rear;
  (void)bp_front;
  (void)steering_wheel_position;

  auto torque_setpoints = Setpoints();
  int32_t torque_left = INVERTER_MAX_POSITIVE_TORQUE * scaled_pedal_position * 10.0;
  int32_t torque_right = INVERTER_MAX_POSITIVE_TORQUE * scaled_pedal_position * 10.0;
  torque_setpoints.torques[0] = torque_left;
  torque_setpoints.torques[1] = torque_right;
  torque_setpoints.torques[2] = torque_left;
  torque_setpoints.torques[3] = torque_right;

  setpoints_publisher->publish(torque_setpoints);
}

void Controller::frontbox_driver_input_topic_callback(const FrontboxDriverInput msg) {
  fbox_driver_input = msg;

}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Controller>());
  rclcpp::shutdown();
}
