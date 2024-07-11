#include "putm_vcl_interfaces/msg/frontbox_driver_input.hpp"
#include "putm_vcl_interfaces/msg/setpoints.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
using namespace putm_vcl_interfaces::msg;

using std::placeholders::_1;

class Controller : public rclcpp::Node {
 public:
  Controller();

 private:
  static constexpr float INVERTER_MAX_TORQUE = 9.8;

  FrontboxDriverInput fbox_driver_input;

  rclcpp::Publisher<Setpoints>::SharedPtr setpoints_publisher;
  rclcpp::Subscription<FrontboxDriverInput>::SharedPtr frontbox_driver_input_subscriber;
  rclcpp::TimerBase::SharedPtr control_loop_timer;

  inline float scale_pedal_position(int32_t pedal_position);
  inline int32_t scale_torque(float torque);
  void frontbox_driver_input_topic_callback(const FrontboxDriverInput msg);
  void control_loop();
};

Controller::Controller()
    : Node("controller"),
      setpoints_publisher(this->create_publisher<Setpoints>("putm_vcl/setpoints", 1)),
      frontbox_driver_input_subscriber(this->create_subscription<FrontboxDriverInput>("putm_vcl/frontbox_driver_input", 1,
                                                                                      std::bind(&Controller::frontbox_driver_input_topic_callback, this, _1))),
      control_loop_timer(this->create_wall_timer(10ms, std::bind(&Controller::control_loop, this))) {
}

void Controller::control_loop() {
  float scaled_pedal_position = scale_pedal_position(fbox_driver_input.pedal_position);  // normalized 0-1
  float steering_wheel_position = fbox_driver_input.steering_wheel_position;             // degrees -val - 0 - val
  float bp_front = fbox_driver_input.brake_pressure_front;                               // voltage 0-5v
  float bp_rear = fbox_driver_input.brake_pressure_rear;                                 // voltage 0-5v

  (void)bp_rear;
  (void)bp_front;
  (void)steering_wheel_position;

  auto setpoints = Setpoints();
  setpoints.torques[0] = scale_torque(scaled_pedal_position * INVERTER_MAX_TORQUE);
  setpoints.torques[1] = scale_torque(scaled_pedal_position * INVERTER_MAX_TORQUE);
  setpoints.torques[2] = scale_torque(scaled_pedal_position * INVERTER_MAX_TORQUE);
  setpoints.torques[3] = scale_torque(scaled_pedal_position * INVERTER_MAX_TORQUE);

  setpoints_publisher->publish(setpoints);
}

void Controller::frontbox_driver_input_topic_callback(const FrontboxDriverInput msg) { fbox_driver_input = msg; }

inline float Controller::scale_pedal_position(int32_t pedal_position) {
  static constexpr int32_t PEDAL_SCALER = 500;
  return (((float)pedal_position) / PEDAL_SCALER);
}

inline int32_t Controller::scale_torque(float torque) {
  static constexpr float TORQUE_SCALER = 10.0;
  return (int32_t)(torque * TORQUE_SCALER);
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Controller>());
  rclcpp::shutdown();
}
