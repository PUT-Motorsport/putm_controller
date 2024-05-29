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
  static constexpr int32_t INVERTER_MAX_POSITIVE_TORQUE = 500;
  static constexpr int32_t INVERTER_MAX_NEGATIVE_TORQUE = -500;

  bool rtd_state = true;  // TODO: Implement

  struct torqueModifiers {
    float front;
    float rear;
  } modifiers;

  rclcpp::Subscription<FrontboxDriverInput>::SharedPtr frontbox_driver_input_subscriber;
  rclcpp::Publisher<Setpoints>::SharedPtr setpoints_publisher;

  void frontbox_driver_input_topic_callback(const FrontboxDriverInput msg);
};

Controller::Controller() : Node("controller"), modifiers{0.5, 0.5} {
  setpoints_publisher = this->create_publisher<Setpoints>("putm_vcl/setpoints", 1);
  frontbox_driver_input_subscriber = this->create_subscription<FrontboxDriverInput>("putm_vcl/frontbox_driver_input", 1, std::bind(&Controller::frontbox_driver_input_topic_callback, this, _1));
}

void Controller::frontbox_driver_input_topic_callback(const FrontboxDriverInput msg) {
  float scaled_pedal_position = ((float)(msg.pedal_position) / 500.0) * 100.0;
  if (rtd_state) {
    auto torque_setpoints = Setpoints();
    int32_t torque_front = INVERTER_MAX_POSITIVE_TORQUE * scaled_pedal_position * modifiers.front;
    int32_t torque_rear = INVERTER_MAX_POSITIVE_TORQUE * scaled_pedal_position * modifiers.rear;
    torque_setpoints.torques[0] = torque_front;
    torque_setpoints.torques[1] = torque_front;
    torque_setpoints.torques[2] = torque_rear;
    torque_setpoints.torques[3] = torque_rear;

    setpoints_publisher->publish(torque_setpoints);
  }
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Controller>());
  rclcpp::shutdown();
}
