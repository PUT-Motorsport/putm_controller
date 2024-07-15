#include "putm_vcl_interfaces/msg/frontbox_driver_input.hpp"
#include "putm_vcl_interfaces/msg/setpoints.hpp"
#include "rclcpp/rclcpp.hpp"
extern "C" {
#include "tv.h"
}

using namespace std::chrono_literals;
using namespace putm_vcl_interfaces::msg;

using std::placeholders::_1;

class Controller : public rclcpp::Node {
 public:
  Controller();
  ~Controller();

 private:
  static constexpr float INVERTER_MAX_TORQUE = 9.8;
  struct TvValues {
    double WheelSp_FL = 0.0;
    double WheelSp_FR = 0.0;
    double WheelSp_RL = 0.0;
    double WheelSp_RR = 0.0;
    double acc_pedal = 0.0;
    double ax = 0.0;
    double ay = 0.0;
    double brake_pedal = 0.0;
    double delta = 0.0;
    double yaw_rate = 0.0;
  };

  FrontboxDriverInput frontbox_driver_input;

  rclcpp::Publisher<Setpoints>::SharedPtr setpoints_publisher;
  rclcpp::Subscription<FrontboxDriverInput>::SharedPtr frontbox_driver_input_subscriber;
  rclcpp::TimerBase::SharedPtr control_loop_timer;

  inline double convert_pedal_position(int16_t pedal_position);
  inline double convert_brake_pressure(int16_t brake_pressure);
  inline double convert_steering_wheel_position(int16_t steering_wheel_position);

  inline int32_t convert_torque(double torque);
  void frontbox_driver_input_topic_callback(const FrontboxDriverInput msg);

  void tv_set_values(const TvValues& tv_values);
  void control_loop();
};

Controller::Controller()
    : Node("controller"),
      setpoints_publisher(this->create_publisher<Setpoints>("putm_vcl/setpoints", 1)),
      frontbox_driver_input_subscriber(this->create_subscription<FrontboxDriverInput>("putm_vcl/frontbox_driver_input", 1,
                                                                                      std::bind(&Controller::frontbox_driver_input_topic_callback, this, _1))),
      control_loop_timer(this->create_wall_timer(10ms, std::bind(&Controller::control_loop, this)))  // TODO: Determine control loop frequency
{
  tv_initialize();
}

Controller::~Controller() { tv_terminate(); }

void Controller::frontbox_driver_input_topic_callback(const FrontboxDriverInput msg) { frontbox_driver_input = msg; }

void Controller::tv_set_values(const TvValues& tv_values) {
  tv_P.WheelSp_FL_Value = tv_values.WheelSp_FL;
  tv_P.WheelSp_FR_Value = tv_values.WheelSp_FR;
  tv_P.WheelSp_RL_Value = tv_values.WheelSp_RL;
  tv_P.WheelSp_RR_Value = tv_values.WheelSp_RR;
  tv_P.acc_pedal_Value = tv_values.acc_pedal;
  tv_P.ax_Value = tv_values.ax;
  tv_P.ay_Value = tv_values.ay;
  tv_P.brake_pedal_Value = tv_values.brake_pedal;
  tv_P.delta_Value = tv_values.delta;
  tv_P.yaw_rate_Value = tv_values.yaw_rate;
}

void Controller::control_loop() {
  if (rtmGetErrorStatus(tv_M) == (NULL) && !rtmGetStopRequested(tv_M)) {
    TvValues tv_values;
    tv_values.acc_pedal = convert_pedal_position(frontbox_driver_input.pedal_position);
    tv_values.brake_pedal = convert_brake_pressure((frontbox_driver_input.brake_pressure_front + frontbox_driver_input.brake_pressure_rear) / 2);
    tv_values.delta = convert_steering_wheel_position(frontbox_driver_input.steering_wheel_position);
    tv_set_values(tv_values);

    tv_step();

    RCLCPP_INFO_STREAM(this->get_logger(), "Trq_FL_Value:" << tv_B.Trq_FL);
    RCLCPP_INFO_STREAM(this->get_logger(), "Trq_FR_Value:" << tv_B.Trq_FR);
    RCLCPP_INFO_STREAM(this->get_logger(), "Trq_RL_Value:" << tv_B.Trq_RL);
    RCLCPP_INFO_STREAM(this->get_logger(), "Trq_RR_Value:" << tv_B.Trq_RR);

    auto setpoints = Setpoints();
    setpoints.front_left.torque = convert_torque(tv_B.Trq_FL);
    setpoints.front_right.torque = convert_torque(tv_B.Trq_FR);
    setpoints.rear_left.torque = convert_torque(tv_B.Trq_RL);
    setpoints.rear_right.torque = convert_torque(tv_B.Trq_RR);

    setpoints_publisher->publish(setpoints);
  } else {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Error in Simulink model");
  }
}

inline double Controller::convert_pedal_position(int16_t pedal_position) {
  static constexpr int32_t PEDAL_SCALER = 500;
  return (((float)pedal_position) / PEDAL_SCALER);
}

inline double Controller::convert_brake_pressure(int16_t brake_pressure) {
  // TODO: Implement brake pressure conversion
  return brake_pressure;
}

inline double Controller::convert_steering_wheel_position(int16_t steering_wheel_position) {
  // TODO: Implement steering wheel position conversion
  return steering_wheel_position;
}

inline int32_t Controller::convert_torque(double torque) {
  static constexpr float TORQUE_SCALER = 10.0;
  return (int32_t)(torque * TORQUE_SCALER);
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Controller>());
  rclcpp::shutdown();
}
