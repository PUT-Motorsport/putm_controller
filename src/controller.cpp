#include "putm_vcl_interfaces/msg/frontbox_driver_input.hpp"
#include "putm_vcl_interfaces/msg/setpoints.hpp"
#include "putm_vcl_interfaces/msg/amk_actual_values1.hpp"
#include "rclcpp/rclcpp.hpp"

extern "C" {
#include "read.h"
#include "tv_code.h"
}

using namespace std::chrono_literals;
using namespace putm_vcl_interfaces::msg;

using std::placeholders::_1;

class Controller : public rclcpp::Node {
 public:
  Controller();
  ~Controller();

 private:
  FrontboxDriverInput frontbox_driver_input;

  rclcpp::Publisher<Setpoints>::SharedPtr setpoints_publisher;
  rclcpp::Subscription<FrontboxDriverInput>::SharedPtr frontbox_driver_input_subscriber;
  rclcpp::TimerBase::SharedPtr control_loop_timer;
  rclcpp::Subscription<AmkActualValues1>::SharedPtr amk_front_left_actual_values1_subscriber;
  rclcpp::Subscription<AmkActualValues1>::SharedPtr amk_front_right_actual_values1_subscriber;
  rclcpp::Subscription<AmkActualValues1>::SharedPtr amk_rear_left_actual_values1_subscriber;
  rclcpp::Subscription<AmkActualValues1>::SharedPtr amk_rear_right_actual_values1_subscriber;

  inline double convert_pedal_position(int16_t pedal_position);
  inline double convert_brake_pressure(int16_t brake_pressure);
  inline double convert_steering_wheel_position(int16_t steering_wheel_position);

  inline int32_t convert_torque(double torque);
  uint8_t speed_fl, speed_fr, speed_rl, speed_rr;

  void frontbox_driver_input_topic_callback(const FrontboxDriverInput msg);
  void amk_actual_values1_callback(const AmkActualValues1 msg);
  void amk_actual_values2_callback(const AmkActualValues1 msg);
  void amk_actual_values3_callback(const AmkActualValues1 msg);
  void amk_actual_values4_callback(const AmkActualValues1 msg);

  void control_loop();
};

Controller::Controller()
    : Node("controller"),
      setpoints_publisher(this->create_publisher<Setpoints>("putm_vcl/setpoints", 1)),
      frontbox_driver_input_subscriber(this->create_subscription<FrontboxDriverInput>("putm_vcl/frontbox_driver_input", 1, std::bind(&Controller::frontbox_driver_input_topic_callback, this, _1))),
      control_loop_timer(this->create_wall_timer(10ms, std::bind(&Controller::control_loop, this))) ,
      amk_front_left_actual_values1_subscriber(this->create_subscription<AmkActualValues1>("putm_vcl/amk/front/left/actual_values1", 1, std::bind(&Controller::amk_actual_values1_callback, this, _1))),
      amk_front_right_actual_values1_subscriber(this->create_subscription<AmkActualValues1>("putm_vcl/amk/front/right/actual_values1", 1, std::bind(&Controller::amk_actual_values2_callback, this, _1))),
      amk_rear_left_actual_values1_subscriber(this->create_subscription<AmkActualValues1>("putm_vcl/amk/rear/left/actual_values1", 1, std::bind(&Controller::amk_actual_values3_callback, this, _1))),
      amk_rear_right_actual_values1_subscriber(this->create_subscription<AmkActualValues1>("putm_vcl/amk/rear/right/actual_values1", 1, std::bind(&Controller::amk_actual_values4_callback, this, _1)))
      {
        tv_code_initialize();
        read_inputs();
      }

Controller::~Controller() { tv_code_terminate(); }

void Controller::frontbox_driver_input_topic_callback(const FrontboxDriverInput msg) { frontbox_driver_input = msg; }

void Controller::amk_actual_values1_callback(const AmkActualValues1 msg) 
{  
  speed_fl = abs(msg.actual_velocity);
}

void Controller::amk_actual_values2_callback(const AmkActualValues1 msg) 
{  
  speed_fr = abs(msg.actual_velocity);
}

void Controller::amk_actual_values3_callback(const AmkActualValues1 msg) 
{  
  speed_rl = abs(msg.actual_velocity);
}

void Controller::amk_actual_values4_callback(const AmkActualValues1 msg) 
{  
  speed_rr = abs(msg.actual_velocity);
}



void Controller::control_loop() {
  if (rtmGetErrorStatus(tv_code_M) == (NULL) && !rtmGetStopRequested(tv_code_M)) {
    tv_code_P.acc_pedal_Value = convert_pedal_position(frontbox_driver_input.pedal_position);
    //tv_code_P.brake_pedal_Value = convert_brake_pressure((frontbox_driver_input.brake_pressure_front + frontbox_driver_input.brake_pressure_rear) / 2);
    tv_code_P.delta_Value = convert_steering_wheel_position(frontbox_driver_input.steering_wheel_position);

    tv_code_P.whl_speed_fl_Value = speed_fl / tv_code_P.drive_ratio;
    tv_code_P.whl_speed_fr_Value = speed_fr / tv_code_P.drive_ratio;
    tv_code_P.whl_speed_rl_Value = speed_rl / tv_code_P.drive_ratio;
    tv_code_P.whl_speed_rr_Value = speed_rr / tv_code_P.drive_ratio;

    tv_code_P.Switch_Threshold = 0;
    tv_code_P.Switch_Threshold_i = 0;

    tv_code_P.ax_Value = 2;
    tv_code_P.ay_Value = 1;
    
    tv_code_step();


    double torque_fl = tv_code_B.trq_fl / tv_code_P.drive_ratio;
    double torque_fr = tv_code_B.trq_fr / tv_code_P.drive_ratio;
    double torque_rl = tv_code_B.trq_rl / tv_code_P.drive_ratio;
    double torque_rr = tv_code_B.trq_rr / tv_code_P.drive_ratio;

    auto setpoints = Setpoints();
    setpoints.front_left.torque = convert_torque(torque_fl);
    setpoints.front_right.torque = convert_torque(torque_fr);
    setpoints.rear_left.torque = convert_torque(torque_rl);
    setpoints.rear_right.torque = convert_torque(torque_rr);

    setpoints_publisher->publish(setpoints);
  } else {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Error in Simulink model");
  }
}

inline double Controller::convert_pedal_position(int16_t pedal_position) {
  static constexpr double PEDAL_SCALER = 500.0;
  return (((double)pedal_position) / PEDAL_SCALER);
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
  static constexpr double TORQUE_SCALER = 10.0;
  return (int32_t)(torque * TORQUE_SCALER);
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Controller>());
  rclcpp::shutdown();
}
