#include "putm_vcl_interfaces/msg/frontbox_driver_input.hpp"
#include "putm_vcl_interfaces/msg/setpoints.hpp"
#include "putm_vcl_interfaces/msg/amk_actual_values1.hpp"
#include "rclcpp/rclcpp.hpp"
#include "putm_vcl_interfaces/msg/xsens_acceleration.hpp"
#include "putm_vcl_interfaces/msg/xsens_rate_of_turn.hpp"

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
  rclcpp::Subscription<XsensAcceleration>::SharedPtr xsens_acceleration_ay_subscriber;
  rclcpp::Subscription<XsensAcceleration>::SharedPtr xsens_acceleration_ax_subscriber;
  rclcpp::Subscription<XsensRateOfTurn>::SharedPtr xsens_rate_of_turn_subscriber;

  inline double convert_pedal_position(int16_t pedal_position);
  inline double convert_brake_pressure(int16_t brake_pressure);
  inline double convert_steering_wheel_position(int16_t steering_wheel_position);

  inline int32_t convert_torque(double torque);
  uint8_t speed_fl, speed_fr, speed_rl, speed_rr;
  double ay, ax, yaw_rate;

  void frontbox_driver_input_topic_callback(const FrontboxDriverInput msg);
  void amk_actual_values1_callback(const AmkActualValues1 msg);
  void amk_actual_values2_callback(const AmkActualValues1 msg);
  void amk_actual_values3_callback(const AmkActualValues1 msg);
  void amk_actual_values4_callback(const AmkActualValues1 msg);
  void xsens_acceleration_ay_callback(const XsensAcceleration msg);
  void xsens_acceleration_ax_callback(const XsensAcceleration msg);
  void xsens_rate_of_turn_callback(const XsensRateOfTurn msg);

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
      amk_rear_right_actual_values1_subscriber(this->create_subscription<AmkActualValues1>("putm_vcl/amk/rear/right/actual_values1", 1, std::bind(&Controller::amk_actual_values4_callback, this, _1))),
      xsens_acceleration_ay_subscriber(this->create_subscription<XsensAcceleration>("putm_vcl/xsens_acceleration", 1, std::bind(&Controller::xsens_acceleration_ay_callback, this, _1))),
      xsens_acceleration_ax_subscriber(this->create_subscription<XsensAcceleration>("putm_vcl/xsens_acceleration", 1, std::bind(&Controller::xsens_acceleration_ax_callback, this, _1))),
      xsens_rate_of_turn_subscriber(this->create_subscription<XsensRateOfTurn>("putm_vcl/xsens_rate_of_turn", 1, std::bind(&Controller::xsens_rate_of_turn_callback, this, _1)))
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

void Controller::xsens_acceleration_ay_callback(const XsensAcceleration msg) 
{  
  ay = msg.acc_y;
}

void Controller::xsens_acceleration_ax_callback(const XsensAcceleration msg) 
{  
  ax = msg.acc_x;
}

void Controller::xsens_rate_of_turn_callback(const XsensRateOfTurn msg) 
{  
  yaw_rate = msg.gyr_x;
}



void Controller::control_loop() {
  if (rtmGetErrorStatus(tv_code_M) == (NULL) && !rtmGetStopRequested(tv_code_M)) {
    tv_code_P.acc_pedal_Value = convert_pedal_position(frontbox_driver_input.pedal_position);
    //tv_code_P.brake_pedal_Value = convert_brake_pressure((frontbox_driver_input.brake_pressure_front + frontbox_driver_input.brake_pressure_rear) / 2);
    tv_code_P.delta_Value = -1*3.1415*convert_steering_wheel_position(frontbox_driver_input.steering_wheel_position)/180;

    tv_code_P.lf = tv_code_P.L - tv_code_P.lr;
    tv_code_P.whl_speed_fl_Value = speed_fl / tv_code_P.drive_ratio;
    tv_code_P.whl_speed_fr_Value = speed_fr / tv_code_P.drive_ratio;
    tv_code_P.whl_speed_rl_Value = speed_rl / tv_code_P.drive_ratio;
    tv_code_P.whl_speed_rr_Value = speed_rr / tv_code_P.drive_ratio;

    tv_code_P.Constant1_Value = 0;
    tv_code_P.Switch_Threshold = 0;
    tv_code_P.Switch_Threshold_i = 1;

    tv_code_P.yaw_rate_Value = yaw_rate;
    tv_code_P.ax_Value = ax;
    tv_code_P.ay_Value = ay;
    
    tv_code_step();

    double torque_fl = tv_code_B.trq_fl / tv_code_P.drive_ratio ;
    double torque_fr = tv_code_B.trq_fr / tv_code_P.drive_ratio;
    double torque_rl = tv_code_B.trq_rl / tv_code_P.drive_ratio ;
    double torque_rr = tv_code_B.trq_rr / tv_code_P.drive_ratio ;

    torque_fl/=tv_code_P.max_moment;
    torque_fr/=tv_code_P.max_moment;
    torque_rl/=tv_code_P.max_moment;
    torque_rr/=tv_code_P.max_moment;

    auto setpoints = Setpoints();
    setpoints.front_left.torque = convert_torque(torque_fl)* -1;
    setpoints.front_right.torque = convert_torque(torque_fr)   ;
    setpoints.rear_left.torque = convert_torque(torque_rl)     ;
    setpoints.rear_right.torque = 0; //convert_torque(torque_rr)* -1;

    //RCLCPP_INFO(this->get_logger(), "RTD: on %f", tv_code_B.Product1);

    /*if(tv_code_P.acc_pedal_Value > 0.1){
    setpoints.front_left.torque = -70;
    setpoints.front_right.torque = 70;
    setpoints.rear_left.torque = 70;
    setpoints.rear_right.torque = -70;
    }
    else{
    setpoints.front_left.torque = 0;
    setpoints.front_right.torque = 0;
    setpoints.rear_left.torque = 0;
    setpoints.rear_right.torque = 0;
    }*/

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
  static constexpr double TORQUE_SCALER = 1000.0;
  return (int32_t)(torque * TORQUE_SCALER);
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Controller>());
  rclcpp::shutdown();
}
