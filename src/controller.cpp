#include "putm_vcl_interfaces/msg/frontbox_driver_input.hpp"
#include "putm_vcl_interfaces/msg/bms_hv_main.hpp"
#include "putm_vcl_interfaces/msg/setpoints.hpp"
#include "putm_vcl_interfaces/msg/amk_actual_values1.hpp"
#include "rclcpp/rclcpp.hpp"
#include "putm_vcl_interfaces/msg/xsens_acceleration.hpp"
#include "putm_vcl_interfaces/msg/xsens_rate_of_turn.hpp"
#include "putm_vcl_interfaces/msg/yaw_ref.hpp"
#include "vectornav_msgs/msg/imu_group.hpp"

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
  int16_t previous_pos;
  FrontboxDriverInput frontbox_driver_input;

  rclcpp::QoS qos_;

  rclcpp::Time last_call_time_;

  rclcpp::Publisher<Setpoints>::SharedPtr setpoints_publisher;
  rclcpp::Publisher<YawRef>::SharedPtr yaw_rate_ref_publisher;
  rclcpp::Subscription<FrontboxDriverInput>::SharedPtr frontbox_driver_input_subscriber;
  rclcpp::TimerBase::SharedPtr control_loop_timer;
  rclcpp::Subscription<AmkActualValues1>::SharedPtr amk_front_left_actual_values1_subscriber;
  rclcpp::Subscription<AmkActualValues1>::SharedPtr amk_front_right_actual_values1_subscriber;
  rclcpp::Subscription<AmkActualValues1>::SharedPtr amk_rear_left_actual_values1_subscriber;
  rclcpp::Subscription<AmkActualValues1>::SharedPtr amk_rear_right_actual_values1_subscriber;
  rclcpp::Subscription<XsensAcceleration>::SharedPtr xsens_acceleration_ay_subscriber;
  rclcpp::Subscription<XsensAcceleration>::SharedPtr xsens_acceleration_ax_subscriber;
  rclcpp::Subscription<XsensRateOfTurn>::SharedPtr xsens_rate_of_turn_subscriber;
  rclcpp::Subscription<vectornav_msgs::msg::ImuGroup>::SharedPtr vn300_rate_of_turn_subscriber;
  rclcpp::Subscription<BmsHvMain>::SharedPtr bms_hv_main_subscriber;

  inline double convert_pedal_position(int16_t pedal_position);
  inline double convert_brake_pressure(int16_t brake_pressure);
  inline double convert_steering_wheel_position(int16_t steering_wheel_position);

  inline int32_t convert_torque(double torque);
  uint8_t speed_fl, speed_fr, speed_rl, speed_rr;
  double ay, ax, yaw_rate, batt_curr;

  void frontbox_driver_input_topic_callback(const FrontboxDriverInput msg);
  void amk_actual_values1_callback(const AmkActualValues1 msg);
  void amk_actual_values2_callback(const AmkActualValues1 msg);
  void amk_actual_values3_callback(const AmkActualValues1 msg);
  void amk_actual_values4_callback(const AmkActualValues1 msg);
  void xsens_acceleration_ay_callback(const XsensAcceleration msg);
  void xsens_acceleration_ax_callback(const XsensAcceleration msg);
  void xsens_rate_of_turn_callback(const XsensRateOfTurn msg);
  void vn300_rate_of_turn_callback(const vectornav_msgs::msg::ImuGroup msg);
  void bms_hv_main_callback(const BmsHvMain msg);

  void control_loop();
  double torque_fl;
  double torque_fr;
  double torque_rl;
  double torque_rr;
};





Controller::Controller()
    : Node("controller"),

      qos_(rclcpp::QoS(1)
              .best_effort()
              .durability_volatile()),

      setpoints_publisher(this->create_publisher<Setpoints>("putm_vcl/setpoints", 1)),
      frontbox_driver_input_subscriber(this->create_subscription<FrontboxDriverInput>("putm_vcl/frontbox_driver_input", 1, std::bind(&Controller::frontbox_driver_input_topic_callback, this, _1))),
      control_loop_timer(this->create_wall_timer(10ms, std::bind(&Controller::control_loop, this))) ,
      amk_front_left_actual_values1_subscriber(this->create_subscription<AmkActualValues1>("putm_vcl/amk/front/left/actual_values1", qos_, std::bind(&Controller::amk_actual_values1_callback, this, _1))),
      amk_front_right_actual_values1_subscriber(this->create_subscription<AmkActualValues1>("putm_vcl/amk/front/right/actual_values1", qos_, std::bind(&Controller::amk_actual_values2_callback, this, _1))),
      amk_rear_left_actual_values1_subscriber(this->create_subscription<AmkActualValues1>("putm_vcl/amk/rear/left/actual_values1", qos_, std::bind(&Controller::amk_actual_values3_callback, this, _1))),
      amk_rear_right_actual_values1_subscriber(this->create_subscription<AmkActualValues1>("putm_vcl/amk/rear/right/actual_values1", qos_, std::bind(&Controller::amk_actual_values4_callback, this, _1))),
      xsens_acceleration_ay_subscriber(this->create_subscription<XsensAcceleration>("putm_vcl/xsens_acceleration", 1, std::bind(&Controller::xsens_acceleration_ay_callback, this, _1))),
      xsens_acceleration_ax_subscriber(this->create_subscription<XsensAcceleration>("putm_vcl/xsens_acceleration", 1, std::bind(&Controller::xsens_acceleration_ax_callback, this, _1))),
      xsens_rate_of_turn_subscriber(this->create_subscription<XsensRateOfTurn>("putm_vcl/xsens_rate_of_turn", 1, std::bind(&Controller::xsens_rate_of_turn_callback, this, _1))),
      yaw_rate_ref_publisher(this->create_publisher<YawRef>("yaw_ref", 1)),
      vn300_rate_of_turn_subscriber(this->create_subscription<vectornav_msgs::msg::ImuGroup>("vectornav/raw/imu", 1,  std::bind(&Controller::vn300_rate_of_turn_callback, this, _1))),
      bms_hv_main_subscriber(this->create_subscription<BmsHvMain>("putm_vcl/bms_hv_main", 1,  std::bind(&Controller::bms_hv_main_callback, this, _1))),
      previous_pos(0)
      {
        rclcpp::QoS qos(1);
        qos.best_effort();
        qos.durability_volatile();
        last_call_time_ = this->now();
        tv_code_initialize();
        read_inputs();
        torque_fl = 0;
        torque_fr = 0;
        torque_rl = 0;
        torque_rr = 0;
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
  // ay = msg.acc_y;
}

void Controller::xsens_acceleration_ax_callback(const XsensAcceleration msg) 
{  
  // ax = msg.acc_x;
}

void Controller::xsens_rate_of_turn_callback(const XsensRateOfTurn msg) 
{  
  // yaw_rate = msg.gyr_z;
}

void Controller::vn300_rate_of_turn_callback(const vectornav_msgs::msg::ImuGroup msg) 
{  
  // yaw_rate = msg.angularrate.z;
  // ay = msg.accel.y * -1;
  // ax = msg.accel.x * -1;
  yaw_rate = 0;
  ay = 0;
  ax = 0;
}

void Controller::bms_hv_main_callback(const BmsHvMain msg) 
{  
  batt_curr = msg.current;
}



void Controller::control_loop() {
  if (rtmGetErrorStatus(tv_code_M) == (NULL) && !rtmGetStopRequested(tv_code_M)) {
    //testing loop duration
    auto now = this->now();
    auto time_since_last_call = now - last_call_time_;
    last_call_time_ = now;

    // RCLCPP_INFO(this->get_logger(), "Time since last call: %f ms", time_since_last_call.seconds() * 1000.0);
    
    tv_code_P.acc_pedal_Value = convert_pedal_position(frontbox_driver_input.pedal_position);
    //tv_code_P.brake_pedal_Value = convert_brake_pressure((frontbox_driver_input.brake_pressure_front + frontbox_driver_input.brake_pressure_rear) / 2);
    tv_code_P.delta_Value = -1*3.1415*convert_steering_wheel_position(frontbox_driver_input.steering_wheel_position)/180;

    tv_code_P.delta_Value/=5;

    tv_code_P.avg_min_speed_switch_CurrentSet = 1;


    tv_code_P.whl_speed_fl_Value = speed_rl;
    tv_code_P.whl_speed_fr_Value = speed_fr;
    tv_code_P.whl_speed_rl_Value = speed_rl;
    tv_code_P.whl_speed_rr_Value = speed_rr;

    // tv_code_P.whl_speed_fl_Value = 1000;
    // tv_code_P.whl_speed_fr_Value = 1000;
    // tv_code_P.whl_speed_rl_Value = 1000;
    // tv_code_P.whl_speed_rr_Value = 1000;

    tv_code_P.speed_switch_Threshold = 0;

    tv_code_P.TT_max_Value = 30;

    tv_code_P.regen_switch_CurrentSetting = 1;
    // tv_code_P.batt_curr_Value = abs(batt_curr/100);
    tv_code_P.yaw_rate_Value = yaw_rate;
    tv_code_P.ax_Value = ax;
    tv_code_P.ay_Value = ay;
    tv_code_P.Mz_p=300;
    tv_code_P.Mz_I=30;
    tv_code_P.Ku=-1/2000;
    // tv_code_P.power_speed_limiter_switch_Thre = 100000000;
    
    tv_code_step();

    // torque_fl = tv_code_P.acc_pedal_Value;
    // torque_fr = tv_code_P.acc_pedal_Value;
    // torque_rl = tv_code_P.acc_pedal_Value;
    // torque_rr = tv_code_P.acc_pedal_Value;

    torque_fl = tv_code_B.trq_fl / tv_code_P.drive_ratio ;
    torque_fr = tv_code_B.trq_fr / tv_code_P.drive_ratio;
    torque_rl = tv_code_B.trq_rl / tv_code_P.drive_ratio ;
    torque_rr = tv_code_B.trq_rr / tv_code_P.drive_ratio ;

    torque_fl/=tv_code_P.max_moment;
    torque_fr/=tv_code_P.max_moment;
    torque_rl/=tv_code_P.max_moment;
    torque_rr/=tv_code_P.max_moment;

    


    auto setpoints = Setpoints();
    auto vpdata = YawRef();
    vpdata.current_change = tv_code_B.speed_filter_fr.speed_filter_fl;
    vpdata.yaw_rate_ref = tv_code_B.avg_min_speed_switch;
    // vpdata.est_power = tv_code_B.est_power;
    // vpdata.torque_fixed = tv_code_B.torque_fixed;
    // vpdata.ifl = tv_code_B.T_max;
    // vpdata.ufl = tv_code_B.UFL;
    // vpdata.ifr = tv_code_B.IFR;
    // vpdata.ufr = tv_code_B.UFR;
    // vpdata.irl = tv_code_B.IRL;
    // vpdata.url = tv_code_B.URL;
    // vpdata.irr = tv_code_B.IRR;
    // vpdata.urr = tv_code_B.URR;

    setpoints.front_left.torque = convert_torque(torque_fl)* -1;
    setpoints.front_right.torque = convert_torque(torque_fr);
    setpoints.rear_left.torque = convert_torque(torque_rl);
    // setpoints.rear_right.torque = convert_torque(torque_rr)* -1;
    //michal
    setpoints.rear_right.torque = convert_torque(torque_rr);

    // RCLCPP_INFO(this->get_logger(), "est batt current: %f %f", tv_code_B.est_bat_current, tv_code_P.P_max / tv_code_P.batt_voltage);


    setpoints_publisher->publish(setpoints);
    yaw_rate_ref_publisher->publish(vpdata);
    // testing loop duration
    auto end_time = this->now();
    auto loop_duration = end_time - now;
    // RCLCPP_INFO(this->get_logger(), "Loop duration: %f ms", loop_duration.seconds() * 1000.0);
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
  if(previous_pos < -100 && steering_wheel_position > 100){
    steering_wheel_position = -135;
  }
  previous_pos = steering_wheel_position;

  
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