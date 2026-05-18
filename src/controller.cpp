#include "putm_vcl_interfaces/msg/frontbox_driver_input.hpp"
#include "putm_vcl_interfaces/msg/bms_hv_main.hpp"
#include "putm_vcl_interfaces/msg/setpoints.hpp"
#include "putm_vcl_interfaces/msg/amk_actual_values1.hpp"
#include "rclcpp/rclcpp.hpp"
#include "putm_vcl_interfaces/msg/xsens_acceleration.hpp"
#include "putm_vcl_interfaces/msg/xsens_rate_of_turn.hpp"
#include "putm_vcl_interfaces/msg/yaw_ref.hpp"
#include "vectornav_msgs/msg/imu_group.hpp"

#define MAX_MOMENT  185.25


extern "C" {
#include "acados_solver_tv_nmpc.h"
#include "acados_c/ocp_nlp_interface.h"
}

// extern "C" {
// #include "read.h"
// #include "tv_code.h"
// }

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
  rclcpp::Publisher<YawRef>::SharedPtr yaw_rate_ref_publisher;
  rclcpp::Subscription<FrontboxDriverInput>::SharedPtr frontbox_driver_input_subscriber;
  rclcpp::Subscription<AmkActualValues1>::SharedPtr amk_front_left_actual_values1_subscriber;
  rclcpp::Subscription<AmkActualValues1>::SharedPtr amk_front_right_actual_values1_subscriber;
  rclcpp::Subscription<AmkActualValues1>::SharedPtr amk_rear_left_actual_values1_subscriber;
  rclcpp::Subscription<AmkActualValues1>::SharedPtr amk_rear_right_actual_values1_subscriber;
  rclcpp::Subscription<XsensAcceleration>::SharedPtr xsens_acceleration_ay_subscriber;
  rclcpp::Subscription<XsensAcceleration>::SharedPtr xsens_acceleration_ax_subscriber;
  rclcpp::Subscription<XsensRateOfTurn>::SharedPtr xsens_rate_of_turn_subscriber;
  rclcpp::Subscription<vectornav_msgs::msg::ImuGroup>::SharedPtr vn300_rate_of_turn_subscriber;
  rclcpp::Subscription<BmsHvMain>::SharedPtr bms_hv_main_subscriber;
  rclcpp::TimerBase::SharedPtr control_loop_timer;

  inline double convert_pedal_position(int16_t pedal_position);
  inline double convert_brake_pressure(int16_t brake_pressure);
  inline double convert_steering_wheel_position(int16_t steering_wheel_position);
  inline int32_t convert_torque(double torque);
  
  // Stany
  bool is_initialized;
  int16_t previous_pos;
  uint8_t speed_fl, speed_fr, speed_rl, speed_rr;
  double ay, ax, yaw_rate, batt_curr;
  Setpoints setpoints;

  // --- Wskaźniki i bufory --
  tv_nmpc_solver_capsule *acados_capsule;
  ocp_nlp_config *nlp_config;
  ocp_nlp_dims *nlp_dims;
  ocp_nlp_in *nlp_in;
  ocp_nlp_out *nlp_out;

  double lbx0[TV_NMPC_NBX0];
  double ubx0[TV_NMPC_NBX0];
  double p_val[TV_NMPC_NP];
  double x_k1[TV_NMPC_NX];

  // --- Zmienne TC ---
  double integral_err[4];
  double tau_final[4];

  // Parametry TC
  const double R_e = 0.193;
  const double dt = 0.01;
  const double Kp = 50.0;
  const double Ki = 100.0;
  const double kappa_limit = 0.15;

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
};

Controller::Controller()
    : Node("controller"),
      setpoints_publisher(this->create_publisher<Setpoints>("putm_vcl/setpoints", 1)),
      yaw_rate_ref_publisher(this->create_publisher<YawRef>("yaw_ref", 1)),
      frontbox_driver_input_subscriber(this->create_subscription<FrontboxDriverInput>("putm_vcl/frontbox_driver_input", 1, std::bind(&Controller::frontbox_driver_input_topic_callback, this, _1))),
      amk_front_left_actual_values1_subscriber(this->create_subscription<AmkActualValues1>("putm_vcl/amk/front/left/actual_values1", 1, std::bind(&Controller::amk_actual_values1_callback, this, _1))),
      amk_front_right_actual_values1_subscriber(this->create_subscription<AmkActualValues1>("putm_vcl/amk/front/right/actual_values1", 1, std::bind(&Controller::amk_actual_values2_callback, this, _1))),
      amk_rear_left_actual_values1_subscriber(this->create_subscription<AmkActualValues1>("putm_vcl/amk/rear/left/actual_values1", 1, std::bind(&Controller::amk_actual_values3_callback, this, _1))),
      amk_rear_right_actual_values1_subscriber(this->create_subscription<AmkActualValues1>("putm_vcl/amk/rear/right/actual_values1", 1, std::bind(&Controller::amk_actual_values4_callback, this, _1))),
      xsens_acceleration_ay_subscriber(this->create_subscription<XsensAcceleration>("putm_vcl/xsens_acceleration", 1, std::bind(&Controller::xsens_acceleration_ay_callback, this, _1))),
      xsens_acceleration_ax_subscriber(this->create_subscription<XsensAcceleration>("putm_vcl/xsens_acceleration", 1, std::bind(&Controller::xsens_acceleration_ax_callback, this, _1))),
      xsens_rate_of_turn_subscriber(this->create_subscription<XsensRateOfTurn>("putm_vcl/xsens_rate_of_turn", 1, std::bind(&Controller::xsens_rate_of_turn_callback, this, _1))),
      vn300_rate_of_turn_subscriber(this->create_subscription<vectornav_msgs::msg::ImuGroup>("vectornav/raw/imu", 1,  std::bind(&Controller::vn300_rate_of_turn_callback, this, _1))),
      bms_hv_main_subscriber(this->create_subscription<BmsHvMain>("putm_vcl/bms_hv_main", 1,  std::bind(&Controller::bms_hv_main_callback, this, _1))),
      control_loop_timer(this->create_wall_timer(5ms, std::bind(&Controller::control_loop, this))),
      is_initialized(false),
      previous_pos(0),
      speed_fl(0), speed_fr(0), speed_rl(0), speed_rr(0),
      ay(0.0), ax(0.0), yaw_rate(0.0), batt_curr(0.0)
      {
    // 1. Inicjalizacja Solvera
      acados_capsule = tv_nmpc_acados_create_capsule();
      int status = tv_nmpc_acados_create(acados_capsule);
      if (status) {
        RCLCPP_FATAL(this->get_logger(), "Acados solver init failed with status: %d", status);
      }

      // 2. Przypisanie wskaźników konfiguracyjnych RAZ
      nlp_config = tv_nmpc_acados_get_nlp_config(acados_capsule);
      nlp_dims   = tv_nmpc_acados_get_nlp_dims(acados_capsule);
      nlp_in     = tv_nmpc_acados_get_nlp_in(acados_capsule);
      nlp_out    = tv_nmpc_acados_get_nlp_out(acados_capsule);

      // 3. Wyzerowanie buforów TC
      for (int i = 0; i < 4; i++) {
        tau_final[i] = 0.0;
        integral_err[i] = 0.0;
      }
      }

Controller::~Controller() {
  tv_nmpc_acados_free(acados_capsule);
  tv_nmpc_acados_free_capsule(acados_capsule);
}

void Controller::frontbox_driver_input_topic_callback(const FrontboxDriverInput msg) { frontbox_driver_input = msg; }
void Controller::amk_actual_values1_callback(const AmkActualValues1 msg) { speed_fl = abs(msg.actual_velocity); }
void Controller::amk_actual_values2_callback(const AmkActualValues1 msg) { speed_fr = abs(msg.actual_velocity); }
void Controller::amk_actual_values3_callback(const AmkActualValues1 msg) { speed_rl = abs(msg.actual_velocity); }
void Controller::amk_actual_values4_callback(const AmkActualValues1 msg) { speed_rr = abs(msg.actual_velocity); }

void Controller::xsens_acceleration_ay_callback(const XsensAcceleration msg) { (void)msg; /* ay = msg.acc_y; */ }
void Controller::xsens_acceleration_ax_callback(const XsensAcceleration msg) { (void)msg; /* ax = msg.acc_x; */ }
void Controller::xsens_rate_of_turn_callback(const XsensRateOfTurn msg) { (void)msg; /* yaw_rate = msg.gyr_z; */ }

void Controller::vn300_rate_of_turn_callback(const vectornav_msgs::msg::ImuGroup msg) {  
  (void)msg;
  // yaw_rate = msg.angularrate.z;
  // ay = msg.accel.y * -1;
  // ax = msg.accel.x * -1;
  yaw_rate = 0;
  ay = 0;
  ax = 0;
}

void Controller::bms_hv_main_callback(const BmsHvMain msg) { batt_curr = msg.current; }



void Controller::control_loop() {

  auto start_time = std::chrono::high_resolution_clock::now();
  bool enable_tc = false;

  // double pedal = convert_pedal_position(frontbox_driver_input.pedal_position);
  // double steering_angle = convert_steering_wheel_position(frontbox_driver_input.steering_wheel_position);
  // double delta_kier = -1.0 * (M_PI * steering_angle / 180.0) / 5.0; 

  // double w_fl = speed_fl * 0.10472;
  // double w_fr = speed_fr * 0.10472;
  // double w_rl = speed_rl * 0.10472;
  // double w_rr = speed_rr * 0.10472;

  double pedal = 0.8;
  double delta_kier = 0.2;
  
  double vx_est = 15.0;
  double vy_est = 0.5;
  yaw_rate = 0.3;
  
  double w_ideal = vx_est / R_e;
  double w_fl = w_ideal;
  double w_fr = w_ideal;
  double w_rl = w_ideal;
  double w_rr = w_ideal;

  // double vx_est = ((w_fl + w_fr) / 2.0) * R_e;
  if(vx_est < 1.0) vx_est = 1.0; // NMPC wymaga V > 0
  // double vy_est = 0.0;

  // Nadpisanie parametrów bufora P
  p_val[0] = 0.0; // r_ref
  p_val[1] = delta_kier;
  p_val[2] = delta_kier;
  p_val[3] = 750.0; // Fz_fl
  p_val[4] = 750.0; // Fz_fr
  p_val[5] = 750.0; // Fz_rl
  p_val[6] = 750.0; // Fz_rr
  p_val[7] = pedal * 4.0 * 13.0 * 14.25; // t_ref

  for (int i = 0; i <= TV_NMPC_N; i++) {
    tv_nmpc_acados_update_params(acados_capsule, i, p_val, 8);
  }

  // Nadpisanie bufora stanu lbx0 / ubx0
  lbx0[0] = vx_est; ubx0[0] = vx_est;
  lbx0[1] = vy_est; ubx0[1] = vy_est;
  lbx0[2] = yaw_rate; ubx0[2] = yaw_rate;
  lbx0[3] = w_fl; ubx0[3] = w_fl;
  lbx0[4] = w_fr; ubx0[4] = w_fr;
  lbx0[5] = w_rl; ubx0[5] = w_rl;
  lbx0[6] = w_rr; ubx0[6] = w_rr;
  
  // Feedback z poprzedniego kroku po TC
  lbx0[7] = tau_final[0]; ubx0[7] = tau_final[0];
  lbx0[8] = tau_final[1]; ubx0[8] = tau_final[1];
  lbx0[9] = tau_final[2]; ubx0[9] = tau_final[2];
  lbx0[10]= tau_final[3]; ubx0[10]= tau_final[3];

  ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0, "lbx", lbx0);
  ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0, "ubx", ubx0);

  // NMPC
  if (!is_initialized) {
    double x_init[TV_NMPC_NX];
    for (int i=0; i<TV_NMPC_NX; i++) x_init[i] = 0.0;
    
    x_init[0] = vx_est;
    x_init[1] = vy_est;
    x_init[2] = yaw_rate;
    x_init[3] = w_fl;
    x_init[4] = w_fr;
    x_init[5] = w_rl;
    x_init[6] = w_rr;
    
    double u_init[TV_NMPC_NU] = {0.0, 0.0, 0.0, 0.0};

    for (int i = 0; i < TV_NMPC_N; i++) {
      ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, i, "x", x_init);
      ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, i, "u", u_init);
    }
    ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, TV_NMPC_N, "x", x_init);
    
    is_initialized = true;
    RCLCPP_INFO(this->get_logger(), "Acados horizon initialized with vx = %.2f", vx_est);
  }
  
  int status = tv_nmpc_acados_solve(acados_capsule);
  
  auto end_time = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double, std::milli> elapsed_ms = end_time - start_time;

  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, 
      "Czas NMPC Solve: %.3f ms | Status: %d", elapsed_ms.count(), status);
  
  if (status != 0) {
    RCLCPP_WARN(this->get_logger(), "Acados solve failed!");
    tau_final[0] = 0; tau_final[1] = 0; tau_final[2] = 0; tau_final[3] = 0;
  } else {
    ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 1, "x", x_k1);
    double tau_nmpc[4] = {x_k1[7], x_k1[8], x_k1[9], x_k1[10]};
    double w_actual[4] = {w_fl, w_fr, w_rl, w_rr};

    // TC
    for (int i = 0; i < 4; i++) {
      if (enable_tc) {
        double kappa_actual = ((w_actual[i] * R_e) - vx_est) / vx_est;
        double error = kappa_actual - kappa_limit;
        double tau_tc = 0.0;

        if (error > 0) {
          integral_err[i] += error * dt;
          tau_tc = (Kp * error) + (Ki * integral_err[i]);
        } else {
          integral_err[i] = 0.0;
          tau_tc = 0.0;
        }

        tau_final[i] = tau_nmpc[i] - tau_tc;
        if (tau_final[i] < 0) tau_final[i] = 0.0;
      } 
      else {
        integral_err[i] = 0.0;
        tau_final[i] = tau_nmpc[i]; 
      }
    
    }
  }

  setpoints.front_left.torque = convert_torque(tau_final[0]) * -1;
  setpoints.front_right.torque = convert_torque(tau_final[1]);
  setpoints.rear_left.torque = convert_torque(tau_final[2]);
  setpoints.rear_right.torque = convert_torque(tau_final[3]);
  setpoints_publisher->publish(setpoints);
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
  return (int32_t)(torque / MAX_MOMENT * TORQUE_SCALER);
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Controller>());
  rclcpp::shutdown();
}