#include "putm_vcl_interfaces/msg/frontbox_driver_input.hpp"
#include "putm_vcl_interfaces/msg/bms_hv_main.hpp"
#include "putm_vcl_interfaces/msg/setpoints.hpp"
#include "putm_vcl_interfaces/msg/amk_actual_values1.hpp"
#include "rclcpp/rclcpp.hpp"
#include "putm_vcl_interfaces/msg/xsens_acceleration.hpp"
#include "putm_vcl_interfaces/msg/xsens_rate_of_turn.hpp"
#include "putm_vcl_interfaces/msg/yaw_ref.hpp"
#include "vectornav_msgs/msg/imu_group.hpp"
#include <Eigen/Dense>

#define MAX_MOMENT  185.25


extern "C" {
#include "acados_solver_tv_nmpc.h"
#include "acados_c/ocp_nlp_interface.h"
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
  inline int32_t convert_torque(double torque);
  inline double convert_wheel_speed(double rpm);
  inline void convert_steering_angle(double steering_wheel_deg, double &delta_l_rad, double &delta_r_rad);
  inline void calculate_load_transfer(double ax, double ay, double &fz_fl, double &fz_fr, double &fz_rl, double &fz_rr);
  inline double calculate_wheel_base_velocity(double w_fl, double w_fr, double w_rl, double w_rr, double delta_l, double delta_r);
  
  // Stany
  bool is_initialized;
  double speed_fl, speed_fr, speed_rl, speed_rr;
  double ay, ax, yaw_rate, batt_curr;
  Setpoints setpoints;
  YawRef yaw_ref;

  // Filtr
  double ax_filtered, ay_filtered, yaw_rate_filtered;
  const double lp_alpha_acc = 0.6;

  // Wskaźniki i bufory ACADOS
  tv_nmpc_solver_capsule *acados_capsule;
  ocp_nlp_config *nlp_config;
  ocp_nlp_dims *nlp_dims;
  ocp_nlp_in *nlp_in;
  ocp_nlp_out *nlp_out;

  double lbx0[TV_NMPC_NBX0];
  double ubx0[TV_NMPC_NBX0];
  double p_val[TV_NMPC_NP];
  double x_k1[TV_NMPC_NX];

  // TC
  double integral_err[4];
  double tau_final[4];

  // Parametry TC
  const double R_e = 0.193;
  const double dt = 0.005;
  const double Kp = 50.0;
  const double Ki = 100.0;
  const double kappa_limit = 0.15;

  // EKF
  Eigen::Vector2d ekf_x;
  Eigen::Matrix2d ekf_P;
  Eigen::Matrix2d ekf_Q;
  Eigen::MatrixXd ekf_R;
  Eigen::MatrixXd ekf_H;
  Eigen::Matrix2d ekf_I;

  inline void estimate_velocity_ekf(double ax, double ay, double r, double w_fl, double w_fr, double w_rl, double w_rr, double delta_l, double delta_r, double &vx_est, double &vy_est);

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
      ay(0.0), ax(0.0), yaw_rate(0.0), batt_curr(0.0),
      ax_filtered(0.0), ay_filtered(0.0), yaw_rate_filtered(0.0)
      {

      acados_capsule = tv_nmpc_acados_create_capsule();
      int status = tv_nmpc_acados_create(acados_capsule);
      if (status) {
        RCLCPP_FATAL(this->get_logger(), "Acados solver init failed with status: %d", status);
      }

      nlp_config = tv_nmpc_acados_get_nlp_config(acados_capsule);
      nlp_dims   = tv_nmpc_acados_get_nlp_dims(acados_capsule);
      nlp_in     = tv_nmpc_acados_get_nlp_in(acados_capsule);
      nlp_out    = tv_nmpc_acados_get_nlp_out(acados_capsule);

      for (int i = 0; i < 4; i++) {
        tau_final[i] = 0.0;
        integral_err[i] = 0.0;
      }

      ekf_x << 1.0, 0.0;
      ekf_P = Eigen::Matrix2d::Identity() * 1.0;
      
      ekf_Q = Eigen::Matrix2d::Identity();
      ekf_Q(0,0) = 0.05; 
      ekf_Q(1,1) = 0.1;
      
      ekf_R = Eigen::MatrixXd::Identity(5, 5) * 0.1;
      ekf_R(4,4) = 0.5;
      
      ekf_H = Eigen::MatrixXd::Zero(5, 2);
      ekf_H.col(0) << 1, 1, 1, 1, 0;
      ekf_H.col(1) << 0, 0, 0, 0, 1;
      
      ekf_I = Eigen::Matrix2d::Identity();
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
  double ax_raw = msg.accel.x * -1;
  double ay_raw = msg.accel.y * -1;
  yaw_rate = msg.angularrate.z;

  ax_filtered = lp_alpha_acc * ax_raw  + (1.0 - lp_alpha_acc) * ax_filtered;
  ay_filtered = lp_alpha_acc * ay_raw  + (1.0 - lp_alpha_acc) * ay_filtered;

  ax = ax_filtered;
  ay = ay_filtered;
}

void Controller::bms_hv_main_callback(const BmsHvMain msg) { batt_curr = msg.current; }



void Controller::control_loop() {
  auto start_time = std::chrono::high_resolution_clock::now();
  
  bool enable_tc = true; 

  double pedal = convert_pedal_position(frontbox_driver_input.pedal_position);
  double steering_angle_deg = frontbox_driver_input.steering_wheel_position;

  double w_fl = convert_wheel_speed(speed_fl);
  double w_fr = convert_wheel_speed(speed_fr);
  double w_rl = convert_wheel_speed(speed_rl);
  double w_rr = convert_wheel_speed(speed_rr);

  double delta_l_rad = 0.0;
  double delta_r_rad = 0.0;
  convert_steering_angle(steering_angle_deg, delta_l_rad, delta_r_rad);
  
  double vx_est = 1.0;
  double vy_est = 0.0;
  estimate_velocity_ekf(ax, ay, yaw_rate, w_fl, w_fr, w_rl, w_rr, delta_l_rad, delta_r_rad, vx_est, vy_est);

  // Low speed mode: poniżej 1 m/s, bez NMPC, bez TC, tylko mapowanie pedału na moment
  if (vx_est < 1.0) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
        "Low-Speed Mode (vx = %.2f). Bypassing NMPC.", vx_est);

    double manual_torque = pedal * MAX_MOMENT;

    tau_final[0] = manual_torque;
    tau_final[1] = manual_torque;
    tau_final[2] = manual_torque;
    tau_final[3] = manual_torque;

    is_initialized = false;
  } 
  // NMPC mode: Prędkość powyżej 1.0 m/s, pełne wektorowanie i Traction Control
  else {
    double fz_fl, fz_fr, fz_rl, fz_rr;
    calculate_load_transfer(ax, ay, fz_fl, fz_fr, fz_rl, fz_rr);

    double t_ref = pedal * MAX_MOMENT * 4; 

    // Nadpisanie parametrów bufora P dla wszystkich kroków horyzontu
    p_val[0] = yaw_rate; 
    p_val[1] = delta_l_rad;
    p_val[2] = delta_r_rad;
    p_val[3] = fz_fl;
    p_val[4] = fz_fr;
    p_val[5] = fz_rl;
    p_val[6] = fz_rr;
    p_val[7] = t_ref;

    for (int i = 0; i <= TV_NMPC_N; i++) {
      tv_nmpc_acados_update_params(acados_capsule, i, p_val, 8);
    }

    // Przygotowanie bufora stanu x0
    lbx0[0] = vx_est; ubx0[0] = vx_est;
    lbx0[1] = vy_est; ubx0[1] = vy_est;
    lbx0[2] = yaw_rate; ubx0[2] = yaw_rate;
    lbx0[3] = w_fl; ubx0[3] = w_fl;
    lbx0[4] = w_fr; ubx0[4] = w_fr;
    lbx0[5] = w_rl; ubx0[5] = w_rl;
    lbx0[6] = w_rr; ubx0[6] = w_rr;
    
    // Feedback stanów wewnętrznych
    lbx0[7] = tau_final[0]; ubx0[7] = tau_final[0];
    lbx0[8] = tau_final[1]; ubx0[8] = tau_final[1];
    lbx0[9] = tau_final[2]; ubx0[9] = tau_final[2];
    lbx0[10]= tau_final[3]; ubx0[10]= tau_final[3];

    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0, "lbx", lbx0);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0, "ubx", ubx0);

    // COLD START 
    if (!is_initialized) {
      double x_init[TV_NMPC_NX] = {vx_est, vy_est, yaw_rate, w_fl, w_fr, w_rl, w_rr, 
                                   tau_final[0], tau_final[1], tau_final[2], tau_final[3]};
      double u_init[TV_NMPC_NU] = {0.0, 0.0, 0.0, 0.0};

      for (int i = 0; i < TV_NMPC_N; i++) {
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, i, "x", x_init);
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, i, "u", u_init);
      }
      ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, TV_NMPC_N, "x", x_init);
      is_initialized = true;
    }
    
    // SOLVE
    int status = tv_nmpc_acados_solve(acados_capsule);
    
    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> elapsed_ms = end_time - start_time;

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, 
        "Czas NMPC: %.3f ms | Status: %d", elapsed_ms.count(), status);
    
    double tau_nmpc[4] = {0.0, 0.0, 0.0, 0.0};

    if (status != 0) {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 500, "NMPC Fail (Status: %d). Failsafe aktywny.", status);
      
      // FAILSAFE
      double x_reset[TV_NMPC_NX] = {vx_est, vy_est, yaw_rate, w_fl, w_fr, w_rl, w_rr, 0.0, 0.0, 0.0, 0.0};
      double u_reset[TV_NMPC_NU] = {0.0, 0.0, 0.0, 0.0};
      for (int k = 0; k <= TV_NMPC_N; k++) {
          ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, k, "x", x_reset);
      }
      for (int k = 0; k < TV_NMPC_N; k++) {
          ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, k, "u", u_reset);
      }

      tau_final[0] = 0; tau_final[1] = 0; tau_final[2] = 0; tau_final[3] = 0;
    } 
    else {
      // WARM START: Przesunięcie horyzontu
      double x_temp[TV_NMPC_NX];
      double u_temp[TV_NMPC_NU];
      for (int k = 0; k < TV_NMPC_N - 1; k++) {
          ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, k + 1, "x", x_temp);
          ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, k + 1, "u", u_temp);
          ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, k, "x", x_temp);
          ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, k, "u", u_temp);
      }
      ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, TV_NMPC_N - 1, "x", x_temp);
      ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, TV_NMPC_N, "x", x_temp);

      // Pobranie zoptymalizowanych momentów
      ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 1, "x", x_k1);
      tau_nmpc[0] = x_k1[7]; 
      tau_nmpc[1] = x_k1[8]; 
      tau_nmpc[2] = x_k1[9]; 
      tau_nmpc[3] = x_k1[10];

      // TC
      double w_actual[4] = {w_fl, w_fr, w_rl, w_rr};
      for (int i = 0; i < 4; i++) {
        if (enable_tc && pedal > 0.05) { 
          double kappa_actual = ((w_actual[i] * R_e) - vx_est) / vx_est;
          double error = kappa_actual - kappa_limit;
          double tau_tc = 0.0;

          if (error > 0) {
            integral_err[i] += error * dt;
            if(integral_err[i] > 5.0) integral_err[i] = 5.0; // Anti windup
            
            tau_tc = (Kp * error) + (Ki * integral_err[i]);
          } else {
            integral_err[i] = 0.0; 
            tau_tc = 0.0;
          }

          tau_final[i] = tau_nmpc[i] - tau_tc;
          if (tau_final[i] < 0.0) tau_final[i] = 0.0;
        } 
        else {
          integral_err[i] = 0.0;
          tau_final[i] = tau_nmpc[i]; 
        }
      }
    }
  }
  
  if (pedal < 0.01) {
    tau_final[0] = 0.0;
    tau_final[1] = 0.0;
    tau_final[2] = 0.0;
    tau_final[3] = 0.0;
  }

  // Publikacja 
  yaw_ref.yaw_rate_ref = yaw_rate; 
  yaw_ref.vx_est = vx_est * -1.0;
  yaw_ref.vy_est = vy_est * -1.0;
  yaw_rate_ref_publisher->publish(yaw_ref);

  setpoints.front_left.torque = convert_torque(tau_final[0]);
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

  return brake_pressure;
}


inline double Controller::convert_wheel_speed(double rpm) {
  
  double gear_ratio = 14.25; 
  return (rpm * (M_PI / 30.0)) / gear_ratio;
}

inline void Controller::convert_steering_angle(double steering_wheel_deg, double &delta_l_rad, double &delta_r_rad) {
  
  // Wielomiany geometrii Ackermanna
  double delta_l_deg = -0.0000942 * std::pow(steering_wheel_deg, 2) + 0.2543 * steering_wheel_deg + 0.0182;
  double delta_r_deg = 0.000410 * std::pow(steering_wheel_deg, 2) + 0.2554 * steering_wheel_deg + 0.0200;

  // Deg to rad
  delta_l_rad = delta_l_deg * (M_PI / 180.0);
  delta_r_rad = delta_r_deg * (M_PI / 180.0);
}

inline void Controller::calculate_load_transfer(double ax_sensor, double ay_sensor, double &fz_fl, double &fz_fr, double &fz_rl, double &fz_rr) {

  const double m = 300.0;     // Masa beniga
  const double g = 9.81;      // przyspieszenie
  const double a = 0.765;     // odleglosc cg od przedniej osi
  const double b = 0.765;     // odleglosc cg od tylnej osi 
  const double c = 0.621;     // polowa rostawu kol
  const double L = a + b;     // rozstaw osi
  const double h = 0.3;     // wysokosc cg

  double Fz_static_front = (m * g * b) / (2.0 * L);
  double Fz_static_rear  = (m * g * a) / (2.0 * L);

  double dFz_long = (m * h * ax_sensor) / (2.0 * L);
  double dFz_lat_front = (m * h * ay_sensor * b) / (2.0 * L * c);
  double dFz_lat_rear  = (m * h * ay_sensor * a) / (2.0 * L * c);

  fz_fl = Fz_static_front - dFz_long - dFz_lat_front;
  fz_fr = Fz_static_front - dFz_long + dFz_lat_front;
  fz_rl = Fz_static_rear  + dFz_long - dFz_lat_rear;
  fz_rr = Fz_static_rear  + dFz_long + dFz_lat_rear;

  if (fz_fl < 10.0) fz_fl = 10.0;
  if (fz_fr < 10.0) fz_fr = 10.0;
  if (fz_rl < 10.0) fz_rl = 10.0;
  if (fz_rr < 10.0) fz_rr = 10.0;
}

inline int32_t Controller::convert_torque(double torque) {
  static constexpr double TORQUE_SCALER = 1000.0;
  return (int32_t)(torque / MAX_MOMENT * TORQUE_SCALER);
}

inline void Controller::estimate_velocity_ekf(double ax, double ay, double r, double w_fl, double w_fr, double w_rl, double w_rr, double delta_l, double delta_r, double &vx_est, double &vy_est) {
  
  const double c = 0.621;  
  const double b = 0.765;  

  // PREDYKCJA
  Eigen::Vector2d x_pred;
  x_pred(0) = ekf_x(0) + (ax + ekf_x(1) * r) * dt;
  x_pred(1) = ekf_x(1) + (ay - ekf_x(0) * r) * dt;

  Eigen::Matrix2d F;
  F << 1.0,        r * dt,
      -r * dt, 1.0;

  Eigen::Matrix2d P_pred = F * ekf_P * F.transpose() + ekf_Q;

  // KOREKCJA - zmianione pomiary, aby lepiej odzwierciedlały prędkość bazującą na kołach i pseudo-pomiarze vy z akceleracji bocznej
  Eigen::VectorXd z(5);
  z(0) = w_fl * R_e * cos(delta_l);
  z(1) = w_fr * R_e * cos(delta_r);
  z(2) = w_rl * R_e;
  z(3) = w_rr * R_e;
  z(4) = r * b; 

  Eigen::VectorXd z_pred(5);
  z_pred(0) = x_pred(0) - r * c;
  z_pred(1) = x_pred(0) + r * c;
  z_pred(2) = x_pred(0) - r * c;
  z_pred(3) = x_pred(0) + r * c;
  z_pred(4) = x_pred(1);

  Eigen::VectorXd y = z - z_pred;

  Eigen::MatrixXd R_adaptive = ekf_R;
  double ax_penalty = std::abs(ax) * 0.5;
  
  for(int i = 0; i < 4; i++) {
    double spike_penalty = 5.0 * (y(i) * y(i)); 
    R_adaptive(i, i) += ax_penalty + spike_penalty;
  } 
  R_adaptive(4, 4) += std::abs(ay) * 0.2;

  Eigen::MatrixXd S = ekf_H * P_pred * ekf_H.transpose() + R_adaptive;
  Eigen::MatrixXd K = P_pred * ekf_H.transpose() * S.inverse();

  ekf_x = x_pred + K * y;
  ekf_P = (ekf_I - K * ekf_H) * P_pred;

  // zabezpieczenie przed zerową prędkością
  if (ekf_x(0) < 0.0) {
    ekf_x(0) = 0.0;
  }

  vx_est = ekf_x(0);
  vy_est = ekf_x(1);
}

inline double Controller::calculate_wheel_base_velocity(double w_fl, double w_fr, double w_rl, double w_rr, double delta_l, double delta_r) {
  double v_fl = w_fl * R_e * cos(delta_l);
  double v_fr = w_fr * R_e * cos(delta_r);
  
  double v_rl = w_rl * R_e;
  double v_rr = w_rr * R_e;

  double vx_wheels = (v_fl + v_fr + v_rl + v_rr) / 4.0;

  return vx_wheels;
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Controller>());
  rclcpp::shutdown();
}
