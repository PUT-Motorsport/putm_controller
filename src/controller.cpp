#include "putm_vcl_interfaces/msg/detail/frontbox__struct.hpp"
#include "putm_vcl_interfaces/msg/frontbox.hpp"
#include "putm_vcl_interfaces/msg/setpoints.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
using namespace putm_vcl_interfaces::msg;

using std::placeholders::_1;

class Controller : public rclcpp::Node
{
public:
  Controller();

private:
  static constexpr int32_t INVERTER_MAX_POSITIVE_TOURQE = 500;
  static constexpr int32_t INVERTER_MAX_NEGATIVE_TOURQE = -500;

  float pedal_position;
  bool rtd_state = true; // TODO: Implement

  struct tourqeModifiers
  {
    float front;
    float rear;
  } modifiers;

  rclcpp::TimerBase::SharedPtr main_loop_timer;
  rclcpp::Subscription<Frontbox>::SharedPtr frontbox_subscriber;
  rclcpp::Publisher<Setpoints>::SharedPtr setpoints_publisher;

  void main_loop_callback();
  void frontbox_topic_callback(const Frontbox msg);
};

Controller::Controller() : Node("controller"), pedal_position(0), modifiers{0.5, 0.5}
{
  setpoints_publisher = this->create_publisher<Setpoints>("setpoints", 10);
  frontbox_subscriber = this->create_subscription<Frontbox>(
    "frontbox", 10, std::bind(&Controller::frontbox_topic_callback, this, _1));
  main_loop_timer = this->create_wall_timer(10ms, std::bind(&Controller::main_loop_callback, this));
}

void Controller::main_loop_callback()
{
  /* continue only if we are in rtd */
  if (rtd_state) {
    auto tourqe_setpoints = Setpoints();
    int32_t tourqe_front = INVERTER_MAX_POSITIVE_TOURQE * pedal_position * modifiers.front;
    int32_t tourqe_rear = INVERTER_MAX_POSITIVE_TOURQE * pedal_position * modifiers.rear;
    tourqe_setpoints.tourqes[0] = tourqe_front;
    tourqe_setpoints.tourqes[1] = tourqe_front;
    tourqe_setpoints.tourqes[2] = tourqe_rear;
    tourqe_setpoints.tourqes[3] = tourqe_rear;

    setpoints_publisher->publish(tourqe_setpoints);
  }
}

void Controller::frontbox_topic_callback(const Frontbox msg)
{
  pedal_position = msg.pedal_position;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Controller>());
  rclcpp::shutdown();
  return 0;
}
