#include <cstdio>
#include "putm_pm09_vcl/msg/setpoints.hpp"
#include "putm_pm09_vcl/msg/detail/frontbox__struct.hpp"
#include "putm_pm09_vcl/msg/frontbox.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

constexpr int32_t INVERTER_MAX_POSITIVE_TOURQE = 500;
constexpr int32_t INVERTER_MAX_NEGATIVE_TOURQE = -500;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class InvertersDriver : public rclcpp::Node
{
  public:
    InvertersDriver(): Node("inverters_river"), pedalPosition(0), modifiers{0.5,0.5}
    {
      setpointsPublisher = this->create_publisher<putm_pm09_vcl::msg::Setpoints>("setpoints", 10);
      frontBoxSubscriber = this->create_subscription<putm_pm09_vcl::msg::Frontbox>("frontbox", 10, std::bind(&InvertersDriver::frontboxTopicCallback, this, _1));
      mainLoopTimer = this->create_wall_timer(10ms, std::bind(&InvertersDriver::MainLoopCallback, this));
    }

  private:
    void MainLoopCallback()
    {
      /* continue only if we are in rtd */
      auto tourqeSetpoints = putm_pm09_vcl::msg::Setpoints();
      int32_t tourqe_front = INVERTER_MAX_POSITIVE_TOURQE * pedalPosition * modifiers.front;
      int32_t tourqe_rear =  INVERTER_MAX_POSITIVE_TOURQE * pedalPosition * modifiers.rear;
      tourqeSetpoints.tourqes[0] = tourqe_front;
      tourqeSetpoints.tourqes[1] = tourqe_front;
      tourqeSetpoints.tourqes[2] = tourqe_rear;
      tourqeSetpoints.tourqes[3] = tourqe_rear;

      setpointsPublisher->publish(tourqeSetpoints);

    }
    void frontboxTopicCallback(const putm_pm09_vcl::msg::Frontbox msg)
    {
      pedalPosition = msg.pedal_position;

    }
    float pedalPosition;
    bool rtd_state;
    struct tourqeModifiers
    {
      float front;
      float rear;
    }modifiers;

    rclcpp::TimerBase::SharedPtr mainLoopTimer;
    rclcpp::Subscription<putm_pm09_vcl::msg::Frontbox>:: SharedPtr frontBoxSubscriber;
    rclcpp::Publisher<putm_pm09_vcl::msg::Setpoints>::SharedPtr setpointsPublisher;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<InvertersDriver>());
  rclcpp::shutdown();
  return 0;
}
