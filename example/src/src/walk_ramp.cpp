// Example 1: Walk forward with slow ramp-up, cruise, then slow ramp-down to stop.
// Uses Move(vx, vy, vyaw). Sequence driven by elapsed time.

#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/sport_mode_state.hpp"
#include "unitree_api/msg/request.hpp"
#include "common/ros2_sport_client.h"

using std::placeholders::_1;

static constexpr double DT          = 0.02;   // 50 Hz control loop
static constexpr double RAMP_TIME   = 2.0;    // seconds to ramp up / ramp down
static constexpr double CRUISE_TIME = 4.0;    // seconds at full speed
static constexpr double MAX_VX      = 0.5;    // m/s forward speed

class WalkRamp : public rclcpp::Node
{
public:
    WalkRamp() : Node("walk_ramp"), t_(0.0), ready_(false)
    {
        state_sub_ = create_subscription<unitree_go::msg::SportModeState>(
            "sportmodestate", 10,
            std::bind(&WalkRamp::state_cb, this, _1));

        req_pub_ = create_publisher<unitree_api::msg::Request>("/api/sport/request", 10);

        timer_ = create_wall_timer(
            std::chrono::milliseconds(int(DT * 1000)),
            std::bind(&WalkRamp::control_cb, this));
    }

private:
    void state_cb(unitree_go::msg::SportModeState::SharedPtr msg)
    {
        if (!ready_) {
            RCLCPP_INFO(get_logger(), "Start pos: x=%.2f y=%.2f yaw=%.2f",
                        msg->position[0], msg->position[1], msg->imu_state.rpy[2]);
            ready_ = true;
        }
    }

    void control_cb()
    {
        if (!ready_) return;

        double total = 2 * RAMP_TIME + CRUISE_TIME;
        double vx = 0.0;

        if (t_ < RAMP_TIME) {
            // Ramp up: smoothly go from 0 to MAX_VX
            vx = MAX_VX * (t_ / RAMP_TIME);
        } else if (t_ < RAMP_TIME + CRUISE_TIME) {
            // Cruise at full speed
            vx = MAX_VX;
        } else if (t_ < total) {
            // Ramp down: smoothly go from MAX_VX to 0
            double ramp_down_t = t_ - RAMP_TIME - CRUISE_TIME;
            vx = MAX_VX * (1.0 - ramp_down_t / RAMP_TIME);
        } else {
            // Stopped -- send zero once then idle
            unitree_api::msg::Request req;
            sport_req_.Move(req, 0.0f, 0.0f, 0.0f);
            req_pub_->publish(req);
            RCLCPP_INFO_ONCE(get_logger(), "Walk complete, robot stopped.");
            return;
        }

        unitree_api::msg::Request req;
        sport_req_.Move(req, static_cast<float>(vx), 0.0f, 0.0f);
        req_pub_->publish(req);

        t_ += DT;
    }

    rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr state_sub_;
    rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr req_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    SportClient sport_req_;
    double t_;
    bool ready_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WalkRamp>());
    rclcpp::shutdown();
    return 0;
}
