// Walk forward while oscillating body height in a sinusoidal pattern.
// Height offset stays within [-0.18, 0.03] m (relative to default standing height).

#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/sport_mode_state.hpp"
#include "unitree_api/msg/request.hpp"
#include "common/ros2_sport_client.h"

using std::placeholders::_1;

static constexpr double DT           = 0.002;   // 500 Hz control loop
static constexpr double SETTLE_TIME  = 2.0;     // s: reach center height before walking
static constexpr double WALK_TIME    = 5.0;     // s: walk with sin-wave height (1 full cycle)
static constexpr double RESTORE_TIME = 2.0;     // s: stop and restore default height

static constexpr float  VX           = 0.1f;    // forward walking speed (m/s)

// Sin-wave height parameters — all values within [-0.18, 0.03] m
static constexpr double H_CENTER     = -0.075;  // midpoint of oscillation (m)
static constexpr double H_AMP        = 0.06;    // amplitude (m); range → [-0.135, -0.015]
static constexpr double H_PERIOD     = 5.0;     // period (s) — slow for first try

class WalkWithSinHeight : public rclcpp::Node
{
public:
    WalkWithSinHeight() : Node("walk_with_sin_height"), t_(-1.0)
    {
        state_sub_ = create_subscription<unitree_go::msg::SportModeState>(
            "sportmodestate", 10,
            std::bind(&WalkWithSinHeight::state_cb, this, _1));

        req_pub_ = create_publisher<unitree_api::msg::Request>("/api/sport/request", 10);

        timer_ = create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(DT * 1000)),
            std::bind(&WalkWithSinHeight::control_cb, this));
    }

private:
    void state_cb(unitree_go::msg::SportModeState::SharedPtr msg)
    {
        if (t_ < 0) {
            RCLCPP_INFO(get_logger(), "Current body height: %.3f m", msg->body_height);
        }
    }

    void control_cb()
    {
        t_ += DT;
        if (t_ < 0) return;

        const double phase2 = SETTLE_TIME + WALK_TIME;
        const double phase3 = phase2 + RESTORE_TIME;

        unitree_api::msg::Request req_h, req_m;

        if (t_ < SETTLE_TIME) {
            sport_req_.BodyHeight(req_h, static_cast<float>(H_CENTER));
            req_pub_->publish(req_h);

        } else if (t_ < phase2) {
            const double walk_t = t_ - SETTLE_TIME;
            const double h = H_CENTER + H_AMP * std::sin(2.0 * M_PI * walk_t / H_PERIOD);

            sport_req_.BodyHeight(req_h, static_cast<float>(h));
            sport_req_.Move(req_m, VX, 0.0f, 0.0f);
            req_pub_->publish(req_h);
            req_pub_->publish(req_m);

        } else if (t_ < phase3) {
            // height at the exact end of walk phase
            const double h_start = H_CENTER + H_AMP * std::sin(2.0 * M_PI * WALK_TIME / H_PERIOD);
            const double alpha = (t_ - phase2) / RESTORE_TIME;  // 0 → 1
            const double h = h_start + alpha * (0.0 - h_start);  // interpolate to default
            sport_req_.BodyHeight(req_h, static_cast<float>(h));
            sport_req_.Move(req_m, 0.0f, 0.0f, 0.0f);
            req_pub_->publish(req_h);
            req_pub_->publish(req_m);

        } else {
            RCLCPP_INFO_ONCE(get_logger(), "Done.");
        }
    }

    rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr state_sub_;
    rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr req_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    SportClient sport_req_;
    double t_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WalkWithSinHeight>());
    rclcpp::shutdown();
    return 0;
}
