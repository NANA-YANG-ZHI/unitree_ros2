// Walk forward while oscillating body roll in a sinusoidal pattern via Euler().
// After walking, clears the tilt back to zero.

#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/sport_mode_state.hpp"
#include "unitree_api/msg/request.hpp"
#include "common/ros2_sport_client.h"

using std::placeholders::_1;

static constexpr double DT           = 0.002;  // 500 Hz control loop
static constexpr double SETTLE_TIME  = 1.0;    // s: reach center roll before walking
static constexpr double WALK_TIME    = 5.0;    // s: walk with sin-wave roll (1 full cycle)
static constexpr double RESTORE_TIME = 1.0;    // s: stop and restore zero roll

static constexpr float  VX           = 0.1f;   // forward walking speed (m/s)

// Sin-wave roll parameters (degrees)
static constexpr float  ROLL_CENTER_DEG = 0.0f;   // midpoint of oscillation
static constexpr float  ROLL_AMP_DEG    = 10.0f;  // amplitude; range -> [-10, 10] deg
static constexpr double ROLL_PERIOD     = 5.0;    // period (s)
// Sin-wave pitch parameters (degrees)
static constexpr float  PITCH_CENTER_DEG = 0.0f;   // midpoint of oscillation
static constexpr float  PITCH_AMP_DEG    = 10.0f;  // amplitude; range -> [-10, 10] deg
static constexpr double PITCH_PERIOD     = 5.0;    // period (s)

static inline float deg2rad(float d) { return d * static_cast<float>(M_PI) / 180.0f; }

class WalkWithSinRoll : public rclcpp::Node
{
public:
    WalkWithSinRoll() : Node("walk_with_sin_roll"), t_(-1.0)
    {
        state_sub_ = create_subscription<unitree_go::msg::SportModeState>(
            "sportmodestate", 10,
            std::bind(&WalkWithSinRoll::state_cb, this, _1));

        req_pub_ = create_publisher<unitree_api::msg::Request>("/api/sport/request", 10);

        timer_ = create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(DT * 1000)),
            std::bind(&WalkWithSinRoll::control_cb, this));
    }

private:
    void state_cb(unitree_go::msg::SportModeState::SharedPtr msg)
    {
        if (t_ < 0) {
            RCLCPP_INFO(get_logger(), "IMU rpy: roll=%.2f pitch=%.2f yaw=%.2f",
                        msg->imu_state.rpy[0], msg->imu_state.rpy[1], msg->imu_state.rpy[2]);
        }
    }

    void control_cb()
    {
        t_ += DT;
        if (t_ < 0) return;

        const double phase2 = SETTLE_TIME + WALK_TIME;
        const double phase3 = phase2 + RESTORE_TIME;

        unitree_api::msg::Request req;

        if (t_ < SETTLE_TIME) {
            sport_req_.Euler(req, deg2rad(ROLL_CENTER_DEG), deg2rad(PITCH_CENTER_DEG), 0.0f);
            req_pub_->publish(req);

        } else if (t_ < phase2) {
            const double walk_t = t_ - SETTLE_TIME;
            const float roll_deg = ROLL_CENTER_DEG +
                ROLL_AMP_DEG * static_cast<float>(std::sin(2.0 * M_PI * walk_t / ROLL_PERIOD));
            const float pitch_deg = PITCH_CENTER_DEG +
                PITCH_AMP_DEG * static_cast<float>(std::sin(2.0 * M_PI * walk_t / PITCH_PERIOD));
            unitree_api::msg::Request req2;
            sport_req_.Euler(req,  deg2rad(roll_deg), deg2rad(pitch_deg), 0.0f);
            sport_req_.Move(req2, VX, 0.0f, 0.0f);
            req_pub_->publish(req);
            req_pub_->publish(req2);

        } else if (t_ < phase3) {
            const float pitch_start = PITCH_CENTER_DEG +
                PITCH_AMP_DEG * static_cast<float>(std::sin(2.0 * M_PI * WALK_TIME / PITCH_PERIOD));
            const float alpha = static_cast<float>((t_ - phase2) / RESTORE_TIME);  // 0 -> 1
            const float pitch_deg = pitch_start + alpha * (0.0f - pitch_start);

            const float roll_start = ROLL_CENTER_DEG +
                ROLL_AMP_DEG * static_cast<float>(std::sin(2.0 * M_PI * WALK_TIME / ROLL_PERIOD));
            const float beta = static_cast<float>((t_ - phase2) / RESTORE_TIME);  // 0 -> 1
            const float roll_deg = roll_start + beta * (0.0f - roll_start);
            sport_req_.Euler(req, deg2rad(roll_deg), deg2rad(pitch_deg), 0.0f);
            req_pub_->publish(req);
            unitree_api::msg::Request req2;
            sport_req_.Move(req2, 0.0f, 0.0f, 0.0f);
            req_pub_->publish(req2);

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
    rclcpp::spin(std::make_shared<WalkWithSinRoll>());
    rclcpp::shutdown();
    return 0;
}
