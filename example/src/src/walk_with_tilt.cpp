// Example 2: Apply a roll or pitch tilt via Euler(), then walk forward while holding it.
// After walking, clears the tilt back to zero.

#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/sport_mode_state.hpp"
#include "unitree_api/msg/request.hpp"
#include "common/ros2_sport_client.h"

using std::placeholders::_1;

static constexpr double DT          = 0.002;  // 500 Hz
static constexpr double SETTLE_TIME = 1.0;    // seconds to hold tilt before walking
static constexpr double WALK_TIME   = 4.0;    // seconds of forward walking
static constexpr double CLEAR_TIME  = 1.0;    // seconds to hold zero tilt after walk
static constexpr float  VX          = 0.1f;   // forward speed while tilted (m/s)
static constexpr float  ROLL_DEG    = 10.0f;   // roll  (degrees) -- set what you want
static constexpr float  PITCH_DEG   = 0.0f;   // pitch (degrees) -- positive tilts nose up

static inline float deg2rad(float d) { return d * static_cast<float>(M_PI) / 180.0f; }

class WalkWithTilt : public rclcpp::Node
{
public:
    WalkWithTilt() : Node("walk_with_tilt"), t_(-1.0)
    {
        state_sub_ = create_subscription<unitree_go::msg::SportModeState>(
            "sportmodestate", 10,
            std::bind(&WalkWithTilt::state_cb, this, _1));

        req_pub_ = create_publisher<unitree_api::msg::Request>("/api/sport/request", 10);

        timer_ = create_wall_timer(
            std::chrono::milliseconds(int(DT * 1000)),
            std::bind(&WalkWithTilt::control_cb, this));
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

        double phase2 = SETTLE_TIME + WALK_TIME;
        double phase3 = phase2 + CLEAR_TIME;

        unitree_api::msg::Request req;

        if (t_ < SETTLE_TIME) {
            // Apply tilt, stand still
            sport_req_.Euler(req, deg2rad(ROLL_DEG), deg2rad(PITCH_DEG), 0.0f);
            req_pub_->publish(req);

        } else if (t_ < phase2) {
            // Walk forward while maintaining tilt
            // Send Euler first, then Move -- both are individual DDS messages
            unitree_api::msg::Request req2;
            sport_req_.Euler(req,  deg2rad(ROLL_DEG), deg2rad(PITCH_DEG), 0.0f);
            sport_req_.Move(req2, VX, 0.0f, 0.0f);
            req_pub_->publish(req);
            req_pub_->publish(req2);

        } else if (t_ < phase3) {
            // Stop walking, clear tilt
            sport_req_.Euler(req, 0.0f, 0.0f, 0.0f);
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
    rclcpp::spin(std::make_shared<WalkWithTilt>());
    rclcpp::shutdown();
    return 0;
}
