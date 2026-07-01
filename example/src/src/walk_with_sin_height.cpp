// Walk forward while oscillating body height in a sinusoidal pattern.
// Height offset stays within [-0.18, 0.03] m (relative to default standing height).

#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/sport_mode_state.hpp"
#include "unitree_api/msg/request.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
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
    WalkWithSinHeight() : Node("walk_with_sin_height"), t_(-1.0), body_height_0_(-1.0)
    {
        state_sub_ = create_subscription<unitree_go::msg::SportModeState>(
            "sportmodestate", 10,
            std::bind(&WalkWithSinHeight::state_cb, this, _1));

        req_pub_     = create_publisher<unitree_api::msg::Request>("/api/sport/request", 10);
        desired_pub_ = create_publisher<geometry_msgs::msg::PointStamped>("/walk_sin_height/desired", 10);

        timer_ = create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(DT * 1000)),
            std::bind(&WalkWithSinHeight::control_cb, this));
    }

private:
    void state_cb(unitree_go::msg::SportModeState::SharedPtr msg)
    {
        if (t_ < 0) {
            RCLCPP_INFO(get_logger(), "Current body height: %.3f m", msg->body_height);
            body_height_0_ = msg->body_height;  // latch nominal height before any commands
        }
    }

    void control_cb()
    {
        t_ += DT;
        if (t_ < 0) return;

        const double phase2 = SETTLE_TIME + WALK_TIME;
        const double phase3 = phase2 + RESTORE_TIME;

        unitree_api::msg::Request req_h, req_m;
        double des_h = 0.0, des_vx = 0.0;

        if (t_ < SETTLE_TIME) {
            des_h  = H_CENTER;
            des_vx = 0.0;
            sport_req_.BodyHeight(req_h, static_cast<float>(des_h));
            req_pub_->publish(req_h);

        } else if (t_ < phase2) {
            const double walk_t = t_ - SETTLE_TIME;
            des_h  = H_CENTER + H_AMP * std::sin(2.0 * M_PI * walk_t / H_PERIOD);
            des_vx = VX;
            sport_req_.BodyHeight(req_h, static_cast<float>(des_h));
            sport_req_.Move(req_m, VX, 0.0f, 0.0f);
            req_pub_->publish(req_h);
            req_pub_->publish(req_m);

        } else if (t_ < phase3) {
            const double h_start = H_CENTER + H_AMP * std::sin(2.0 * M_PI * WALK_TIME / H_PERIOD);
            const double alpha = (t_ - phase2) / RESTORE_TIME;  // 0 → 1
            des_h  = h_start + alpha * (0.0 - h_start);
            des_vx = 0.0;
            sport_req_.BodyHeight(req_h, static_cast<float>(des_h));
            sport_req_.Move(req_m, 0.0f, 0.0f, 0.0f);
            req_pub_->publish(req_h);
            req_pub_->publish(req_m);

        } else {
            RCLCPP_INFO_ONCE(get_logger(), "Done.");
            return;
        }

        // Publish desired commands; z = absolute body height for direct comparison with state
        if (body_height_0_ >= 0.0) {
            geometry_msgs::msg::PointStamped des_msg;
            des_msg.header.stamp = now();
            des_msg.point.x = des_vx;
            des_msg.point.y = 0.0;
            des_msg.point.z = body_height_0_ + des_h;
            desired_pub_->publish(des_msg);
        }
    }

    rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr state_sub_;
    rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr req_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr desired_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    SportClient sport_req_;
    double t_;
    double body_height_0_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WalkWithSinHeight>());
    rclcpp::shutdown();
    return 0;
}
