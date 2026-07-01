// Example 3: Change body height via BodyHeight(), then walk forward while holding it.
// After walking, restores the default height.

#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/sport_mode_state.hpp"
#include "unitree_api/msg/request.hpp"
#include "common/ros2_sport_client.h"

using std::placeholders::_1;

static constexpr double DT           = 0.002;  // 500 Hz
static constexpr double SETTLE_TIME  = 1.5;    // seconds to reach new height before walking
static constexpr double WALK_TIME    = 4.0;    // seconds of forward walking
static constexpr double RESTORE_TIME = 1.5;    // seconds to restore default height
static constexpr float  VX           = 0.3f;   // forward speed (m/s)
// BodyHeight offset is relative to default standing height (0.0 = default).
// Positive raises the body, negative lowers it. Typical range: -0.1 to +0.1 m.
static constexpr float  HEIGHT_OFFSET = 0.05f; // raise 8 cm above default

class WalkWithHeight : public rclcpp::Node
{
public:
    WalkWithHeight() : Node("walk_with_height"), t_(-1.0)
    {
        state_sub_ = create_subscription<unitree_go::msg::SportModeState>(
            "sportmodestate", 10,
            std::bind(&WalkWithHeight::state_cb, this, _1));

        req_pub_ = create_publisher<unitree_api::msg::Request>("/api/sport/request", 10);

        timer_ = create_wall_timer(
            std::chrono::milliseconds(int(DT * 1000)),
            std::bind(&WalkWithHeight::control_cb, this));
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

        double phase2 = SETTLE_TIME + WALK_TIME;
        double phase3 = phase2 + RESTORE_TIME;

        unitree_api::msg::Request req;

        if (t_ < SETTLE_TIME) {
            // Set new height, stand still
            sport_req_.BodyHeight(req, HEIGHT_OFFSET);
            req_pub_->publish(req);

        } else if (t_ < phase2) {
            // Walk forward while holding height
            unitree_api::msg::Request req2;
            sport_req_.BodyHeight(req,  HEIGHT_OFFSET);
            sport_req_.Move(req2, VX, 0.0f, 0.0f);
            req_pub_->publish(req);
            req_pub_->publish(req2);

        } else if (t_ < phase3) {
            // Stop walking, restore default height
            sport_req_.BodyHeight(req, 0.0f);
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
    rclcpp::spin(std::make_shared<WalkWithHeight>());
    rclcpp::shutdown();
    return 0;
}
