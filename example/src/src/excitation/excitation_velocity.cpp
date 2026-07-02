// Excite forward/lateral/yaw velocity (vx, vy, vyaw) with a random Fourier-series
// trajectory sent directly to Move().
//
// The Fourier coefficients are randomized per run (see SEED below); this node
// writes them to excitation_velocity_params.json on startup so
// plot/excitation_velocity.py can reconstruct and plot the exact commanded
// trajectory offline.

#include <cmath>
#include <fstream>
#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/sport_mode_state.hpp"
#include "unitree_api/msg/request.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "common/ros2_sport_client.h"
#include "common/excitation.hpp"

using std::placeholders::_1;

static constexpr double DT           = 0.002;  // 500 Hz control loop
static constexpr double SETTLE_TIME  = 1.0;    // s: hold still before exciting
static constexpr double EXCITE_TIME  = 10.0;   // s: excitation duration
static constexpr double RESTORE_TIME = 2.0;    // s: ramp back to zero velocity

// Excitation tuning (see excitation.hpp for what each parameter controls).
// njoints = 3: [vx, vy, vyaw].
static constexpr int    ORDER        = 3;
static constexpr int    NJOINTS      = 3;
static const std::vector<double> PARAM_RANGE = {0.25, 0.15, 0.4};  // m/s, m/s, rad/s; tune from the plot
static constexpr unsigned SEED       = 42;                          // change for a different random "tryout"

// Safety clamp (not a published hardware spec, just a conservative sanity bound).
static constexpr double VX_LIMIT   = 0.4;   // m/s
static constexpr double VY_LIMIT   = 0.25;  // m/s
static constexpr double VYAW_LIMIT = 0.8;   // rad/s

static const char *PARAMS_FILE = "excitation_velocity_params.json";

class ExcitationVelocity : public rclcpp::Node
{
public:
    ExcitationVelocity()
        : Node("excitation_velocity"), t_(-1.0),
          exc_(ORDER, NJOINTS, PARAM_RANGE)
    {
        exc_.generate_random_param(SEED);
        exc_.set_duration(EXCITE_TIME);
        exc_.set_offset({0.0, 0.0, 0.0});
        write_params_file();

        state_sub_ = create_subscription<unitree_go::msg::SportModeState>(
            "sportmodestate", 10,
            std::bind(&ExcitationVelocity::state_cb, this, _1));

        req_pub_     = create_publisher<unitree_api::msg::Request>("/api/sport/request", 10);
        desired_pub_ = create_publisher<geometry_msgs::msg::PointStamped>("/excitation_velocity/desired", 10);

        timer_ = create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(DT * 1000)),
            std::bind(&ExcitationVelocity::control_cb, this));
    }

private:
    void write_params_file()
    {
        nlohmann::json j = exc_.to_json();
        j["dt"] = DT;
        j["settle_time"] = SETTLE_TIME;
        j["excite_time"] = EXCITE_TIME;
        j["restore_time"] = RESTORE_TIME;
        j["vx_limit"] = VX_LIMIT;
        j["vy_limit"] = VY_LIMIT;
        j["vyaw_limit"] = VYAW_LIMIT;
        std::ofstream out(PARAMS_FILE);
        out << j.dump(2);
        RCLCPP_INFO(get_logger(), "Wrote excitation params to %s", PARAMS_FILE);
    }

    void state_cb(unitree_go::msg::SportModeState::SharedPtr msg)
    {
        if (t_ < 0) {
            RCLCPP_INFO(get_logger(), "Current velocity: vx=%.2f vy=%.2f vyaw=%.2f",
                        msg->velocity[0], msg->velocity[1], msg->yaw_speed);
        }
    }

    void control_cb()
    {
        t_ += DT;
        if (t_ < 0) return;

        const double phase2 = SETTLE_TIME + EXCITE_TIME;
        const double phase3 = phase2 + RESTORE_TIME;

        unitree_api::msg::Request req;
        double vx = 0.0, vy = 0.0, vyaw = 0.0;

        if (t_ < SETTLE_TIME) {
            vx = vy = vyaw = 0.0;

        } else if (t_ < phase2) {
            const double excite_t = t_ - SETTLE_TIME;
            const std::vector<double> q = exc_.eval(excite_t);
            vx = q[0]; vy = q[1]; vyaw = q[2];

        } else if (t_ < phase3) {
            const std::vector<double> q_end = exc_.eval(EXCITE_TIME);
            const double alpha = (t_ - phase2) / RESTORE_TIME;  // 0 -> 1
            vx   = q_end[0] + alpha * (0.0 - q_end[0]);
            vy   = q_end[1] + alpha * (0.0 - q_end[1]);
            vyaw = q_end[2] + alpha * (0.0 - q_end[2]);

        } else {
            RCLCPP_INFO_ONCE(get_logger(), "Done.");
            return;
        }

        vx   = clamp(vx, VX_LIMIT);
        vy   = clamp(vy, VY_LIMIT);
        vyaw = clamp(vyaw, VYAW_LIMIT);

        sport_req_.Move(req, static_cast<float>(vx), static_cast<float>(vy), static_cast<float>(vyaw));
        req_pub_->publish(req);

        geometry_msgs::msg::PointStamped des_msg;
        des_msg.header.stamp = now();
        des_msg.point.x = vx;
        des_msg.point.y = vy;
        des_msg.point.z = vyaw;
        desired_pub_->publish(des_msg);
    }

    static double clamp(double v, double limit)
    {
        if (v < -limit) return -limit;
        if (v >  limit) return  limit;
        return v;
    }

    rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr state_sub_;
    rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr req_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr desired_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    SportClient sport_req_;
    double t_;
    FourierExcitation exc_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ExcitationVelocity>());
    rclcpp::shutdown();
    return 0;
}
