// Walk forward while exciting body height with a random Fourier-series trajectory.
// Height offset is clamped to [-0.18, 0.03] m (relative to default standing height).
//
// The Fourier coefficients are randomized per run (see SEED below); this node
// writes them to excitation_height_params.json on startup so
// plot/excitation_height.py can reconstruct and plot the exact commanded
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
static constexpr double SETTLE_TIME  = 1.0;    // s: hold center height before exciting
static constexpr double EXCITE_TIME  = 10.0;   // s: excitation duration
static constexpr double RESTORE_TIME = 2.0;    // s: stop and restore default height

static constexpr float  VX           = 0.1f;   // forward walking speed during excitation (m/s)

// Excitation tuning (see excitation.hpp for what each parameter controls).
static constexpr int    ORDER        = 3;
static constexpr int    NJOINTS      = 1;
static const std::vector<double> PARAM_RANGE = {0.05};  // m; tune from the plot
static constexpr double H_CENTER     = -0.075;           // m; offset the excitation oscillates around
static constexpr unsigned SEED       = 42;                // change for a different random "tryout"

static constexpr double H_MIN = -0.18;  // m, hardware spec
static constexpr double H_MAX =  0.03;  // m, hardware spec

static const char *PARAMS_FILE = "excitation_height_params.json";

class ExcitationHeight : public rclcpp::Node
{
public:
    ExcitationHeight()
        : Node("excitation_height"), t_(-1.0), body_height_0_(-1.0),
          exc_(ORDER, NJOINTS, PARAM_RANGE)
    {
        exc_.generate_random_param(SEED);
        exc_.set_duration(EXCITE_TIME);
        exc_.set_offset({H_CENTER});
        write_params_file();

        state_sub_ = create_subscription<unitree_go::msg::SportModeState>(
            "sportmodestate", 10,
            std::bind(&ExcitationHeight::state_cb, this, _1));

        req_pub_     = create_publisher<unitree_api::msg::Request>("/api/sport/request", 10);
        desired_pub_ = create_publisher<geometry_msgs::msg::PointStamped>("/excitation_height/desired", 10);

        timer_ = create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(DT * 1000)),
            std::bind(&ExcitationHeight::control_cb, this));
    }

private:
    void write_params_file()
    {
        nlohmann::json j = exc_.to_json();
        j["dt"] = DT;
        j["settle_time"] = SETTLE_TIME;
        j["excite_time"] = EXCITE_TIME;
        j["restore_time"] = RESTORE_TIME;
        j["vx"] = VX;
        j["h_min"] = H_MIN;
        j["h_max"] = H_MAX;
        std::ofstream out(PARAMS_FILE);
        out << j.dump(2);
        RCLCPP_INFO(get_logger(), "Wrote excitation params to %s", PARAMS_FILE);
    }

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

        const double phase2 = SETTLE_TIME + EXCITE_TIME;
        const double phase3 = phase2 + RESTORE_TIME;

        unitree_api::msg::Request req_h, req_m;
        double des_h = 0.0, des_vx = 0.0;

        if (t_ < SETTLE_TIME) {
            des_h  = H_CENTER;
            des_vx = 0.0;
            sport_req_.BodyHeight(req_h, clamp_height(des_h));
            req_pub_->publish(req_h);

        } else if (t_ < phase2) {
            const double excite_t = t_ - SETTLE_TIME;
            des_h  = exc_.eval(excite_t)[0];
            des_vx = VX;
            sport_req_.BodyHeight(req_h, clamp_height(des_h));
            sport_req_.Move(req_m, VX, 0.0f, 0.0f);
            req_pub_->publish(req_h);
            req_pub_->publish(req_m);

        } else if (t_ < phase3) {
            const double h_start = exc_.eval(EXCITE_TIME)[0];
            const double alpha = (t_ - phase2) / RESTORE_TIME;  // 0 -> 1
            des_h  = h_start + alpha * (0.0 - h_start);
            des_vx = 0.0;
            sport_req_.BodyHeight(req_h, clamp_height(des_h));
            sport_req_.Move(req_m, 0.0f, 0.0f, 0.0f);
            req_pub_->publish(req_h);
            req_pub_->publish(req_m);

        } else {
            RCLCPP_INFO_ONCE(get_logger(), "Done.");
            return;
        }

        if (body_height_0_ >= 0.0) {
            geometry_msgs::msg::PointStamped des_msg;
            des_msg.header.stamp = now();
            des_msg.point.x = des_vx;
            des_msg.point.y = 0.0;
            des_msg.point.z = body_height_0_ + des_h;
            desired_pub_->publish(des_msg);
        }
    }

    static float clamp_height(double h)
    {
        if (h < H_MIN) h = H_MIN;
        if (h > H_MAX) h = H_MAX;
        return static_cast<float>(h);
    }

    rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr state_sub_;
    rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr req_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr desired_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    SportClient sport_req_;
    double t_;
    double body_height_0_;
    FourierExcitation exc_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ExcitationHeight>());
    rclcpp::shutdown();
    return 0;
}
