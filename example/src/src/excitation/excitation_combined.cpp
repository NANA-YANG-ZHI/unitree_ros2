// Simultaneous multi-axis excitation: body height, roll/pitch (via Euler()),
// and vx/vy/vyaw (via Move()) are each driven by their own independent random
// Fourier-series trajectory, all evaluated in one control loop and commanded
// together. Merge of excitation_height.cpp, excitation_pitch_roll.cpp and
// excitation_velocity.cpp into one node so all axes can be excited at once
// for system identification.
//
// Move() is owned entirely by the velocity excitation here -- there is no
// separate constant forward-walk speed like the standalone excitation_height
// node used, since that would conflict with the velocity excitation's own
// vx output.
//
// Each axis keeps its own order/param_range/seed so it can still be tuned
// independently via ROS params without touching the others; settle/excite/
// restore timing is shared since all three axes start and stop together.
// Coefficients are written to the same three excitation_*_params.json files
// the standalone nodes used, so the existing plot/excitation_*.py scripts
// work unchanged.

#include <cmath>
#include <fstream>
#include <stdexcept>
#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/sport_mode_state.hpp"
#include "unitree_api/msg/request.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "common/ros2_sport_client.h"
#include "common/excitation.hpp"
#include "common/param_helpers.hpp"

using std::placeholders::_1;

static constexpr double DT           = 0.002;  // 500 Hz control loop
static constexpr double SETTLE_TIME  = 2.0;    // s: hold neutral pose before exciting
static constexpr double EXCITE_TIME  = 35.0;   // s: excitation duration (shared by all axes)
static constexpr double RESTORE_TIME = 2.0;    // s: ramp all axes back to neutral

static inline float deg2rad(double d) { return static_cast<float>(d * M_PI / 180.0); }

// -- Height axis (see excitation_height.cpp for original single-axis version) --
static constexpr int    ORDER_H       = 3;
static const std::vector<double> PARAM_RANGE_H = {0.2};  // m; tune from the plot
static constexpr double H_CENTER      = -0.075;            // m; offset the excitation oscillates around
static constexpr unsigned SEED_H      = 42;
static constexpr double H_MIN = -0.14;  // m, hardware spec
static constexpr double H_MAX =  0.01;  // m, hardware spec

// -- Roll/pitch axis (see excitation_pitch_roll.cpp) --
static constexpr int    ORDER_PR         = 3;
static constexpr double PARAM_RANGE_DEG_PITCH  = 20.0;  // deg; see excitation_pitch_roll.cpp for scale note
static constexpr double PARAM_RANGE_DEG_ROLL = 15.0;
static constexpr double CLAMP_LIMIT_DEG_PITCH  = 20.0;  // deg; hard safety clamp on commanded Euler roll/pitch
static constexpr double CLAMP_LIMIT_DEG_ROLL = 15.0;
static constexpr unsigned SEED_PR        = 108;

// -- Velocity axis (see excitation_velocity.cpp) --
static constexpr int    ORDER_V       = 3;
static const std::vector<double> PARAM_RANGE_V = {0.13, 0.09, 0.2};  // m/s, m/s, rad/s
static constexpr unsigned SEED_V      = 208;
static constexpr double VX_LIMIT   = 0.4;   // m/s
static constexpr double VY_LIMIT   = 0.25;  // m/s
static constexpr double VYAW_LIMIT = 0.8;   // rad/s

static const char *PARAMS_FILE_H  = "excitation_height_params.json";
static const char *PARAMS_FILE_PR = "excitation_pitch_roll_params.json";
static const char *PARAMS_FILE_V  = "excitation_velocity_params.json";

static std::vector<double> deg_vec_to_rad(const std::vector<double> &deg)
{
    std::vector<double> rad(deg.size());
    for (std::size_t i = 0; i < deg.size(); ++i) rad[i] = deg[i] * M_PI / 180.0;
    return rad;
}

class ExcitationCombined : public rclcpp::Node
{
public:
    ExcitationCombined()
        : Node("excitation_combined"), t_(-1.0), body_height_0_(-1.0),
          order_h_(declare_and_get_int(this, "height_order", ORDER_H)),
          param_range_h_(declare_and_get_double_array(this, "height_param_range", PARAM_RANGE_H)),
          seed_h_(declare_and_get_uint(this, "height_seed", SEED_H)),
          h_center_(declare_and_get_double(this, "height_h_center", H_CENTER)),
          order_pr_(declare_and_get_int(this, "pr_order", ORDER_PR)),
          param_range_deg_pr_(declare_and_get_double_array(this, "pr_param_range_deg",
                                                            {PARAM_RANGE_DEG_PITCH, PARAM_RANGE_DEG_ROLL})),
          seed_pr_(declare_and_get_uint(this, "pr_seed", SEED_PR)),
          order_v_(declare_and_get_int(this, "vel_order", ORDER_V)),
          param_range_v_(declare_and_get_double_array(this, "vel_param_range", PARAM_RANGE_V)),
          seed_v_(declare_and_get_uint(this, "vel_seed", SEED_V)),
          excite_time_(declare_and_get_double(this, "excite_time", EXCITE_TIME)),
          exc_h_(order_h_, 1, param_range_h_),
          exc_pr_(order_pr_, 2, deg_vec_to_rad(param_range_deg_pr_)),
          exc_v_(order_v_, 3, param_range_v_)
    {
        if (static_cast<int>(param_range_h_.size()) != 1) {
            throw std::invalid_argument("height_param_range must have exactly 1 element(s)");
        }
        if (static_cast<int>(param_range_deg_pr_.size()) != 2) {
            throw std::invalid_argument("pr_param_range_deg must have exactly 2 element(s)");
        }
        if (static_cast<int>(param_range_v_.size()) != 3) {
            throw std::invalid_argument("vel_param_range must have exactly 3 element(s)");
        }

        exc_h_.generate_random_param(seed_h_);
        exc_h_.set_duration(excite_time_);
        exc_h_.set_offset({h_center_});

        exc_pr_.generate_random_param(seed_pr_);
        exc_pr_.set_duration(excite_time_);
        exc_pr_.set_offset({0.0, 0.0});

        exc_v_.generate_random_param(seed_v_);
        exc_v_.set_duration(excite_time_);
        exc_v_.set_offset({0.0, 0.0, 0.0});

        write_params_files();

        state_sub_ = create_subscription<unitree_go::msg::SportModeState>(
            "sportmodestate", 10,
            std::bind(&ExcitationCombined::state_cb, this, _1));

        req_pub_        = create_publisher<unitree_api::msg::Request>("/api/sport/request", 10);
        desired_h_pub_  = create_publisher<geometry_msgs::msg::PointStamped>("/excitation_height/desired", 10);
        desired_pr_pub_ = create_publisher<geometry_msgs::msg::PointStamped>("/excitation_pitch_roll/desired", 10);
        desired_v_pub_  = create_publisher<geometry_msgs::msg::PointStamped>("/excitation_velocity/desired", 10);

        timer_ = create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(DT * 1000)),
            std::bind(&ExcitationCombined::control_cb, this));
    }

private:
    void write_params_files()
    {
        nlohmann::json jh = exc_h_.to_json();
        jh["dt"] = DT;
        jh["settle_time"] = SETTLE_TIME;
        jh["excite_time"] = excite_time_;
        jh["restore_time"] = RESTORE_TIME;
        jh["h_min"] = H_MIN;
        jh["h_max"] = H_MAX;
        std::ofstream(PARAMS_FILE_H) << jh.dump(2);
        RCLCPP_INFO(get_logger(), "Wrote excitation params to %s", PARAMS_FILE_H);

        nlohmann::json jpr = exc_pr_.to_json();
        jpr["dt"] = DT;
        jpr["settle_time"] = SETTLE_TIME;
        jpr["excite_time"] = excite_time_;
        jpr["restore_time"] = RESTORE_TIME;
        jpr["param_range_deg"] = param_range_deg_pr_;
        jpr["clamp_limit_deg_roll"] = CLAMP_LIMIT_DEG_ROLL;
        jpr["clamp_limit_deg_pitch"] = CLAMP_LIMIT_DEG_PITCH;
        std::ofstream(PARAMS_FILE_PR) << jpr.dump(2);
        RCLCPP_INFO(get_logger(), "Wrote excitation params to %s", PARAMS_FILE_PR);

        nlohmann::json jv = exc_v_.to_json();
        jv["dt"] = DT;
        jv["settle_time"] = SETTLE_TIME;
        jv["excite_time"] = excite_time_;
        jv["restore_time"] = RESTORE_TIME;
        jv["vx_limit"] = VX_LIMIT;
        jv["vy_limit"] = VY_LIMIT;
        jv["vyaw_limit"] = VYAW_LIMIT;
        std::ofstream(PARAMS_FILE_V) << jv.dump(2);
        RCLCPP_INFO(get_logger(), "Wrote excitation params to %s", PARAMS_FILE_V);
    }

    void state_cb(unitree_go::msg::SportModeState::SharedPtr msg)
    {
        if (t_ < 0) {
            RCLCPP_INFO(get_logger(),
                        "Current body height: %.3f m, IMU rpy: roll=%.2f pitch=%.2f yaw=%.2f, "
                        "velocity: vx=%.2f vy=%.2f vyaw=%.2f",
                        msg->body_height,
                        msg->imu_state.rpy[0], msg->imu_state.rpy[1], msg->imu_state.rpy[2],
                        msg->velocity[0], msg->velocity[1], msg->yaw_speed);
            body_height_0_ = msg->body_height;  // latch nominal height before any commands
        }
    }

    void control_cb()
    {
        t_ += DT;
        if (t_ < 0) return;

        const double phase2 = SETTLE_TIME + excite_time_;
        const double phase3 = phase2 + RESTORE_TIME;

        double des_h = h_center_;
        double roll = 0.0, pitch = 0.0;
        double vx = 0.0, vy = 0.0, vyaw = 0.0;

        if (t_ < SETTLE_TIME) {
            // hold neutral: des_h/roll/pitch/vx/vy/vyaw already at their defaults above

        } else if (t_ < phase2) {
            const double excite_t = t_ - SETTLE_TIME;
            des_h = exc_h_.eval(excite_t)[0];
            const std::vector<double> q_pr = exc_pr_.eval(excite_t);
            roll = q_pr[0]; pitch = q_pr[1];
            const std::vector<double> q_v = exc_v_.eval(excite_t);
            vx = q_v[0]; vy = q_v[1]; vyaw = q_v[2];

        } else if (t_ < phase3) {
            const double alpha = (t_ - phase2) / RESTORE_TIME;  // 0 -> 1

            const double h_start = exc_h_.eval(excite_time_)[0];
            des_h = h_start + alpha * (0.0 - h_start);

            const std::vector<double> q_pr_end = exc_pr_.eval(excite_time_);
            roll  = q_pr_end[0] + alpha * (0.0 - q_pr_end[0]);
            pitch = q_pr_end[1] + alpha * (0.0 - q_pr_end[1]);

            const std::vector<double> q_v_end = exc_v_.eval(excite_time_);
            vx   = q_v_end[0] + alpha * (0.0 - q_v_end[0]);
            vy   = q_v_end[1] + alpha * (0.0 - q_v_end[1]);
            vyaw = q_v_end[2] + alpha * (0.0 - q_v_end[2]);

        } else {
            RCLCPP_INFO_ONCE(get_logger(), "Done.");
            return;
        }

        roll  = clamp(roll,  deg2rad(CLAMP_LIMIT_DEG_ROLL));
        pitch = clamp(pitch, deg2rad(CLAMP_LIMIT_DEG_PITCH));
        vx    = clamp(vx,   VX_LIMIT);
        vy    = clamp(vy,   VY_LIMIT);
        vyaw  = clamp(vyaw, VYAW_LIMIT);

        unitree_api::msg::Request req_h, req_e, req_m;
        sport_req_.BodyHeight(req_h, clamp_height(des_h));
        sport_req_.Euler(req_e, static_cast<float>(roll), static_cast<float>(pitch), 0.0f);
        sport_req_.Move(req_m, static_cast<float>(vx), static_cast<float>(vy), static_cast<float>(vyaw));
        req_pub_->publish(req_h);
        req_pub_->publish(req_e);
        req_pub_->publish(req_m);

        
        geometry_msgs::msg::PointStamped des_h_msg;
        des_h_msg.header.stamp = now();
        des_h_msg.point.x = 0.0;
        des_h_msg.point.y = 0.0;
        des_h_msg.point.z = 0.33 + des_h;
        desired_h_pub_->publish(des_h_msg);
        

        geometry_msgs::msg::PointStamped des_pr_msg;
        des_pr_msg.header.stamp = now();
        des_pr_msg.point.x = roll;
        des_pr_msg.point.y = pitch;
        des_pr_msg.point.z = 0.0;
        desired_pr_pub_->publish(des_pr_msg);

        geometry_msgs::msg::PointStamped des_v_msg;
        des_v_msg.header.stamp = now();
        des_v_msg.point.x = vx;
        des_v_msg.point.y = vy;
        des_v_msg.point.z = vyaw;
        desired_v_pub_->publish(des_v_msg);
    }

    static float clamp_height(double h)
    {
        if (h < H_MIN) h = H_MIN;
        if (h > H_MAX) h = H_MAX;
        return static_cast<float>(h);
    }

    static double clamp(double v, double limit)
    {
        if (v < -limit) return -limit;
        if (v >  limit) return  limit;
        return v;
    }

    rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr state_sub_;
    rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr req_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr desired_h_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr desired_pr_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr desired_v_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    SportClient sport_req_;
    double t_;
    double body_height_0_;

    int order_h_;
    std::vector<double> param_range_h_;
    unsigned seed_h_;
    double h_center_;

    int order_pr_;
    std::vector<double> param_range_deg_pr_;
    unsigned seed_pr_;

    int order_v_;
    std::vector<double> param_range_v_;
    unsigned seed_v_;

    double excite_time_;

    FourierExcitation exc_h_;
    FourierExcitation exc_pr_;
    FourierExcitation exc_v_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ExcitationCombined>());
    rclcpp::shutdown();
    return 0;
}
