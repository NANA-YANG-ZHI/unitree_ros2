// Excite body roll and pitch (via Euler()) with a random Fourier-series
// trajectory while standing in place.
//
// The Fourier coefficients are randomized per run (see SEED below); this node
// writes them to excitation_pitch_roll_params.json on startup so
// plot/excitation_pitch_roll.py can reconstruct and plot the exact commanded
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
static constexpr double SETTLE_TIME  = 1.0;    // s: hold level before exciting
static constexpr double EXCITE_TIME  = 10.0;   // s: excitation duration
static constexpr double RESTORE_TIME = 1.0;    // s: ramp back to level

static inline float deg2rad(double d) { return static_cast<float>(d * M_PI / 180.0); }

// Excitation tuning (see excitation.hpp for what each parameter controls).
// njoints = 2: [roll, pitch] (rad).
static constexpr int    ORDER          = 3;
static constexpr int    NJOINTS        = 2;
static constexpr double AMP_LIMIT_DEG  = 10.0;  // deg, matches walk_with_sin_roll.cpp spec
static const std::vector<double> PARAM_RANGE = {
    AMP_LIMIT_DEG * M_PI / 180.0, AMP_LIMIT_DEG * M_PI / 180.0};  // rad; tune from the plot
static constexpr unsigned SEED = 42;  // change for a different random "tryout"

static const char *PARAMS_FILE = "excitation_pitch_roll_params.json";

class ExcitationPitchRoll : public rclcpp::Node
{
public:
    ExcitationPitchRoll()
        : Node("excitation_pitch_roll"), t_(-1.0),
          exc_(ORDER, NJOINTS, PARAM_RANGE)
    {
        exc_.generate_random_param(SEED);
        exc_.set_duration(EXCITE_TIME);
        exc_.set_offset({0.0, 0.0});
        write_params_file();

        state_sub_ = create_subscription<unitree_go::msg::SportModeState>(
            "sportmodestate", 10,
            std::bind(&ExcitationPitchRoll::state_cb, this, _1));

        req_pub_     = create_publisher<unitree_api::msg::Request>("/api/sport/request", 10);
        desired_pub_ = create_publisher<geometry_msgs::msg::PointStamped>("/excitation_pitch_roll/desired", 10);

        timer_ = create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(DT * 1000)),
            std::bind(&ExcitationPitchRoll::control_cb, this));
    }

private:
    void write_params_file()
    {
        nlohmann::json j = exc_.to_json();
        j["dt"] = DT;
        j["settle_time"] = SETTLE_TIME;
        j["excite_time"] = EXCITE_TIME;
        j["restore_time"] = RESTORE_TIME;
        j["amp_limit_deg"] = AMP_LIMIT_DEG;
        std::ofstream out(PARAMS_FILE);
        out << j.dump(2);
        RCLCPP_INFO(get_logger(), "Wrote excitation params to %s", PARAMS_FILE);
    }

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

        const double phase2 = SETTLE_TIME + EXCITE_TIME;
        const double phase3 = phase2 + RESTORE_TIME;

        double roll = 0.0, pitch = 0.0;

        if (t_ < SETTLE_TIME) {
            roll = pitch = 0.0;

        } else if (t_ < phase2) {
            const double excite_t = t_ - SETTLE_TIME;
            const std::vector<double> q = exc_.eval(excite_t);
            roll = q[0]; pitch = q[1];

        } else if (t_ < phase3) {
            const std::vector<double> q_end = exc_.eval(EXCITE_TIME);
            const double alpha = (t_ - phase2) / RESTORE_TIME;  // 0 -> 1
            roll  = q_end[0] + alpha * (0.0 - q_end[0]);
            pitch = q_end[1] + alpha * (0.0 - q_end[1]);

        } else {
            RCLCPP_INFO_ONCE(get_logger(), "Done.");
            return;
        }

        roll  = clamp(roll,  deg2rad(AMP_LIMIT_DEG));
        pitch = clamp(pitch, deg2rad(AMP_LIMIT_DEG));

        unitree_api::msg::Request req_e, req_m;
        sport_req_.Euler(req_e, static_cast<float>(roll), static_cast<float>(pitch), 0.0f);
        sport_req_.Move(req_m, 0.0f, 0.0f, 0.0f);
        req_pub_->publish(req_e);
        req_pub_->publish(req_m);

        geometry_msgs::msg::PointStamped des_msg;
        des_msg.header.stamp = now();
        des_msg.point.x = roll;
        des_msg.point.y = pitch;
        des_msg.point.z = 0.0;
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
    rclcpp::spin(std::make_shared<ExcitationPitchRoll>());
    rclcpp::shutdown();
    return 0;
}
