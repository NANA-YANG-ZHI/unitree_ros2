// Excite body roll and pitch (via Euler()) with a random Fourier-series
// trajectory while standing in place.
//
// The Fourier coefficients are randomized per run (see SEED below); this node
// writes them to excitation_pitch_roll_params.json on startup so
// plot/excitation_pitch_roll.py can reconstruct and plot the exact commanded
// trajectory offline.

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
static constexpr double SETTLE_TIME  = 1.0;    // s: hold level before exciting
static constexpr double EXCITE_TIME  = 10.0;   // s: excitation duration
static constexpr double RESTORE_TIME = 1.0;    // s: ramp back to level

static inline float deg2rad(double d) { return static_cast<float>(d * M_PI / 180.0); }

// Excitation tuning (see excitation.hpp for what each parameter controls).
// Defaults below; override via ROS 2 params (order, param_range_deg, excite_time,
// seed) instead of rebuilding, e.g.:
//   ros2 run unitree_ros2_example excitation_pitch_roll --ros-args -p param_range_deg:=[20,20] -p seed:=7
// njoints = 2: [roll, pitch] (rad).
static constexpr int    ORDER          = 3;
static constexpr int    NJOINTS        = 2;
// Raw coefficient scale (excitation.cpp attenuates this by ~1/(2*omega_f*k) in
// eval(), so the *actual* swing comes out well below PARAM_RANGE_DEG -- e.g.
// with order=3 and duration=10s, actual swing is roughly ~25% of this value.
// Tune from the plot: bigger PARAM_RANGE_DEG or bigger EXCITE_TIME both make
// the actual swing bigger.
static constexpr double PARAM_RANGE_DEG = 30.0;  // deg
// Hard safety clamp applied to the final commanded value, independent of
// PARAM_RANGE_DEG above. Unitree Euler() spec is roll/pitch +-0.75 rad
// (~43 deg); this stays well under that while leaving headroom above the
// expected actual swing so the sinusoid isn't clipped into a flat top.
static constexpr double CLAMP_LIMIT_DEG = 20.0;  // deg
static constexpr unsigned SEED = 42;  // change for a different random "tryout"

static const char *PARAMS_FILE = "excitation_pitch_roll_params.json";

static std::vector<double> deg_vec_to_rad(const std::vector<double> &deg)
{
    std::vector<double> rad(deg.size());
    for (std::size_t i = 0; i < deg.size(); ++i) rad[i] = deg[i] * M_PI / 180.0;
    return rad;
}

class ExcitationPitchRoll : public rclcpp::Node
{
public:
    ExcitationPitchRoll()
        : Node("excitation_pitch_roll"), t_(-1.0),
          order_(declare_and_get_int(this, "order", ORDER)),
          param_range_deg_(declare_and_get_double_array(this, "param_range_deg",
                                                         {PARAM_RANGE_DEG, PARAM_RANGE_DEG})),
          excite_time_(declare_and_get_double(this, "excite_time", EXCITE_TIME)),
          seed_(declare_and_get_uint(this, "seed", SEED)),
          exc_(order_, NJOINTS, deg_vec_to_rad(param_range_deg_))
    {
        if (static_cast<int>(param_range_deg_.size()) != NJOINTS) {
            throw std::invalid_argument("param_range_deg must have exactly " + std::to_string(NJOINTS) + " element(s)");
        }
        exc_.generate_random_param(seed_);
        exc_.set_duration(excite_time_);
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
        j["excite_time"] = excite_time_;
        j["restore_time"] = RESTORE_TIME;
        j["param_range_deg"] = param_range_deg_;
        j["clamp_limit_deg"] = CLAMP_LIMIT_DEG;
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

        const double phase2 = SETTLE_TIME + excite_time_;
        const double phase3 = phase2 + RESTORE_TIME;

        double roll = 0.0, pitch = 0.0;

        if (t_ < SETTLE_TIME) {
            roll = pitch = 0.0;

        } else if (t_ < phase2) {
            const double excite_t = t_ - SETTLE_TIME;
            const std::vector<double> q = exc_.eval(excite_t);
            roll = q[0]; pitch = q[1];

        } else if (t_ < phase3) {
            const std::vector<double> q_end = exc_.eval(excite_time_);
            const double alpha = (t_ - phase2) / RESTORE_TIME;  // 0 -> 1
            roll  = q_end[0] + alpha * (0.0 - q_end[0]);
            pitch = q_end[1] + alpha * (0.0 - q_end[1]);

        } else {
            RCLCPP_INFO_ONCE(get_logger(), "Done.");
            return;
        }

        roll  = clamp(roll,  deg2rad(CLAMP_LIMIT_DEG));
        pitch = clamp(pitch, deg2rad(CLAMP_LIMIT_DEG));

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

    int order_;
    std::vector<double> param_range_deg_;
    double excite_time_;
    unsigned seed_;
    FourierExcitation exc_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ExcitationPitchRoll>());
    rclcpp::shutdown();
    return 0;
}
