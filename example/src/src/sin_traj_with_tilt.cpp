// Sin trajectory + constant body tilt (roll / pitch via Euler()).
// Follows the same TrajectoryFollow pattern as sport_mode_ctrl.cpp,
// and sends Euler() each control tick alongside the trajectory.

#include <unistd.h>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/sport_mode_state.hpp"
#include "unitree_api/msg/request.hpp"
#include "common/ros2_sport_client.h"

using std::placeholders::_1;

static constexpr float ROLL_DEG  = 0.0f;  // roll  in degrees -- set what you want
static constexpr float PITCH_DEG = 5.0f;  // pitch in degrees -- positive tilts nose up

static inline float deg2rad(float d) { return d * static_cast<float>(M_PI) / 180.0f; }

class SinTrajWithTilt : public rclcpp::Node
{
public:
    SinTrajWithTilt() : Node("sin_traj_with_tilt")
    {
        state_suber = create_subscription<unitree_go::msg::SportModeState>(
            "sportmodestate", 10,
            std::bind(&SinTrajWithTilt::state_callback, this, _1));

        req_puber = create_publisher<unitree_api::msg::Request>("/api/sport/request", 10);

        timer_ = create_wall_timer(
            std::chrono::milliseconds(int(dt * 1000)),
            std::bind(&SinTrajWithTilt::timer_callback, this));

        t = -1;
    }

private:
    void timer_callback()
    {
        t += dt;
        if (t > 0)
        {
            // --- body tilt: send every tick so the controller holds it ---
            unitree_api::msg::Request euler_req;
            sport_req.Euler(euler_req, deg2rad(ROLL_DEG), deg2rad(PITCH_DEG), 0.0f);
            req_puber->publish(euler_req);

            // --- sin trajectory (identical to sport_mode_ctrl.cpp) ---
            double time_seg  = 0.2;
            double time_temp = t - time_seg;

            std::vector<PathPoint> path;
            for (int i = 0; i < 30; i++)
            {
                PathPoint pt;
                time_temp += time_seg;

                float px_local   =  0.5f * sin(0.5 * time_temp);
                float py_local   =  0.0f;
                float yaw_local  =  0.0f;
                float vx_local   =  0.5f * cos(0.5 * time_temp);
                float vy_local   =  0.0f;
                float vyaw_local =  0.0f;

                pt.timeFromStart = i * time_seg;
                pt.x    = px_local * cos(yaw0) - py_local * sin(yaw0) + px0;
                pt.y    = px_local * sin(yaw0) + py_local * cos(yaw0) + py0;
                pt.yaw  = yaw_local + yaw0;
                pt.vx   = vx_local * cos(yaw0) - vy_local * sin(yaw0);
                pt.vy   = vx_local * sin(yaw0) + vy_local * cos(yaw0);
                pt.vyaw = vyaw_local;
                path.push_back(pt);
            }

            unitree_api::msg::Request traj_req;
            sport_req.TrajectoryFollow(traj_req, path);
            req_puber->publish(traj_req);
        }
    }

    void state_callback(unitree_go::msg::SportModeState::SharedPtr data)
    {
        if (t < 0)
        {
            px0  = data->position[0];
            py0  = data->position[1];
            yaw0 = data->imu_state.rpy[2];
            RCLCPP_INFO(get_logger(), "Init pos: x=%.2f y=%.2f yaw=%.2f  rpy=[%.2f %.2f %.2f]",
                        px0, py0, yaw0,
                        data->imu_state.rpy[0], data->imu_state.rpy[1], data->imu_state.rpy[2]);
        }
    }

    rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr state_suber;
    rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr req_puber;
    rclcpp::TimerBase::SharedPtr timer_;

    SportClient sport_req;

    double t  = -1;
    double dt = 0.002;

    double px0  = 0;
    double py0  = 0;
    double yaw0 = 0;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SinTrajWithTilt>());
    rclcpp::shutdown();
    return 0;
}
