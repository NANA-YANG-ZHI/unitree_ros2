
#include <unistd.h>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/sport_mode_state.hpp"

#include "unitree_api/msg/request.hpp"
#include "common/ros2_sport_client.h"

using std::placeholders::_1;

class RotateInPlace : public rclcpp::Node
{
public:
    RotateInPlace() : Node("rotate_in_place")
    {
        state_suber = this->create_subscription<unitree_go::msg::SportModeState>(
            "sportmodestate", 10, std::bind(&RotateInPlace::state_callback, this, _1));
        req_puber = this->create_publisher<unitree_api::msg::Request>("/api/sport/request", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(int(dt * 1000)),
            std::bind(&RotateInPlace::timer_callback, this));

        t = -1;
    };

private:
    void timer_callback()
    {
        t += dt;
        if (t > 0)
        {
            double time_seg = 0.2;
            double time_temp = t - time_seg;

            std::vector<PathPoint> path;

            for (int i = 0; i < 30; i++)
            {
                PathPoint path_point_tmp;
                time_temp += time_seg;

                // Rotate in place: no translation, constant yaw rate
                float vyaw_local = 0.2f; // rad/s, positive = counter-clockwise
                float yaw_local = vyaw_local * time_temp;
                float vx_local = 0;
                float vy_local = 0;

                path_point_tmp.timeFromStart = i * time_seg;
                path_point_tmp.x = px0;
                path_point_tmp.y = py0;
                path_point_tmp.yaw = yaw_local + yaw0;
                path_point_tmp.vx = vx_local;
                path_point_tmp.vy = vy_local;
                path_point_tmp.vyaw = vyaw_local;
                path.push_back(path_point_tmp);
            }

            sport_req.TrajectoryFollow(req, path);
            req_puber->publish(req);
        }
    };

    void state_callback(unitree_go::msg::SportModeState::SharedPtr data)
    {
        if (t < 0)
        {
            px0 = data->position[0];
            py0 = data->position[1];
            yaw0 = data->imu_state.rpy[2];
            std::cout << "Initial pose: " << px0 << ", " << py0 << ", " << yaw0 << std::endl;
        }
    }

    rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr state_suber;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr req_puber;

    unitree_api::msg::Request req;
    SportClient sport_req;

    double t;
    double dt = 0.002;

    double px0 = 0;
    double py0 = 0;
    double yaw0 = 0;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RotateInPlace>());
    rclcpp::shutdown();
    return 0;
}
