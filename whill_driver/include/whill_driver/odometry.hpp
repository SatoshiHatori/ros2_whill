#pragma once

#include <cmath>
#include <math.h>
#include <limits>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace whill_driver
{

    class Odometry
    {
    private:
        typedef struct
        {
            long double x;
            long double y;
            long double yaw;
            long double linear_x;
            long double angular_z;
        } VehiclePose2D;

        double wheel_radius_;
        double wheel_tread_;
        double base_link_height_;

        VehiclePose2D vehicle_;
        long double confineRadian(long double rad);

    public:
        explicit Odometry();
        explicit Odometry(double wheel_radius, double wheel_tread);
        void update(double right_wheel_vel_rps, double left_wheel_vel_rps, double dt);
        void set(double x, double y, double yaw, double linear_x, double angular_z);
        void reset();
        nav_msgs::msg::Odometry get();
    };

} // namespace whill_driver
