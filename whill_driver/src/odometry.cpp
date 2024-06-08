#include "whill_driver/odometry.hpp"

namespace whill_driver
{

    Odometry::Odometry()
    {
        vehicle_.x = vehicle_.y = vehicle_.yaw = 0.0;
        vehicle_.linear_x = vehicle_.angular_z = 0.0;
        wheel_radius_ = 0.1;
        wheel_tread_ = 0.5;
        base_link_height_ = wheel_radius_;
    }

    Odometry::Odometry(double wheel_radius, double wheel_tread)
    {
        vehicle_.x = vehicle_.y = vehicle_.yaw = 0.0;
        vehicle_.linear_x = vehicle_.angular_z = 0.0;
        wheel_radius_ = wheel_radius;
        wheel_tread_ = wheel_tread;
        base_link_height_ = wheel_radius_;
    }

    long double Odometry::confineRadian(long double rad)
    {
        if (rad >= M_PI)
        {
            rad -= 2.0 * M_PI;
        }
        else if (rad <= -M_PI)
        {
            rad += 2.0 * M_PI;
        }
        return rad;
    }

    void Odometry::update(double right_wheel_vel_mps, double left_wheel_vel_mps, double dt)
    {
        if (dt < 0.0001)
            return;

        long double delta_linear = (right_wheel_vel_mps + left_wheel_vel_mps) / 2.0;
        long double delta_yaw = (right_wheel_vel_mps - left_wheel_vel_mps) / (wheel_tread_);

        vehicle_.x += delta_linear * dt * std::cos(vehicle_.yaw + delta_yaw * dt / 2.0);
        vehicle_.y += delta_linear * dt * std::sin(vehicle_.yaw + delta_yaw * dt / 2.0);

        vehicle_.linear_x = delta_linear;
        vehicle_.angular_z = delta_yaw;
        vehicle_.yaw = confineRadian(vehicle_.yaw + delta_yaw * dt);

        return;
    }

    void Odometry::set(double x, double y, double yaw, double linear_x, double angular_z)
    {
        vehicle_ = {x, y, yaw, linear_x, angular_z};
        return;
    }

    void Odometry::reset()
    {
        set(0.0, 0.0, 0.0, 0.0, 0.0);
    }

    nav_msgs::msg::Odometry Odometry::get()
    {
        nav_msgs::msg::Odometry odom;

        tf2::Quaternion quat_tf;
        quat_tf.setRPY(0.0, 0.0, vehicle_.yaw);

        odom.pose.pose.position.x = vehicle_.x;
        odom.pose.pose.position.y = vehicle_.y;
        odom.pose.pose.position.z = base_link_height_;
        odom.pose.pose.orientation = tf2::toMsg(quat_tf);
        odom.twist.twist.linear.x = vehicle_.linear_x;
        odom.twist.twist.linear.y = 0.0;
        odom.twist.twist.linear.z = 0.0;
        odom.twist.twist.angular.x = 0.0;
        odom.twist.twist.angular.y = 0.0;
        odom.twist.twist.angular.z = vehicle_.angular_z;

        return odom;
    }

} // namespace whill_driver
