// Copyright (c) 2024 WHILL, Inc.
// Released under the MIT license
// https://opensource.org/licenses/mit-license.php

/**
 * @file    whill_node.hpp
 * @brief   Definitions of the unique node that works as WHILL
 */
#ifndef WHILL_DRIVER_WHILL_NODE_H_
#define WHILL_DRIVER_WHILL_NODE_H_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "whill_msgs/msg/model_cr2_state.hpp"
#include "whill_msgs/srv/set_battery_voltage_out.hpp"
#include "whill_msgs/srv/set_power.hpp"
#include "whill_msgs/srv/set_speed_profile.hpp"

#include "model_cr2/whill.hpp"
#include "odometry.hpp"
namespace whill_driver
{
    // The default serial port name
    const std::string kDefaultPortName = "/dev/ttyUSB0";

    // The default odometry frame name
    const std::string kDefaultOdometryFrame = "odom";

    // The default publish interval [ms]
    constexpr uint16_t kDefaultStatePublishIntervalMs = 100;

    // The default state send interval [ms]
    constexpr uint16_t kDefaultSendIntervalMs = 10;

    // WHILL tread [m]
    constexpr double kDefaultWhillTread = 0.496;

    // WHILL wheel radius [m]
    constexpr double kDefaultWhillWheelRadius = 0.1325;
    

    class WhillNode : public rclcpp::Node
    {
    public:
        RCLCPP_PUBLIC
        WhillNode();

        explicit WhillNode(const rclcpp::NodeOptions &options);

        RCLCPP_PUBLIC
        ~WhillNode();

    private:
        void Initialize();
        std::shared_ptr<model_cr2::Whill> whill_;
        std::chrono::duration<double, std::milli> publish_duration_;
        std::chrono::duration<double, std::milli> send_interval_;
        std::string odom_frame_;
        Odometry whill_odom_;
        double wheel_radius_, wheel_tread_;

        void
        OnStatesModelCr2Timer();
        rclcpp::TimerBase::SharedPtr states_model_cr2_timer_;
        rclcpp::Publisher<whill_msgs::msg::ModelCr2State>::SharedPtr states_model_cr2_pub_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;

        void OnControllerJoy(const sensor_msgs::msg::Joy::SharedPtr joy);
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr controller_joy_sub_;

        void OnControllerCmdVel(const geometry_msgs::msg::Twist::SharedPtr cmd_vel);
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr controller_cmd_vel_sub_;

        void OnSetPowerSrv(
            const std::shared_ptr<rmw_request_id_t> request_header,
            const std::shared_ptr<whill_msgs::srv::SetPower::Request> request,
            const std::shared_ptr<whill_msgs::srv::SetPower::Response> response);
        rclcpp::Service<whill_msgs::srv::SetPower>::SharedPtr set_power_srv_;

        void OnSetSpeedProfileSrv(
            const std::shared_ptr<rmw_request_id_t> request_header,
            const std::shared_ptr<whill_msgs::srv::SetSpeedProfile::Request> request,
            const std::shared_ptr<whill_msgs::srv::SetSpeedProfile::Response> response);
        rclcpp::Service<whill_msgs::srv::SetSpeedProfile>::SharedPtr set_speed_profile_srv_;

        void OnSetBatteryVoltageOutSrv(
            const std::shared_ptr<rmw_request_id_t> request_header,
            const std::shared_ptr<whill_msgs::srv::SetBatteryVoltageOut::Request> request,
            const std::shared_ptr<whill_msgs::srv::SetBatteryVoltageOut::Response> response);
        rclcpp::Service<whill_msgs::srv::SetBatteryVoltageOut>::SharedPtr set_battery_voltage_out_srv_;

        int ConvertToWhillJoy(float raw_joy);
        bool IsOutside(uint8_t target, uint8_t end1, uint8_t end2);
    };

} // namespace whill_driver

#endif // WHILL_DRIVER_WHILL_NODE_H_
