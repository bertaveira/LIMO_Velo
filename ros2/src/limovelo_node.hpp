#pragma once

#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "Limovelo.hpp"

#include "Publishers.hpp"

class LimoVeloNode : public rclcpp::Node
{
public:
    LimoVeloNode();

private:
    void setup_params();
    void init_timer_callback();
    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);

    void logger(std::string msg);

    void run_slam();

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointCloudSub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imuSub_;
    rclcpp::TimerBase::SharedPtr timer_;

    limovelo::Params params_;
    limovelo::LimoVelo * limoVelo_;
    Publishers * publishers_;
};
