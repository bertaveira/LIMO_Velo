#include <pcl_conversions/pcl_conversions.h>

#include "PointCloudProcessor.hpp"

#include "Common.hpp"
#include "Objects.hpp"
#include "Utils.hpp"

Points PointCloudProcessor::msg2points(const PointCloud_msg & msg)
{
    if (config_.LiDAR_type == limovelo::LIDAR_TYPE::Velodyne) {
        return this->velodynemsg2points(msg);
    }
    if (config_.LiDAR_type == limovelo::LIDAR_TYPE::Hesai) {
        return this->hesaimsg2points(msg);
    }
    if (config_.LiDAR_type == limovelo::LIDAR_TYPE::Ouster) {
        return this->oustermsg2points(msg);
    }
    if (config_.LiDAR_type == limovelo::LIDAR_TYPE::Custom) {
        return this->custommsg2points(msg);
    }

    // Unknown LiDAR type
    RCLCPP_ERROR(
        rclcpp::get_logger(
            "limovelo"), "Unknown LiDAR type! Change your YAML parameters file.");
    return Points();
}

Points PointCloudProcessor::downsample(const Points & points)
{
    // Downsample points close depending on their time distance
    return this->temporal_downsample(points);
}

Points PointCloudProcessor::sort_points(const Points & points)
{
    Points sorted_points = points;
    // Use std::sort included in <algorithms>
    std::sort(sorted_points.begin(), sorted_points.end(), this->time_sort);
    return sorted_points;
}

// Velodyne specific
Points PointCloudProcessor::velodynemsg2points(const PointCloud_msg & msg)
{
    pcl::PointCloud<velodyne_ros::Point>::Ptr raw_pcl(new pcl::PointCloud<velodyne_ros::Point>());
    pcl::fromROSMsg(*msg, *raw_pcl);
    return this->to_points(*raw_pcl);
}

double PointCloudProcessor::get_begin_time(const pcl::PointCloud<velodyne_ros::Point> & pcl)
{
    // Velodyne points have relative time
    if (config_.stamp_beginning) {
        return Conversions::microsec2Sec(pcl.header.stamp) + pcl.points.front().time;
    } else {
        return Conversions::microsec2Sec(pcl.header.stamp) + pcl.points.front().time -
               pcl.points.back().time;
    }
}

// HESAI specific
Points PointCloudProcessor::hesaimsg2points(const PointCloud_msg & msg)
{
    pcl::PointCloud<hesai_ros::Point>::Ptr raw_pcl(new pcl::PointCloud<hesai_ros::Point>());
    pcl::fromROSMsg(*msg, *raw_pcl);
    return this->to_points(*raw_pcl);
}

double PointCloudProcessor::get_begin_time(const pcl::PointCloud<hesai_ros::Point> & pcl)
{
    // HESAI points have absolute time
    return 0.d;
}

// Ouster specific
Points PointCloudProcessor::oustermsg2points(const PointCloud_msg & msg)
{
    pcl::PointCloud<ouster_ros::Point>::Ptr raw_pcl(new pcl::PointCloud<ouster_ros::Point>());
    pcl::fromROSMsg(*msg, *raw_pcl);
    return this->to_points(*raw_pcl);
}

double PointCloudProcessor::get_begin_time(const pcl::PointCloud<ouster_ros::Point> & pcl)
{
    // Ouster points have relative time
    if (config_.stamp_beginning) {
        return Conversions::microsec2Sec(pcl.header.stamp) + Conversions::nanosec2Sec(
            pcl.points.front().t);
    } else {
        return Conversions::microsec2Sec(pcl.header.stamp) + Conversions::nanosec2Sec(
            pcl.points.front().t) - Conversions::nanosec2Sec(pcl.points.back().t);
    }
}

// Custom specific
Points PointCloudProcessor::custommsg2points(const PointCloud_msg & msg)
{
    pcl::PointCloud<custom::Point>::Ptr raw_pcl(new pcl::PointCloud<custom::Point>());
    pcl::fromROSMsg(*msg, *raw_pcl);
    return this->to_points(*raw_pcl);
}

// Change this to fit the timestamp of the first point
double PointCloudProcessor::get_begin_time(const pcl::PointCloud<custom::Point> & pcl)
{
    // // Example: Points with relative time
    // if (config_.stamp_beginning) return Conversions::microsec2Sec(pcl.header.stamp) + pcl.points.front().time;
    // else return Conversions::microsec2Sec(pcl.header.stamp) + pcl.points.front().time - pcl.points.back().time;

    // Example: Points with absolute time
    return 0.d;
}

template<typename PointType>
Points PointCloudProcessor::to_points(const typename pcl::PointCloud<PointType> & pcl)
{
    Points pts;

    // Special case: Velodyne points have relative time, we need to get the time stamp from the earliest point
    double begin_time = this->get_begin_time(pcl);

    for (PointType p : pcl.points) {
        pts.push_back(Point(p, begin_time));
    }
    return pts;
}

Points PointCloudProcessor::temporal_downsample(const Points & points)
{
    Points downsampled;
    int ds_counter = 0;

    for (Point p : points) {
        // Keep point if counter is multiple of downsample_rate
        bool keep_point = config_.downsample_rate <=
          1 or++ ds_counter % config_.downsample_rate == 0;
        if (keep_point and config_.min_dist < p.norm()) {downsampled.push_back(p);}
    }

    return downsampled;
}

bool PointCloudProcessor::time_sort(const Point & a, const Point & b)
{
    return a.time < b.time;
}
