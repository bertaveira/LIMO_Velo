#pragma once

#include <vector>
#include <string>


namespace limovelo
{

namespace LIDAR_TYPE
{
const std::string Velodyne = "velodyne";
const std::string Hesai = "hesai";
const std::string Ouster = "ouster";
const std::string Custom = "custom";
}

struct InitializationParams
{
    std::vector<double> times;
    std::vector<double> deltas;
};

struct Params
{
    bool mapping_online;
    bool real_time;

    bool estimate_extrinsics;
    bool print_extrinsics;
    std::vector<double> initial_gravity;
    std::vector<double> I_Rotation_L;
    std::vector<double> I_Translation_L;

    double empty_lidar_time;
    double real_time_delay;

    double full_rotation_time;
    double imu_rate;

    int downsample_rate;
    float downsample_prec;
    bool high_quality_publish;

    double min_dist;
    std::string LiDAR_type;

    bool offset_beginning;
    bool stamp_beginning;

    double degeneracy_threshold;
    bool print_degeneracy_values;

    int MAX_NUM_ITERS;
    int MAX_POINTS2MATCH;
    std::vector<double> LIMITS;
    int NUM_MATCH_POINTS;
    double MAX_DIST_PLANE;
    float PLANES_THRESHOLD;
    float PLANES_CHOOSE_CONSTANT;

    double wx_MULTIPLIER;
    double wy_MULTIPLIER;
    double wz_MULTIPLIER;

    double cov_acc;
    double cov_gyro;
    double cov_bias_acc;
    double cov_bias_gyro;
    double LiDAR_noise;

    std::string points_topic;
    std::string imus_topic;

    InitializationParams Initialization;
};

extern Params Config;

} // namespace limovelo
