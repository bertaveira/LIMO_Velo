#include "limovelo_node.hpp"

#include "PointCloudProcessor.hpp"

using namespace std::chrono_literals;

LimoVeloNode::LimoVeloNode()
  : Node("limovelo")
{
    RCLCPP_INFO(this->get_logger(), "LimoVeloNode created");

    // Initialize parameters
    setup_params();

    // Setup params
    limovelo::set_params(params_);

    // Initialize limovelo
    limoVelo_ =
      new limovelo::LimoVelo(std::bind(&LimoVeloNode::logger, this, std::placeholders::_1));

    // Can't get node pointer in constructor, so we need to use a timer to initialize the publishers
    timer_ = this->create_wall_timer(100ms, std::bind(&LimoVeloNode::init_timer_callback, this));
}

void LimoVeloNode::init_timer_callback()
{
    publishers_ = new Publishers(this->shared_from_this());

    // Initialize subscribers
    pointCloudSub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        params_.points_topic, 10,
        std::bind(&LimoVeloNode::pointcloud_callback, this, std::placeholders::_1)
    );

    imuSub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        params_.imus_topic, 10, std::bind(&LimoVeloNode::imu_callback, this, std::placeholders::_1)
    );

    // cancel the timer
    timer_->cancel();
}

void LimoVeloNode::logger(std::string msg)
{
    RCLCPP_INFO(this->get_logger(), msg.c_str());
}

void LimoVeloNode::setup_params()
{
    // Read YAML parameters
    this->declare_parameter("mapping_online", true);
    this->get_parameter("mapping_online", params_.mapping_online);

    this->declare_parameter("real_time", false);
    this->get_parameter("real_time", params_.real_time);

    this->declare_parameter("estimate_extrinsics", false);
    this->get_parameter("estimate_extrinsics", params_.estimate_extrinsics);

    this->declare_parameter("print_extrinsics", false);
    this->get_parameter("print_extrinsics", params_.print_extrinsics);

    this->declare_parameter("downsample_rate", 4);
    this->get_parameter("downsample_rate", params_.downsample_rate);

    this->declare_parameter("downsample_prec", 0.2);
    this->get_parameter("downsample_prec", params_.downsample_prec);

    this->declare_parameter("high_quality_publish", true);
    this->get_parameter("high_quality_publish", params_.high_quality_publish);

    this->declare_parameter("MAX_NUM_ITERS", 3);
    this->get_parameter("MAX_NUM_ITERS", params_.MAX_NUM_ITERS);

    this->declare_parameter("LIMITS", std::vector<double>{23, 0.001});
    this->get_parameter("LIMITS", params_.LIMITS);

    this->declare_parameter("NUM_MATCH_POINTS", 5);
    this->get_parameter("NUM_MATCH_POINTS", params_.NUM_MATCH_POINTS);

    this->declare_parameter("MAX_POINTS2MATCH", 10);
    this->get_parameter("MAX_POINTS2MATCH", params_.MAX_POINTS2MATCH);

    this->declare_parameter("MAX_DIST_PLANE", 2.0);
    this->get_parameter("MAX_DIST_PLANE", params_.MAX_DIST_PLANE);

    this->declare_parameter("PLANES_THRESHOLD", 0.1f);
    this->get_parameter("PLANES_THRESHOLD", params_.PLANES_THRESHOLD);

    this->declare_parameter("PLANES_CHOOSE_CONSTANT", 9.0f);
    this->get_parameter("PLANES_CHOOSE_CONSTANT", params_.PLANES_CHOOSE_CONSTANT);

    this->declare_parameter("LiDAR_type", "unknown");
    this->get_parameter("LiDAR_type", params_.LiDAR_type);

    this->declare_parameter("LiDAR_noise", 0.001);
    this->get_parameter("LiDAR_noise", params_.LiDAR_noise);

    this->declare_parameter("min_dist", 3.);
    this->get_parameter("min_dist", params_.min_dist);

    this->declare_parameter("imu_rate", 400.0);
    this->get_parameter("imu_rate", params_.imu_rate);

    this->declare_parameter("degeneracy_threshold", 5.d);
    this->get_parameter("degeneracy_threshold", params_.degeneracy_threshold);

    this->declare_parameter("print_degeneracy_values", false);
    this->get_parameter("print_degeneracy_values", params_.print_degeneracy_values);

    this->declare_parameter("full_rotation_time", 0.1);
    this->get_parameter("full_rotation_time", params_.full_rotation_time);

    this->declare_parameter("empty_lidar_time", 20.);
    this->get_parameter("empty_lidar_time", params_.empty_lidar_time);

    this->declare_parameter("real_time_delay", 1.);
    this->get_parameter("real_time_delay", params_.real_time_delay);

    this->declare_parameter("covariance_gyroscope", 1.0e-4);
    this->get_parameter("covariance_gyroscope", params_.cov_gyro);

    this->declare_parameter("covariance_acceleration", 1.0e-2);
    this->get_parameter("covariance_acceleration", params_.cov_acc);

    this->declare_parameter("covariance_bias_gyroscope", 1.0e-5);
    this->get_parameter("covariance_bias_gyroscope", params_.cov_bias_gyro);

    this->declare_parameter("covariance_bias_acceleration", 1.0e-4);
    this->get_parameter("covariance_bias_acceleration", params_.cov_bias_acc);

    this->declare_parameter("wx_MULTIPLIER", 1.);
    this->get_parameter("wx_MULTIPLIER", params_.wx_MULTIPLIER);

    this->declare_parameter("wy_MULTIPLIER", 1.);
    this->get_parameter("wy_MULTIPLIER", params_.wy_MULTIPLIER);

    this->declare_parameter("wz_MULTIPLIER", 1.);
    this->get_parameter("wz_MULTIPLIER", params_.wz_MULTIPLIER);

    this->declare_parameter("points_topic", "/velodyne_points");
    this->get_parameter("points_topic", params_.points_topic);

    this->declare_parameter("imus_topic", "/vectornav/IMU");
    this->get_parameter("imus_topic", params_.imus_topic);

    this->declare_parameter("offset_beginning", false);
    this->get_parameter("offset_beginning", params_.offset_beginning);

    this->declare_parameter("stamp_beginning", false);
    this->get_parameter("stamp_beginning", params_.stamp_beginning);

    this->declare_parameter("Initialization.times", std::vector<double>{});
    this->get_parameter("Initialization.times", params_.Initialization.times);

    this->declare_parameter(
        "Initialization.deltas",
        std::vector<double>{params_.full_rotation_time});
    this->get_parameter("Initialization.deltas", params_.Initialization.deltas);

    this->declare_parameter("initial_gravity", std::vector<double>{0.0, 0.0, -9.807});
    this->get_parameter("initial_gravity", params_.initial_gravity);

    this->declare_parameter("I_Translation_L", std::vector<double>{3, 0.});
    this->get_parameter("I_Translation_L", params_.I_Translation_L);

    this->declare_parameter("I_Rotation_L", std::vector<double>{9, 0.});
    this->get_parameter("I_Rotation_L", params_.I_Rotation_L);
}

static void timeIt(std::string text, bool print = true)
{
    static std::chrono::time_point<std::chrono::system_clock> start =
      std::chrono::system_clock::now();

    if (!print) {
        start = std::chrono::system_clock::now();
        return;
    }

    auto end = std::chrono::system_clock::now();
    std::chrono::duration<double, std::milli> duration = end - start;
    std::cout << text << " " << duration.count() << "ms" << std::endl;

    // start = std::chrono::system_clock::now();
}

void LimoVeloNode::pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    std::chrono::time_point<std::chrono::system_clock> start = std::chrono::system_clock::now();

    timeIt("", false);
    // Create a temporal object to process the pointcloud message
    PointCloudProcessor processor(params_);
    Points points = processor.msg2points(msg);
    timeIt("msg2points");

    Points downsampled_points = processor.downsample(points);
    timeIt("downsample");
    Points sorted_points = processor.sort_points(downsampled_points);
    timeIt("sort_points");

    limoVelo_->add_points(sorted_points);
    timeIt("add_points");

    // The accumulator received enough data to start
    if (limoVelo_->initialized()) {
        timeIt("initialized");
        run_slam();
    }

    // log execution time
    auto end = std::chrono::system_clock::now();
    std::chrono::duration<double, std::milli> duration = end - start;
    logger("Total execution time: " + std::to_string(duration.count()) + "ms");
}

void LimoVeloNode::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    IMU imu;
    // Linear accelerations
    imu.a(0) = msg->linear_acceleration.x;
    imu.a(1) = msg->linear_acceleration.y;
    imu.a(2) = msg->linear_acceleration.z;

    // Gyroscope
    imu.w(0) = msg->angular_velocity.x;
    imu.w(1) = msg->angular_velocity.y;
    imu.w(2) = msg->angular_velocity.z;

    // Orientation
    imu.q.x() = msg->orientation.x;
    imu.q.y() = msg->orientation.y;
    imu.q.z() = msg->orientation.z;
    imu.q.w() = msg->orientation.w;

    // Time
    imu.time = Conversions::nanosec2Sec(msg->header.stamp.nanosec) + msg->header.stamp.sec;

    limoVelo_->add_imu(imu);
}


void LimoVeloNode::run_slam()
{
    State state = limoVelo_->run_localization();
    timeIt("run_localization");
    if (state.time < 0) {
        return;
    }

    // Publish state
    publishers_->state(state, false);
    publishers_->tf(state);

    timeIt("state publishers");

    // Publish points
    Points global_ds_compensated = limoVelo_->get_global_downsampled_compensated_points();
    publishers_->pointcloud(global_ds_compensated, true);

    if (params_.print_extrinsics) {publishers_->extrinsics(state);}

    timeIt("pointcloud publishers");

    limoVelo_->run_mapping();

    timeIt("run_mapping");

    // Publish map
    Points global_compensated = limoVelo_->get_global_compensated_points();
    if (params_.high_quality_publish) {publishers_->pointcloud(global_compensated, false);} else {
        publishers_->pointcloud(global_ds_compensated, false);
    }

    timeIt("map publishers");
}


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LimoVeloNode>());
    rclcpp::shutdown();
    return 0;
}
