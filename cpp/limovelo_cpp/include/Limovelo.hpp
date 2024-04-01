#pragma once

#include "Common.hpp"

namespace limovelo
{

void set_params(Params & params);

class LimoVelo
{
public:
    LimoVelo(std::function<void(std::string)> logger);

    void reset();
    void setup_logger(std::function<void(std::string)> logger) {this->logger_ = logger;}
    bool initialized();

    void add_imu(const IMU & imu);
    void add_points(const Points & points);

    State run_localization();
    Points get_compsensated_points();
    Points get_downsampled_compensated_points();
    Points get_global_compensated_points();
    Points get_global_downsampled_compensated_points();

    void run_mapping();

private:
    bool initialized_ = false;
    std::function<void(std::string)> logger_;

    double t1_, t2_, delta_;
    Points compensated_, ds_compensated_, global_compensated_, global_ds_compensated_;

    std::shared_ptr<Accumulator> accumulator_;
    std::shared_ptr<Localizator> localizer_;
    std::shared_ptr<Compensator> compensator_;
    std::shared_ptr<Mapper> mapper_;

};

} // namespace limovelo
