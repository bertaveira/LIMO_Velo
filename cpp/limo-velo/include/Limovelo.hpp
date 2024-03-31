#pragma once

#include "Common.hpp"
#include "Objects.hpp"
#include "Accumulator.hpp"
#include "Localizator.hpp"
#include "Compensator.hpp"
#include "Mapper.hpp"

class LimoVelo
{
public:
    LimoVelo(std::function<void(std::string)> logger);

    void reset();
    void setup_logger(std::function<void(std::string)> logger) {this->logger_ = logger;}
    bool initialized();

    void add_imu(const IMU & imu);
    void add_points(const Points & points);

    State run_localize();
    Points get_compsensated_points();
    Points get_downsampled_compensated_points();
    Points get_global_compensated_points();
    Points get_global_downsampled_compensated_points();

    void run_mapping(double t);

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
