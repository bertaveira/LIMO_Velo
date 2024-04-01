#include "Limovelo.hpp"

#include "Objects.hpp"
#include "Accumulator.hpp"
#include "Compensator.hpp"
#include "Localizator.hpp"
#include "Mapper.hpp"

namespace limovelo
{

void set_params(Params & params)
{
    Config = params;
}

LimoVelo::LimoVelo(std::function<void(std::string)> logger)
  : logger_(logger)
{
    accumulator_ = AccumulatorInstance;
    localizer_ = LocalizatorInstance;
    compensator_ = std::make_shared<Compensator>();
    mapper_ = MapperInstance;
}

void LimoVelo::reset()
{
    accumulator_->reset();

    // Delete and realocate objects
    AccumulatorInstance = std::make_shared<Accumulator>(logger_);
    accumulator_ = AccumulatorInstance;
    LocalizatorInstance = std::make_shared<Localizator>();
    localizer_ = LocalizatorInstance;
    MapperInstance = std::make_shared<Mapper>();
    mapper_ = MapperInstance;
    compensator_ = std::make_shared<Compensator>();

    initialized_ = false;
}

void LimoVelo::add_imu(const IMU & imu)
{
    accumulator_->push(imu);
}

void LimoVelo::add_points(const Points & points)
{
    accumulator_->push(points);
}

bool LimoVelo::initialized()
{
    if (initialized_) {
        return true;
    }

    if (accumulator_->ready()) {
        double t = accumulator_->set_initial_time();
        if (t < 0) {return false;}

        IMUs imus = accumulator_->get_imus(-1, t);
        localizer_->initialize(t, imus);

        t2_ = accumulator_->latest_points_time();

        delta_ = Config.Initialization.deltas.front();

        initialized_ = true;
        return true;
    }

    return false;
}

State LimoVelo::run_localization()
{
    if (!initialized()) {
        logger_(
            "LimoVelo not yet initialized. Use initialized() method to check and keep feeding data.");
        return State();
    }


    if (Config.real_time) {
        // Take latest time window of data potentilly dropping old things
        t2_ = accumulator_->latest_time();
        t1_ = std::max(t2_ - delta_, localizer_->last_time_updated);
    } else {
        // Take the next available data window
        t1_ = t2_;
        t2_ = std::min(t2_ + delta_, accumulator_->latest_time());
    }

    // Update delta value
    delta_ = accumulator_->update_delta(Config.Initialization, t2_);

    // Check if interval has enough field of view
    if (t2_ - t1_ < 1e-3) {return State();}

    /*********** GET THE DATA ***********/

    // Get points
    Points points = accumulator_->get_points(t1_, t2_);

    // Get states just before t1 to t2
    States states = accumulator_->get_states(t1_, t2_); // TODO: [Bernardo] isn't it always empty between t1 and t2???
    {
        State latestState = localizer_->latest_state();
        states.push_front(accumulator_->get_prev_state(t1_, latestState));
    }

    // Get IMUs after last state
    IMUs imus = accumulator_->get_imus(states.front().time, t2_);
    imus.push_back(accumulator_->get_next_imu(t2_));

    compensated_ = compensator_->compensate(states, imus, t2_, points);
    ds_compensated_ = compensator_->downsample(compensated_);
    if (ds_compensated_.size() < Config.MAX_POINTS2MATCH) {
        logger_("Not enough points to localize.");
        return State();
    }

    /*********** LOCALIZATION ***********/

    // Integrate IMUs up to t2
    imus = accumulator_->get_imus(localizer_->last_time_integrated, t2_);
    localizer_->propagate_to(imus, t2_);

    // Localize points in map
    localizer_->correct(ds_compensated_, t2_);
    State Xt2 = localizer_->latest_state();
    Xt2.time = t2_;
    accumulator_->push(Xt2);

    // Publish pointcloud used to localize
    global_compensated_ = Xt2 * Xt2.I_Rt_L() * compensated_;
    global_ds_compensated_ = Xt2 * Xt2.I_Rt_L() * ds_compensated_;

    return Xt2;
}

Points LimoVelo::get_compsensated_points()
{
    return compensated_;
}

Points LimoVelo::get_downsampled_compensated_points()
{
    return ds_compensated_;
}

Points LimoVelo::get_global_compensated_points()
{
    return global_compensated_;
}

Points LimoVelo::get_global_downsampled_compensated_points()
{
    return global_ds_compensated_;
}

void LimoVelo::run_mapping()
{
    if (!initialized()) {
        logger_(
            "LimoVelo not yet initialized. Use initialized() method to check and keep feeding data.");
        return;
    }

    // Add updated points to map (mapping online)
    mapper_->add(global_ds_compensated_, t2_, true);

    /* Step 3. ERASE OLD DATA */

    // Empty too old LiDAR points
    accumulator_->clear_lidar(t2_ - Config.empty_lidar_time);
    accumulator_->clear_buffers(t2_ - Config.empty_lidar_time);
}

} // namespace limovelo
