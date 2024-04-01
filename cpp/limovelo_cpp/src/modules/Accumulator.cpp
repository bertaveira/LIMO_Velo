#include "Accumulator.hpp"

// Push content
void Accumulator::push(const State & state) {this->BUFFER_X.push(state);}
void Accumulator::push(const IMU & imu) {this->BUFFER_I.push(imu);}
void Accumulator::push(const Point & point) {this->BUFFER_L.push(point);}
void Accumulator::push(const Points & points)
{
    // Check if missing data
    if (this->missing_data(points)) {this->throw_warning(points);}

    for (Point p : points) {
        this->push(p);
    }
}

void Accumulator::reset()
{
    this->BUFFER_X.clear();
    this->BUFFER_I.clear();
    this->BUFFER_L.clear();
    this->is_ready = false;
    this->has_warned_lidar = false;
}

// Empty buffers
void Accumulator::clear_buffers()
{
    this->BUFFER_L.clear();
    this->BUFFER_I.clear();
}

void Accumulator::clear_buffers(TimeType t)
{
    this->BUFFER_L.clear(t);
    this->BUFFER_I.clear(t);
}

void Accumulator::clear_lidar(TimeType t)
{
    this->BUFFER_L.clear(t);
}

/////////////////////////////////

State Accumulator::get_prev_state(double t, State & latestState)
{
    if (this->BUFFER_X.empty()) {
        latestState.time = t;
        this->push(latestState);
        return latestState;
    }

    return this->get_prev(this->BUFFER_X, t);
}

IMU Accumulator::get_next_imu(double t)
{
    return this->get_next(this->BUFFER_I, t);
}

States Accumulator::get_states(double t1, double t2)
{
    return this->get(this->BUFFER_X, t1, t2);
}

Points Accumulator::get_points(double t1, double t2)
{
    return this->get(this->BUFFER_L, t1, t2);
}

IMUs Accumulator::get_imus(double t1, double t2)
{
    return this->get(this->BUFFER_I, t1, t2);
}


//////////////////////////

bool Accumulator::ready()
{
    // Only check it once
    if (this->is_ready) {return true;}

    // Ready if there's enough IMUs to fit the delay
    if (this->enough_imus() && this->BUFFER_L.size() > 0) {
        return this->is_ready = true;
    }

    return this->is_ready = false;
}

double Accumulator::update_delta(const limovelo::InitializationParams & initialization, double t)
{
    assert(
        ("There has to be exactly one more delta value than time delimiters",
        initialization.times.size() + 1 == initialization.deltas.size()));
    return this->interpret_initialization(initialization, t);
}

double Accumulator::latest_time()
{
    return std::min(this->latest_imu_time(), this->latest_points_time());
}

double Accumulator::latest_imu_time()
{
    // // Ideally should be ros::Time::now() - delay, but it's easier the other way with rosbags (no need for use_sim_time=true)
    // return ros::Time::now() - limovelo::Config.real_time_delay;

    // Latest IMU timestamp - delay
    return this->BUFFER_I.front().time;
}

double Accumulator::latest_points_time()
{
    return this->BUFFER_L.front().time;
}

double Accumulator::oldest_points_time()
{
    return this->BUFFER_L.back().time;
}

double Accumulator::set_initial_time()
{
    if (this->BUFFER_I.size() < 1) {return -1;}
    double latest_imu_time = this->BUFFER_I.front().time;

    this->initial_time = latest_imu_time - limovelo::Config.real_time_delay;

    return this->initial_time;
}

////////////////////////// PRIVATE //////////////////////////

bool Accumulator::enough_imus()
{
    return this->BUFFER_I.size() > limovelo::Config.real_time_delay * limovelo::Config.imu_rate;
}

double Accumulator::interpret_initialization(
    const limovelo::InitializationParams & initialization,
    double t)
{
    // If is after last time
    if (initialization.times.empty()) {return initialization.deltas.back();}
    if (t - this->initial_time >= initialization.times.back()) {
        return initialization.deltas.back();
    }

    // If we have to find it in the list
    for (int k = 0; k < initialization.times.size(); ++k) {
        if (t - this->initial_time < initialization.times[k]) {
            return initialization.deltas[k];
        }
    }

    return initialization.deltas.back();
}

bool Accumulator::missing_data(const Points & time_sorted_points)
{
    if (time_sorted_points.size() < limovelo::Config.MAX_POINTS2MATCH) {return false;}

    // Check missing 'time' information
    if (time_sorted_points.front().time == 0 and time_sorted_points.back().time == 0) {
        // Remove Initialization
        limovelo::Config.Initialization.times = {};
        limovelo::Config.Initialization.deltas = {limovelo::Config.full_rotation_time};

        return true;
    }

    return false;
}

void Accumulator::throw_warning(const Points & time_sorted_points)
{
    // Warn once
    if (not this->has_warned_lidar) {this->has_warned_lidar = true;} else {return;}

    // Warn missing 'time' information FIXME: UNCOMMENT THIS!!! TODO:
    // this->logger_("LiDAR points are missing 'time' information.");
    // this->logger_(
    //     "Delta has been fixed to %f (s) leading to a fixed %d (Hz) localization.",
    //     limovelo::Config.full_rotation_time,
    //     (int) 1. / limovelo::Config.full_rotation_time);
}


std::shared_ptr<Accumulator> AccumulatorInstance = std::make_shared<Accumulator>();
