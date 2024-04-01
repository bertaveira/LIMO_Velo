#pragma once

#include <deque>
#include <functional>

#include "Objects.hpp"
#include "Utils.hpp"

class Accumulator
{
public:
    Accumulator(std::function<void(std::string)> logger = defaultPrintFunction)
      : logger_(logger) {}

    Buffer<Point> BUFFER_L;
    Buffer<IMU> BUFFER_I;
    Buffer<State> BUFFER_X;

    double initial_time;

    // Push content
    void push(const Points &);
    void push(const IMU &);
    void push(const State &);
    void push(const Point &);

    // Empty buffers
    void reset();
    void clear_buffers();
    void clear_buffers(TimeType);
    void clear_lidar(TimeType);

    // Get content given time intervals
    State get_prev_state(double t, State & latestState);
    IMU get_next_imu(double t);

    States get_states(double t1, double t2);
    Points get_points(double t1, double t2);
    IMUs get_imus(double t1, double t2);

    template<typename ContentType>
    int before_t(Buffer<ContentType> & source, double t)
    {
        return Algorithms::binary_search(source.content, t, true);
    }

    template<typename ArrayType>
    int before_t(const ArrayType & array, double t, bool desc = false)
    {
        return Algorithms::binary_search(array, t, desc);
    }

    // Start/stop
    bool ready();

    // Time management
    double set_initial_time();
    double update_delta(const limovelo::InitializationParams &, double t);
    double latest_time();
    double latest_imu_time();
    double latest_points_time();
    double oldest_points_time();

private:
    bool is_ready = false;
    bool has_warned_lidar = false;

    // Default print function
    static void defaultPrintFunction(const std::string & str)
    {
        std::cout << str << std::endl;
    }

    template<typename ContentType>
    std::deque<ContentType> get(Buffer<ContentType> & source, double t1, double t2)
    {
        std::deque<ContentType> result;
        int k_t2 = std::max(0, before_t(source, t2));

        // Get content between t1 from t2 sorted new to old
        for (int k = k_t2; k < source.content.size(); ++k) {
            ContentType cnt = source.content[k];
            if (t1 > cnt.time) {break;}
            if (t2 >= cnt.time) {result.push_front(cnt);}
        }

        return result;
    }

    template<typename ContentType>
    ContentType get_next(Buffer<ContentType> & source, double t)
    {
        if (source.content.empty()) {return ContentType();}
        if (source.content.back().time > t) {return ContentType();}
        if (t > source.content.front().time) {return source.content.front();}

        int k_t = std::max(0, before_t(source, t));
        if (k_t == 0) {return source.content[k_t];}

        // Get rightest content left to t (sorted new to old)
        for (int k = k_t; k < source.content.size(); ++k) {
            ContentType cnt = source.content[k];
            ContentType next_cnt = source.content[k - 1];
            if (t >= cnt.time) {return next_cnt;}
        }

        return ContentType();
    }

    template<typename ContentType>
    ContentType get_prev(Buffer<ContentType> & source, double t)
    {
        int k_t = before_t(source, t) + 1;
        if (k_t >= source.content.size()) {k_t = source.content.size() - 1;}

        // Get leftest (newest) content right (previous) to t (sorted new to old)
        for (int k = k_t; k >= 0; --k) {
            ContentType cnt = source.content[k];
            if (t > cnt.time) {return cnt;}
        }

        // If not a content found, push an empty one at t
        return ContentType();
    }

    bool enough_imus();
    double interpret_initialization(const limovelo::InitializationParams &, double t);

    bool missing_data(const Points &);
    void throw_warning(const Points &);

    std::function<void(std::string)> logger_;

};

extern std::shared_ptr<Accumulator> AccumulatorInstance;
