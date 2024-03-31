#pragma once

#include "Common.hpp"

class Compensator
{
public:
    Compensator();

    // Main constructor
    Points compensate(const States & states, const IMUs & imus, double t2, const Points & points);

    Points downsample(const Points &);

private:
    State get_t2(const States &, double t2);
    States upsample(const States &, const IMUs &);

    Points voxelgrid_downsample(const Points &);
    Points onion_downsample(const Points &);
};
