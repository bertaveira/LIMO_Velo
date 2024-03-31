#pragma once

// SLAM libraries
#include "use-ikfom.hpp"
#include "ikd_Tree.h"

#include "Objects.hpp"

class Mapper
{
public:
    double last_map_time = -1;

private:
    KD_TREE<Point>::Ptr map;

public:
    Mapper();
    bool exists();
    int size();

    void add(Points &, double time, bool downsample = false);
    void add(const State &, Points &, bool downsample = false);
    Matches match(const State &, const Points &);
    bool hasToMap(double t);

private:
    void init_tree();
    void build_tree(Points &);
    bool exists_tree();

    void add_points(Points &, bool downsample = false);
    Match match_plane(const Point &);
};

extern std::shared_ptr<Mapper> MapperInstance;
