#include "Common.hpp"
#include "Objects.hpp"

// class Match {
// public:
Match::Match(const Point & p, const Plane & H)
{
    this->point = p;
    this->plane = H;
    this->distance = H.dist_to_plane(p);
}

bool Match::is_chosen()
{
    return this->plane.is_plane;
}
