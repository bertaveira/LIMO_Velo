#include "Common.hpp"
#include "Objects.hpp"

// class RotTransl {
// public:

RotTransl::RotTransl(const State & S)
{
    this->R = S.R;
    this->t = S.pos;
}

RotTransl::RotTransl(const Eigen::Matrix3f & dR, const Eigen::Vector3f & dt)
{
    this->R = dR;
    this->t = dt;
}

RotTransl RotTransl::inv()
{
    return RotTransl(
        this->R.transpose(),
        -this->R.transpose() * this->t
    );
}

RotTransl operator*(const RotTransl & RT1, const RotTransl & RT2)
{
    return RotTransl(
        RT1.R * RT2.R,
        RT1.R * RT2.t + RT1.t
    );
}

Point operator*(const RotTransl & RT, const Point & p)
{
    return Point(
        RT.R * p.toEigen() + RT.t,
        p
    );
}

Points operator*(const RotTransl & RT, const Points & points)
{
    Points moved_pts = points;
    for (Point & p : moved_pts) {
        p = RT * p;
    }
    return moved_pts;
}
