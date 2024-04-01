#include "Common.hpp"
#include "Objects.hpp"

// class IMU {
// public:
IMU::IMU()
  : IMU::IMU(0.) {}

IMU::IMU(const Eigen::Vector3f & a, const Eigen::Vector3f & w, double time)
  : IMU::IMU(a, w, Eigen::Quaternionf(1, 0, 0, 0), time)
{
    this->a = a;
    this->w = w;
    this->q = q;
    this->time = time;
}

IMU::IMU(
    const Eigen::Vector3f & a, const Eigen::Vector3f & w, const Eigen::Quaternionf & q,
    double time)
{
    this->a = a;
    this->w = w;
    this->q = q;
    this->time = time;
}

IMU::IMU(double time)
  : IMU::IMU(Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), Eigen::Quaternionf(1, 0, 0, 0), time)
{
}

bool IMU::has_orientation()
{
    return not (this->q.x() == 0 and this->q.y() == 0 and this->q.z() == 0 and this->q.w() == 0);
}
