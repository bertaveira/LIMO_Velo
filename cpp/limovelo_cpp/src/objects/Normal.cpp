#include "Common.hpp"
#include "Objects.hpp"

Normal::Normal() {}

Normal::Normal(const Eigen::Matrix<float, 4, 1> & ABCD)
{
    this->A = ABCD(0);
    this->B = ABCD(1);
    this->C = ABCD(2);
    this->D = ABCD(3);
}

Eigen::Matrix<float, 3, 1> Normal::vect() const
{
    return Eigen::Matrix<float, 3, 1>(
        this->A,
        this->B,
        this->C
    );
}

Eigen::Matrix<double, 3, 1> operator*(const Eigen::Matrix<double, 3, 3> & R, const Normal & n)
{
    return R * n.vect().cast<double>();
}
