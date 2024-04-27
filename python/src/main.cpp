#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>

#include "Limovelo.hpp"
#include "Common.hpp"
#include "Objects.hpp"

namespace py = pybind11;

class LimoVeloPy : public limovelo::LimoVelo
{
public:
    LimoVeloPy(std::function<void(std::string)> logger)
      : limovelo::LimoVelo(logger) {}

    void add_points_numpy(py::array_t<float> points)
    {
        // Assert the array is N x 5
        if (points.ndim() != 2 || points.shape(1) != 5) {
            throw std::runtime_error(
                      "Input array must be of shape N x 5, where N is the number of points and 5 is the number of columns (x, y, z, intensity, time)");
        }

        // Build the Points object
        Points points_obj;
        auto buffer = points.request();
        float * ptr = (float *) buffer.ptr;
        for (size_t i = 0; i < buffer.shape[0]; i++) {
            Point point;
            point.x = ptr[i * 5];
            point.y = ptr[i * 5 + 1];
            point.z = ptr[i * 5 + 2];
            point.intensity = ptr[i * 5 + 3];
            point.time = ptr[i * 5 + 4];
            points_obj.push_back(point);
        }

        // Add the points
        this->add_points(points_obj);
    }

    void add_imu_py(
        float ax, float ay, float az, float wx, float wy, float wz, float qx, float qy,
        float qz, float qw, float time)
    {
        IMU imu;
        imu.a << ax, ay, az;
        imu.w << wx, wy, wz;
        imu.q.x() = qx;
        imu.q.y() = qy;
        imu.q.z() = qz;
        imu.time = time;
        this->add_imu(imu);
    }

    py::array _limovelo_to_numpy_points(Points points)
    {
        size_t n = points.size();

        std::vector<double> data(5 * n);

        // Loop through points and populate the array
        for (size_t i = 0; i < n; ++i) {
            data[i * 5] = points[i].x;
            data[i * 5 + 1] = points[i].y;
            data[i * 5 + 2] = points[i].z;
            data[i * 5 + 3] = points[i].intensity;
            data[i * 5 + 4] = points[i].time;
        }

        ssize_t ndim = 2;
        std::vector<ssize_t> shape = {n, 5};
        std::vector<ssize_t> strides = {sizeof(double) * 5, sizeof(double)};

        return py::array(
            py::buffer_info(
                data.data(),                       /* data as contiguous array  */
                sizeof(double),                      /* size of one scalar        */
                py::format_descriptor<double>::format(), /* data type                 */
                ndim,                                /* number of dimensions      */
                shape,                               /* shape of the matrix       */
                strides                              /* strides for each axis     */
        ));
    }

    py::array_t<float> get_compensated_pointcloud_py(
        bool global_frame = false,
        bool downsample = false)
    {
        Points points;
        if (downsample) {
            points =
              global_frame ? this->get_global_downsampled_compensated_points() : this->
              get_downsampled_compensated_points();
        } else {
            points =
              global_frame ? this->get_global_compensated_points() : this->get_compsensated_points();
        }
        return _limovelo_to_numpy_points(points);
    }

};


PYBIND11_MODULE(limovelo, m) {

    py::class_<limovelo::InitializationParams>(m, "InitializationParams")
    .def_readwrite("times", &limovelo::InitializationParams::times)
    .def_readwrite("deltas", &limovelo::InitializationParams::deltas);

    py::class_<limovelo::Params>(m, "Params")
    .def_readwrite("real_time", &limovelo::Params::real_time)
    .def_readwrite("estimate_extrinsics", &limovelo::Params::estimate_extrinsics)
    .def_readwrite("initial_gravity", &limovelo::Params::initial_gravity)
    .def_readwrite("I_Rotation_L", &limovelo::Params::I_Rotation_L)
    .def_readwrite("I_Translation_L", &limovelo::Params::I_Translation_L)
    .def_readwrite("empty_lidar_time", &limovelo::Params::empty_lidar_time)
    .def_readwrite("real_time_delay", &limovelo::Params::real_time_delay)
    .def_readwrite("full_rotation_time", &limovelo::Params::full_rotation_time)
    .def_readwrite("imu_rate", &limovelo::Params::imu_rate)
    .def_readwrite("downsample_rate", &limovelo::Params::downsample_rate)
    .def_readwrite("downsample_prec", &limovelo::Params::downsample_prec)
    .def_readwrite("min_dist", &limovelo::Params::min_dist)
    .def_readwrite("LiDAR_type", &limovelo::Params::LiDAR_type)
    .def_readwrite("offset_beginning", &limovelo::Params::offset_beginning)
    .def_readwrite("stamp_beginning", &limovelo::Params::stamp_beginning)
    .def_readwrite("degeneracy_threshold", &limovelo::Params::degeneracy_threshold)
    .def_readwrite("print_degeneracy_values", &limovelo::Params::print_degeneracy_values)
    .def_readwrite("MAX_NUM_ITERS", &limovelo::Params::MAX_NUM_ITERS)
    .def_readwrite("MAX_POINTS2MATCH", &limovelo::Params::MAX_POINTS2MATCH)
    .def_readwrite("LIMITS", &limovelo::Params::LIMITS)
    .def_readwrite("NUM_MATCH_POINTS", &limovelo::Params::NUM_MATCH_POINTS)
    .def_readwrite("MAX_DIST_PLANE", &limovelo::Params::MAX_DIST_PLANE)
    .def_readwrite("PLANES_THRESHOLD", &limovelo::Params::PLANES_THRESHOLD)
    .def_readwrite("PLANES_CHOOSE_CONSTANT", &limovelo::Params::PLANES_CHOOSE_CONSTANT)
    .def_readwrite("wx_MULTIPLIER", &limovelo::Params::wx_MULTIPLIER)
    .def_readwrite("wy_MULTIPLIER", &limovelo::Params::wy_MULTIPLIER)
    .def_readwrite("wz_MULTIPLIER", &limovelo::Params::wz_MULTIPLIER)
    .def_readwrite("cov_acc", &limovelo::Params::cov_acc)
    .def_readwrite("cov_gyro", &limovelo::Params::cov_gyro)
    .def_readwrite("cov_bias_acc", &limovelo::Params::cov_bias_acc)
    .def_readwrite("cov_bias_gyro", &limovelo::Params::cov_bias_gyro)
    .def_readwrite("LiDAR_noise", &limovelo::Params::LiDAR_noise)
    .def_readwrite("points_topic", &limovelo::Params::points_topic)
    .def_readwrite("imus_topic", &limovelo::Params::imus_topic)
    .def_readwrite("Initialization", &limovelo::Params::Initialization);

    py::class_<LimoVeloPy> limovelo_class(m, "LimoVelo");
    limovelo_class.def(py::init<std::function<void(std::string)>>());
    limovelo_class.def("reset", &LimoVeloPy::reset);
    limovelo_class.def("initialized", &LimoVeloPy::initialized);
    limovelo_class.def("add_imu", &LimoVeloPy::add_imu_py);
    limovelo_class.def("add_points", &LimoVeloPy::add_points_numpy);
    limovelo_class.def("run_localization", &LimoVeloPy::run_localization);
    limovelo_class.def("run_mapping", &LimoVeloPy::run_mapping);
    limovelo_class.def(
        "get_compensated_pointcloud", &LimoVeloPy::get_compensated_pointcloud_py, py::arg(
            "global_frame") = false, py::arg("downsample") = false);
}
