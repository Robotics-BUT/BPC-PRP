#include "nodes/planar_imu_integrator.hpp"
#include <vector>

namespace algorithms {

    void PlanarImuIntegrator::update(float gyro_z, double dt) {
        // odečti kalibrační offset a proveď numerickou integraci (Eulerova metoda)
        float corrected_gyro = gyro_z - gyro_offset_;
        theta_ += corrected_gyro * static_cast<float>(dt);
    }

    void PlanarImuIntegrator::setCalibration(std::vector<float> gyro) {
        if (gyro.empty()) {
            gyro_offset_ = 0.0f;
            return;
        }
        float sum = std::accumulate(gyro.begin(), gyro.end(), 0.0f);
        gyro_offset_ = sum / static_cast<float>(gyro.size());
    }

    float PlanarImuIntegrator::getYaw() const {
        return theta_;  // yaw angle in radians
    }

    void PlanarImuIntegrator::reset() {
        theta_ = 0.0f;
        gyro_offset_ = 0.0f;
    }

}  // namespace algorithms
