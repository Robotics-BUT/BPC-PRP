//
// Created by kuko on 16.4.25.
//

#ifndef IMU_NODE_HPP
#define IMU_NODE_HPP


#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include "nodes/planar_imu_integrator.hpp"

namespace nodes {

    enum class ImuNodeMode {
        CALIBRATE,
        INTEGRATE,
    };

    class ImuNode : public rclcpp::Node {
    public:
        ImuNode();
        ~ImuNode() override = default;

        // Set the IMU Mode
        void setMode(const ImuNodeMode setMode);

        // Get the current IMU Mode
        ImuNodeMode getMode();

        // Get the results after Integration
        float getIntegratedResults();

        // Reset the class
        void reset_imu();

    private:

        void calibrate();
        void integrate();

        ImuNodeMode mode = ImuNodeMode::INTEGRATE;

        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
        algorithms::PlanarImuIntegrator planar_integrator_;

        std::vector<float> gyro_calibration_samples_;

        void on_imu_msg(const sensor_msgs::msg::Imu::SharedPtr msg);
    };
}



#endif //IMU_NODE_HPP
