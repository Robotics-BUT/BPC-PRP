#include "../include/nodes/kinematics_odometry.hpp"

    Kinematics::Kinematics(double wheel_radius, double wheel_base, int ticks_revolution)
        : wheel_radius_(wheel_radius),
          wheel_base_(wheel_base),
          ticks_revolution_(ticks_revolution),
          wheel_circumference_(2 * M_PI * wheel_radius) {}

    RobotSpeed Kinematics::forward(WheelSpeed x) const {
        RobotSpeed result;
        result.v = (wheel_radius_ / 2.0) * (x.l + x.r);
        result.w = (wheel_radius_ / wheel_base_) * (x.r - x.l);
        return result;
    }

    WheelSpeed Kinematics::inverse(RobotSpeed x) const {
        WheelSpeed result;
        result.l = (x.v - (x.w * (wheel_base_ / 2.0))) / wheel_radius_;
        result.r = (x.v + (x.w * (wheel_base_ / 2.0))) / wheel_radius_;
        return result;
    }


    Coordinates Kinematics::forward(Encoders x) const {
        Coordinates result;
        result.x = (static_cast<double>(x.l) / ticks_revolution_) * wheel_circumference_;
        result.y = (static_cast<double>(x.r) / ticks_revolution_) * wheel_circumference_;
        return result;
    }

    Encoders Kinematics::inverse(Coordinates x) const {
        Encoders result;
        result.l = static_cast<int>((x.x / wheel_circumference_) * ticks_revolution_);
        result.r = static_cast<int>((x.y / wheel_circumference_) * ticks_revolution_);
        return result;
    }