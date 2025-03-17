#ifndef KINEMATICS_HPP
#define KINEMATICS_HPP

#include <cmath>
#include <cstdint>

struct RobotSpeed {
    float v; // Linear velocity [m/s]
    float w; // Angular velocity [rad/s]
};

struct WheelSpeed {
    float l; // Left wheel speed [rad/s]
    float r; // Right wheel speed [rad/s]
};

struct Encoders {
    int l; // Left wheel encoder ticks
    int r; // Right wheel encoder ticks
};

struct Coordinates {
    float x; // Cartesian x-coordinate
    float y; // Cartesian y-coordinate
};

class Kinematics {
public:
    Kinematics(double wheel_radius, double wheel_base, int ticks_revolution);

    RobotSpeed forward(WheelSpeed x) const;
    WheelSpeed inverse(RobotSpeed x) const;
    Coordinates forward(Encoders x) const;
    Encoders inverse(Coordinates x) const;

private:
    double wheel_radius_;
    double wheel_base_;
    int ticks_revolution_;
    double wheel_circumference_;
};

#endif // KINEMATICS_HPP
