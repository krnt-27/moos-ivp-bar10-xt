#include "SimpleKalman.h"
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

SimpleKalman1D::SimpleKalman1D(double process_variance, double measurement_variance, double estimate) {
    q = process_variance;
    r = measurement_variance;
    x = estimate;
    p = 1.0;
}

double SimpleKalman1D::update(double measurement) {
    // Predict
    p += q;
    // Update
    double k = p / (p + r);
    x += k * (measurement - x);
    p *= (1.0 - k);
    return x;
}

SensorKalmanFilter::SensorKalmanFilter() :
    acc_x(), acc_y(), acc_z(),
    gyro_x(), gyro_y(), gyro_z(),
    mag_x(), mag_y(), mag_z(),
    roll_sin(), roll_cos(),
    pitch_sin(), pitch_cos(),
    yaw_sin(), yaw_cos()
{
}

double SensorKalmanFilter::filterAngle(double angle_deg, SimpleKalman1D& kalman_sin, SimpleKalman1D& kalman_cos) {
    double angle_rad = angle_deg * M_PI / 180.0;
    double sin_val = std::sin(angle_rad);
    double cos_val = std::cos(angle_rad);

    double filtered_sin = kalman_sin.update(sin_val);
    double filtered_cos = kalman_cos.update(cos_val);

    double filtered_angle_rad = std::atan2(filtered_sin, filtered_cos);
    double filtered_angle_deg = filtered_angle_rad * 180.0 / M_PI;
    
    // Equivalent to % 360 for double
    filtered_angle_deg = std::fmod(filtered_angle_deg, 360.0);
    if (filtered_angle_deg < 0) {
        filtered_angle_deg += 360.0;
    }
    
    return filtered_angle_deg;
}
