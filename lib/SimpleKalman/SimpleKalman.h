#ifndef SIMPLE_KALMAN_H
#define SIMPLE_KALMAN_H

class SimpleKalman1D {
public:
    SimpleKalman1D(double process_variance = 0.01, double measurement_variance = 0.1, double estimate = 0.0);
    double update(double measurement);

private:
    double q;
    double r;
    double x;
    double p;
};

class SensorKalmanFilter {
public:
    SensorKalmanFilter();
    
    // Updates internal state and returns filtered angle
    double filterAngle(double angle_deg, SimpleKalman1D& kalman_sin, SimpleKalman1D& kalman_cos);

    SimpleKalman1D acc_x;
    SimpleKalman1D acc_y;
    SimpleKalman1D acc_z;
    
    SimpleKalman1D gyro_x;
    SimpleKalman1D gyro_y;
    SimpleKalman1D gyro_z;
    
    SimpleKalman1D mag_x;
    SimpleKalman1D mag_y;
    SimpleKalman1D mag_z;
    
    SimpleKalman1D roll_sin;
    SimpleKalman1D roll_cos;
    SimpleKalman1D pitch_sin;
    SimpleKalman1D pitch_cos;
    SimpleKalman1D yaw_sin;
    SimpleKalman1D yaw_cos;
};

#endif
