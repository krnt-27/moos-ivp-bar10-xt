#ifndef MADGWICK_AHRS_H
#define MADGWICK_AHRS_H

class MadgwickAHRS {
public:
    MadgwickAHRS(float sampleFreq = 10.0f, float beta = 0.1f);
    void updateMARG(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
    void updateIMU(float gx, float gy, float gz, float ax, float ay, float az);
    
    // Quaternion in scalar-first format [q0, q1, q2, q3] -> [w, x, y, z]
    float q0, q1, q2, q3; 

    float getRoll();
    float getPitch();
    float getYaw();

private:
    float sampleFreq;
    float beta;
    float invSampleFreq;
    float invSqrt(float x);
    void computeAngles();
    
    float roll, pitch, yaw;
    bool anglesComputed;
};

#endif
