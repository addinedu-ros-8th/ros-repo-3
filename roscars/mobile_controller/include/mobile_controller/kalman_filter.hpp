#ifndef KALMAN_FILTER_HPP
#define KALMAN_FILTER_HPP

class KalmanFilter {
public:
    KalmanFilter(float process_noise = 1e-5, float measurement_noise = 1e-2, float estimate_error = 1.0, float initial_value = 0.0);

    float update(float measurement);

    float getEstimate() const;

private:
    float q;  // Process noise covariance
    float r;  // Measurement noise covariance
    float p;  // Estimate error covariance
    float x;  // Value
    float k;  // Kalman gain
};

#endif // KALMAN_FILTER_HPP