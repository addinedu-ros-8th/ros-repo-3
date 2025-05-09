#include "kalman_filter.hpp"

KalmanFilter::KalmanFilter(float process_noise, float measurement_noise, float estimate_error, float initial_value)
    : q(process_noise), r(measurement_noise), p(estimate_error), x(initial_value), k(0.0f) {}

float KalmanFilter::update(float measurement) {
    // Prediction update
    p += q;

    // Measurement update
    k = p / (p + r);
    x += k * (measurement - x);
    p *= (1 - k);

    return x;
}

float KalmanFilter::getEstimate() const {
    return x;
}
