// Estimate a constant from noisy measurements and print each updated estimate.

#include <cstdio>
#include "KalmanFilter.h"

// Initialize a one-state model, then predict and correct for each measurement.
int main() {
    KalmanFilter<1> filter;
    const KalmanFilter<1>::State initial_state = {0.0f};
    const KalmanFilter<1>::Matrix initial_covariance = {{100.0f}};
    const KalmanFilter<1>::Matrix transition = {{1.0f}};
    const KalmanFilter<1>::Matrix process_noise = {{0.0f}};
    const KalmanFilter<1>::State measurement_model = {1.0f};
    const float measurement_variance = 1.0f;
    const float measurements[] = {9.0f, 11.0f, 8.0f, 12.0f, 10.0f};

    // The constant-state model carries the estimate forward without process noise.
    filter.initialize(initial_state, initial_covariance);
    for (float measurement : measurements) {
        filter.predict(transition, process_noise);
        if (!filter.correct(measurement, measurement_model, measurement_variance)) {
            return 1;
        }
        std::printf("Measured: %.2f  Estimated: %.2f\n", measurement, filter.getState(0));
    }
    return 0;
}
