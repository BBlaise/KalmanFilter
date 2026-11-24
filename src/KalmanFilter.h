#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

#include <stddef.h>

// Linear Kalman filter with a fixed state size and scalar measurements.
// The caller supplies the model, noise, and samples for each step.
template <size_t num_states, typename Scalar = float>
class KalmanFilter {
public:
    using State = Scalar[num_states];
    using Matrix = Scalar[num_states][num_states];

    // Set the starting estimate and its covariance.
    void initialize(const State& initial_state, const Matrix& initial_covariance);

    // Predict without an external input.
    void predict(const Matrix& transition, const Matrix& process_noise);

    // Predict the state (F*x + Bu) and covariance (F*P*F^T + Q).
    void predict(const Matrix& transition, const State& input_effect, const Matrix& process_noise);

    // Correct with one measurement. The measurement model is the row H.
    // Returns false if the innovation variance is unusable.
    bool correct(Scalar measurement, const State& measurement_model, Scalar measurement_variance);

    // Return the estimate at the caller's state index.
    Scalar getState(size_t index) const;

private:
    static_assert(num_states > 0, "The filter needs at least one state.");

    State state = {};
    Matrix covariance = {};
};

// Template definitions must be visible for each state count and scalar type.
#include "KalmanFilter.tpp"

#endif // KALMAN_FILTER_H_
