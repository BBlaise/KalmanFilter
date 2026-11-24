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

// Set the starting estimate and its covariance.
template <size_t num_states, typename Scalar>
void KalmanFilter<num_states, Scalar>::initialize(const State& initial_state, const Matrix& initial_covariance) {
    for (size_t i = 0; i < num_states; i++) {
        state[i] = initial_state[i];
        for (size_t j = 0; j < num_states; j++) {
            covariance[i][j] = initial_covariance[i][j];
        }
    }
}

// Predict without an external input.
template <size_t num_states, typename Scalar>
void KalmanFilter<num_states, Scalar>::predict(const Matrix& transition, const Matrix& process_noise) {
    const State input_effect = {};
    predict(transition, input_effect, process_noise);
}

// Predict with the supplied input effect Bu and process noise Q.
template <size_t num_states, typename Scalar>
void KalmanFilter<num_states, Scalar>::predict(const Matrix& transition, const State& input_effect, const Matrix& process_noise) {
    State predicted_state = {};
    Matrix transition_covariance = {};

    // Compute F*x + Bu and F*P before overwriting the previous estimate.
    for (size_t i = 0; i < num_states; i++) {
        predicted_state[i] = input_effect[i];
        for (size_t k = 0; k < num_states; k++) {
            predicted_state[i] += transition[i][k] * state[k];
            for (size_t j = 0; j < num_states; j++) {
                transition_covariance[i][j] += transition[i][k] * covariance[k][j];
            }
        }
    }

    // Finish F*P*F^T + Q using rows of F as columns of F^T.
    for (size_t i = 0; i < num_states; i++) {
        state[i] = predicted_state[i];
        for (size_t j = 0; j < num_states; j++) {
            covariance[i][j] = process_noise[i][j];
            for (size_t k = 0; k < num_states; k++) {
                covariance[i][j] += transition_covariance[i][k] * transition[j][k];
            }
        }
    }
}

// Correct with one measurement. The measurement model is the row H.
// Returns false if the innovation variance is unusable.
template <size_t num_states, typename Scalar>
bool KalmanFilter<num_states, Scalar>::correct(Scalar measurement, const State& measurement_model, Scalar measurement_variance) {
    State covariance_projection = {};  // P*H^T
    Scalar innovation = measurement;
    Scalar innovation_variance = measurement_variance;

    // Compute the measurement residual z - H*x and its variance H*P*H^T + R.
    for (size_t i = 0; i < num_states; i++) {
        innovation -= measurement_model[i] * state[i];
        for (size_t j = 0; j < num_states; j++) {
            covariance_projection[i] += covariance[i][j] * measurement_model[j];
        }
        innovation_variance += measurement_model[i] * covariance_projection[i];
    }

    // A zero, negative, or NaN variance leaves the estimate unchanged.
    if (!(innovation_variance > Scalar(0))) return false;

    for (size_t i = 0; i < num_states; i++) {
        const Scalar gain = covariance_projection[i] / innovation_variance;
        state[i] += gain * innovation;

        // Update one triangle and mirror it to keep P symmetric.
        for (size_t j = i; j < num_states; j++) {
            covariance[i][j] -= gain * covariance_projection[j];
            covariance[j][i] = covariance[i][j];
        }
    }
    return true;
}

// Return the estimate at the caller's state index.
template <size_t num_states, typename Scalar>
Scalar KalmanFilter<num_states, Scalar>::getState(size_t index) const {
    return state[index];
}

#endif // KALMAN_FILTER_H_
