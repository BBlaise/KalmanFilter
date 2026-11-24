# KalmanFilter

A header-only, fixed-size linear Kalman filter for C++ that predicts a state and corrects it one scalar measurement at a time. The caller supplies the model and noise parameters.

## Install

Copy `src/KalmanFilter.h` to your project or add `src/` to your include path. Include `KalmanFilter.h` in a C++11 or newer project.

## Usage

Choose the number of states with `KalmanFilter<N>`. Call `initialize()` with a starting estimate and covariance, then call `predict()` and `correct()` as measurements arrive. Read the estimate with `getState(index)`.

Build the basic example from the repository root with:

```sh
g++ -std=c++11 -Isrc examples/kalman_filter_basic.cpp -o kalman_filter_basic
```

Run the executable to see the estimate settle near 10.

## Examples

- [Basic C++](examples/kalman_filter_basic.cpp) estimates a constant from a short sequence of noisy measurements.
- [STM32L476RG](examples/kalman_filter_STM32L476RG.cpp) estimates X-axis tilt and angular rate from an MPU6050 and prints them over UART.

The STM32 example uses an STM32CubeL4 HAL project and the STM32I2C and MPU6050 libraries.
