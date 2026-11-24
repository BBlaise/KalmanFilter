/*
 * Estimate the MPU6050 X-axis tilt and angular rate at 100 Hz.
 * USART2 reports raw and filtered values every 200 ms (115200 baud, 8N1).
 * I2C1 uses PB8 (SCL) and PB9 (SDA). Build this file in place of main.cpp.
 */

#include <math.h>
#include <stdio.h>
#include "main.h"
#include "STM32I2C.h"
#include "MPU6050.h"
#include "KalmanFilter.h"

static I2C_HandleTypeDef hi2c1;
static UART_HandleTypeDef huart2;

// The shared TIM6 interrupt handler uses this handle.
extern "C" {
TIM_HandleTypeDef htim6;
}

static STM32I2C i2c(&hi2c1);
static MPU6050<STM32I2C*> imu(&i2c);

static KalmanFilter<2> tilt_filter;
static KalmanFilter<1> rate_filter;
static uint32_t previous_sample_ms = 0;
static volatile uint32_t kalman_timer_ticks = 0;

static void SystemClock_Config(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM6_Init(void);
static void initializeKalmanFilters(void);
static void updateKalmanFilters(void);

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_USART2_UART_Init();
    MX_TIM6_Init();
    MX_I2C1_Init();
    initializeKalmanFilters();

    // Service TIM6 samples in the foreground so I2C and UART can block safely.
    uint32_t serviced_tick = 0;
    while (1)
    {
        const uint32_t timer_tick = kalman_timer_ticks;
        if (timer_tick == serviced_tick) continue;

        serviced_tick = timer_tick;
        updateKalmanFilters();
    }
}

// Calculate the accelerometer roll angle about X, as in mpu6050kf.c.
static float calculateAccelTiltDeg(void) {
    const float accel_y = imu.accel_y_g;
    const float accel_x = imu.accel_x_g;
    const float accel_z = imu.accel_z_g;
    const float roll_denominator = sqrtf(accel_x * accel_x + accel_z * accel_z);
    return atan2f(accel_y, roll_denominator) * 57.2957795f;
}

// Seed both estimates, then start TIM6 at 100 Hz.
static void initializeKalmanFilters(void) {
    imu.initialize();
    float readings[7];
    imu.readSample(readings);

    tilt_filter.initialize({calculateAccelTiltDeg(), 0.0f}, {{1.0f, 0.0f}, {0.0f, 1.0f}});
    rate_filter.initialize({imu.gyro_x_dps}, {{1.0f}});
    previous_sample_ms = HAL_GetTick();

    // APB1 runs without a divider, so TIM6 uses the APB1 clock.
    const uint32_t timer_count_hz = HAL_RCC_GetPCLK1Freq() / (htim6.Init.Prescaler + 1);
    __HAL_TIM_SET_AUTORELOAD(&htim6, timer_count_hz / 100 - 1);
    __HAL_TIM_SET_COUNTER(&htim6, 0);
    __HAL_TIM_CLEAR_FLAG(&htim6, TIM_FLAG_UPDATE);
    if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK) Error_Handler();
}

// Signal a sample from the TIM6 interrupt, as in the Lab 6 timer callback.
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
    if (htim == &htim6) kalman_timer_ticks++;
}

// Run one filter update and report the results over serial.
static void updateKalmanFilters(void) {
    float readings[7];
    imu.readSample(readings);
    const uint32_t sample_ms = HAL_GetTick();
    const float dt = (sample_ms - previous_sample_ms) * 0.001f;
    previous_sample_ms = sample_ms;

    const float accel_tilt_deg = calculateAccelTiltDeg();
    const float gyro_rate_dps = imu.gyro_x_dps;

    // Tilt state order: angle in degrees, then gyro bias in deg/s.
    const float tilt_transition[2][2] = {{1.0f, -dt}, {0.0f, 1.0f}};  // Subtract integrated bias.
    const float tilt_gyro_input[2] = {dt * gyro_rate_dps, 0.0f};       // Integrate the measured rate.
    const float tilt_process_noise[2][2] = {
        {0.001f * dt, 0.0f},  // Added angle variance over this step.
        {0.0f, 0.003f * dt}   // Added bias variance over this step.
    };
    const float tilt_measurement_model[2] = {1.0f, 0.0f};  // The accelerometer observes angle.
    const float tilt_measurement_variance = 0.03f;

    tilt_filter.predict(tilt_transition, tilt_gyro_input, tilt_process_noise);
    tilt_filter.correct(accel_tilt_deg, tilt_measurement_model, tilt_measurement_variance);

    const float rate_transition[1][1] = {{1.0f}};     // Carry the previous rate forward.
    const float rate_process_noise[1][1] = {{dt}};    // Add rate variance over this step.
    const float rate_measurement_model[1] = {1.0f};   // The gyro directly observes rate.
    const float rate_measurement_variance = 0.25f;

    rate_filter.predict(rate_transition, rate_process_noise);
    rate_filter.correct(gyro_rate_dps, rate_measurement_model, rate_measurement_variance);

    // Print less frequently than the filter updates.
    static uint32_t previous_print_ms = 0;
    if (sample_ms - previous_print_ms >= 200) {
        previous_print_ms = sample_ms;
        char message[96];
        const int length = snprintf(message, sizeof(message), "Tilt X (deg): %.2f -> %.2f | Rate X (deg/s): %.2f -> %.2f\r\n",
               accel_tilt_deg, tilt_filter.getState(0),
               gyro_rate_dps, rate_filter.getState(0));
        if (length > 0 && length < static_cast<int>(sizeof(message))) {
            HAL_UART_Transmit(&huart2, reinterpret_cast<uint8_t*>(message), length, HAL_MAX_DELAY);
        }
    }
}

// Stops execution if clock or peripheral initialization fails.
void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
    }
}

// Configures the 80 MHz system clock.
static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage
    */
    if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 1;
    RCC_OscInitStruct.PLL.PLLN = 10;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
    RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
    {
        Error_Handler();
    }
}

// Configures I2C1 for the MPU6050.
static void MX_I2C1_Init(void)
{

    hi2c1.Instance = I2C1;
    hi2c1.Init.Timing = 0x00F12981;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK)
    {
        Error_Handler();
    }

    /** Configure Analogue filter
    */
    if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
    {
        Error_Handler();
    }

    /** Configure Digital filter
    */
    if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
    {
        Error_Handler();
    }

}

// Configures USART2 for sensor output.
static void MX_USART2_UART_Init(void)
{

    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart2) != HAL_OK)
    {
        Error_Handler();
    }

}

// Configures the timer used to schedule filter updates.
static void MX_TIM6_Init(void)
{
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    htim6.Instance = TIM6;
    htim6.Init.Prescaler = 8000-1;
    htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim6.Init.Period = 10000;
    htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
    {
      Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
    {
      Error_Handler();
    }

}
