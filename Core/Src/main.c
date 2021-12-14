/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <math.h>
#include "iks01a3_motion_sensors.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ACC_GYRO IKS01A3_LSM6DSO_0
#define ACC IKS01A3_LIS2DW12_0
#define MAG IKS01A3_LIS2MDL_0
#define max_out_str_len 128
#define Ts_ticks 500 // Sensor sampling period (ticks, 100 ticks = 1 ms)
#define Ts_s 5E-3f // Sensor sampling period (s)
#define beta 2.0f // Amplification factor of gradient descent corrective step
#define window_size 5 // Moving average window size
#define window_size_inv 0.2f // Inverse of moving average window size
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

// Global variables
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
volatile float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;

// Flag variables used in ISRs:
volatile uint8_t get_sensor_data = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

void AHRS_update_quat(float wx, float wy, float wz, float ax, float ay, float az, float mx, float my, float mz);
float fast_inv_sqrtf(float x);
void quat2rpy();
int get_tx_str_len(uint8_t* out_str);
int index_incr(int i);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_USART2_UART_Init();
	MX_TIM3_Init();
	/* USER CODE BEGIN 2 */

	HAL_TIM_OC_Start_IT(&htim3, TIM_CHANNEL_1); // Start TIM3-CH1 in OC mode with IT
	__HAL_DBGMCU_FREEZE_TIM3(); // Freeze TIM3 during debug

	// Sensors init and enabling
	IKS01A3_MOTION_SENSOR_Init(ACC, MOTION_ACCELERO);
	IKS01A3_MOTION_SENSOR_Enable(ACC, MOTION_ACCELERO);
	IKS01A3_MOTION_SENSOR_Init(ACC_GYRO, MOTION_GYRO);
	IKS01A3_MOTION_SENSOR_Enable(ACC_GYRO, MOTION_GYRO);
	IKS01A3_MOTION_SENSOR_Init(MAG, MOTION_MAGNETO);
	IKS01A3_MOTION_SENSOR_Enable(MAG, MOTION_MAGNETO);

	IKS01A3_MOTION_SENSOR_Axes_t lin_acc_axes;
	IKS01A3_MOTION_SENSOR_Axes_t ang_vel_axes;
	IKS01A3_MOTION_SENSOR_Axes_t mag_axes;

	float a[3] = {0.0f}; // Acceleration (mg)
	float w[3] = {0.0f}; // Angular velocity (rad/s)
	float m[3] = {0.0f}; // Magnetic field (mG)

	int sensor_data[9][window_size + 1] = {{0}}; // For moving average calculations
	int i = 0; // Index for sensor_data
	uint8_t start_ma = 0, first_ma = 1; // Moving average flags

	uint8_t out_str[max_out_str_len]; // Output string for UART TX
	int tx_str_len = 0; // Number of chars/bytes to be TX

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
	/* USER CODE END WHILE */

	/* USER CODE BEGIN 3 */

		if(get_sensor_data == 1) // Flag set by TIM3-CH1 OC ISR (sensor sampling period has elapsed)
		{
			get_sensor_data = 0;

			// Sensor data acquisition and calibration

			// Raw sensor data acquisition
			IKS01A3_MOTION_SENSOR_GetAxes(ACC, MOTION_ACCELERO, &lin_acc_axes); // Output unit: mg
			IKS01A3_MOTION_SENSOR_GetAxes(ACC_GYRO, MOTION_GYRO, &ang_vel_axes); // Output unit: mdeg/s
			IKS01A3_MOTION_SENSOR_GetAxes(MAG, MOTION_MAGNETO, &mag_axes); // Output unit: mG

			// Data acquisition w/ moving average (window size = 5)

			// First calibration: offsets removal
			sensor_data[0][i] = lin_acc_axes.x - 31;
			sensor_data[1][i] = lin_acc_axes.y - 33;
			sensor_data[2][i] = lin_acc_axes.z + 33;
			sensor_data[3][i] = ang_vel_axes.x - 280;
			sensor_data[4][i] = ang_vel_axes.y + 490;
			sensor_data[5][i] = ang_vel_axes.z;
			sensor_data[6][i] = mag_axes.x + 95;
			sensor_data[7][i] = mag_axes.y + 63;
			sensor_data[8][i] = mag_axes.z - 70;

			if(i == window_size-1 && start_ma != 1) // The first 5 sensor values have been collected
				start_ma = 1; // We have enough sensor values to start computing the 1st moving average value

			if(start_ma == 1)
			{
				if(first_ma == 1) // Compute 1st moving avarage value
				{
					first_ma = 0;
					for(int r=0; r<=8; r++)
						for(int c=0; c<=window_size-1; c++)
							sensor_data[r][window_size] += sensor_data[r][c];

					a[1] = window_size_inv * (float)sensor_data[0][window_size];
					a[0] = -window_size_inv * (float)sensor_data[1][window_size];
					a[2] = window_size_inv * (float)sensor_data[2][window_size];
					w[0] = 1.74533E-5f * window_size_inv * (float)sensor_data[3][window_size];
					w[1] = 1.74533E-5f * window_size_inv * (float)sensor_data[4][window_size];
					w[2] = 1.74533E-5f * window_size_inv * (float)sensor_data[5][window_size];
					m[0] = -0.9434f * window_size_inv * (float)sensor_data[6][window_size];
					m[1] = 1.1691f * window_size_inv * (float)sensor_data[7][window_size];
					m[2] = 0.9220f * window_size_inv * (float)sensor_data[8][window_size];
				}
				else // Compute next moving average values
				{
					a[1] += window_size_inv * (float)(sensor_data[0][i] - sensor_data[0][index_incr(i)]);
					a[0] += -window_size_inv * (float)(sensor_data[1][i] - sensor_data[1][index_incr(i)]);
					a[2] += window_size_inv * (float)(sensor_data[2][i] - sensor_data[2][index_incr(i)]);
					w[0] += 1.74533E-5f * window_size_inv * (float)(sensor_data[3][i] - sensor_data[3][index_incr(i)]);
					w[1] += 1.74533E-5f * window_size_inv * (float)(sensor_data[4][i] - sensor_data[4][index_incr(i)]);
					w[2] += 1.74533E-5f * window_size_inv * (float)(sensor_data[5][i] - sensor_data[5][index_incr(i)]);
					m[0] += -0.9434f * window_size_inv * (float)(sensor_data[6][i] - sensor_data[6][index_incr(i)]);
					m[1] += 1.1691f * window_size_inv * (float)(sensor_data[7][i] - sensor_data[7][index_incr(i)]);
					m[2] += 0.9220f * window_size_inv * (float)(sensor_data[8][i] - sensor_data[8][index_incr(i)]);
				}
				// Second calibration has been performed: magnetic field rescaling
				// Corrections have been applied to align sensors frames to body frame
				// Ang velocity is converted from mdeg/s to rad/s through the constant 1.74533E-5 = 0.001*pi/180

				AHRS_update_quat(w[0], w[1], w[2], a[0], a[1], a[2], m[0], m[1], m[2]);

				quat2rpy();

				// Build the output string containing the RPY angles values
				snprintf((char*)out_str, max_out_str_len, "%7.2f %6.2f %7.2f\r\n", roll, pitch, yaw);

				// Compute the string length up to the terminal newline character; in this way, no undefined chars are TX
				tx_str_len = get_tx_str_len(out_str);

				HAL_UART_Transmit_IT(&huart2, out_str, tx_str_len); // UART TX on serial port COM3
			}

			i = index_incr(i);

			// Data acquisition w/o moving average
			/*
			a[1] = (float)(lin_acc_axes.x - 31);
			a[0] = -(float)(lin_acc_axes.y - 33);
			a[2] = (float)(lin_acc_axes.z + 33);
			w[0] = 1.74533E-5f*(float)(ang_vel_axes.x - 280);
			w[1] = 1.74533E-5f*(float)(ang_vel_axes.y + 490);
			w[2] = 1.74533E-5f*(float)(ang_vel_axes.z);
			m[0] = -0.9434f*(float)(mag_axes.x + 95);
			m[1] = 1.1691f*(float)(mag_axes.y + 63);
			m[2] = 0.9220f*(float)(mag_axes.z - 70);
			*/
		}
	}

	/* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 839;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void AHRS_update_quat(float wx, float wy, float wz, float ax, float ay, float az, float mx, float my, float mz)
{
	// Updates the current quaternion to the new orientation, using fused gyroscope, accelerometer and magnetometer data
	float norm_rec;
	float s0, s1, s2, s3;
	float q_dot1, q_dot2, q_dot3, q_dot4;
	float hx, hy;
	float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

	// Quaternion derivative from angular velocity (gyroscope)
	q_dot1 = 0.5f * (-q1 * wx - q2 * wy - q3 * wz);
	q_dot2 = 0.5f * ( q0 * wx + q2 * wz - q3 * wy);
	q_dot3 = 0.5f * ( q0 * wy - q1 * wz + q3 * wx);
	q_dot4 = 0.5f * ( q0 * wz + q1 * wy - q2 * wx);

	// Normalize acceleration
	norm_rec = fast_inv_sqrtf(ax * ax + ay * ay + az * az);
	ax *= norm_rec;
	ay *= norm_rec;
	az *= norm_rec;

	// Normalize magnetic field
	norm_rec = fast_inv_sqrtf(mx * mx + my * my + mz * mz);
	mx *= norm_rec;
	my *= norm_rec;
	mz *= norm_rec;

	// Auxiliary variables to avoid operations repetition
	_2q0mx = 2.0f * q0 * mx;
	_2q0my = 2.0f * q0 * my;
	_2q0mz = 2.0f * q0 * mz;
	_2q1mx = 2.0f * q1 * mx;
	_2q0 = 2.0f * q0;
	_2q1 = 2.0f * q1;
	_2q2 = 2.0f * q2;
	_2q3 = 2.0f * q3;
	_2q0q2 = 2.0f * q0 * q2;
	_2q2q3 = 2.0f * q2 * q3;
	q0q0 = q0 * q0;
	q0q1 = q0 * q1;
	q0q2 = q0 * q2;
	q0q3 = q0 * q3;
	q1q1 = q1 * q1;
	q1q2 = q1 * q2;
	q1q3 = q1 * q3;
	q2q2 = q2 * q2;
	q2q3 = q2 * q3;
	q3q3 = q3 * q3;

	// Reference direction of Earth magnetic field
	hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
	hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
	_2bx = sqrtf(hx * hx + hy * hy);
	_2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
	_4bx = 2.0f * _2bx;
	_4bz = 2.0f * _2bz;

	// Gradient descent algorithm corrective step (accelerometer + magnetometer)
	s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
	s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
	s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
	s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);

	norm_rec = fast_inv_sqrtf(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // Normalize step magnitude

	s0 *= norm_rec;
	s1 *= norm_rec;
	s2 *= norm_rec;
	s3 *= norm_rec;

	// Apply corrective step with amplification factor beta (sensor fusion)
	q_dot1 -= beta * s0;
	q_dot2 -= beta * s1;
	q_dot3 -= beta * s2;
	q_dot4 -= beta * s3;

	// Basic discrete integration of quaterion derivative
	q0 += q_dot1 * Ts_s;
	q1 += q_dot2 * Ts_s;
	q2 += q_dot3 * Ts_s;
	q3 += q_dot4 * Ts_s;

	// Normalize quaternion
	norm_rec = fast_inv_sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= norm_rec;
	q1 *= norm_rec;
	q2 *= norm_rec;
	q3 *= norm_rec;
}

float fast_inv_sqrtf(float x)
{
	// Implements the fast inverse square root algorithm
	float x_half = 0.5f * x;
	float y = x;
	int i = *(int*)&y; // Floating-point bits pointed as integer bits
	i = 0x5f3759df - (i>>1); // 1st estimate of inv sqrt (error < 5%)
	y = *(float*)&i;
	y = y * (1.5f - (x_half * y * y)); // 1 iteration of Newton method to get 2nd estimate of inv sqrt (error < 0.5%)
	return y;
}

void quat2rpy()
{
	// Computes RPY angles from quaternion
	roll = 57.29578f * atan2f( 2.0f*(q0*q1 + q2*q3) , 1.0f - 2.0f*(q1*q1 + q2*q2) ); // Range: -180/180
	pitch = 57.29578f * asinf( 2.0f*(q0*q2 - q1*q3) ); // Range: -90/90
	yaw = 57.29578f * atan2f( 2.0f*(q0*q3 + q1*q2) , 1.0f - 2.0f*(q2*q2 + q3*q3) ); // Range: -180/180
	// 57.29578 = 180/pi to convert rad to deg
}

int index_incr(int i)
{
	if(i == window_size)
		i=0;
	else
		i++;
	return i;
}

int get_tx_str_len(uint8_t* out_str)
{
	// Counts the number of chars/bytes in the string before the terminal newline character
	int tx_str_len = 0;
	for(int i = 0; i<max_out_str_len; i++)
	{
		if(out_str[i] != '\r')
			tx_str_len++;
		else
		{
			tx_str_len+=2;
			i=max_out_str_len;
		}
	}
	return tx_str_len;
}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef* htim) // TIM OC ISR
{
	if(htim == &htim3)
	{
		if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
		{
			get_sensor_data = 1;
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, __HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_1) + Ts_ticks);
		}
	}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

