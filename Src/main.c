/* USER CODE BEGIN Header */
/**
 * PROJE: A-NAV (Autonomous Navigation & AHRS)
 * YAZAR: Hatice Kübra Yılgın - 1030220765
 * DETAY: IMU Kalibrasyonu, PA4 Pil Ölçümü, PB12 Flow CS ve PID Sistemi.
 */
/* USER CODE END Header */

#include "main.h"
#include "adc.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include "ssd1306.h"
#include "ssd1306_fonts.h"

/* USER CODE BEGIN PV */
// 1. Sensör ve AHRS Verileri
int16_t Accel_X_RAW, Accel_Y_RAW, Accel_Z_RAW, Gyro_X_RAW, Gyro_Y_RAW;
float Ax, Ay, Az, Gx, Gy, roll = 0, pitch = 0, dt = 0.01f;

// 2. Kalibrasyon Değerleri
float ax_off = 0, ay_off = 0, gx_off = 0, gy_off = 0;
int cal_samples = 500;

// 3. Enerji Takibi (PA4 ADC)
uint32_t adc_val;
float v_bat = 0.0f;
const float v_ratio = 5.54f;

// 4. PID ve Kontrol Parametreleri
float Kp_Roll = 1.2f, Ki_Roll = 0.01f, Kd_Roll = 8.0f;
float roll_integral = 0, prev_error_roll = 0, pid_roll_out;
float Kp_Pitch = 1.2f, Ki_Pitch = 0.01f, Kd_Pitch = 8.0f;
float pitch_integral = 0, prev_error_pitch = 0, pid_pitch_out;
float throttle = 1200, motor1, motor2, motor3, motor4;

// 5. Navigasyon ve Ekran Deposu
uint8_t gps_data;
char gps_buffer[100];
int buffer_index = 0;
float latitude = 0, longitude = 0;
uint32_t last_gps_tick = 0;
uint16_t MS5611_C[8];
float altitude_m = 0, heading_deg = 0;
int16_t mag_x, mag_y, mag_z, flow_x, flow_y;
char oled_buf[32];
/* USER CODE END PV */

/* --- FONKSİYON PROTOTİPLERİ --- */
/* USER CODE BEGIN PFP */
void SystemClock_Config(void);
void Parse_GPS_Data(char *buffer);
void MS5611_Init(void);
void QMC5883L_Init(void);
void Read_Magnetometer(void);
void Read_Optical_Flow(void);
/* USER CODE END PFP */

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();

  /* USER CODE BEGIN 2 */
  ssd1306_Init();
  ssd1306_Fill(Black);
  ssd1306_SetCursor(10, 10);
  ssd1306_WriteString("CALIBRATING...", Font_7x10, White);
  ssd1306_UpdateScreen();

  uint8_t pwr = 0x00;
  HAL_I2C_Mem_Write(&hi2c1, (0x68 << 1), 0x6B, 1, &pwr, 1, 100);

  // MPU6050 Otomatik Sıfırlama
  for (int i = 0; i < cal_samples; i++) {
      uint8_t r[14];
      HAL_I2C_Mem_Read(&hi2c1, (0x68 << 1), 0x3B, 1, r, 14, 100);
      ax_off += (int16_t)(r[0] << 8 | r[1]);
      ay_off += (int16_t)(r[2] << 8 | r[3]);
      gx_off += (int16_t)(r[8] << 8 | r[9]);
      gy_off += (int16_t)(r[10] << 8 | r[11]);
      HAL_Delay(2);
  }
  ax_off /= cal_samples; ay_off /= cal_samples;
  gx_off /= cal_samples; gy_off /= cal_samples;

  MS5611_Init();
  QMC5883L_Init();
  HAL_UART_Receive_IT(&huart2, &gps_data, 1);

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
  /* USER CODE END 2 */

  while (1)
  {
    /* USER CODE BEGIN 3 */
    uint8_t imu_raw[14];
    HAL_I2C_Mem_Read(&hi2c1, (0x68 << 1), 0x3B, 1, imu_raw, 14, 100);

    Ax = ((int16_t)(imu_raw[0] << 8 | imu_raw[1]) - ax_off) / 16384.0f;
    Ay = ((int16_t)(imu_raw[2] << 8 | imu_raw[3]) - ay_off) / 16384.0f;
    Az = (int16_t)(imu_raw[4] << 8 | imu_raw[5]) / 16384.0f;
    Gx = ((int16_t)(imu_raw[8] << 8 | imu_raw[9]) - gx_off) / 131.0f;
    Gy = ((int16_t)(imu_raw[10] << 8 | imu_raw[11]) - gy_off) / 131.0f;

    // Tamamlayıcı Filtre Denklemimiz:
    // $$Angle = 0.96 \cdot (Angle + Gyro \cdot dt) + 0.04 \cdot (AccelAngle)$$
    roll  = 0.96f*(roll + Gx*dt) + 0.04f*(atan2f(Ay, sqrtf(Ax*Ax + Az*Az))*57.29578f);
    pitch = 0.96f*(pitch + Gy*dt) + 0.04f*(atan2f(-Ax, sqrtf(Ay*Ay + Az*Az))*57.29578f);

    Read_Magnetometer();
    Read_Optical_Flow();

    // PA4 Pil Voltaj Takibi
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
        adc_val = HAL_ADC_GetValue(&hadc1);
        v_bat = (adc_val * 3.3f / 4095.0f) * v_ratio;
    }

    // PID Dengeleme Algoritması
    // $$Output(t) = K_p e(t) + K_i \int e(t)dt + K_d \frac{de(t)}{dt}$$
    float er = 0.0f - roll; float ep = 0.0f - pitch;
    roll_integral += er * dt; pitch_integral += ep * dt;
    pid_roll_out = (Kp_Roll*er) + (Ki_Roll*roll_integral) + (Kd_Roll*(er - prev_error_roll)/dt);
    pid_pitch_out = (Kp_Pitch*ep) + (Ki_Pitch*pitch_integral) + (Kd_Pitch*(ep - prev_error_pitch)/dt);
    prev_error_roll = er; prev_error_pitch = ep;

    motor1 = throttle + pid_pitch_out + pid_roll_out;
    motor2 = throttle + pid_pitch_out - pid_roll_out;
    motor3 = throttle - pid_pitch_out - pid_roll_out;
    motor4 = throttle - pid_pitch_out + pid_roll_out;

    // --- Warning Karşıtı Güvenlik Blokları ---
    if (motor1 > 2400) { motor1 = 2400; }
    if (motor1 < 1000) { motor1 = 1000; }
    if (motor2 > 2400) { motor2 = 2400; }
    if (motor2 < 1000) { motor2 = 1000; }
    if (motor3 > 2400) { motor3 = 2400; }
    if (motor3 < 1000) { motor3 = 1000; }
    if (motor4 > 2400) { motor4 = 2400; }
    if (motor4 < 1000) { motor4 = 1000; }

    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (uint32_t)motor1);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, (uint32_t)motor2);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, (uint32_t)motor3);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, (uint32_t)motor4);

    ssd1306_Fill(Black);
    sprintf(oled_buf, "R:%.1f P:%.1f B:%.1fV", roll, pitch, v_bat);
    ssd1306_SetCursor(0, 0); ssd1306_WriteString(oled_buf, Font_7x10, White);
    sprintf(oled_buf, "LAT:%.5f", latitude);
    ssd1306_SetCursor(0, 16); ssd1306_WriteString(oled_buf, Font_7x10, White);
    sprintf(oled_buf, "LON:%.5f", longitude);
    ssd1306_SetCursor(0, 32); ssd1306_WriteString(oled_buf, Font_7x10, White);
    ssd1306_UpdateScreen();

    HAL_Delay(10);
    /* USER CODE END 3 */
  }
}

/* USER CODE BEGIN 4 */
void Read_Optical_Flow(void) {
    uint8_t reg = 0x02; uint8_t raw[4];
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); // Yeni CS pini: PB12
    HAL_SPI_Transmit(&hspi1, &reg, 1, 100);
    HAL_SPI_Receive(&hspi1, raw, 4, 100);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
    flow_x = (int16_t)(raw[1] << 8 | raw[0]);
    flow_y = (int16_t)(raw[3] << 8 | raw[2]);
}

void QMC5883L_Init(void) {
    uint8_t cfg[2] = {0x09, 0x1D};
    HAL_I2C_Master_Transmit(&hi2c1, (0x0D << 1), cfg, 2, 100);
}

void Read_Magnetometer(void) {
    uint8_t m[6];
    HAL_I2C_Mem_Read(&hi2c1, (0x0D << 1), 0x00, 1, m, 6, 100);
    mag_x = (int16_t)(m[1] << 8 | m[0]);
    mag_y = (int16_t)(m[3] << 8 | m[2]);
    heading_deg = atan2f(mag_y, mag_x) * 57.29578f;
    if(heading_deg < 0) heading_deg += 360;
}

void MS5611_Init(void) {
    uint8_t p[2];
    for (int i = 0; i < 8; i++) {
        uint8_t cmd = 0xA0 + (i * 2);
        HAL_I2C_Master_Transmit(&hi2c1, (0x77 << 1), &cmd, 1, 100);
        HAL_I2C_Master_Receive(&hi2c1, (0x77 << 1), p, 2, 100);
        MS5611_C[i] = (p[0] << 8) | p[1];
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        if (buffer_index < 100) gps_buffer[buffer_index++] = gps_data;
        if (gps_data == '\n') {
            gps_buffer[buffer_index] = '\0'; Parse_GPS_Data(gps_buffer);
            buffer_index = 0; last_gps_tick = HAL_GetTick();
        }
        HAL_UART_Receive_IT(&huart2, &gps_data, 1);
    }
}

void Parse_GPS_Data(char *buffer) {
    if (strstr(buffer, "$GPRMC")) {
        char *t = strtok(buffer, ","); int c = 0;
        while (t != NULL) {
            c++; t = strtok(NULL, ",");
            if (c == 3 && t) latitude = (int)(atof(t)/100) + (atof(t) - (int)(atof(t)/100)*100)/60.0f;
            if (c == 5 && t) longitude = (int)(atof(t)/100) + (atof(t) - (int)(atof(t)/100)*100)/60.0f;
        }
    }
}

void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);
}

void Error_Handler(void) {
  __disable_irq();
  while (1) {}
}
/* USER CODE END 4 */
