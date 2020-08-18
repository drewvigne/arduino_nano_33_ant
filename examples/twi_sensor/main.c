/**
 * Copyright (c) 2015 - 2018, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/** @file
 * @defgroup tw_sensor_example main.c
 * @{
 * @ingroup nrf_twi_example
 * @brief TWI lsm9ds1 Example main file.
 *
 * This file contains the source code for a sample application using TWI.
 *
 */

#include <stdio.h>
#include "boards.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "nrf_drv_twi.h"
#include "nrf_delay.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "lsm9ds1_reg.h"

// TWI instance ID.
#define TWI_INSTANCE_ID     0

// Private variables.
static axis3bit16_t data_raw_acceleration;
static axis3bit16_t data_raw_angular_rate;
static axis3bit16_t data_raw_magnetic_field;
static float acceleration_mg[3];
static float angular_rate_mdps[3];
static float magnetic_field_mgauss[3];
static lsm9ds1_id_t whoamI;
static lsm9ds1_status_t reg;
static uint8_t rst;
char tx_buffer[100];

// TWI instance.
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);
uint8_t register_address = 0x0F; // Address of the who am I register

static lsm9ds1_id_t whoamI;

static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len){
  uint8_t *i2c_address = handle;
  ret_code_t err_code;
  uint16_t reg16 = reg;
  err_code = nrf_drv_twi_tx(&m_twi, *i2c_address, (uint8_t *)&reg16, 1, true);
  if (NRF_SUCCESS != err_code){
    return 0;
  }
  err_code = nrf_drv_twi_rx(&m_twi, *i2c_address, bufp, len); 
  return 0;
}

static int32_t platform_write(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len){
  uint8_t *i2c_address = handle;
  ret_code_t err_code;
  uint8_t buffer[1 + len];
  memcpy(buffer, &reg, 1);
  memcpy(buffer + 1, bufp, len);
    err_code = nrf_drv_twi_tx(&m_twi, *i2c_address, buffer, len + 1, true);
    if(err_code == NRF_SUCCESS){
      printf("Device Address and Register Address and Data sent.\r\n");
      NRF_LOG_INFO("Device Address and Register Address and Data sent.");
    }
    NRF_LOG_FLUSH();
  return 0;
}

/**
 * @brief Powering up capacitors and initializing pullup resistor.
 */
void powerup_init(void)
{
    nrf_gpio_pin_set(VDD_ENV_PIN);
    nrf_gpio_pin_set(R_PULLUP_PIN);
    // Power up and wait for voltage to rise
    nrf_gpio_cfg(VDD_ENV_PIN,  NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_S0H1, NRF_GPIO_PIN_NOSENSE);
    nrf_gpio_cfg(R_PULLUP_PIN, NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_S0H1, NRF_GPIO_PIN_NOSENSE);
    nrf_delay_ms(4);
}

/**
 * @brief TWI initialization.
 */
void twi_init (void)
{
    ret_code_t err_code;
    const nrf_drv_twi_config_t twi_config = {
       .scl                = ARDUINO_SCL1_PIN,
       .sda                = ARDUINO_SDA1_PIN,
       .frequency          = NRF_DRV_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };
    err_code = nrf_drv_twi_init(&m_twi, &twi_config, NULL, NULL);
    APP_ERROR_CHECK(err_code);
    nrf_drv_twi_enable(&m_twi);
    // To secure correct signal levels on the pins used by the TWI master when the system is in OFF mode, and
    // when the TWI master is disabled, SDA and SCL pins must be configured in the GPIO peripheral as follows:
    // SCL As specified in PSEL.SCL: Input H0D1
    // SDA As specified in PSEL.SDA: Input H0D1
    nrf_gpio_cfg(ARDUINO_SDA1_PIN, NRF_GPIO_PIN_DIR_INPUT, NRF_GPIO_PIN_INPUT_CONNECT, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_H0D1, NRF_GPIO_PIN_NOSENSE);
    nrf_gpio_cfg(ARDUINO_SCL1_PIN, NRF_GPIO_PIN_DIR_INPUT, NRF_GPIO_PIN_INPUT_CONNECT, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_H0D1, NRF_GPIO_PIN_NOSENSE);
}

/**
 * @brief Function for main application entry.
 */
int main(void)
{
    ret_code_t err_code;
    uint8_t sample_data;
    bool detected_device = false;

    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    NRF_LOG_DEFAULT_BACKENDS_INIT();

    printf("TWI sensor example started.\r\n");
    NRF_LOG_INFO("TWI sensor example started.");
    NRF_LOG_FLUSH();

    powerup_init();
    printf("Powering up device.\r\n");
    NRF_LOG_INFO("Powering up device.");
    NRF_LOG_FLUSH();
    
    twi_init();
    printf("TWI initialized.\r\n");
    NRF_LOG_INFO("TWI initialized.");
    NRF_LOG_FLUSH();

    // Initialize inertial sensors (IMU) driver interface
    uint8_t i2c_add_mag = LSM9DS1_MAG_I2C_ADD_L >> 1;
    lsm9ds1_ctx_t dev_ctx_mag;
    dev_ctx_mag.write_reg = platform_write;
    dev_ctx_mag.read_reg = platform_read;
    dev_ctx_mag.handle = (void*)&i2c_add_mag;
    printf("IMU initialized.\r\n");
    NRF_LOG_INFO("IMU initialized.");
    NRF_LOG_FLUSH();

    // Initialize magnetic sensors (MAG) driver interface
    uint8_t i2c_add_imu = LSM9DS1_IMU_I2C_ADD_H >> 1;
    lsm9ds1_ctx_t dev_ctx_imu;
    dev_ctx_imu.write_reg = platform_write;
    dev_ctx_imu.read_reg = platform_read;
    dev_ctx_imu.handle = (void*)&i2c_add_imu;
    printf("Magnetic sensors initialized\r\n");
    NRF_LOG_INFO("Magnetic sensors initialized.");
    NRF_LOG_FLUSH();

    // Check device ID
    lsm9ds1_dev_id_get(&dev_ctx_mag, &dev_ctx_imu, &whoamI);
    if (whoamI.imu != LSM9DS1_IMU_ID || whoamI.mag != LSM9DS1_MAG_ID){
      while(1){
        printf("\r\nCannot find the LSM9DS1.\r\n");
        NRF_LOG_INFO("\r\nCannot find the LSM9DS1.");
      }
      NRF_LOG_FLUSH();
    }
    printf("Who am I register [IMU]: 0x%x [MAG]: 0x%x \r\n", whoamI.imu, whoamI.mag);   

    // Restore default configuration
    lsm9ds1_dev_reset_set(&dev_ctx_mag, &dev_ctx_imu, PROPERTY_ENABLE);
    do {
      lsm9ds1_dev_reset_get(&dev_ctx_mag, &dev_ctx_imu, &rst);
    } while (rst);

    // Enable block data update
    lsm9ds1_block_data_update_set(&dev_ctx_mag, &dev_ctx_imu, PROPERTY_ENABLE);

    // Set full scale
    lsm9ds1_xl_full_scale_set(&dev_ctx_imu, LSM9DS1_4g);
    lsm9ds1_gy_full_scale_set(&dev_ctx_imu, LSM9DS1_2000dps);
    lsm9ds1_mag_full_scale_set(&dev_ctx_mag, LSM9DS1_16Ga);

    // Configure filtering chain - see datasheet for filtering chain details
    // Accelerometer filtering chain
    lsm9ds1_xl_filter_aalias_bandwidth_set(&dev_ctx_imu, LSM9DS1_AUTO);
    lsm9ds1_xl_filter_lp_bandwidth_set(&dev_ctx_imu, LSM9DS1_LP_ODR_DIV_50);
    lsm9ds1_xl_filter_out_path_set(&dev_ctx_imu, LSM9DS1_LP_OUT);
    // Gyroscope filtering chain
    lsm9ds1_gy_filter_lp_bandwidth_set(&dev_ctx_imu, LSM9DS1_LP_ULTRA_LIGHT);
    lsm9ds1_gy_filter_hp_bandwidth_set(&dev_ctx_imu, LSM9DS1_HP_MEDIUM);
    lsm9ds1_gy_filter_out_path_set(&dev_ctx_imu, LSM9DS1_LPF1_HPF_LPF2_OUT);
    // Set output data rate
    lsm9ds1_imu_data_rate_set(&dev_ctx_imu, LSM9DS1_IMU_59Hz5);
    lsm9ds1_mag_data_rate_set(&dev_ctx_mag, LSM9DS1_MAG_UHP_10Hz);

    while (true)
    {
      // Read device status register
      lsm9ds1_dev_status_get(&dev_ctx_mag, &dev_ctx_imu, &reg);

      if ( reg.status_imu.xlda && reg.status_imu.gda ){
        // Read IMU data
        memset(data_raw_acceleration.u8bit, 0x00, 3 * sizeof(int16_t));
        memset(data_raw_angular_rate.u8bit, 0x00, 3 * sizeof(int16_t));

        lsm9ds1_acceleration_raw_get(&dev_ctx_imu, data_raw_acceleration.u8bit);
        lsm9ds1_angular_rate_raw_get(&dev_ctx_imu, data_raw_angular_rate.u8bit);

        acceleration_mg[0] = lsm9ds1_from_fs4g_to_mg(data_raw_acceleration.i16bit[0]);
        acceleration_mg[1] = lsm9ds1_from_fs4g_to_mg(data_raw_acceleration.i16bit[1]);
        acceleration_mg[2] = lsm9ds1_from_fs4g_to_mg(data_raw_acceleration.i16bit[2]);

        angular_rate_mdps[0] = lsm9ds1_from_fs2000dps_to_mdps(data_raw_angular_rate.i16bit[0]);
        angular_rate_mdps[1] = lsm9ds1_from_fs2000dps_to_mdps(data_raw_angular_rate.i16bit[1]);
        angular_rate_mdps[2] = lsm9ds1_from_fs2000dps_to_mdps(data_raw_angular_rate.i16bit[2]);

        sprintf((char*)tx_buffer, "IMU - [mg]:%4.2f\t%4.2f\t%4.2f\t[mdps]:%4.2f\t%4.2f\t%4.2f\r\n",
                acceleration_mg[0], acceleration_mg[1], acceleration_mg[2],
                angular_rate_mdps[0], angular_rate_mdps[1], angular_rate_mdps[2]);
        //sprintf((char *)tx_buffer, "Moj IMU - mg[0]: %4.2f \r\n", acceleration_mg[0]);

        printf(tx_buffer/*, strlen((char const*)tx_buffer)*/);
        NRF_LOG_INFO(tx_buffer);
        NRF_LOG_FLUSH();
      }

      if ( reg.status_mag.zyxda ){
        // Read MAG data
        memset(data_raw_magnetic_field.u8bit, 0x00, 3 * sizeof(int16_t));

        lsm9ds1_magnetic_raw_get(&dev_ctx_mag, data_raw_magnetic_field.u8bit);

        magnetic_field_mgauss[0] = lsm9ds1_from_fs16gauss_to_mG(data_raw_magnetic_field.i16bit[0]);
        magnetic_field_mgauss[1] = lsm9ds1_from_fs16gauss_to_mG(data_raw_magnetic_field.i16bit[1]);
        magnetic_field_mgauss[2] = lsm9ds1_from_fs16gauss_to_mG(data_raw_magnetic_field.i16bit[2]);

        sprintf(tx_buffer, "MAG - [mG]:%4.2f\t%4.2f\t%4.2f\r\n",
                magnetic_field_mgauss[0], magnetic_field_mgauss[1], magnetic_field_mgauss[2]);
        printf(tx_buffer/*, strlen((char const*)tx_buffer)*/);
        NRF_LOG_INFO(tx_buffer);
        NRF_LOG_FLUSH();
      }
    }
}

/** @} */