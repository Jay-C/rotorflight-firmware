/*
 * This file is part of Rotorflight.
 *
 * Rotorflight is free software. You can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Rotorflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software. If not, see <https://www.gnu.org/licenses/>.
 */

#pragma once

#define TARGET_BOARD_IDENTIFIER "DEVEBOXH743"

#define USBD_PRODUCT_STRING     "DevEBoxH743"

#define USE_TARGET_CONFIG

#define LED0_PIN                PA1

#define USE_BUTTONS

#define	BUTTON_A_PIN            PE3
#define BUTTON_A_PIN_INVERTED

#define	BUTTON_B_PIN            PC5
#define BUTTON_B_PIN_INVERTED

#define USE_VCP
#define USE_USB_ID
#define USE_USB_DETECT

#define USE_UART
#define USE_UART1
#define USE_UART2
#define USE_UART3
#define USE_UART4
#define USE_UART5
#define USE_UART6
#define USE_UART7
#define USE_UART8

#define USE_SOFTSERIAL1
#define USE_SOFTSERIAL2

#define UNIFIED_SERIAL_PORT_COUNT   3
#define SERIAL_PORT_COUNT           11

#define USE_I2C
#define USE_I2C_DEVICE_1
#define USE_I2C_DEVICE_2
#define USE_I2C_DEVICE_3
#define USE_I2C_DEVICE_4
#define I2C_FULL_RECONFIGURABILITY

#define USE_SPI
#define USE_SPI_DEVICE_1
#define USE_SPI_DEVICE_2
#define USE_SPI_DEVICE_3
#define USE_SPI_DEVICE_4
#define USE_SPI_DEVICE_5
#define USE_SPI_DEVICE_6
#define SPI_FULL_RECONFIGURABILITY

#define USE_QUADSPI
#define USE_QUADSPI_DEVICE_1
//#define USE_QUADSPI_DEVICE_2

#define FLASH_QUADSPI_INSTANCE  QUADSPI

#define QUADSPI1_MODE           QUADSPI_MODE_BK1_ONLY
#define QUADSPI1_CS_FLAGS       (QUADSPI_BK1_CS_HARDWARE | QUADSPI_BK2_CS_NONE | QUADSPI_CS_MODE_LINKED)

#define QUADSPI1_SCK_PIN        PB2

#define QUADSPI1_BK1_CS_PIN     PB6
#define QUADSPI1_BK1_IO0_PIN    PD11
#define QUADSPI1_BK1_IO1_PIN    PD12
#define QUADSPI1_BK1_IO2_PIN    PE2
#define QUADSPI1_BK1_IO3_PIN    PD13

#define QUADSPI1_BK2_CS_PIN     NONE /* NONE */
#define QUADSPI1_BK2_IO0_PIN    NONE /* PE7 */
#define QUADSPI1_BK2_IO1_PIN    NONE /* PE8 */
#define QUADSPI1_BK2_IO2_PIN    NONE /* PE9 */
#define QUADSPI1_BK2_IO3_PIN    NONE /* PE10 */

#define USE_SDCARD
#define USE_SDCARD_SDIO

#define SDCARD_DETECT_PIN       NONE

#define USE_BLACKBOX

#define USE_EXTI
#define USE_MULTI_GYRO
#define USE_MPU_DATA_READY_SIGNAL

#define USE_FAKE_ACC
#define USE_FAKE_GYRO

#define USE_ACC
#define USE_ACC_MPU6050
#define USE_ACC_MPU6500
#define USE_ACC_SPI_MPU6000
#define USE_ACC_SPI_MPU6500
#define USE_ACC_SPI_MPU9250
#define USE_ACC_SPI_ICM42605

#define USE_GYRO
#define USE_GYRO_EXTI
#define USE_GYRO_REGISTER_DUMP
#define USE_GYRO_SPI_MPU6000
#define USE_GYRO_SPI_MPU6500
#define USE_GYRO_SPI_MPU9250
#define USE_GYRO_SPI_ICM42605

#define USE_MAG
#define USE_MAG_DATA_READY_SIGNAL
#define USE_MAG_HMC5883
#define USE_MAG_SPI_HMC5883
#define USE_MAG_QMC5883
#define USE_MAG_LIS3MDL
#define USE_MAG_AK8963
#define USE_MAG_SPI_AK8963

#define USE_BARO
#define USE_BARO_MS5611
#define USE_BARO_SPI_MS5611
#define USE_BARO_BMP280
#define USE_BARO_SPI_BMP280
#define USE_BARO_BMP388
#define USE_BARO_SPI_BMP388
#define USE_BARO_LPS
#define USE_BARO_SPI_LPS
#define USE_BARO_QMP6988
#define USE_BARO_SPI_QMP6988

#define USE_FLASHFS
#define USE_FLASH_CHIP
#define USE_FLASH_TOOLS
#define USE_FLASH_M25P16
#define USE_FLASH_W25N01G
#define USE_FLASH_W25M
#define USE_FLASH_W25M512
#define USE_FLASH_W25M02G

#define USE_RANGEFINDER
#define USE_RANGEFINDER_HCSR04
#define USE_RANGEFINDER_TF

#define USE_DMA

#define USE_ADC

#define ADC1_INSTANCE           ADC1
#define ADC2_INSTANCE           ADC2
#define ADC3_INSTANCE           ADC3

#define USE_DSHOT
#define USE_DSHOT_DMAR

#define USE_FREQ_SENSOR

#define USE_BEEPER

#define DEFAULT_RX_FEATURE	0
#define DEFAULT_FEATURE		0

#define TARGET_IO_PORTA		0xffff
#define TARGET_IO_PORTB		0xffff
#define TARGET_IO_PORTC		0xffff
#define TARGET_IO_PORTD		0xffff
#define TARGET_IO_PORTE		0xffff
#define TARGET_IO_PORTF		0xffff
#define TARGET_IO_PORTG		0xffff

#define USABLE_TIMER_CHANNEL_COUNT 1

