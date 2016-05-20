/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#define TARGET_BOARD_IDENTIFIER "VRCR" // STM Discovery F3

#define USBD_PRODUCT_STRING "STM32 F3 Discovery"

#define LED0    PE8
#define LED1    PE10
#define LED2    PE14
#define LED0_GPIO   GPIOE
#define LED0_PIN    Pin_8|Pin_12 // Blue LEDs - PE8/PE12
#define LED0_PERIPHERAL RCC_AHBPeriph_GPIOE
#define LED0_INVERTED
#define LED1_GPIO   GPIOE
#define LED1_PIN    Pin_10|Pin_14  // Orange LEDs - PE10/PE14
#define LED1_PERIPHERAL RCC_AHBPeriph_GPIOE
#define LED1_INVERTED

#define BEEPER  PE9

#define BEEP_GPIO   GPIOE
#define BEEP_PIN    Pin_9|Pin_13 // Red LEDs - PE9/PE13
#define BEEP_PERIPHERAL RCC_AHBPeriph_GPIOE
#define BEEPER_INVERTED

#define USE_SPI
#define USE_SPI_DEVICE_1
#define USE_SPI_DEVICE_2

// SPI1
// PB5  SPI1_MOSI
// PB4  SPI1_MISO
// PB3  SPI1_SCK
// PA15 SPI1_NSS

// SPI2
// PB15 SPI2_MOSI
// PB14 SPI2_MISO
// PB13 SPI2_SCK
// PB12 SPI2_NSS

//#define USE_FLASHFS
//#define USE_FLASH_M25P16

//#define M25P16_CS_GPIO          GPIOB
//#define M25P16_CS_PIN           GPIO_Pin_12
//#define M25P16_SPI_INSTANCE     SPI2

#define GYRO
#define USE_GYRO_L3GD20

#define L3GD20_SPI                      SPI1
#define L3GD20_CS_GPIO_CLK_PERIPHERAL   RCC_AHBPeriph_GPIOE
#define L3GD20_CS_GPIO                  GPIOE
#define L3GD20_CS_PIN                   PE3

#define GYRO_L3GD20_ALIGN CW270_DEG

#define USE_SDCARD

#define SDCARD_SPI_INSTANCE                  SPI2
#define SDCARD_SPI_CS_PIN                    PB12

// SPI2 is on the APB1 bus whose clock runs at 36MHz. Divide to under 400kHz for init:
#define SDCARD_SPI_INITIALIZATION_CLOCK_DIVIDER 128
// Divide to under 25MHz for normal operation:
#define SDCARD_SPI_FULL_SPEED_CLOCK_DIVIDER 2

// Note, this is the same DMA channel as USART1_RX. Luckily we don't use DMA for USART Rx.
#define SDCARD_DMA_CHANNEL_TX               DMA1_Channel5
#define SDCARD_DMA_CHANNEL_TX_COMPLETE_FLAG DMA1_FLAG_TC5

// Performance logging for SD card operations:
// #define AFATFS_USE_INTROSPECTIVE_LOGGING

#define ACC
#define USE_ACC_LSM303DLHC

#define MAG
#define USE_MAG_HMC5883


#define USE_VCP
#define USE_USART1
#define USE_USART2
#define SERIAL_PORT_COUNT 3

#define USE_I2C
#define I2C_DEVICE (I2CDEV_1)

#define USE_ADC

#define ADC_INSTANCE                ADC1
#define ADC_AHB_PERIPHERAL          RCC_AHBPeriph_DMA1
#define ADC_DMA_CHANNEL             DMA1_Channel1

#define VBAT_ADC_GPIO               GPIOC
#define VBAT_ADC_GPIO_PIN           GPIO_Pin_0
#define VBAT_ADC_CHANNEL            ADC_Channel_6

#define CURRENT_METER_ADC_GPIO      GPIOC
#define CURRENT_METER_ADC_GPIO_PIN  GPIO_Pin_1
#define CURRENT_METER_ADC_CHANNEL   ADC_Channel_7

#define RSSI_ADC_GPIO               GPIOC
#define RSSI_ADC_GPIO_PIN           GPIO_Pin_2
#define RSSI_ADC_CHANNEL            ADC_Channel_8

#define EXTERNAL1_ADC_GPIO          GPIOC
#define EXTERNAL1_ADC_GPIO_PIN      GPIO_Pin_3
#define EXTERNAL1_ADC_CHANNEL       ADC_Channel_9

#define BLACKBOX
#define GPS
#define GTUNE
#define LED_STRIP
#define LED_STRIP_TIMER TIM16
#define TELEMETRY
#define SERIAL_RX
#define USE_SERVOS
#define USE_CLI

#define USE_SERIAL_4WAY_BLHELI_INTERFACE

#define USE_QUATERNION

#define TARGET_IO_PORTA 0xffff
#define TARGET_IO_PORTB 0xffff
#define TARGET_IO_PORTC 0xffff
#define TARGET_IO_PORTE 0xffff
#define TARGET_IO_PORTF 0xffff