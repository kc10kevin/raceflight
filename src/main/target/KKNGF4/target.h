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
#define TARGET_BOARD_IDENTIFIER "REVO"
#define CONFIG_START_FLASH_ADDRESS (0x08080000) //0x08080000 to 0x080A0000 (FLASH_Sector_8)
#define CONFIG_SERIALRX_PROVIDER SERIALRX_SBUS
#define CONFIG_BLACKBOX_DEVICE BLACKBOX_DEVICE_SDCARD
#define CONFIG_FEATURE_RX_SERIAL
#define CONFIG_FEATURE_ONESHOT125
#define CONFIG_MSP_PORT 2
#define CONFIG_RX_SERIAL_PORT 1

#define USBD_PRODUCT_STRING "KopterKontrolNG"

#define LED0 PB5
#define LED1 PB4
#define BEEPER PA8
#define BEEPER_INVERTED

#define INVERTER PC0 // PC0 used as inverter select GPIO
#define INVERTER_USART USART1

#define MPU6000_CS_PIN        PA4
#define MPU6000_SPI_INSTANCE  SPI1

#define ACC
#define USE_ACC_SPI_MPU6000
#define GYRO_MPU6000_ALIGN CW180_DEG

#define GYRO
#define USE_GYRO_SPI_MPU6000
#define ACC_MPU6000_ALIGN CW180_DEG

// MPU6000 interrupts
#define USE_MPU_DATA_READY_SIGNAL
//#define ENSURE_MPU_DATA_READY_IS_LOW
#define EXTI_CALLBACK_HANDLER_COUNT 1 // MPU data ready (mag disabled)
#define MPU_INT_EXTI PC4
#define USE_EXTI

#define BARO
#define USE_BARO_MS5611
#define MS5611_I2C_INSTANCE I2CDEV_1

#define USE_SDCARD

#define SDCARD_DETECT_INVERTED

#define SDCARD_DETECT_PIN                   PD2
#define SDCARD_DETECT_EXTI_LINE             EXTI_Line2
#define SDCARD_DETECT_EXTI_PIN_SOURCE       EXTI_PinSource2
#define SDCARD_DETECT_EXTI_PORT_SOURCE      EXTI_PortSourceGPIOD
#define SDCARD_DETECT_EXTI_IRQn             EXTI2_IRQn

#define SDCARD_SPI_INSTANCE                 SPI3
#define SDCARD_SPI_CS_PIN                   PB3

// SPI2 is on the APB1 bus whose clock runs at 84MHz. Divide to under 400kHz for init:
#define SDCARD_SPI_INITIALIZATION_CLOCK_DIVIDER 256 // 328kHz
// Divide to under 25MHz for normal operation:
#define SDCARD_SPI_FULL_SPEED_CLOCK_DIVIDER     4 // 21MHz

#define SDCARD_DMA_CHANNEL_TX               DMA1_Stream5
#define SDCARD_DMA_CHANNEL_TX_COMPLETE_FLAG DMA_FLAG_TCIF5
#define SDCARD_DMA_CLK                      RCC_AHB1Periph_DMA1
#define SDCARD_DMA_CHANNEL                  DMA_Channel_0

//#define USE_FLASHFS
//#define USE_FLASH_M25P16
//#define M25P16_CS_PIN         PB3
//#define M25P16_SPI_INSTANCE   SPI3

#define USABLE_TIMER_CHANNEL_COUNT 6  //  chane to 6

#define USE_VCP
#define VBUS_SENSING_PIN PC5
#define VBUS_SENSING_ENABLED

#define USE_USART1
#define USART1_RX_PIN PA10
#define USART1_TX_PIN PA9
#define USART1_AHB1_PERIPHERALS RCC_AHB1Periph_DMA2

#define USE_USART3
#define USART3_RX_PIN PB11
#define USART3_TX_PIN PB10

#define USE_USART6
#define USART6_RX_PIN PC7
#define USART6_TX_PIN PC6

#define SERIAL_PORT_COUNT 4 //VCP, USART1, USART3, USART6

#define USE_SPI

#define USE_SPI_DEVICE_1

#define USE_SPI_DEVICE_2
#define SPI2_NSS_PIN            PB12
#define SPI2_SCK_PIN            PB13
#define SPI2_MISO_PIN           PB14
#define SPI2_MOSI_PIN           PB15

#define USE_SPI_DEVICE_3
#define SPI3_NSS_PIN            PB3
#define SPI3_SCK_PIN            PC10
#define SPI3_MISO_PIN           PC11
#define SPI3_MOSI_PIN           PC12

#define USE_I2C
#define I2C_DEVICE (I2CDEV_1)  // PB6-SCL, PB7-SDA
#define USE_I2C_PULLUP
#define I2C1_SCL             PB6
#define I2C1_SDA             PB7

#define USE_ADC
#define BOARD_HAS_VOLTAGE_DIVIDER

#define VBAT_ADC_PIN           PC1
#define VBAT_ADC_CHANNEL       ADC_Channel_11

#define RSSI_ADC_GPIO_PIN      PC2
#define RSSI_ADC_CHANNEL       ADC_Channel_12

#define CURRENT_METER_ADC_PIN       PC3
#define CURRENT_METER_ADC_CHANNEL   ADC_Channel_13

//#define SENSORS_SET (SENSOR_ACC)

#define LED_STRIP
//#define LED_STRIP_TIMER TIM5

#define WS2811_GPIO_AF                  GPIO_AF_TIM5
#define WS2811_PIN                      PA0 
#define WS2811_TIMER                    TIM5
#define WS2811_TIMER_APB1_PERIPHERAL    RCC_APB1Periph_TIM5
#define WS2811_DMA_HANDLER_IDENTIFER    DMA1_ST2_HANDLER
#define WS2811_DMA_STREAM               DMA1_Stream2
#define WS2811_DMA_CHANNEL              DMA_Channel_6
#define WS2811_DMA_IRQ                  DMA1_Stream2_IRQn


//#define SOFTSERIAL
//#define GPS
#define BLACKBOX
//#define TELEMETRY
#define SERIAL_RX
#define AUTOTUNE
//#define USE_SERVOS
#define USE_QUAD_MIXER_ONLY
#define USE_CLI

#define SPEKTRUM_BIND
// USART2, PA3
#define BIND_PORT            GPIOB
#define BIND_PIN             Pin_11

#define USE_ESCSERIAL
#define ESCSERIAL_TIMER_TX_HARDWARE 0

#define USE_SERIAL_4WAY_BLHELI_INTERFACE

//#define USE_SERIAL_1WIRE
//#define ESC_COUNT 4
//#define S1W_TX_GPIO         GPIOB
//#define S1W_TX_PIN          GPIO_Pin_10
//#define S1W_RX_GPIO         GPIOB
//#define S1W_RX_PIN          GPIO_Pin_11

#define USE_QUATERNION

#define TARGET_IO_PORTA 0xffff
#define TARGET_IO_PORTB 0xffff
#define TARGET_IO_PORTC 0xffff
#define TARGET_IO_PORTD 0xffff