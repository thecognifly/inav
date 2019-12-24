/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once
#define TARGET_BOARD_IDENTIFIER "CLR7"
#define USBD_PRODUCT_STRING "CLRACINGF7"

#define USE_TARGET_CONFIG

/*** Indicators ***/
#define LED0                PB0
#define USE_BEEPER
#define BEEPER              PB4
#define BEEPER_INVERTED

/*** IMU sensors ***/
#define USE_EXTI
#define USE_ACC
#define USE_GYRO

// We use dual IMU sensors, they have to be described in the target file
#define USE_TARGET_IMU_HARDWARE_DESCRIPTORS
#define USE_MPU_DATA_READY_SIGNAL
#define USE_DUAL_GYRO

// MPU6000
#define USE_GYRO_MPU6000
#define USE_ACC_MPU6000

// #define GYRO_1_CS_PIN           PA4
#define MPU6000_CS_PIN          PA4 //GYRO_1_CS_PIN
// #define GYRO_1_SPI_INSTANCE     SPI1
#define MPU6000_SPI_BUS         BUS_SPI1 //GYRO_1_SPI_INSTANCE
// #define GYRO_1_EXTI_PIN         PC4
#define MPU6000_EXTI_PIN        PC4 //GYRO_1_EXTI_PIN
// #define GYRO_1_ALIGN            CW0_DEG
#define GYRO_MPU6000_ALIGN      CW0_DEG //GYRO_1_ALIGN
#define ACC_MPU6000_ALIGN       CW0_DEG //GYRO_1_ALIGN?? 

// ICM20602 - handled by MPU6500 driver
#define USE_GYRO_MPU6500
#define USE_ACC_MPU6500

// #define GYRO_2_CS_PIN            PC13
#define MPU6500_CS_PIN          PC13 //GYRO_2_CS_PIN
// #define GYRO_2_SPI_INSTANCE     SPI1
#define MPU6500_SPI_BUS         BUS_SPI1 //GYRO_2_SPI_INSTANCE
// #define GYRO_2_EXTI_PIN         PC14
#define MPU6500_EXTI_PIN        PC14 //GYRO_2_EXTI_PIN
// #define GYRO_2_ALIGN            CW90_DEG
#define GYRO_MPU6500_ALIGN      CW90_DEG //GYRO_2_ALIGN
#define ACC_MPU6500_ALIGN       CW90_DEG //GYRO_2_ALIGN??

/*** SPI/I2C bus ***/
#define USE_SPI

#define USE_SPI_DEVICE_1
#define SPI1_NSS_PIN             PA4
#define SPI1_SCK_PIN             PA5
#define SPI1_MISO_PIN            PA6
#define SPI1_MOSI_PIN            PA7

#define USE_SPI_DEVICE_2
#define SPI2_NSS_PIN             PB12
#define SPI2_SCK_PIN             PB13
#define SPI2_MISO_PIN            PB14
#define SPI2_MOSI_PIN            PB15

#define USE_SPI_DEVICE_3
#define SPI3_NSS_PIN             PA15
#define SPI3_SCK_PIN             PC10
#define SPI3_MISO_PIN            PC11
#define SPI3_MOSI_PIN            PB5

#define USE_I2C
#define USE_I2C_DEVICE_2       // External I2C
#define I2C_DEVICE              (I2CDEV_2)
#define I2C2_SCL                NONE        // PB10 (UART3_TX)
#define I2C2_SDA                NONE        // PB11 (UART3_RX)

/*** Onboard flash ***/
#define USE_FLASHFS
#define USE_FLASH_M25P16
// #define FLASH_CS_PIN            PB12
#define M25P16_CS_PIN           PB12
// #define FLASH_SPI_INSTANCE      SPI2
#define M25P16_SPI_BUS          BUS_SPI2
// #define USE_FLASH_W25M
// #define USE_FLASH_W25N01G       // 1G NAND flash support


/*** OSD ***/
// #define USE_OSD
#define USE_MAX7456
// #define MAX7456_SPI_INSTANCE    SPI3
#define MAX7456_SPI_BUS         BUS_SPI3
// #define MAX7456_SPI_CS_PIN      PA15
#define MAX7456_CS_PIN          PA15

/*** Serial ports ***/
#define USE_VCP

#define USE_UART1
#define UART1_RX_PIN            PA10
#define UART1_TX_PIN            PA9

#define USE_UART2
#define UART2_RX_PIN            PA3
#define UART2_TX_PIN            PA2

#define USE_UART3
#define UART3_RX_PIN            PB11
#define UART3_TX_PIN            PB10

#define USE_UART4
#define UART4_RX_PIN            PA1
#define UART4_TX_PIN            PA0

#define USE_UART5
#define UART5_RX_PIN            PD2
#define UART5_TX_PIN            PC12

#define USE_UART6
#define UART6_RX_PIN            PC7
#define UART6_TX_PIN            PC6

#define SERIAL_PORT_COUNT       7 //VCP, USART1, USART2,USART3,USART4,USART5,USART6

// /*** BARO & MAG ***/
// #define USE_BARO
// #define BARO_I2C_BUS            BUS_I2C1
// #define USE_BARO_BMP280
// #define USE_BARO_MS5611

// #define USE_MAG
// #define USE_MAG_HMC5883
// #define USE_MAG_QMC5883
// #define MAG_I2C_INSTANCE         (I2CDEV_2)

/*** ADC ***/
#define USE_ADC
// #define VBAT_ADC_PIN            PC2
#define ADC_CHANNEL_1_PIN               PC2 //VBAT_ADC_PIN
// #define CURRENT_METER_ADC_PIN   PC1
#define ADC_CHANNEL_2_PIN               PC1 //CURRENT_METER_ADC_PIN
// #define RSSI_ADC_PIN            PC3
#define ADC_CHANNEL_3_PIN               PC3 //RSSI_ADC_PIN

#define VBAT_ADC_CHANNEL                ADC_CHN_1
#define CURRENT_METER_ADC_CHANNEL       ADC_CHN_2
#define RSSI_ADC_CHANNEL                ADC_CHN_3


// #define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
// #define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_ADC


/*-------------ESCs----------------*/
#define USE_ESC_SENSOR
#define USE_SERIAL_4WAY_BLHELI_INTERFACE
#define USE_ESCSERIAL
#define ESCSERIAL_TIMER_TX_PIN  PB0  // (HARDARE=0)

// #define CURRENT_METER_SCALE_DEFAULT 250  // 3.3/120A  = 25mv/A
#define CURRENT_METER_SCALE     250


// *************** PINIO ***************************
#define USE_PINIO
#define USE_PINIOBOX
#define PINIO1_PIN              PA14  // VTX power switcher

#define HARDWARE_BIND_PLUG
#define BINDPLUG_PIN            PB2

/*** Default settings ***/
#define ENABLE_BLACKBOX_LOGGING_ON_SPIFLASH_BY_DEFAULT
// #define DEFAULT_FEATURES        (FEATURE_OSD)
// #define SERIALRX_UART           SERIAL_PORT_UART5
#define SERIALRX_UART           SERIAL_PORT_USART5
#define SERIALRX_PROVIDER       SERIALRX_SBUS
// #define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL
#define DEFAULT_RX_TYPE         RX_TYPE_SERIAL


// #define ENABLE_DSHOT_DMAR       DSHOT_DMAR_ON
#define MAX_PWM_OUTPUT_PORTS       6
#define USE_DSHOT
#define USE_SERIALSHOT
#define USE_ESC_SENSOR


#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         0xffff
#define TARGET_IO_PORTD         (BIT(2))

#define USABLE_TIMER_CHANNEL_COUNT      9
#define USED_TIMERS             ( TIM_N(2) | TIM_N(3) | TIM_N(4) | TIM_N(5)  | TIM_N(8)   )