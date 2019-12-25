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
#include <stdint.h>

#include "platform.h"
#include "drivers/bus.h"
#include "drivers/io.h"

#include "drivers/dma.h"
#include "drivers/timer.h"
#include "drivers/timer_def.h"

#include "drivers/pinio.h"

#include "drivers/pwm_mapping.h"


// BUSDEV_REGISTER_SPI_TAG(busdev_mpu6000,     DEVHW_MPU6000,      MPU6000_SPI_BUS,    MPU6000_CS_PIN,     MPU6000_EXTI_PIN,       0,  DEVFLAGS_NONE);
// BUSDEV_REGISTER_SPI_TAG(busdev_mpu6500,     DEVHW_MPU6500,      MPU6500_SPI_BUS,    MPU6500_CS_PIN,     MPU6500_EXTI_PIN,       1,  DEVFLAGS_NONE);

// BUSDEV_REGISTER_SPI(    busdev_max7456,     DEVHW_MAX7456,      MAX7456_SPI_BUS,    MAX7456_CS_PIN,     NONE,                       DEVFLAGS_USE_RAW_REGISTERS);

const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {

    //DEF_TIM(TIM2,  CH2,   PB3, TIM_USE_CAMERA_CONTROL,     0, 0), // USE FOR CAMERA CONTROL

    DEF_TIM(TIM4,  CH1,  PB6, TIM_USE_MC_MOTOR,               0, 0), // D1-ST0                   MOTOR1
    DEF_TIM(TIM4,  CH2,  PB7, TIM_USE_MC_MOTOR,               0, 0), // D1-ST3                   MOTOR2
    DEF_TIM(TIM4,  CH3,  PB8, TIM_USE_MC_MOTOR,               0, 0), // D1-ST7                   MOTOR3
    DEF_TIM(TIM4,  CH4,  PB9, TIM_USE_MC_MOTOR,               0, 0), // NONE  TIM4_UP_D1-ST6     MOTOR4
    DEF_TIM(TIM5,  CH2,  PA1, TIM_USE_MC_MOTOR,               0, 0), // D1-ST4                   MOTOR5
    DEF_TIM(TIM8,  CH3,  PC8, TIM_USE_MC_MOTOR,               0, 0), // D2-ST2/D2-ST4            MOTOR6
    DEF_TIM(TIM8,  CH4,  PC9, TIM_USE_MC_MOTOR,               0, 0), // D2-ST7                   MOTOR7

    DEF_TIM(TIM3,  CH4, PB1, TIM_USE_MC_MOTOR | TIM_USE_LED,  0, 0), // D1-ST2                   LED/MOTOR5


};

const int timerHardwareCount = sizeof(timerHardware) / sizeof(timerHardware[0]);