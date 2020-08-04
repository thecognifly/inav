/*
 * This file is part of iNavFlight.
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

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "platform.h"
#include "build/debug.h"

#include "common/axis.h"
#include "common/maths.h"

#include "drivers/system.h"
#include "drivers/time.h"
#include "drivers/io.h"
#include "drivers/exti.h"
#include "drivers/bus.h"

#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/accgyro_mpu.h"
#include "drivers/accgyro/accgyro_icm20689.h"

#if (defined(USE_GYRO_ICM20689) || defined(USE_ACC_ICM20689))

static uint8_t icm20689DeviceDetect(const busDevice_t *busDev)
{
    busSetSpeed(busDev, BUS_SPEED_INITIALIZATION);

    busWrite(busDev, MPU_RA_PWR_MGMT_1, ICM20689_BIT_RESET);

    uint8_t attemptsRemaining = 20;
    uint8_t in;
    do {
        delay(150);
        busRead(busDev, MPU_RA_WHO_AM_I, &in);
        switch (in) {
        case ICM20601_WHO_AM_I_CONST:
        case ICM20602_WHO_AM_I_CONST:
        case ICM20608G_WHO_AM_I_CONST:
        case ICM20689_WHO_AM_I_CONST:
            return true;
        }
    } while (attemptsRemaining--);

    return false;
}

static void icm20689AccInit(accDev_t *acc)
{
    acc->acc_1G = 512 * 4;
}

bool icm20689AccDetect(accDev_t *acc)
{
    acc->busDev = busDeviceOpen(BUSTYPE_ANY, DEVHW_ICM20689, acc->imuSensorToUse);
    if (acc->busDev == NULL) {
        return false;
    }

    mpuContextData_t * ctx = busDeviceGetScratchpadMemory(acc->busDev);
    if (ctx->chipMagicNumber != 0x50D1) {
        return false;
    }

    acc->initFn = icm20689AccInit;
    acc->readFn = mpuAccReadScratchpad;

    return true;
}

static void icm20689AccAndGyroInit(gyroDev_t *gyro)
{
    busDevice_t * busDev = gyro->busDev;
    const gyroFilterAndRateConfig_t * config = mpuChooseGyroConfig(gyro->lpf, 1000000 / gyro->requestedSampleIntervalUs);
    gyro->sampleRateIntervalUs = 1000000 / config->gyroRateHz;

    gyroIntExtiInit(gyro);

    busSetSpeed(busDev, BUS_SPEED_INITIALIZATION);

    busWrite(busDev, MPU_RA_PWR_MGMT_1, ICM20689_BIT_RESET);
    delay(100);
    busWrite(busDev, MPU_RA_SIGNAL_PATH_RESET, 0x03);
    delay(100);
    busWrite(busDev, MPU_RA_PWR_MGMT_1, INV_CLK_PLL);
    delay(15);
    busWrite(busDev, MPU_RA_GYRO_CONFIG, INV_FSR_2000DPS << 3); // ADDR 0x1B: 00011000
    delay(15);
    busWrite(busDev, MPU_RA_ACCEL_CONFIG, INV_FSR_16G << 3); // ADDR 0x1C: 00011000 (±2g (00), ±4g (01), ±8g (10), ±16g (11))
    delay(15);
    busWrite(busDev, MPU_RA_CONFIG, config->gyroConfigValues[0]); // ADDR 0x1A: DLPF_CFG[2:0] => gyro at BW 250Hz
    delay(15);
    
    // If MPU_RA_FF_THR is zero by default, if means the acc is using A_DLPF_CFG = 0 and BW = 218.1Hz
    // busWrite(busDev, MPU_RA_FF_THR, 1 << 3); // ADDR 0x1D: ACCEL_FCHOICE_B=1, disables A_DLPF_CFG  and BW = 1046Hz
    // delay(15);

    busWrite(busDev, MPU_RA_SMPLRT_DIV, config->gyroConfigValues[1]); // ADDR 0x19:
                                                                      // Get Divider Drops
                                                                      // SAMPLE_RATE = 1kHz / (1 + SMPLRT_DIV)
                                                                      // SAMPLE_RATE = 500Hz
    delay(100);

    // Data ready interrupt configuration
    busWrite(busDev, MPU_RA_INT_PIN_CFG, 0x10);  // INT_ANYRD_2CLEAR, BYPASS_EN

    delay(15);

#ifdef USE_MPU_DATA_READY_SIGNAL
    busWrite(busDev, MPU_RA_INT_ENABLE, 0x01); // RAW_RDY_EN interrupt enable
#endif
}

bool icm20689GyroDetect(gyroDev_t *gyro)
{
    gyro->busDev = busDeviceInit(BUSTYPE_ANY, DEVHW_ICM20689, gyro->imuSensorToUse, OWNER_MPU);
    if (gyro->busDev == NULL) {
        return false;
    }

    if (!icm20689DeviceDetect(gyro->busDev)) {
        busDeviceDeInit(gyro->busDev);
        return false;
    }

    // Magic number for ACC detection to indicate that we have detected MPU6000 gyro
    mpuContextData_t * ctx = busDeviceGetScratchpadMemory(gyro->busDev);
    ctx->chipMagicNumber = 0x50D1;

    gyro->initFn = icm20689AccAndGyroInit;
    gyro->readFn = mpuGyroReadScratchpad;
    gyro->intStatusFn = gyroCheckDataReady;
    gyro->temperatureFn = mpuTemperatureReadScratchpad;
    gyro->scale = 1.0f / 16.4f;     // 16.4 dps/lsb scalefactor

    return true;
}

#endif
