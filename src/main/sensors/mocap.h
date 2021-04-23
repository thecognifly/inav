#pragma once
#include <stdint.h>
#include <stdbool.h>

#include "common/time.h"
#include "drivers/sensor.h"

typedef struct {
    bool valid;
    bool reading;
    uint16_t counter;
    int16_t X;
    int16_t Y;
    int16_t Z;
    int16_t YAW;
} __mocap_received_values_t;


extern __mocap_received_values_t mocap_received_values_t;