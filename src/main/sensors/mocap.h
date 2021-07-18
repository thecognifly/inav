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
    timeUs_t lastUpdateTime;
} __mocap_received_values_t;


/*this is the container that contains the desired position to pursue, 
using mocap as a position feedback. The desired position is in cm.*/
typedef struct{
    bool active;
    float x;
    float y;
    float z;
    timeUs_t lastUpdateTime;
} __mocap_desired_pos_t; 


extern __mocap_received_values_t mocap_received_values_t;
extern __mocap_desired_pos_t mocap_desired_pos_t;