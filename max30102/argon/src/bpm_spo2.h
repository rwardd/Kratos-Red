#ifndef BPM_SP02_H
#define BPM_SP02_H


#include "MAX30102.h"
#include "algorithm.h"



int bpm_spo2_thread_start();

struct max30102_data {
    void* fifo_reserved;
    uint16_t heartRate;
    uint8_t spo2;
};

#endif