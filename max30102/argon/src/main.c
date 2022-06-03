
#include <zephyr.h>
#include <device.h>
#include "bpm_spo2.h"
#include "argon_ble.h"
#include "argon_i2c.h"

#define BPM_STACK 1024
#define BPM_PRIO 2

#define BLE_CONNECT_STACK_SIZE 2048
#define BLE_CONNECT_PRIORITY -1

#define DATA_COMM_STACK 512
#define DATA_COMM_PRIO 3

#define RR_STACK_SIZE 512
#define RR_PRIO 1


K_THREAD_DEFINE(bpm_spo2, BPM_STACK, bpm_spo2_thread_start, NULL, NULL, NULL, BPM_PRIO, 0, 100);
K_THREAD_DEFINE(ble_entry, BLE_CONNECT_STACK_SIZE, scu_ble_connect_thread_entry, NULL, NULL, NULL, BLE_CONNECT_PRIORITY, 0, 50);
K_THREAD_DEFINE(data_comm_entry, DATA_COMM_STACK, update_data_thread, NULL, NULL, NULL, DATA_COMM_PRIO, 0, 120);

K_THREAD_DEFINE(argonTID, RR_STACK_SIZE,
		amg8833_thread_entry, NULL, NULL, NULL,
		RR_PRIO, 0, 75);


int main() {
    return 1;
}
