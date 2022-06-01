/*
 * @file oslib/scu_ble/scu_ble.h
 * @author Thomas Salpietro 45822490
 * @date 31/03/2022
 * @brief ble scu driver header file
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef SCU_BLE_H
#define SCU_BLE_H
#include <zephyr.h>
#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <sys/printk.h>
#include <sys/byteorder.h>


#include <drivers/gpio.h>

#include <settings/settings.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>
#include <bluetooth/services/bas.h>
#include <bluetooth/services/hrs.h>
#include "bpm_spo2.h"

void scu_ble_connect_thread_entry(void);
void update_data_thread(void);

extern struct k_fifo bpm_spo2_fifo;
extern int16_t row1[8];
extern int16_t row2[8];
extern int16_t row3[8];
extern int16_t row4[8];
extern int16_t row5[8];
extern int16_t row6[8];
extern int16_t row7[8];
extern int16_t row8[8];

//gatt characteristic user values

#endif
