/**
 ************************************************************************
 * @file ahu_ble.h
 * @author Ryan Ward
 * @date 31.03.2022
 * @brief Contains Macros and definitions for the AHU ble drivers
 **********************************************************************
 * */

#ifndef AHU_BLE
#define AHU_BLE
#include <zephyr/types.h>
#include <stddef.h>
#include <errno.h>
#include <zephyr.h>
#include <sys/printk.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>
#include <sys/byteorder.h>
#include <usb/usb_device.h>
#include <drivers/uart.h> 
#include <drivers/gpio.h>
#include <sys/util.h>


 /* Search, Connect and Gatt Discover */
 /* Bluetooth Thread stack size and priority */
#define BLE_STACK_SIZE 2048
#define BLE_PRIORITY -1

/* Bluetooth send stack size and priority */
#define READ_STACK_SIZE 2048
#define READ_PRIORITY 1

/* JSON continuous output thread stack size */
/* and priority */
#define CONT_STACK_SIZE 1024
#define CONT_PRIORITY 3

/* Buffer of 16 bit ints */
#define UUID_BUFFER_SIZE 16


/* Function Signatures */
void thread_ble_entry(void);


void thread_base_read(void);

#endif