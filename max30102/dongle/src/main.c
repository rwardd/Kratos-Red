/**
 ************************************************************************
 * @file main.c
 * @author Ryan Ward
 * @date 31.03.2022
 * @brief Main function file for 4011 prac1
 **********************************************************************
 * */
#include <zephyr.h>
#include <sys/printk.h>
#include <device.h>
#include <drivers/pwm.h>
#include "ahu_led.h"
#include "ahu_ble.h"


 /*------------ COMPILE TIME THREAD DEFINITIONS ------------- */

K_THREAD_DEFINE(thread2_tid, BLE_STACK_SIZE,
	thread_ble_entry, NULL, NULL, NULL,
	BLE_PRIORITY, 0, 0);

K_THREAD_DEFINE(thread3_tid, CONT_STACK_SIZE,
	thread_base_read, NULL, NULL, NULL,
	CONT_PRIORITY, 0, 100);


K_THREAD_DEFINE(thread4_tid, BLINKY_STACK_SIZE,
	blinky_thread, NULL, NULL, NULL,
	BLINKY_PRIORITY, 0, 100);

/* Main Function */
void main(void) {
	const struct device* console_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
	if (usb_enable(NULL)) {
		return 1;
	}

}