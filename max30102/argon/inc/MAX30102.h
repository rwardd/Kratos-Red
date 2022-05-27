//MAX30102 Pulse detection header files
#ifndef MAX30102_H
#define MAX30102_H
#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>
#include <usb/usb_device.h>
#include <zephyr/types.h>
#include <drivers/i2c.h>


#define REG_INTR_ENABLE_1 0x02
#define REG_INTR_ENABLE_2 0x03
#define REG_FIFO_WR_PTR 0x04
#define REG_OVF_COUNTER 0x05
#define REG_FIFO_RD_PTR 0x06
#define REG_FIFO_DATA 0x07
#define REG_FIFO_CONFIG 0x08
#define REG_MODE_CONFIG 0x09
#define REG_SPO2_CONFIG 0x0A
#define REG_LED1_PA 0x0C
#define REG_LED2_PA 0x0D
#define REG_PILOT_PA 0x10
#define REG_MULTI_LED_CTRL1 0x11
#define REG_MULTI_LED_CTRL2 0x12
#define REG_TEMP_INTR 0x1F
#define REG_TEMP_FRAC 0x20
#define REG_TEMP_CONFIG 0x21
#define REG_PROX_INT_THRESH 0x30
#define REG_REV_ID 0xFE
#define REG_PART_ID 0xFF
void max30102_init();
void MAX30102_setup(const struct device* dev);
bool readFIFO(const struct device* dev);
uint32_t getIRVal(const struct device* dev);
uint32_t getRedVal(const struct device* dev);
uint8_t available();
void nextSample();
uint8_t check(const struct device* dev);
bool readFIFOv2(const struct device* dev, uint32_t* pun_red_led, uint32_t* pun_ir_led);
void reset(const struct device* dev);

#endif
