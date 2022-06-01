




#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <stdio.h>
#include <sys/printk.h>
#include <drivers/i2c.h>
#include <errno.h>
#include "argon_i2c.h"

#include <drivers/gpio.h>
#include <usb/usb_device.h>
#include <drivers/uart.h>   
#include <sys_clock.h>
#include <timing/timing.h>

#define AMG8833_I2C_ADDR 0x69

#define TEMP_25 25
#define TEMP_26 26
#define TEMP_27 27
#define TEMP_28 28
#define TEMP_29 29
#define TEMP_30 30
#define TEMP_31 31
#define TEMP_32 32
#define TEMP_33 33


int16_t read_pixel_temp_reg(const struct device * dev, unsigned char reg) {
    //read reg from amg8833
    int16_t result, ret;
	int8_t buff, lsb, msb;	

	ret = i2c_reg_read_byte(dev, AMG8833_I2C_ADDR, reg, &buff);
	if (ret) {
		printk("Failure to read reg 0x%X", reg);
		return ret;
	}
	lsb = buff;
	ret = i2c_reg_read_byte(dev, AMG8833_I2C_ADDR, reg+1, &buff);
	if (ret) {
		printk("Failure to read reg 0x%X", reg+1);
		return ret;
	}
	msb = buff;

	result = (uint16_t)msb << 8 | lsb;

	return result;


}

void print_grid(uint16_t* grid) {
	for (int i = 0; i < 64; i++) {
		if (i % 8 == 0) {
			printk("\n");
			printk("%d,", i/8);
		}
		printk("%f,", 0.25* grid[i]);
	}
	return;
}

void print_grid_mapped(int8_t* grid) {
	for (int i = 0; i < 64; i++) {
		if (i % 8 == 0) {
			printk("\n");
			printk("%d,", i/8);
		}
		printk("%d,", grid[i]);
	}
	return;
}

uint8_t map_grid(float pixel) {
	if (pixel < TEMP_25) {
		return 0;
	} else if (pixel < TEMP_27 && pixel >= TEMP_25) {
		return 1;
	} else if (pixel < TEMP_29 && pixel >= TEMP_27) {
		return 2;
	} else if (pixel < TEMP_31 && pixel >= TEMP_29) {
		return 3;
	} else if (pixel < TEMP_33 && pixel >= TEMP_31) {
		return 4;
	} else if (pixel >= TEMP_33) {
		return 5;
	} 
				
}


void amg8833_thread_entry(void) {
    k_msleep(100);

    const struct device *i2cDev = DEVICE_DT_GET(DT_NODELABEL(i2c1));

	const struct device *console_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));

	if (usb_enable(NULL)) {
        return;
	}


    if (!device_is_ready(i2cDev)) {
		printk("I2C: Device is not ready.\n");
		return;
	}
	

	uint16_t pixelGrid[64] = {0};
	uint8_t mappedGrid[64] = {0};
    unsigned char pixelLowReg;
	float holder;
	
	while (1) {
    	for (int i = 0; i < MAX_PIXELS; i++) {
        	pixelLowReg = TEMPERATURE_REGISTER_START + (2 * i);
			pixelGrid[i] = read_pixel_temp_reg(i2cDev, pixelLowReg);
			//check if temp is -ve
			if (pixelGrid[i] & (1<<11)) {
				pixelGrid[i] &= ~(1 << 11);
				pixelGrid[i] = pixelGrid[i] * -1;
			}
			
    	}
		//print_grid(pixelGrid);
		//print_grid_float(tempGrid);
		for (int i = 0; i < MAX_PIXELS; i++) {
			mappedGrid[i] = map_grid(pixelGrid[i] * 0.25);
		}
		//print_grid_mapped(mappedGrid);
		print_grid(pixelGrid);
		k_msleep(10);
		//printk("\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");

		
	}

}
