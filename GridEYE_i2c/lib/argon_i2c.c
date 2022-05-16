




#include <zephyr.h>
#include <device.h>
#include <stdio.h>
#include <sys/printk.h>
#include <drivers/i2c.h>
#include <errno.h>
#include "argon_i2c.h"

#define AMG8833_I2C_ADDR 0x69


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
		}
		printk("%f, ", 0.25* grid[i]);
	}
	return;
}

void print_grid_float(float* grid) {
	for (int i = 0; i < 64; i++) {
		if (i % 8 == 0) {
			printk("\n");
		}
		printk("%f, ", grid[i]);
	}
	return;
}

void amg8833_thread_entry(void) {
    //k_msleep(2000);

    const struct device *i2cDev = DEVICE_DT_GET(DT_NODELABEL(i2c0));

    if (!device_is_ready(i2cDev)) {
		printk("I2C: Device is not ready.\n");
		return;
	}
	
	uint8_t data[64];
    for (int i = 0; i < sizeof(data); i++) {
        data[i] = 0;
    }

	uint16_t pixelGrid[64] = {0};
	float tempGrid[64] = {0};
    unsigned char pixelLowReg;
	printk("wegood\n");
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
		print_grid(pixelGrid);
		//print_grid_float(tempGrid);
		k_msleep(500);
		printk("\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");
		
	}

}
