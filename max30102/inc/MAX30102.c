//MAX30102 Pulse detection sensor driver
#include "MAX30102.h"

#define MAX30102_MODE_MULTILED 0x07
#define MAX30102_ADDRESS_WRITE 0xAE
#define MAX30102_MODECONFIG 0x09
#define MAX30102_MODE_MASK 0xF8
#define MAX30102_DIETEMPCONFIG 0x21
#define MAX30102_ADDRESS 0x57
#define MAX30102_PARTID 0xFF
#define MAX30102_PARTID_VALUE 0x15
#define MAX30102_SHUTDOWN_MASK 0x7F
#define MAX30102_SHUTDOWN 0x80
#define MAX30102_WAKEUP 0x00
#define MAX30102_RESET_MASK 0xBF
#define MAX30102_RESET 0x40
#define MAX30102_FIFOCONFIG 0x08
#define MAX30102_SAMPLEAVG_MASK 0b00011111
#define MAX30102_SAMPLEAVG_4 0x40
#define MAX30102_SAMPLEAVG_1 0x00
#define MAX30102_ROLLOVER_MASK 0xEF
#define MAX30102_ROLLOVER_ENABLE 0x10
#define MAX30102_ROLLOVER_DISABLE 0x00
#define MAX30102_MODE_REDIRONLY 0x03
#define MAX30102_PARTICLECONFIG 0x0A
#define MAX30102_ADCRANGE_MASK 0x9F
#define MAX30102_ADCRANGE_4096 0x20
#define MAX30102_ADCRANGE_61384 0x60
#define MAX30102_SAMPLERATE_MASK 0xE3
#define MAX30102_SAMPLERATE_100 0x04
#define MAX30102_SAMPLERATE_400 0x0C
#define MAX30102_PULSEWIDTH_MASK 0xFC
#define MAX30102_PULSEWIDTH_411 0x03

#define MAX30102_LED1_PULSEAMP 0x0C
#define MAX30102_LED2_PULSEAMP 0x0D 
#define MAX30102_LED3_PULSEAMP 0x0E
#define MAX30102_LED_PROX_AMP 0x10
#define MAX30102_MULTILEDCONFIG1 0x11 
#define MAX30102_MULTILEDCONFIG2 0x12

#define MAX30105_SLOT1_MASK 0xF8
#define MAX30105_SLOT2_MASK 0x8F

#define SLOT_RED_LED 0x01 
#define SLOT_IR_LED 0x02

#define MAX30102_FIFOREADPTR 0x06 
#define MAX30102_FIFOWRITEPTR 0x04 
#define MAX30102_FIFOOVERFLOW 0x05
#define MAX30102_FIFODATA 0x07 

#define MAX30102_FIFOWRITEPTR 0x04
#define MAX30102_FIFOREADPTR 0x06

#define MAX30102_INTSTAT1 0x00 
#define MAX30102_INTSTAT2 0x01
#define REG_INTR_ENABLE_1 0x02
#define REG_INTR_ENABLE_2 0x03

#define MAX30102_INT_A_FULL_MASK (uint8_t)~0b100000000
#define MAX30102_INT_A_FULL_ENABLE 0x80

#define STORAGE_SIZE 4
#define I2C_BUFFER_LENGTH 32


uint32_t i2c_cfg = I2C_SPEED_SET(I2C_SPEED_FAST) | I2C_MODE_MASTER;
void bitMask(const struct device* dev_i2c, uint8_t reg, uint8_t mask, uint8_t thing);
uint8_t readPartID(const struct device* dev);

//struct for storing sensor state

typedef struct Record {
    uint32_t red[STORAGE_SIZE];
    uint32_t IR[STORAGE_SIZE];
    uint8_t head;
    uint8_t tail;
} sense_struct;

sense_struct sense;

uint8_t activeLEDS;

// Initialise the Sensor and I2C device
void max30102_init(const struct device* i2c_dev) {



    //configure for master mode
    i2c_configure(i2c_dev, i2c_cfg);

    if (!device_is_ready(i2c_dev)) {
        printk("Could not get I2C device\n");
        return;
    }

    //check the correct device is found
    if (readPartID(i2c_dev) != MAX30102_PARTID_VALUE) {
        printk("Unable to find device\n");
    }
    else {
        printk("Device Found!\n");
    }

}

// Read Part ID
uint8_t readPartID(const struct device* dev) {
    uint8_t partid = 0;
    int ret = i2c_reg_read_byte(dev, MAX30102_ADDRESS, MAX30102_PARTID, &partid);
    if (ret) {
        printk("Unable to read deviceID register\n");
    }
    return partid;
}



/*============== Begin Library Functions ====================*/

void shutdown(const struct device* dev) {
    bitMask(dev, MAX30102_MODECONFIG, MAX30102_SHUTDOWN_MASK, MAX30102_SHUTDOWN);
}

void wakeup(const struct device* dev) {
    bitMask(dev, MAX30102_MODECONFIG, MAX30102_SHUTDOWN_MASK, MAX30102_WAKEUP);
}

void softReset(const struct device* dev) {
    bitMask(dev, MAX30102_MODECONFIG, MAX30102_RESET_MASK, MAX30102_RESET);

    uint32_t start_time = k_uptime_get_32();
    uint8_t response = 1;
    while (k_uptime_get_32() - start_time < 100) {
        i2c_reg_read_byte(dev, MAX30102_ADDRESS, MAX30102_MODECONFIG, &response);
        if ((response & MAX30102_RESET) == 0) {
            break;
        }
        k_sleep(K_MSEC(1));
    }

}

void setFIFOAverage(const struct device* dev, uint8_t numberOfSamples) {
    bitMask(dev, MAX30102_FIFOCONFIG, MAX30102_SAMPLEAVG_MASK, numberOfSamples);

}


void enableFIFORollover(const struct device* dev) {
    bitMask(dev, MAX30102_FIFOCONFIG, MAX30102_ROLLOVER_MASK, MAX30102_ROLLOVER_ENABLE);
}

void setLEDMode(const struct device* dev, uint8_t mode) {
    bitMask(dev, MAX30102_MODECONFIG, MAX30102_MODE_MASK, mode);

}

void setADCRange(const struct device* dev, uint8_t adcRange) {
    bitMask(dev, MAX30102_PARTICLECONFIG, MAX30102_ADCRANGE_MASK, adcRange);

}

void setSampleRate(const struct device* dev, uint8_t sampleRate) {
    bitMask(dev, MAX30102_PARTICLECONFIG, MAX30102_SAMPLERATE_MASK, sampleRate);
}

void setPulseWidth(const struct device* dev, uint8_t pulseWidth) {
    bitMask(dev, MAX30102_PARTICLECONFIG, MAX30102_PULSEWIDTH_MASK, pulseWidth);

}

void setPulseAmplitudeRed(const struct device* dev, uint8_t powerLevel) {
    if (i2c_reg_write_byte(dev, MAX30102_ADDRESS, MAX30102_LED1_PULSEAMP, powerLevel)) {
        printk("Write failed :( \n");
    }
}

void setPulseAmplitudeIR(const struct device* dev, uint8_t powerLevel) {
    if (i2c_reg_write_byte(dev, MAX30102_ADDRESS, MAX30102_LED2_PULSEAMP, powerLevel)) {
        printk("Write failed :( \n");
    }
}

void setPulseAmplitudeGreen(const struct device* dev, uint8_t powerLevel) {
    if (i2c_reg_write_byte(dev, MAX30102_ADDRESS, MAX30102_LED3_PULSEAMP, powerLevel)) {
        printk("Write failed :( \n");
    }
}

void setPulseAmplitudeProximity(const struct device* dev, uint8_t powerLevel) {
    if (i2c_reg_write_byte(dev, MAX30102_ADDRESS, MAX30102_LED_PROX_AMP, powerLevel)) {
        printk("Write failed :( \n");
    }
}

void enableSlot(const struct device* dev, uint8_t slot, uint8_t device) {
    switch (slot) {
    case (1):
        bitMask(dev, MAX30102_MULTILEDCONFIG1, MAX30105_SLOT1_MASK, device);
        break;
    case (2):
        bitMask(dev, MAX30102_MULTILEDCONFIG1, MAX30105_SLOT2_MASK, device << 4);
        break;
    default:
        //should never be here
        break;
    }

}

void configureFIFO(const struct device* dev) {
    int ret = i2c_reg_write_byte(dev, MAX30102_ADDRESS, MAX30102_FIFOCONFIG, 0x0f);
    if (ret) {
        printk("Error configuring FIFO\n");
    }
}

void clearFIFO(const struct device* dev) {
    int ret = i2c_reg_write_byte(dev, MAX30102_ADDRESS, MAX30102_FIFOWRITEPTR, 0);
    ret += i2c_reg_write_byte(dev, MAX30102_ADDRESS, MAX30102_FIFOOVERFLOW, 0);
    ret += i2c_reg_write_byte(dev, MAX30102_ADDRESS, MAX30102_FIFOREADPTR, 0);
    if (ret) {
        printk("Error clearing FIFO r/w pointers (ERR: %i)\n", ret);

    }

}

void setInterrupts(const struct device* dev) {
    int ret = i2c_reg_write_byte(dev, MAX30102_ADDRESS, REG_INTR_ENABLE_1, 0xc0);
    if (ret) {
        printk("Error setting interrupt 1\n");
    }
    ret = i2c_reg_write_byte(dev, MAX30102_ADDRESS, REG_INTR_ENABLE_2, 0x00);
    if (ret) {
        printk("Error setting interrupt 2\n");
    }

}

void enableAFull(const struct device* dev) {
    int ret = i2c_reg_write_byte(dev, MAX30102_ADDRESS, REG_INTR_ENABLE_1, 0x40);
    if (ret) {
        printk("Error setting interrupt 1\n");
    }
}

void setFIFOAlmostFull(const struct device* dev, uint8_t samples) {
    bitMask(dev, MAX30102_FIFOCONFIG, 0xF0, samples);
}

bool maxim_max30102_init(const struct device* dev)
/**
* \brief        Initialize the MAX30102
* \par          Details
*               This function initializes the MAX30102
*
* \param        None
*
* \retval       true on success
*/
{
    if (i2c_reg_write_byte(dev, MAX30102_ADDRESS, REG_INTR_ENABLE_1, 0xc0)) // INTR setting
        return false;
    if (i2c_reg_write_byte(dev, MAX30102_ADDRESS, REG_INTR_ENABLE_2, 0x00))
        return false;
    if (i2c_reg_write_byte(dev, MAX30102_ADDRESS, REG_FIFO_WR_PTR, 0x00))  //FIFO_WR_PTR[4:0]
        return false;
    if (i2c_reg_write_byte(dev, MAX30102_ADDRESS, REG_OVF_COUNTER, 0x00))  //OVF_COUNTER[4:0]
        return false;
    if (i2c_reg_write_byte(dev, MAX30102_ADDRESS, REG_FIFO_RD_PTR, 0x00))  //FIFO_RD_PTR[4:0]
        return false;
    if (i2c_reg_write_byte(dev, MAX30102_ADDRESS, REG_FIFO_CONFIG, 0x0f))  //sample avg = 1, fifo rollover=false, fifo almost full = 17
        return false;
    if (i2c_reg_write_byte(dev, MAX30102_ADDRESS, REG_MODE_CONFIG, 0x03))   //0x02 for Red only, 0x03 for SpO2 mode 0x07 multimode LED
        return false;
    if (i2c_reg_write_byte(dev, MAX30102_ADDRESS, REG_SPO2_CONFIG, 0x27))  // SPO2_ADC range = 4096nA, SPO2 sample rate (100 Hz), LED pulseWidth (400uS)
        return false;

    if (i2c_reg_write_byte(dev, MAX30102_ADDRESS, REG_LED1_PA, 0x24))   //Choose value for ~ 7mA for LED1
        return false;
    if (i2c_reg_write_byte(dev, MAX30102_ADDRESS, REG_LED2_PA, 0x24))   // Choose value for ~ 7mA for LED2
        return false;
    if (i2c_reg_write_byte(dev, MAX30102_ADDRESS, REG_PILOT_PA, 0x7f))   // Choose value for ~ 25mA for Pilot LED
        return false;
    return true;
}


void MAX30102_setup(const struct device* dev) {
    uint8_t dummyVal;
    i2c_reg_read_byte(dev, MAX30102_ADDRESS, 0, &dummyVal);
    //softReset(dev);
    uint8_t ledBrightness = 36; //Options: 0=Off to 255=50mA
    uint8_t heartRateRedLED = 36;

    // uint8_t sampleAverage = 4; //Options: 1, 2, 4, 8, 16, 32
    // uint8_t ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
    // uint8_t sampleRate = 100; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
    // int pulseWidth = 411; //Options: 69, 118, 215, 411
    // int adcRange = 4096; //Options: 2048, 4096, 8192, 16384
    maxim_max30102_init(dev);
    //     setFIFOAverage(dev, MAX30102_SAMPLEAVG_1);
    //     enableFIFORollover(dev);

    //     //     setInterrupts(dev);

    //    // configureFIFO(dev);

    //     setLEDMode(dev, MAX30102_MODE_REDIRONLY);
    //     activeLEDS = 2;
    //     setADCRange(dev, MAX30102_ADCRANGE_4096);
    //     setSampleRate(dev, MAX30102_SAMPLERATE_100);// < -SPO2
    // //    //     //setSampleRate(dev, MAX30102_SAMPLERATE_400); //<- Heart rate
    //     setPulseWidth(dev, MAX30102_PULSEWIDTH_411);
    //     setPulseAmplitudeRed(dev, heartRateRedLED);
    //     setPulseAmplitudeIR(dev, ledBrightness);
    //     enableSlot(dev, 1, SLOT_RED_LED);
    //     enableSlot(dev, 2, SLOT_IR_LED);
    //     clearFIFO(dev);


        //enableAFull(dev);
        //setFIFOAlmostFull(dev, 1);



}



uint8_t available() {
    int8_t numberOfSamples = sense.head - sense.tail;
    if (numberOfSamples < 0) {
        numberOfSamples += STORAGE_SIZE;
    }
    return numberOfSamples;
}

uint8_t getReadPointer(const struct device* dev) {
    uint8_t rptr = 0;
    int ret = i2c_reg_read_byte(dev, MAX30102_ADDRESS, MAX30102_FIFOREADPTR, &rptr);
    if (ret) {
        printk("Failed to read device read pointer\n");
    }
    return rptr;
}

uint8_t getWritePointer(const struct device* dev) {
    uint8_t wptr = 0;
    int ret = i2c_reg_read_byte(dev, MAX30102_ADDRESS, MAX30102_FIFOWRITEPTR, &wptr);
    if (ret) {
        printk("Failed to read device write pointer\n");
    }
    return wptr;
}

uint8_t readINT1(const struct device* dev) {
    uint8_t status;
    int ret = i2c_reg_read_byte(dev, MAX30102_ADDRESS, MAX30102_INTSTAT1, &status);
    if (ret) {
        printk("Error reading INT1 Status \n");
    }
    return status;
}

uint8_t readINT2(const struct device* dev) {
    uint8_t status;
    int ret = i2c_reg_read_byte(dev, MAX30102_ADDRESS, MAX30102_INTSTAT2, &status);
    if (ret) {
        printk("Error reading INT1 Status \n");
    }
    return status;
}

bool readFIFO(const struct device* dev) {
    uint32_t redLED = 0;
    uint32_t irLED = 0;
    uint8_t ch_i2c_data[6];
    uint32_t un_temp;
    readINT1(dev);
    readINT2(dev);
    int ret = i2c_burst_read(dev, MAX30102_ADDRESS, MAX30102_FIFODATA, ch_i2c_data, 6);
    if (ret) {
        printk("Unable to read FIFO data (ERR: %i)\n", ret);
    }
    un_temp = (unsigned char)ch_i2c_data[0];
    un_temp <<= 16;
    redLED += un_temp;
    un_temp = (unsigned char)ch_i2c_data[1];
    un_temp <<= 8;
    redLED += un_temp;
    un_temp = (unsigned char)ch_i2c_data[2];
    redLED += un_temp;

    un_temp = (unsigned char)ch_i2c_data[3];
    un_temp <<= 16;
    irLED += un_temp;
    un_temp = (unsigned char)ch_i2c_data[4];
    un_temp <<= 8;
    irLED += un_temp;
    un_temp = (unsigned char)ch_i2c_data[5];
    irLED += un_temp;
    redLED &= 0x03FFFF;  //Mask MSB [23:18]
    irLED &= 0x03FFFF;

    sense.red[sense.head] = redLED;
    sense.IR[sense.head] = irLED;
    return true;
}

bool readFIFOv2(const struct device* dev, uint32_t* pun_red_led, uint32_t* pun_ir_led) {
    uint32_t un_temp;
    unsigned char uch_temp;
    *pun_red_led = 0;
    *pun_ir_led = 0;
    char ach_i2c_data[6];

    //read and clear status register
    readINT1(dev);
    readINT2(dev);

    int ret = i2c_burst_read(dev, MAX30102_ADDRESS, MAX30102_FIFODATA, ach_i2c_data, 6);
    if (ret) {
        printk("Unable to read FIFO data (ERR: %i)\n", ret);
    }
    un_temp = (unsigned char)ach_i2c_data[0];
    un_temp <<= 16;
    *pun_red_led += un_temp;
    un_temp = (unsigned char)ach_i2c_data[1];
    un_temp <<= 8;
    *pun_red_led += un_temp;
    un_temp = (unsigned char)ach_i2c_data[2];
    *pun_red_led += un_temp;

    un_temp = (unsigned char)ach_i2c_data[3];
    un_temp <<= 16;
    *pun_ir_led += un_temp;
    un_temp = (unsigned char)ach_i2c_data[4];
    un_temp <<= 8;
    *pun_ir_led += un_temp;
    un_temp = (unsigned char)ach_i2c_data[5];
    *pun_ir_led += un_temp;
    *pun_red_led &= 0x03FFFF;  //Mask MSB [23:18]
    *pun_ir_led &= 0x03FFFF;  //Mask MSB [23:18]


    return true;
}

void reset(const struct device* dev) {
    int ret = i2c_reg_write_byte(dev, MAX30102_ADDRESS, REG_MODE_CONFIG, 0x40);
}

uint8_t check(const struct device* dev) {
    uint8_t readPointer = getReadPointer(dev);
    uint8_t writePointer = getWritePointer(dev);

    int numberOfSamples = 0;

    if (readPointer != writePointer) {
        numberOfSamples = writePointer - readPointer;
        if (numberOfSamples < 0) numberOfSamples += 32; //Wrap condition
        int bytesLeftToRead = numberOfSamples * activeLEDS * 3;

        while (bytesLeftToRead > 0) {
            int toGet = bytesLeftToRead;
            if (toGet > I2C_BUFFER_LENGTH)
            {
                //If toGet is 32 this is bad because we read 6 bytes (Red+IR * 3 = 6) at a time
                //32 % 6 = 2 left over. We don't want to request 32 bytes, we want to request 30.
                //32 % 9 (Red+IR+GREEN) = 5 left over. We want to request 27.

                toGet = I2C_BUFFER_LENGTH - (I2C_BUFFER_LENGTH % (activeLEDS * 3)); //Trim toGet to be a multiple of the samples we need to read
            }

            bytesLeftToRead -= toGet;
            while (toGet > 0) {
                sense.head++;
                sense.head %= STORAGE_SIZE;
                readFIFO(dev);
                toGet -= activeLEDS * 3;
            }
        }



    }
    return 1;
}



bool safeCheck(const struct device* dev, uint8_t timeToCheck) {
    uint32_t markTime = k_uptime_get_32();

    while (1) {
        if ((k_uptime_get_32() - markTime) > timeToCheck) return false;
        if (check(dev) == true) {
            return true;

        }
        k_sleep(K_MSEC(1));
    }
}

uint32_t getIRVal(const struct device* dev) {
    if (safeCheck(dev, 250)) {
        return (sense.IR[sense.head]);
    }
    else {
        return 0;
    }
}

uint32_t getRedVal(const struct device* dev) {
    if (safeCheck(dev, 250)) {
        return (sense.red[sense.head]);
    }
    else {
        return 0;
    }
}



void nextSample() {
    if (available()) { //Only advance the tail if new data is available{
        sense.tail++;
        sense.tail %= STORAGE_SIZE; //Wrap condition
    }
}


/*============== Begin Data fetch functions ==================*/



void bitMask(const struct device* dev_i2c, uint8_t reg, uint8_t mask, uint8_t thing) {
    // Grab current register context
    uint8_t originalContents = 0;
    int ret = i2c_reg_read_byte(dev_i2c, MAX30102_ADDRESS, reg, &originalContents);

    if (ret) {
        printk("Unable to get og contents (err %i), ID: (%x)\n", ret, mask);

    }
    // Zero-out the portions of the register we're interested in
    originalContents = originalContents & mask;

    // Change contents
    ret = i2c_reg_write_byte(dev_i2c, MAX30102_ADDRESS, reg, originalContents | thing);
    if (ret) {
        printk("Unable to write (err %i)\n", ret);
    }
}