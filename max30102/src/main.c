//main file for kratos-red 4011 project

#include "MAX30102.h"
#include "algorithm.h"
//#include "heart_rate.h"
#define INTPIN 10

#define MAX_BRIGHTNESS 255
#define RATE_SIZE 4


uint32_t aun_ir_buffer[500]; //IR LED sensor data
int32_t n_ir_buffer_length;    //data length
uint32_t aun_red_buffer[500];    //Red LED sensor data
int32_t n_sp02; //SPO2 value
int8_t ch_spo2_valid;   //indicator to show if the SP02 calculation is valid
int32_t n_heart_rate;   //heart rate value
int8_t  ch_hr_valid;    //indicator to show if the heart rate calculation is valid
uint8_t uch_dummy;

//Increase this for more averaging. 4 is good.
uint8_t rates[RATE_SIZE]; //Array of heart rates
uint8_t rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred

float beatsPerMinute;
int beatAvg;
float ratio;
float correl;
//mainloop
int main() {
    const struct device* console_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
    if (usb_enable(NULL)) {
        return 1;
    }
    const struct device* i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c1));
    max30102_init(i2c_dev);

    const struct device* gpio_dev = device_get_binding("GPIO_1");
    gpio_pin_configure(gpio_dev, INTPIN, GPIO_INPUT | GPIO_INT_DEBOUNCE);
    reset(i2c_dev);
    MAX30102_setup(i2c_dev);


    uint32_t irValue = 0;
    uint32_t redValue = 0;

    n_ir_buffer_length = 500;
    int i = 0;


    for (i = 0; i < n_ir_buffer_length;i++)
    {
        while (gpio_pin_get(gpio_dev, INTPIN) == 1) { printk("WAITING\n"); };   //wait until the interrupt pin asserts

          //readFIFO(i2c_dev, (aun_red_buffer + i), (aun_ir_buffer + i));  //read from MAX30102 FIFO

        // while (available() == false) {
        //     check(i2c_dev);
        // }
        // aun_ir_buffer[i] = getIRVal(i2c_dev);
        // aun_red_buffer[i] = getRedVal(i2c_dev);
        // nextSample();
        readFIFOv2(i2c_dev, (aun_ir_buffer + i), (aun_red_buffer + i));


        printk("red=");
        printk("%i", aun_red_buffer[i]);
        printk(", ir=");
        printk("%i\n\r", aun_ir_buffer[i]);
    }
    maxim_heart_rate_and_oxygen_saturation(aun_ir_buffer, n_ir_buffer_length, aun_red_buffer, &n_sp02, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid);
    while (1)
    {
        i = 0;


        //dumping the first 100 sets of samples in the memory and shift the last 400 sets of samples to the top
        for (i = 100;i < 500;i++)
        {
            aun_red_buffer[i - 100] = aun_red_buffer[i];
            aun_ir_buffer[i - 100] = aun_ir_buffer[i];

        }

        //take 100 sets of samples before calculating the heart rate.
        for (i = 400;i < 500;i++)
        {

            while (gpio_pin_get(gpio_dev, INTPIN) == 1) { printk("WAITING\n"); };
            readFIFOv2(i2c_dev, (aun_ir_buffer + i), (aun_red_buffer + i));

            // while (available() == false) {
            //     check(i2c_dev);
            // }
            // aun_ir_buffer[i] = getIRVal(i2c_dev);
            // aun_red_buffer[i] = getRedVal(i2c_dev);
            // nextSample();


            //send samples and calculation result to terminal program through UART
            printk("red=");
            printk("%i", aun_red_buffer[i]);
            printk(", ir=");
            printk("%i", aun_ir_buffer[i]);
            printk(", HR=%i, ", n_heart_rate);
            printk("HRvalid=%i, ", ch_hr_valid);
            printk("SpO2=%i, ", n_sp02);
            printk("SPO2Valid=%i\n\r", ch_spo2_valid);
        }
        maxim_heart_rate_and_oxygen_saturation(aun_ir_buffer, n_ir_buffer_length, aun_red_buffer, &n_sp02, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid);

    }


    return 1;
}

























// while (1) {
//     irValue = 0;
//     redValue = 0;
//     if (gpio_pin_get(gpio_dev, INTPIN) == 0) {
//         printk("INT\n");
//     }   //wait until the interrupt pin asserts

//     irValue = getIRVal(i2c_dev);

//     if (checkForBeat(irValue) == true) {
//         uint32_t delta = k_uptime_get_32() - lastBeat;
//         lastBeat = k_uptime_get_32();

//         beatsPerMinute = 60 / (delta / 1000.0);

//         if (beatsPerMinute < 255 && beatsPerMinute > 20) {
//             rates[rateSpot++] = (uint8_t)beatsPerMinute;
//             rateSpot %= RATE_SIZE;

//             beatAvg = 0;
//             for (uint8_t x = 0; x < RATE_SIZE; x++) {
//                 beatAvg += rates[x];
//             }
//             beatAvg /= RATE_SIZE;
//         }
//     }
//     printk("IR=");
//     printk("%d", irValue);
//     printk(", BPM=");
//     printk("%f", beatsPerMinute);
//     printk(", Avg BPM=");
//     printk("%i", beatAvg);

//     if (irValue < 50000) {
//         printk(" No finger? :(");
//     }
//     printk("\n");
// }

