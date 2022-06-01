/**
 ************************************************************************
 * @file ahu_led.c
 * @author Ryan Ward
 * @date 31.03.2022
 * @brief Contains Source code for handling LEDs on NRF Dongle
 **********************************************************************
 * */


#include "./ahu_led.h"
 /* Register LOG Module */
LOG_MODULE_REGISTER(led_module, LOG_LEVEL_DBG);

/* Keep alive thread */
void blinky_thread(void) {
    
    static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
    int ret;
    ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);

    if (ret < 0) {
        return;
    }

    while (1) {
        ret = gpio_pin_toggle_dt(&led);
        if (ret < 0) {
            return;
        }

        k_sleep(K_SECONDS(1));
    }
}
