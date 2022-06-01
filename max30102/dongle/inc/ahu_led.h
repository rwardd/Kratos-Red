/**
 ************************************************************************
 * @file ahu_led.h
 * @author Ryan Ward
 * @date 31.03.2022
 * @brief Contains Macros and Definitions for handling LEDs on NRF Dongle
 **********************************************************************
 * */
#ifndef AHU_LED
#define AHU_LED
 /*
  * Extract devicetree configuration.
  */
#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>

#include <logging/log.h>


  /* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)

#if DT_NODE_HAS_STATUS(LED0_NODE, okay)
#define LED0	DT_GPIO_LABEL(LED0_NODE, gpios)
#define PIN	DT_GPIO_PIN(LED0_NODE, gpios)
#define FLAGS	DT_GPIO_FLAGS(LED0_NODE, gpios)
#else
/* A build error here means your board isn't set up to blink an LED. */
#error "Unsupported board: led0 devicetree alias is not defined"
#define LED0	""
#define PIN	0
#define FLAGS	0
#endif
/*-------------------------------------------------------------- */
/* LED  */
#define PERIOD_USEC	(USEC_PER_SEC / 50U)
#define STEPSIZE_USEC	2000

/* LED Stack size and priority */
#define LED_STACK_SIZE 512
#define LED_PRIORITY 3

/* Blink thread stack size and priority */
#define BLINKY_STACK_SIZE 512
#define BLINKY_PRIORITY 5


/* Function signatures */
void blinky_thread(void);

#endif

