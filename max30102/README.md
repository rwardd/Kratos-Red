# CSSE4011 Prac1: Ryan Ward 45813685 AHU Module
## Design Tasks
### 1. System Timer and LED Shell Commands
The AHU NRF Dongle has an RGB LED that is can be toggled on using three PWM channels - Red, Green and Blue. The user can enter the command led <o, f, t> <r, g, b> to turn on, off or toggle the red, blue or green LEDs.

The Zephyr RTOS contains a system call to get the current kernel uptime, and when the user inputs the 'time' command, this function gets called and the uptime is formatted to the screen.

### 2. Logging
The Zephyr RTOS provides API Macros to assist with the logging of data to the terminal.  Throughout the program, various log statements are included for warnings, errors and information.  The are able to be enabled/disabled using the provided 'log' command from the RTOS.  

### 3. Shell Command Interface
The shell interface was constructed with the Zephyr RTOS shell API. This allowed for the creation of all commands and their handler functions, which would pass various data structures along FIFOs to processing threads. An example command call would be 'hts221 r t', which would send a request to the ble thread, which would call a funtion to constuct the HCI data packet, then write to the connected device's GATT characteristic.  When receiving data from the device, the data is put into a FIFO which is read by the HCI processing thread, and then formatted and printed to the terminal.

### 4. Bluetooth Communication
The Bluetooth communication was handled through writing to each device's GATT characteristics. For example, a read request from the terminal would write to the Thingy52's GATT, which would trigger a callback, and the Thingy would then parse the written data and send a packet back depending on the type of request.

The Bluetooth discovery was handled by various connect and disconnect callbacks, and operated by the Thingy putting out an advertisement, and the dongle reading the advertisement data and connecting if the data was correct. Then, the devices discover each others GATT characteristics so that communication is possible throught the GATT handles.



## Folder Structure
```
repo
|
├── myoslib
│   ├── ahu_hci
│   │   ├── hci.c
|   |   ├── hci.h
│   ├── ahu_ble
|   │   ├── ahu_ble.c
|   |   ├── ahu_ble.h
│   ├── ahu_led
|   │   ├── ahu_led.c
|   │   ├── ahu_led.h
│   ├── ahu_shell
|   │   ├── ahu_shell.c
|   │   ├── ahu_shell.h
├── prac1
│   ├── ahu
│   │   ├── src
|   │   |   ├── main.c
|   │   ├── CMakeLists.txt
|   │   ├── prj.conf
|   │   ├── dtc_shell.overlay
│   ├── README.md
```
## References used
The Zephyr Project documentation: [zephyrproject.org](zephyrproject.org) 

Provided Open source example code: [ConnectionExample](https://github.com/uqembeddedsys/zephyr-examples/tree/main/ble_connect_sample) [GATTWriteExample](https://github.com/zephyrproject-rtos/zephyr/blob/main/tests/bluetooth/bsim_bt/bsim_test_gatt/src/gatt_client_test.c)

## Building and Flashing Instructions
The source code builds in the repo/prac1/ahu/ using the command:

```console
$ west build -p auto
```
And can be flashed using the nrfutil tools.


## User Instructions
The terminal can be interacted with using putty or screen

```console
$ screen /dev/tty* 115200
```
Once the terminal is running, press TAB to see available commands.