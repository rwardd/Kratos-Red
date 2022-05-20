

#include <zephyr.h>
#include "argon_i2c.h"

#define ARGON_PRIO 1
#define ARGON_STACK_SIZE 512

K_THREAD_DEFINE(argonTID, ARGON_STACK_SIZE,
		amg8833_thread_entry, NULL, NULL, NULL,
		ARGON_PRIO, 0, 0);