#include "utilities.h"

static volatile mode_struc mode = IDLE;

// define a new structure to store various modes
mode_struc get_mode(void) {
	return mode;
}

// function to set mode
void set_mode(mode_struc m) {
	__builtin_disable_interrupts();
	mode = m;
	__builtin_enable_interrupts();
}

