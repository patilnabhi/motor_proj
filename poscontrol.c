#include "poscontrol.h"
#include "NU32.h"

void poscontrol_init(void) {
	T4CONbits.TCKPS = 6;
	PR4 = 6249; // freq: 200 Hz
	TMR4 = 0;
	T4CONbits.ON = 1;

	IPC4bits.T4IP = 5;
	IPC4bits.T4IS = 0;
	IFS0bits.T4IF = 0;
	IEC0bits.T4IE = 1;
}