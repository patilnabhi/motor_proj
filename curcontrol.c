#include "curcontrol.h"
#include "NU32.h"

void curcontrol_init(void) {
	unsigned int OCRS_val;

	PR3 = 3999; // freq: 20kHz 
	TMR3 = 0;
	T3CONbits.TCKPS = 0;
	T3CONbits.ON = 1;

	OC1CONbits.OCM = 0b110;
	OCRS_val = 2000; // initial value of OCRS to be 1/2 of total value
	OC1RS = OCRS_val;
	OC1R = OCRS_val;
	OC1CONbits.OCTSEL = 1;	
	OC1CONbits.ON = 1;

	TRISDbits.TRISD5 = 0;
	LATDbits.LATD5 = 0;

	PR2 = 3999; // freq: 5kHz
	TMR2 = 0;
	T2CONbits.TCKPS = 2;
	T2CONbits.ON = 1;

	IPC2bits.T2IP = 4;
	IPC2bits.T2IS = 0;
	IFS0bits.T2IF = 0;
	IEC0bits.T2IE = 1;
}

void curcontrol_set_pwm(int duty_cycle) {
	if (duty_cycle >= 0)
	{
		LATDbits.LATD5 = 1;
		OC1RS = duty_cycle * 40;
	}
	if (duty_cycle < 0)
	{
		LATDbits.LATD5 = 0;
		OC1RS = -1 * duty_cycle * 40;
	}
}