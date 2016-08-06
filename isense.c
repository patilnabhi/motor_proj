#include "isense.h"
#include "NU32.h"

#define SAMPLE_TIME 10

void isense_init(void) {
	AD1PCFGbits.PCFG0 = 0;
	AD1CON3bits.SAMC = 2;
	AD1CON3bits.ADCS = 2;
	AD1CON1bits.SSRC = 0b111;
	AD1CON2bits.BUFM = 1;
	AD1CON1bits.ADON = 1;
}

int isense_counts(void) {
	unsigned int current_counts = 0;
	unsigned int elapsed=0, finish_time=0;
	int i;
	for (i = 1; i <= 5; i++)
	{
		AD1CHSbits.CH0SA = 0;
		AD1CON1bits.SAMP = 1;
		elapsed = _CP0_GET_COUNT();
		finish_time = elapsed + SAMPLE_TIME;
		while (_CP0_GET_COUNT() < finish_time) {
			;
		}
		AD1CON1bits.SAMP = 0;
		while (!AD1CON1bits.DONE) { ; }
		current_counts = current_counts + ADC1BUF0;
	}
	return current_counts/5;
}

int isense_amps(void) {
	unsigned int current_counts = 0, current = 0;
	unsigned int elapsed=0, finish_time=0;
	int i;
	for (i = 1; i <= 5; i++)
	{
		AD1CHSbits.CH0SA = 0;
		AD1CON1bits.SAMP = 1;
		elapsed = _CP0_GET_COUNT();
		finish_time = elapsed + SAMPLE_TIME;
		while (_CP0_GET_COUNT() < finish_time) {
			;
		}
		AD1CON1bits.SAMP = 0;
		while (!AD1CON1bits.DONE) { ; }
		current_counts = current_counts + ADC1BUF0;
	}
	current_counts = current_counts / 5;
	current = 2139 * current_counts - 1076083; // equation from calibrations results (multiply by 1000 to prevent floating point math)
	return current;
}