#ifndef UTILITIES_H
#define UTILITIES_H

typedef enum { IDLE, PWM, ITEST, HOLD, TRACK } mode_struc;

typedef struct {
	int refcur;
	int actcur;
	int refpos;
	int actpos;
} control_data_t;

mode_struc get_mode(void);
void set_mode(mode_struc m);

#endif