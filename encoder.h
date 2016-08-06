#ifndef ENCODER_H
#define ENCODER_H

static int encoder_command(int read);
int encoder_counts(void);
void encoder_reset(void);
int encoder_deg(void);
void encoder_init(void);

#endif