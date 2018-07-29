#ifndef __INPUTS_H__
#define __INPUTS_H__

#include <stdint.h>


void ADC_AnalogRead(uint16_t *a, uint16_t *b);
void ADC_init(void);


void ButtonInit(void);
int ButtonIsPress(void);


#endif
