#ifndef __COLOR_H__
#define __COLOR_H__

#include <stdint.h>

void HSV_to_RGB(int hue, int sat, int val, uint8_t *rc, uint8_t *gc, uint8_t *bc);
void TColorToRGB(uint8_t tindex, uint8_t *rc, uint8_t *gc, uint8_t *bc);

#endif
