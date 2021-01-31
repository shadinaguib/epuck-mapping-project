#include <math.h>
#include <stdlib.h>
#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>
#include <utilities.h>
#include <motors.h>

uint16_t absolute_difference(uint16_t a, uint16_t b)
{
  return abs(a - b);
}

uint16_t steps_to_degrees(uint16_t steps)
{
  return steps*360/PERIMETER_EPUCK;
}

uint16_t degrees_to_steps(uint16_t degrees)
{
  return PERIMETER_EPUCK*degrees/360;
}

float deg2rad(uint16_t degrees)
{
  return degrees*PI/180;
}

float steps2rad(uint16_t steps)
{
  return 2*PI*steps/PERIMETER_EPUCK;
}

float rads2steps(float rads)
{
  return rads*PERIMETER_EPUCK/(2*PI);
}

uint16_t mm2steps(uint16_t millimeters)
{
  return millimeters*NSTEP_ONE_TURN/WHEEL_PERIMETER_MM;
}

uint16_t steps2mm(uint16_t steps)
{
  return steps*WHEEL_PERIMETER_MM/NSTEP_ONE_TURN;
}

uint16_t minimum(uint16_t a, uint16_t b)
{
if(a<b)
    return a;
else
    return b;
}
