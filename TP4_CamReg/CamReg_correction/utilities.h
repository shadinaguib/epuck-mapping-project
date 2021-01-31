#ifndef UTILITIES_H
#define UTILITIES_H


//start the PI regulator thread
uint16_t absolute_difference(uint16_t a, uint16_t b);
uint16_t steps_to_degrees(uint16_t steps);
uint16_t degrees_to_steps(uint16_t degrees);
float    deg2rad(uint16_t degrees);
float rads2steps(float rads);
float steps2rad(uint16_t steps);
uint16_t mm2steps(uint16_t millimeters);
uint16_t steps2mm(uint16_t steps);
uint16_t minimum(uint16_t a, uint16_t b);





#endif /* UTILITIES_H */
