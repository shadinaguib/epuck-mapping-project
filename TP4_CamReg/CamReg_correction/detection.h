#ifndef DETECTION_H
#define DETECTION_H

#define THICKNESS_OBJECT        60
#define DIST_TO_READ_COLORS     900
#define NUMBER_OF_MEASUREMENTS  324
#define OBJECT_RED              5


void init_sensors(void);
void detect_and_explore_start(void);
/**
* @brief   starts thread to detect and explore objects
*/
void detect_objects(void);
void process_objects(void);
void face_first_object(void);
/**
* @brief   Makes the robot turn and face the first object detected
*/
void advance_to_first_object(void);
void move_to_next_object(uint8_t i);
/**
* @brief  Robot faces next object and advances to it
*
* @param i corresponds to the object where the robot already is, it will move to object i+1
*/
void scan_walls(void);
/**
* @brief scans distance with walls and stores values in array
*
*/
void scan_for_objects(void);

 // @brief   Scans where objects are while making a turn and stores data in OBJECT structure

#endif /* DETECTION_H */
