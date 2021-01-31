#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>
#include <main.h>
#include <motors.h>
#include <utilities.h>
#include <process_image.h>
#include <sensors/proximity.h>
#include <sensors/VL53L0X/VL53L0X.h>
#include <detection.h>
#include <stdbool.h>
#include <audio/play_melody.h>
#include <leds.h>
#include <selector.h>

bool stop = false;

static uint8_t code = 0;
static uint8_t number_of_objects = 0;

uint16_t distance_from_walls[NUMBER_OF_MEASUREMENTS] = {
    0
};
uint16_t distance_from_objects[NUMBER_OF_MEASUREMENTS] = {
    0
};

typedef struct object {
    uint16_t beginning;
    uint16_t end;
    uint16_t center;
    uint16_t distance_to_center;
}
OBJECT;

OBJECT object[6];

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

void init_sensors(void) {
    // TOF sensor
    VL53L0X_start();
    // Proximity sensors
    messagebus_init( & bus, & bus_lock, & bus_condvar);
    proximity_start();
    calibrate_ir();
    chThdSleepMilliseconds(500);
}

static THD_WORKING_AREA(waDetectAndExplore, 4096);
static THD_FUNCTION(DetectAndExplore, arg) {
    chRegSetThreadName(__FUNCTION__);
    (void) arg;

    //messagebus_topic_t *prox_topic = messagebus_find_topic_blocking(&bus, "/proximity");
    //proximity_msg_t prox_values;
    while(get_selector() == 0)
      chThdSleepMilliseconds(100);

    scan_walls();

    scan_for_objects();

    // for (uint16_t i = 0; i < NUMBER_OF_MEASUREMENTS; i++) {
    //     chprintf((BaseSequentialStream * ) & SD3, "Difference between distances at position %d is %d - %d = %d\n\r", i * 4,
    //         distance_from_walls[i], distance_from_objects[i], absolute_difference(distance_from_walls[i], distance_from_objects[i]));
    // }

    detect_objects();
    process_objects();

    chprintf((BaseSequentialStream * ) & SD3, "Going to first object\n\r");
    if (number_of_objects > 0) {
        face_first_object();
        advance_to_first_object();

        for (uint8_t i = 0; i < number_of_objects - 1; i++) {
            move_to_next_object(i);
            chThdSleepMilliseconds(3000);
            if (color_seen() == OBJECT_RED) {
                playPompier();
            } else // Color is GREEN
            {
                set_rgb_led(0, 0, 10, 0);
                set_rgb_led(1, 0, 10, 0);
                set_rgb_led(2, 0, 10, 0);
                set_rgb_led(3, 0, 10, 0);

                chThdSleepMilliseconds(1000);
                clear_leds();
                chThdSleepMilliseconds(50);
            }

            chThdSleepMilliseconds(1000);
        }
    } else
        chprintf((BaseSequentialStream * ) & SD3, "No objects found!\n\r");

    while (1) {
        chprintf((BaseSequentialStream * ) & SD3, "SLEEPING\n\r");
        chThdSleepMilliseconds(1000);
    }

} // End of Function

void detect_and_explore_start(void) {
    chThdCreateStatic(waDetectAndExplore, sizeof(waDetectAndExplore), NORMALPRIO, DetectAndExplore, NULL);
}

void detect_objects(void) {
    for (uint16_t i = 0; i < NUMBER_OF_MEASUREMENTS - 10; i++) { // detects objet only if 10 measurements confirm!

        if (absolute_difference(distance_from_walls[i], distance_from_objects[i]) < THICKNESS_OBJECT &&
            absolute_difference(distance_from_walls[i + 1], distance_from_objects[i + 1]) > THICKNESS_OBJECT &&
            absolute_difference(distance_from_walls[i + 2], distance_from_objects[i + 2]) > THICKNESS_OBJECT &&
            absolute_difference(distance_from_walls[i + 3], distance_from_objects[i + 3]) > THICKNESS_OBJECT &&
            absolute_difference(distance_from_walls[i + 4], distance_from_objects[i + 4]) > THICKNESS_OBJECT &&
            absolute_difference(distance_from_walls[i + 5], distance_from_objects[i + 5]) > THICKNESS_OBJECT &&
            absolute_difference(distance_from_walls[i + 6], distance_from_objects[i + 6]) > THICKNESS_OBJECT &&
            absolute_difference(distance_from_walls[i + 7], distance_from_objects[i + 7]) > THICKNESS_OBJECT &&
            absolute_difference(distance_from_walls[i + 8], distance_from_objects[i + 8]) > THICKNESS_OBJECT &&
            absolute_difference(distance_from_walls[i + 9], distance_from_objects[i + 9]) > THICKNESS_OBJECT &&
            absolute_difference(distance_from_walls[i + 10], distance_from_objects[i + 10]) > THICKNESS_OBJECT) {
                chprintf((BaseSequentialStream * ) & SD3, "I FOUND AN OBJECT! starts at %d\n\r", 4 * (i + 1));
                object[code].beginning = 4 * (i + 1);
        } else if (absolute_difference(distance_from_walls[i], distance_from_objects[i]) > THICKNESS_OBJECT &&
            absolute_difference(distance_from_walls[i + 1], distance_from_objects[i + 1]) > THICKNESS_OBJECT &&
            absolute_difference(distance_from_walls[i + 2], distance_from_objects[i + 2]) > THICKNESS_OBJECT &&
            absolute_difference(distance_from_walls[i + 3], distance_from_objects[i + 3]) > THICKNESS_OBJECT &&
            absolute_difference(distance_from_walls[i + 4], distance_from_objects[i + 4]) > THICKNESS_OBJECT &&
            absolute_difference(distance_from_walls[i + 5], distance_from_objects[i + 5]) > THICKNESS_OBJECT &&
            absolute_difference(distance_from_walls[i + 6], distance_from_objects[i + 6]) > THICKNESS_OBJECT &&
            absolute_difference(distance_from_walls[i + 7], distance_from_objects[i + 7]) > THICKNESS_OBJECT &&
            absolute_difference(distance_from_walls[i + 8], distance_from_objects[i + 8]) > THICKNESS_OBJECT &&
            absolute_difference(distance_from_walls[i + 9], distance_from_objects[i + 9]) > THICKNESS_OBJECT &&
            absolute_difference(distance_from_walls[i + 10], distance_from_objects[i + 10]) < THICKNESS_OBJECT) {
                chprintf((BaseSequentialStream * ) & SD3, "END OF OBJECT! ends at %d\n\r", 4 * (i + 1));
                object[code++].end = 4 * (i + 1);
        }
    }

    chprintf((BaseSequentialStream * ) & SD3, "A total of %d objects were found.\n\r", code);
    number_of_objects = code;

}

void process_objects(void) {
    for (uint8_t i = 0; i < number_of_objects; i++) // We take three measurements and take the minimal distance between the three to make sure the robot doesn't run into the objects.
    {
        object[i].center = (object[i].beginning + object[i].end) / 2;
        object[i].distance_to_center = mm2steps(minimum(minimum(distance_from_objects[object[i].center / 4], distance_from_objects[object[i].center / 4 + 4]), distance_from_objects[object[i].center / 4 + 8]));
        chprintf((BaseSequentialStream * ) & SD3, "Object %d center is at position %d\n\r", i, object[i].center);
    }
}

void face_first_object(void) {
    // For the first object :
    stop = false;
    right_motor_set_pos(0);
    left_motor_set_pos(0);
    right_motor_set_speed(0);
    left_motor_set_speed(0);
    chThdSleepMilliseconds(1000);

    while (!stop) {
        systime_t time = chVTGetSystemTime();
        if (abs(left_motor_get_pos()) > object[0].center) {
            right_motor_set_speed(0);
            left_motor_set_speed(0);
            stop = true;
            chprintf((BaseSequentialStream * ) & SD3, "FACING FIRST OBJECT\n\r");
        } else {

            // if ( (abs(left_motor_get_pos()) == 0) || ((abs(left_motor_get_pos()) % STEPS_PER_DEGREE) < 1) )
            //   if(!case remplie)
            //   {
            //     scan_walls(i++);      //   }
            right_motor_set_speed(-200);
            left_motor_set_speed(200);
        }
        //scan_walls();
        chThdSleepUntilWindowed(time, time + MS2ST(10)); // Refresh @ 100 Hz.
    }
    //advance_to_first_object();
}

void advance_to_first_object(void) {
    stop = false;
    right_motor_set_pos(0);
    left_motor_set_pos(0);
    right_motor_set_speed(0);
    left_motor_set_speed(0);
    chThdSleepMilliseconds(1000);
    //chprintf((BaseSequentialStream *)&SD3, "Init values : 0 = %d, 7 = %d.\n\r", prox_values.initValue[0],  prox_values.initValue[7]);

    //uint16_t minimum_distance_allowed = VL53L0X_get_dist_mm()*NSTEP_ONE_TURN/WHEEL_PERIMETER_MM - 750;
    //uint16_t distance = mm2steps(VL53L0X_get_dist_mm());

    while (!stop) {
        systime_t time = chVTGetSystemTime();
        //messagebus_topic_wait(prox_topic, &prox_values, sizeof(prox_values));

        if ((get_calibrated_prox(0) > 40) && (get_calibrated_prox(7) > 40)) {
            right_motor_set_speed(0);
            left_motor_set_speed(0);
            stop = true;
            chprintf((BaseSequentialStream * ) & SD3, "Arrived to FIRST object\n\r");
            object[0].distance_to_center = abs(left_motor_get_pos());
        } else {
            right_motor_set_speed(500);
            left_motor_set_speed(500);
        }
        chThdSleepUntilWindowed(time, time + MS2ST(10)); // Refresh @ 100 Hz.
    }

    chThdSleepMilliseconds(3000);
    if (color_seen() == OBJECT_RED) {
        playPompier();
    } else {
        set_rgb_led(0, 0, 10, 0);
        set_rgb_led(1, 0, 10, 0);
        set_rgb_led(2, 0, 10, 0);
        set_rgb_led(3, 0, 10, 0);

        chThdSleepMilliseconds(1000);
        clear_leds();
        chThdSleepMilliseconds(50);
    }
}

void move_to_next_object(uint8_t i) {

    float alpha = steps2rad(object[i + 1].center - object[i].center);
    uint32_t distance_to_next_center = 0;
    float angle_gamma = 0;

    if (i == 0) // if moving from first to second object
    {
        distance_to_next_center = sqrt((object[i].distance_to_center) * (object[i].distance_to_center) +
            (object[i + 1].distance_to_center - DIST_TO_READ_COLORS) * (object[i + 1].distance_to_center - DIST_TO_READ_COLORS) -
            (2 * (object[i].distance_to_center) * (object[i + 1].distance_to_center - DIST_TO_READ_COLORS) * cos(alpha))); // Distance between the center of the robot at first object and the center at second object
        angle_gamma = acos(((object[i].distance_to_center) * (object[i].distance_to_center) +
            distance_to_next_center * distance_to_next_center -
            (float)(object[i + 1].distance_to_center - DIST_TO_READ_COLORS) * (object[i + 1].distance_to_center - DIST_TO_READ_COLORS)) / (2 * (object[i].distance_to_center) * distance_to_next_center));
    } else {
        distance_to_next_center = sqrt((object[i].distance_to_center - DIST_TO_READ_COLORS) * (object[i].distance_to_center - DIST_TO_READ_COLORS) +
            (object[i + 1].distance_to_center - DIST_TO_READ_COLORS) * (object[i + 1].distance_to_center - DIST_TO_READ_COLORS) -
            (2 * (object[i].distance_to_center - DIST_TO_READ_COLORS) * (object[i + 1].distance_to_center - DIST_TO_READ_COLORS) * cos(alpha))
        ); // Distance between the center of the robot at first object and the center at second object
        angle_gamma = acos(((object[i].distance_to_center - DIST_TO_READ_COLORS) * (object[i].distance_to_center - DIST_TO_READ_COLORS) +
            distance_to_next_center * distance_to_next_center -
            (float)(object[i + 1].distance_to_center - DIST_TO_READ_COLORS) * (object[i + 1].distance_to_center - DIST_TO_READ_COLORS)) / (2 * (object[i].distance_to_center - DIST_TO_READ_COLORS) * distance_to_next_center));
    }

    //uint32_t distance_to_next_center = mm2steps(distance_to_next_center_mm);

    float angle_to_next_center = rads2steps(PI - angle_gamma);

    float angle_to_face_center = rads2steps(PI - angle_gamma) - object[i + 1].center + object[i].center;


    chprintf((BaseSequentialStream * ) & SD3, "I have to turn %f in steps and advance for %d\n\r", angle_to_next_center, distance_to_next_center);

    stop = false;
    right_motor_set_pos(0);
    left_motor_set_pos(0);
    right_motor_set_speed(0);
    left_motor_set_speed(0);
    chThdSleepMilliseconds(1000);

    while (!stop) { // FACE NEXT OBJECT
        systime_t time = chVTGetSystemTime();
        if (abs(left_motor_get_pos()) > angle_to_next_center) {
            right_motor_set_speed(0);
            left_motor_set_speed(0);
            stop = true;
            chprintf((BaseSequentialStream * ) & SD3, "FACING NEXT OBJECT\n\r");
        } else {

            // if ( (abs(left_motor_get_pos()) == 0) || ((abs(left_motor_get_pos()) % STEPS_PER_DEGREE) < 1) )
            //   if(!case remplie)
            //   {
            //     scan_walls(i++);      //   }
            right_motor_set_speed(-200);
            left_motor_set_speed(200);
        }
        //scan_walls();
        chThdSleepUntilWindowed(time, time + MS2ST(10)); // Refresh @ 100 Hz.
    }

    stop = false; // CASE :  GO TO NEXT OBJECT
    right_motor_set_pos(0);
    left_motor_set_pos(0);
    right_motor_set_speed(0);
    left_motor_set_speed(0);
    chThdSleepMilliseconds(1000);
    chprintf((BaseSequentialStream * ) & SD3, "Going to next object.\n\r");

    while (!stop) { // Go to next object
        systime_t time = chVTGetSystemTime();
        if (abs(left_motor_get_pos()) > distance_to_next_center) {
            right_motor_set_speed(0);
            left_motor_set_speed(0);
            stop = true;
            chprintf((BaseSequentialStream * ) & SD3, "Arrived to object\n\r");
        } else {
            right_motor_set_speed(500);
            left_motor_set_speed(500);
        }
        //scan_walls();
        chThdSleepUntilWindowed(time, time + MS2ST(10)); // Refresh @ 100 Hz.
    }

    stop = false; //TURN TOWARDS CENTER
    right_motor_set_pos(0);
    left_motor_set_pos(0);
    right_motor_set_speed(0);
    left_motor_set_speed(0);
    chThdSleepMilliseconds(1000);

    while (!stop) {
        systime_t time = chVTGetSystemTime();
        if (-abs(left_motor_get_pos()) < -angle_to_face_center) {
            right_motor_set_speed(0);
            left_motor_set_speed(0);
            stop = true;
            chprintf((BaseSequentialStream * ) & SD3, "Arrived to object\n\r");
        } else {
            right_motor_set_speed(200);
            left_motor_set_speed(-200);
        }
        chThdSleepUntilWindowed(time, time + MS2ST(10)); // Refresh @ 100 Hz.
    }

}

void scan_walls(void) {
    stop = false;
    right_motor_set_pos(0);
    left_motor_set_pos(0);
    right_motor_set_speed(0);
    left_motor_set_speed(0);
    chThdSleepMilliseconds(1000);
    distance_from_walls[0] = VL53L0X_get_dist_mm();

    while (!stop) {
        systime_t time = chVTGetSystemTime();
        if ((-abs(right_motor_get_pos()) < -PERIMETER_EPUCK) && (abs(left_motor_get_pos()) > PERIMETER_EPUCK)) {
            right_motor_set_speed(0);
            left_motor_set_speed(0);
            stop = true;
            chprintf((BaseSequentialStream * ) & SD3, "STOPPING WALL SCAN\n\r");
        } else {
            if ((left_motor_get_pos() + 1) % 4 == 0) // BONNE CASE !
            {
                distance_from_walls[(left_motor_get_pos() + 1) / 4] = VL53L0X_get_dist_mm();
            }

            right_motor_set_speed(-200);
            left_motor_set_speed(200);
        }
        chThdSleepUntilWindowed(time, time + MS2ST(10)); // Refresh @ 100 Hz.
    }
}

void scan_for_objects(void) {
    stop = false;
    chThdSleepMilliseconds(7000);
    distance_from_objects[0] = VL53L0X_get_dist_mm();

    right_motor_set_pos(0);
    left_motor_set_pos(0);

    while (!stop) {
        systime_t time = chVTGetSystemTime();
        if ((-abs(right_motor_get_pos()) < -PERIMETER_EPUCK) && (abs(left_motor_get_pos()) > PERIMETER_EPUCK)) {
            right_motor_set_speed(0);
            left_motor_set_speed(0);
            stop = true;
        } else {
            if ((left_motor_get_pos() % 4 == 0) && (left_motor_get_pos() < 1294) && (left_motor_get_pos() != 0)) // BONNE CASE !
            {
                distance_from_objects[left_motor_get_pos() / 4] = VL53L0X_get_dist_mm();
            }

            right_motor_set_speed(-200);
            left_motor_set_speed(200);
        }

        chThdSleepUntilWindowed(time, time + MS2ST(10)); // Refresh @ 100 Hz.
    }
}
