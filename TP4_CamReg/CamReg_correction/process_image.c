#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>
#include <main.h>
#include <camera/po8030.h>
#include <process_image.h>

static uint8_t color;
enum {
    WHITE,
    MAGENTA,
    CYAN,
    YELLOW,
    BLUE,
    RED,
    GREEN,
    BLACK
};

static float distance_cm = 0;
static uint16_t line_position = IMAGE_BUFFER_SIZE / 2; //middle
static uint8_t counter = 0;

//semaphore
static BSEMAPHORE_DECL(image_ready_sem, TRUE);


bool extract_line_width(uint8_t * buffer) {

    uint16_t i = 0, begin = 0, end = 0;
    uint8_t stop = 0, wrong_line = 0, line_not_found = 0;
		uint16_t low_mean = 0, high_mean = 0;
    uint32_t mean = 0;

    //performs an average
    for (uint16_t i = 0; i < IMAGE_BUFFER_SIZE; i++) {
        mean += buffer[i];
    }
    mean /= IMAGE_BUFFER_SIZE;
    low_mean = mean * 7;
    high_mean = mean * 13;

    do {
        wrong_line = 0;
        //search for a begin
        while (stop == 0 && i < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE)) {
            //the slope must at least be WIDTH_SLOPE wide and is compared
            //to the mean of the image
            if (10*buffer[i] < low_mean && 10*buffer[i + WIDTH_SLOPE] > high_mean) {
                begin = i;
                stop = 1;
            }
            i++;
        }
        //if a begin was found, search for an end
        if (i < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE) && begin) {
            stop = 0;

            while (stop == 0 && i < IMAGE_BUFFER_SIZE) {
                if (10*buffer[i] < low_mean && 10*buffer[i - WIDTH_SLOPE] > high_mean) {
                    end = i;
                    stop = 1;
                }
                i++;
            }
            //if an end was not found
            if (i > IMAGE_BUFFER_SIZE || !end) {
                line_not_found = 1;
            }
        } else //if no begin was found
        {
            line_not_found = 1;
        }

        //if a line too small has been detected, continues the search
        if (!line_not_found && (end - begin) < MIN_LINE_WIDTH) {
            i = end;
            begin = 0;
            end = 0;
            stop = 0;
            wrong_line = 1;
        }
    } while (wrong_line);

    if (line_not_found) {
        begin = 0;
        end = 0;
        return false;
    } else {
        line_position = (begin + end) / 2;
        return true;
    }
}

static THD_WORKING_AREA(waCaptureImage, 256);
static THD_FUNCTION(CaptureImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void) arg;

    //Takes pixels 0 to IMAGE_BUFFER_SIZE of the line 10 + 11 (minimum 2 lines because reasons)
    po8030_advanced_config(FORMAT_RGB565, 0, 10, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1);
    dcmi_enable_double_buffering();
    dcmi_set_capture_mode(CAPTURE_ONE_SHOT);
    dcmi_prepare();

    while (1) {
        //starts a capture
        dcmi_capture_start();
        //waits for the capture to be done
        wait_image_ready();
        //signals an image has been captured
        chBSemSignal( & image_ready_sem);
    }
}

static THD_WORKING_AREA(waProcessImage, 1024);
static THD_FUNCTION(ProcessImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void) arg;

    uint8_t * img_buff_ptr;
    uint8_t image[IMAGE_BUFFER_SIZE] = {
        0
    };
    uint32_t red_mean = 0, blue_mean = 0, green_mean = 0;
    systime_t time;

    bool red = false, blue = false, green = false; //send_to_computer = true,

    while (1) {
        //waits until an image has been captured
        chBSemWait( & image_ready_sem);
        time = chVTGetSystemTime();
        //gets the pointer to the array filled with the last image in RGB565
        img_buff_ptr = dcmi_get_last_image_ptr();

        //Extracts only the red pixels
        for (uint16_t i = 0; i < (2 * IMAGE_BUFFER_SIZE); i += 2) {
            //extracts first 5bits of the first byte
            //takes nothing from the second byte
            image[i / 2] = (uint8_t) img_buff_ptr[i] & 0xF8;
        }
        red = extract_line_width(image);

        //Extracts only the green pixels
        for (uint16_t i = 0; i < (2 * IMAGE_BUFFER_SIZE); i++) {
            //extracts last 3bits of the first byte
            //extracts first 3 bits of  the second byte
            if (i % 2 == 0) {
                image[i / 2] = img_buff_ptr[i] & 0x07;
                image[i / 2] = (uint8_t)(image[i / 2] << 5);
            } else {
                image[(i - 1) / 2] = (uint8_t)(image[(i - 1) / 2] | (((img_buff_ptr[i] & 0xE0)) >> 3));
            }
        }
        green = extract_line_width(image);

        for (uint16_t i = 1; i < (2 * IMAGE_BUFFER_SIZE); i += 2) {
            //extracts last 5bits of the second byte
            //takes nothing from the first byte
            image[(i - 1) / 2] = ((uint8_t) img_buff_ptr[i] & 0x1F) << 3;
        }
        blue = extract_line_width(image);

        if (blue && red && green) { // every color gets 33 points
            green_mean += 33;
            blue_mean += 33;
            red_mean += 33;
        }
        if (blue && red) { // blue and red get 50 points each
            blue_mean += 50;
            red_mean += 50;
        }
        if (blue && green) {
            blue_mean += 50;
            green_mean += 50;
        }
        if (green && red) {
            green_mean += 50;
            red_mean += 50;
        }
        if (green) { // green gets 100 points
            green_mean += 100;
        }
        if (red) {
            red_mean += 100;
        }
        if (blue) {
            blue_mean += 100;
        }
        counter++;

        if (counter == 20) { // The color with the more points in 20 measurements wins!
            counter = 0;
            if (red_mean > green_mean) {
                if (blue_mean > red_mean) color = BLUE;
                else color = RED;
            } else if (green_mean > blue_mean) color = GREEN;
            else color = BLUE;
            green_mean = 0;
            red_mean = 0;
            blue_mean = 0;

            // if (color == GREEN)
            //     chprintf((BaseSequentialStream * ) & SD3, "COLOR = GREEN\n\r");
            // if (color == RED)
            //     chprintf((BaseSequentialStream * ) & SD3, "COLOR = RED\n\r");
            // if (color == BLUE)
            //     chprintf((BaseSequentialStream * ) & SD3, "COLOR = BLUE\n\r");
        }

        chThdSleepUntilWindowed(time, time + MS2ST(100));

        //SendUint8ToComputer(image, IMAGE_BUFFER_SIZE);
    }
}

float get_distance_cm(void) {
    return distance_cm;
}

uint16_t get_line_position(void) {
    return line_position;
}

void process_image_start(void) {
    chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO, ProcessImage, NULL);
    chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO, CaptureImage, NULL);
}

uint8_t color_seen(void) {
    return color;
}
