#include "modules/safest_bearing/safest_bearing.h"
#include "modules/computer_vision/cv.h"
#include "modules/core/abi.h"
#include "std.h"
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include <stdlib.h>
#include "pthread.h"

static pthread_mutex_t mutex;

float bearings_tensor[1][2];
float center_y;
float safest_bearing_rad_instruction = 0;
int16_t turn = 0;

#define N_INTERVALS 9
#define SLIDING_WINDOW 10

int center_y_history[SLIDING_WINDOW] = {0};
int interval_count[N_INTERVALS] = {0};  // Count predictions per interval
int history_index = 0;

void update_safest_bearing(int interval);
int get_bearing_interval(float center_y);
void draw_bearing_box(struct image_t *img, float norm_min_bearing, float norm_max_bearing, float (*tensor_41)[1][2]);

struct image_t *random_draw(struct image_t *img, uint8_t camera_id);
struct image_t *random_draw(struct image_t *img, uint8_t camera_id __attribute__((unused)))
{
    turn = 1;
    draw_bearing_box(img, 0.3, 0.7, &bearings_tensor);
    return img;
}

void safest_bearing_init(void)
{
  #define COLOR_OBJECT_DETECTOR_FPS1 0
  cv_add_to_device(&COLOR_OBJECT_DETECTOR_CAMERA1, random_draw, COLOR_OBJECT_DETECTOR_FPS1, 0);
}

void entry(const float tensor_input_1[1][3][104][48], float tensor_41[1][2]);

void draw_bearing_box(struct image_t *img, float norm_min_bearing, float norm_max_bearing, float (*tensor_41)[1][2])
{
    uint8_t *buffer = img->buf;

    // Prepare tensor input for the entry function
    float tensor_input_1[1][3][104][48];  

    // Convert YUV image to tensor format
    for (int y = 0; y < img->h; y++) {
        for (int x = 0; x < img->w; x++) {
            uint8_t *yp, *up, *vp;
            if (x % 2 == 0) {
                up = &buffer[y * 2 * img->w + 2 * x];      
                yp = &buffer[y * 2 * img->w + 2 * x + 1];  
                vp = &buffer[y * 2 * img->w + 2 * x + 2];  
            } else {
                up = &buffer[y * 2 * img->w + 2 * x - 2];  
                yp = &buffer[y * 2 * img->w + 2 * x + 1];  
                vp = &buffer[y * 2 * img->w + 2 * x];      
            }

            tensor_input_1[0][0][y][x] = *yp;   
            tensor_input_1[0][1][y][x] = *up;   
            tensor_input_1[0][2][y][x] = *vp;   
        }
    }

    entry(tensor_input_1, *tensor_41);

    float min_bearing = (*tensor_41)[0][0];
    float max_bearing = (*tensor_41)[0][1];

    // Clamp the bearings to [0, 1]
    if (min_bearing < 0) min_bearing = 0;
    if (min_bearing > 1) min_bearing = 1;
    if (max_bearing < 0) max_bearing = 0;
    if (max_bearing > 1) max_bearing = 1;

    uint8_t min_y = (uint8_t)(min_bearing * img->h);
    uint8_t max_y = (uint8_t)(max_bearing * img->h);

    uint8_t y_value = 76;   
    uint8_t u_value = 84;   
    uint8_t v_value = 255;  

    if (min_y > max_y) {
        uint16_t temp = min_y;
        min_y = max_y;
        max_y = temp;
    }

    uint16_t thickness = 2;

    for (uint16_t x = 0; x < img->w; x++) {
        for (uint16_t i = 0; i < thickness; i++) {
            uint8_t *yp_top, *up_top, *vp_top;
            if (x % 2 == 0) {
                up_top = &buffer[(min_y + i) * 2 * img->w + 2 * x];      
                yp_top = &buffer[(min_y + i) * 2 * img->w + 2 * x + 1];  
                vp_top = &buffer[(min_y + i) * 2 * img->w + 2 * x + 2];  
            } else {
                up_top = &buffer[(min_y + i) * 2 * img->w + 2 * x - 2];  
                yp_top = &buffer[(min_y + i) * 2 * img->w + 2 * x + 1];  
                vp_top = &buffer[(min_y + i) * 2 * img->w + 2 * x];      
            }
            *yp_top = y_value;
            *up_top = u_value;
            *vp_top = v_value;
        }

        for (uint16_t i = 0; i < thickness; i++) {
            uint8_t *yp_bottom, *up_bottom, *vp_bottom;
            if (x % 2 == 0) {
                up_bottom = &buffer[(max_y + i) * 2 * img->w + 2 * x];      
                yp_bottom = &buffer[(max_y + i) * 2 * img->w + 2 * x + 1];  
                vp_bottom = &buffer[(max_y + i) * 2 * img->w + 2 * x + 2];  
            } else {
                up_bottom = &buffer[(max_y + i) * 2 * img->w + 2 * x - 2];  
                yp_bottom = &buffer[(max_y + i) * 2 * img->w + 2 * x + 1];  
                vp_bottom = &buffer[(max_y + i) * 2 * img->w + 2 * x];      
            }
            *yp_bottom = y_value;
            *up_bottom = u_value;
            *vp_bottom = v_value;
        }
    }

    pthread_mutex_lock(&mutex);
    center_y = (min_bearing + max_bearing) / 2;  
    pthread_mutex_unlock(&mutex);

    int current_interval = get_bearing_interval(center_y);
    printf("Center Y: %.2f, Current Interval: %d\n", center_y, current_interval);

    // Update history with the current interval
    center_y_history[history_index] = current_interval;
    history_index = (history_index + 1) % SLIDING_WINDOW;

    // Update interval count
    interval_count[current_interval]++;

    // After every 10 predictions, select the interval with the most predictions
    if (history_index == 0) {
        int max_count = 0;
        int max_interval = 0;

        // Find the interval with the most predictions
        for (int i = 0; i < N_INTERVALS; i++) {
            if (interval_count[i] > max_count) {
                max_count = interval_count[i];
                max_interval = i;
            }
        }

        update_safest_bearing(max_interval);

        // Reset the interval counts after each batch of 10 predictions
        memset(interval_count, 0, sizeof(interval_count));
    }
}

void update_safest_bearing(int interval)
{
    float dy = (interval + 0.5f) / N_INTERVALS - 0.5f;  
    float dx = 0.5f;             

    float safest_bearing_rad_instruction = atan2(dy, dx);

    pthread_mutex_lock(&mutex);
    safest_bearing_rad_instruction = safest_bearing_rad_instruction;
    pthread_mutex_unlock(&mutex);

    printf("Safest Bearing Instruction: %.2f degrees\n", safest_bearing_rad_instruction * 180.0f / M_PI);
}

int get_bearing_interval(float center_y)
{
    return (int)(center_y * N_INTERVALS);
}

void safest_bearing_periodic(void)
{
    pthread_mutex_lock(&mutex);
    AbiSendMsgVISUAL_DETECTION(4, turn, 0, 0, 0, (int32_t) (-0.10f * safest_bearing_rad_instruction * 180.0f / M_PI), 0);
    pthread_mutex_unlock(&mutex);
}
