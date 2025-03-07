// Own header
#include "modules/safest_bearing/safest_bearing.h"
#include "modules/computer_vision/cv.h"
#include "modules/core/abi.h"
#include "std.h"
#include <stdint.h>

#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include <stdlib.h>  // Make sure to include this header for malloc
#include "pthread.h"

// Function prototype
void draw_bearing_box(struct image_t *img, float norm_min_bearing, float norm_max_bearing);

struct image_t *random_draw1(struct image_t *img, uint8_t camera_id);
struct image_t *random_draw1(struct image_t *img, uint8_t camera_id __attribute__((unused)))
{
  draw_bearing_box(img, 0.3, 0.7);

  

  return img;
}

void safest_bearing_init(void)
{
  #define COLOR_OBJECT_DETECTOR_FPS1 0 ///< Default FPS (zero means run at camera fps)
  // #ifdef COLOR_OBJECT_DETECTOR_CAMERA1
  cv_add_to_device(&COLOR_OBJECT_DETECTOR_CAMERA1, random_draw1, COLOR_OBJECT_DETECTOR_FPS1, 0);
}

void entry(const float tensor_input_1[1][3][485][224], float tensor_41[1][2]);

void draw_bearing_box(struct image_t *img, float norm_min_bearing, float norm_max_bearing)
{
    uint8_t *buffer = img->buf;

    // Prepare tensor input for the entry function, assuming the tensor is [1][3][485][224]
    float tensor_input_1[1][3][485][224];  // Adjust the size based on your image size
    float tensor_41[1][2];

    // Convert YUV image to tensor format
    for (int y = 0; y < img->h; y++) {
        for (int x = 0; x < img->w; x++) {
            uint8_t *yp, *up, *vp;
            if (x % 2 == 0) {
                up = &buffer[y * 2 * img->w + 2 * x];      // U
                yp = &buffer[y * 2 * img->w + 2 * x + 1];  // Y1
                vp = &buffer[y * 2 * img->w + 2 * x + 2];  // V
            } else {
                up = &buffer[y * 2 * img->w + 2 * x - 2];  // U
                yp = &buffer[y * 2 * img->w + 2 * x + 1];  // Y2
                vp = &buffer[y * 2 * img->w + 2 * x];      // V
            }

            // Assuming tensor_input_1 follows the structure [1][3][h][w], storing Y, U, V values separately
            tensor_input_1[0][0][y][x] = *yp;   // Y (Luminance)
            tensor_input_1[0][1][y][x] = *up;   // U (Chrominance)
            tensor_input_1[0][2][y][x] = *vp;   // V (Chrominance)
        }
    }

    // Call the entry function with the tensor input
    entry(tensor_input_1, tensor_41);

    float max_bearing = tensor_41[0][0];
    float min_bearing = tensor_41[0][1];

    // Print the values of min_bearing and max_bearing
    printf("Min Bearing: %f\n", min_bearing);
    printf("Max Bearing: %f\n", max_bearing);

    // Convert normalized bearings to pixel locations
    uint16_t min_y = (uint16_t)((min_bearing / 360.0) * img->h);
    uint16_t max_y = (uint16_t)((max_bearing / 360.0) * img->h);

    // // Convert normalized bearings to pixel locations
    // uint16_t min_y = (uint16_t)(norm_min_bearing * img->h);
    // uint16_t max_y = (uint16_t)(norm_max_bearing * img->h);

    // Green color in YUV
    uint8_t y_value = 76;   // Luminance (brightness) for red
    uint8_t u_value = 84;   // U chrominance for red
    uint8_t v_value = 255;  // V chrominance for red

    // Ensure min_y is less than max_y
    if (min_y > max_y) {
        uint16_t temp = min_y;
        min_y = max_y;
        max_y = temp;
    }

    // Adjust thickness of drawn box
    uint16_t thickness = 2;  // Make the box 2 pixels thick

    // Draw the top and bottom borders of the box (hollow)
    for (uint16_t x = 0; x < img->w; x++) {
        // Top border (min_y)
        for (uint16_t i = 0; i < thickness; i++) {
            uint8_t *yp_top, *up_top, *vp_top;
            if (x % 2 == 0) {
                up_top = &buffer[(min_y + i) * 2 * img->w + 2 * x];      // U
                yp_top = &buffer[(min_y + i) * 2 * img->w + 2 * x + 1];  // Y1
                vp_top = &buffer[(min_y + i) * 2 * img->w + 2 * x + 2];  // V
            } else {
                up_top = &buffer[(min_y + i) * 2 * img->w + 2 * x - 2];  // U
                yp_top = &buffer[(min_y + i) * 2 * img->w + 2 * x + 1];  // Y2
                vp_top = &buffer[(min_y + i) * 2 * img->w + 2 * x];      // V
            }

            // Set the color for the top border
            *yp_top = y_value;
            *up_top = u_value;
            *vp_top = v_value;
        }

        // Bottom border (max_y)
        for (uint16_t i = 0; i < thickness; i++) {
            uint8_t *yp_bottom, *up_bottom, *vp_bottom;
            if (x % 2 == 0) {
                up_bottom = &buffer[(max_y + i) * 2 * img->w + 2 * x];      // U
                yp_bottom = &buffer[(max_y + i) * 2 * img->w + 2 * x + 1];  // Y1
                vp_bottom = &buffer[(max_y + i) * 2 * img->w + 2 * x + 2];  // V
            } else {
                up_bottom = &buffer[(max_y + i) * 2 * img->w + 2 * x - 2];  // U
                yp_bottom = &buffer[(max_y + i) * 2 * img->w + 2 * x + 1];  // Y2
                vp_bottom = &buffer[(max_y + i) * 2 * img->w + 2 * x];      // V
            }

            // Set the color for the bottom border
            *yp_bottom = y_value;
            *up_bottom = u_value;
            *vp_bottom = v_value;
        }
    }

    // Draw the left and right borders of the box (hollow)
    for (uint16_t y = min_y; y <= max_y; y++) {
        // Left border (min_x)
        for (uint16_t i = 0; i < thickness; i++) {
            uint8_t *yp_left, *up_left, *vp_left;
            if ((0 + i) % 2 == 0) {
                up_left = &buffer[(y + i) * 2 * img->w];          // U
                yp_left = &buffer[(y + i) * 2 * img->w + 1];      // Y1
                vp_left = &buffer[(y + i) * 2 * img->w + 2];      // V
            } else {
                up_left = &buffer[(y + i) * 2 * img->w - 2];      // U
                yp_left = &buffer[(y + i) * 2 * img->w + 1];      // Y2
                vp_left = &buffer[(y + i) * 2 * img->w];          // V
            }

            // Set the color for the left border
            *yp_left = y_value;
            *up_left = u_value;
            *vp_left = v_value;
        }

        // Right border (max_x)
        for (uint16_t i = 0; i < thickness; i++) {
            uint8_t *yp_right, *up_right, *vp_right;
            if ((img->w - 1 + i) % 2 == 0) {
                up_right = &buffer[(y + i) * 2 * img->w + 2 * (img->w - 1)];  // U
                yp_right = &buffer[(y + i) * 2 * img->w + 2 * (img->w - 1) + 1];  // Y1
                vp_right = &buffer[(y + i) * 2 * img->w + 2 * (img->w - 1) + 2];  // V
            } else {
                up_right = &buffer[(y + i) * 2 * img->w + 2 * (img->w - 1) - 2];  // U
                yp_right = &buffer[(y + i) * 2 * img->w + 2 * (img->w - 1) + 1];  // Y2
                vp_right = &buffer[(y + i) * 2 * img->w + 2 * (img->w - 1)];      // V
            }

            // Set the color for the right border
            *yp_right = y_value;
            *up_right = u_value;
            *vp_right = v_value;
        }
    }
}

void safest_bearing_periodic(void)
{

}