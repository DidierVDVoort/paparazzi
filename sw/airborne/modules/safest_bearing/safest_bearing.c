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
void rotate_image_90_counterclockwise(struct image_t *img);

struct image_t *random_draw1(struct image_t *img, uint8_t camera_id);
struct image_t *random_draw1(struct image_t *img, uint8_t camera_id __attribute__((unused)))
{
  rotate_image_90_counterclockwise(img);
  return img;
}

void safest_bearing_init(void)
{
  #define COLOR_OBJECT_DETECTOR_FPS1 0 ///< Default FPS (zero means run at camera fps)
  // #ifdef COLOR_OBJECT_DETECTOR_CAMERA1
  cv_add_to_device(&COLOR_OBJECT_DETECTOR_CAMERA1, random_draw1, COLOR_OBJECT_DETECTOR_FPS1, 0);
}

void rotate_image_90_counterclockwise(struct image_t *img)
{
    // Assume img->buf is in YUV422 format
    uint32_t y_plane_size = img->w * img->h;  // Full resolution for Y plane
    uint32_t uv_plane_size = y_plane_size / 2;  // Half resolution for U and V planes (since they are subsampled)

    // Allocate memory for the rotated image
    uint8_t *rotated_buffer = (uint8_t*)malloc(y_plane_size + uv_plane_size);
    if (rotated_buffer == NULL) {
        fprintf(stderr, "Memory allocation for rotated buffer failed.\n");
        return;
    }

    uint8_t *y_plane = img->buf;                  // Start of Y plane in original buffer
    uint8_t *uv_plane = img->buf + y_plane_size;  // Start of UV plane in original buffer

    uint8_t *rotated_y_plane = rotated_buffer;        // Start of Y plane in rotated buffer
    uint8_t *rotated_uv_plane = rotated_buffer + y_plane_size;  // Start of UV plane in rotated buffer

    // Rotate Y plane (full resolution)
    for (uint16_t y = 0; y < img->h; y++) {
        for (uint16_t x = 0; x < img->w; x++) {
            uint16_t new_x = y;
            uint16_t new_y = img->w - 1 - x;
            rotated_y_plane[new_y * img->h + new_x] = y_plane[y * img->w + x];
        }
    }

    // Rotate UV plane (half resolution)
    for (uint16_t y = 0; y < img->h; y += 2) {
        for (uint16_t x = 0; x < img->w; x += 2) {
            // Each pair of pixels shares the same U and V values
            uint16_t new_x = y / 2;
            uint16_t new_y = (img->w / 2) - 1 - x / 2;

            rotated_uv_plane[2 * (new_y * img->h / 2 + new_x)] = uv_plane[2 * (y * img->w / 2 + x / 2)];     // U
            rotated_uv_plane[2 * (new_y * img->h / 2 + new_x) + 1] = uv_plane[2 * (y * img->w / 2 + x / 2) + 1];  // V
        }
    }

    // Free the original buffer and update the img with the new rotated buffer
    free(img->buf);
    img->buf = rotated_buffer;

    // Swap width and height for the rotated image
    uint16_t temp = img->w;
    img->w = img->h;
    img->h = temp;

    // Print the image dimensions after rotating
    fprintf(stderr, "Image rotated. New width: %d, New height: %d\n", img->w, img->h);

}

void safest_bearing_periodic(void)
{

}