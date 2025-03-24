// Own header
#include "modules/ray_paths_calibration/ray_paths_calibration.h"
#include "modules/computer_vision/cv.h"
#include "modules/core/abi.h"
#include "std.h"
#include <stdint.h>

#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include "pthread.h"

bool calibration_mode = false;
bool green_draw_calibration = false;
bool orange_draw_calibration = false;
bool purple_draw_calibration = false;
bool brown_draw_calibration = false;

// Function
void show_color_filter(struct image_t *img);

struct image_t *image_func(struct image_t *img, uint8_t camera_id);
struct image_t *image_func(struct image_t *img, uint8_t camera_id __attribute__((unused)))
{
  if (calibration_mode){
    show_color_filter(img);
  }
  return img;
}

void ray_paths_calibration_init(void)
{
  #ifndef RAY_PATH_CALIBRATION_FPS
  #define RAY_PATH_CALIBRATION_FPS 0 ///< Default FPS (zero means run at camera fps)
  #endif
  
  #ifdef RAY_PATH_FINDER_GREEN_LUM_MIN
    green.lum_min = RAY_PATH_FINDER_GREEN_LUM_MIN;
    green.lum_max = RAY_PATH_FINDER_GREEN_LUM_MAX;
    green.cb_min = RAY_PATH_FINDER_GREEN_CB_MIN;
    green.cb_max = RAY_PATH_FINDER_GREEN_CB_MAX;
    green.cr_min = RAY_PATH_FINDER_GREEN_CR_MIN;
    green.cr_max = RAY_PATH_FINDER_GREEN_CR_MAX;
    green.draw = RAY_PATH_FINDER_GREEN_DRAW;
  #endif

  #ifdef RAY_PATH_FINDER_ORANGE_LUM_MIN
    orange.lum_min = RAY_PATH_FINDER_ORANGE_LUM_MIN;
    orange.lum_max = RAY_PATH_FINDER_ORANGE_LUM_MAX;
    orange.cb_min = RAY_PATH_FINDER_ORANGE_CB_MIN;
    orange.cb_max = RAY_PATH_FINDER_ORANGE_CB_MAX;
    orange.cr_min = RAY_PATH_FINDER_ORANGE_CR_MIN;
    orange.cr_max = RAY_PATH_FINDER_ORANGE_CR_MAX;
    orange.draw = RAY_PATH_FINDER_ORANGE_DRAW;
  #endif

  #ifdef RAY_PATH_FINDER_PURPLE_LUM_MIN
    purple.lum_min = RAY_PATH_FINDER_PURPLE_LUM_MIN;
    purple.lum_max = RAY_PATH_FINDER_PURPLE_LUM_MAX;
    purple.cb_min = RAY_PATH_FINDER_PURPLE_CB_MIN;
    purple.cb_max = RAY_PATH_FINDER_PURPLE_CB_MAX;
    purple.cr_min = RAY_PATH_FINDER_PURPLE_CR_MIN;
    purple.cr_max = RAY_PATH_FINDER_PURPLE_CR_MAX;
    purple.draw = RAY_PATH_FINDER_PURPLE_DRAW;
  #endif

  #ifdef RAY_PATH_FINDER_BROWN_LUM_MIN
    brown.lum_min = RAY_PATH_FINDER_BROWN_LUM_MIN;
    brown.lum_max = RAY_PATH_FINDER_BROWN_LUM_MAX;
    brown.cb_min = RAY_PATH_FINDER_BROWN_CB_MIN;
    brown.cb_max = RAY_PATH_FINDER_BROWN_CB_MAX;
    brown.cr_min = RAY_PATH_FINDER_BROWN_CR_MIN;
    brown.cr_max = RAY_PATH_FINDER_BROWN_CR_MAX;
    brown.draw = RAY_PATH_FINDER_BROWN_DRAW;
  #endif

  cv_add_to_device(&RAY_PATH_FINDER_CAMERA1, image_func, RAY_PATH_CALIBRATION_FPS, 0);
}

void show_color_filter(struct image_t *img)
{
  uint8_t *buffer = img->buf;

  // Define color settings
  ColorSettings colors[] = {green, orange, purple, brown};
  const size_t color_count = sizeof(colors) / sizeof(colors[0]);

  // Struct to store computed min/max values
  struct {
      uint8_t lum_min, lum_max;
      uint8_t cb_min, cb_max;
      uint8_t cr_min, cr_max;
      bool draw;
  } ranges[sizeof(colors) / sizeof(colors[0])];

  // Precompute min/max values
  for (size_t i = 0; i < color_count; i++) {
      ranges[i].lum_min = colors[i].lum_min;
      ranges[i].lum_max = colors[i].lum_max;
      ranges[i].cb_min = colors[i].cb_min;
      ranges[i].cb_max = colors[i].cb_max;
      ranges[i].cr_min = colors[i].cr_min;
      ranges[i].cr_max = colors[i].cr_max;
      ranges[i].draw = colors[i].draw;
  }

  // Loop through all pixels
  for (uint16_t y = 0; y < img->h; y++) {
    for (uint16_t x = 0; x < img->w; x++) {
        uint8_t *yp, *up, *vp;

        // Access the U, Y1, and V values
        if (x % 2 == 0) {
            up = &buffer[y * 2 * img->w + 2 * x];      // U
            yp = &buffer[y * 2 * img->w + 2 * x + 1];  // Y1
            vp = &buffer[y * 2 * img->w + 2 * x + 2];  // V
        } else {
            up = &buffer[y * 2 * img->w + 2 * x - 2];  // U
            vp = &buffer[y * 2 * img->w + 2 * x];      // V
            yp = &buffer[y * 2 * img->w + 2 * x + 1];  // Y2
        }

        // Check each color range
        for (size_t i = 0; i < color_count; i++) {
            if (ranges[i].draw &&
                (*yp >= ranges[i].lum_min && *yp <= ranges[i].lum_max) &&
                (*up >= ranges[i].cb_min && *up <= ranges[i].cb_max) &&
                (*vp >= ranges[i].cr_min && *vp <= ranges[i].cr_max)) {
                *yp = 255;  // Highlight the pixel
            }
        }
    }
  }
}

void ray_paths_calibration_periodic(void)
{
  // Do nothing
}

