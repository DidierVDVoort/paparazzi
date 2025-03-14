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

static pthread_mutex_t mutex;

#ifndef RAY_PATH_FINDER_FPS1
#define RAY_PATH_FINDER_FPS1 0 ///< Default FPS (zero means run at camera fps)
#endif

// Filter Settings
uint8_t margin_gr_calibration = 0;
uint8_t lum_gr_calibration = 0;
uint8_t cb_gr_calibration = 0;
uint8_t cr_gr_calibration = 0;

uint8_t margin_or_calibration = 0;
uint8_t lum_or_calibration = 0;
uint8_t cb_or_calibration = 0;
uint8_t cr_or_calibration = 0;

bool calibration_mode = true;
bool green_draw_calibration = true;
bool orange_draw_calibration = false;

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
  pthread_mutex_init(&mutex, NULL); // TODO: Check if this is necessary and work?

  #ifndef RAY_PATH_FINDER_FPS1
  #define RAY_PATH_FINDER_FPS1 0 ///< Default FPS (zero means run at camera fps)
  #endif
  
  #ifdef RAY_PATH_FINDER_GREEN_LUM
    lum_gr_calibration = RAY_PATH_FINDER_GREEN_LUM;
    cb_gr_calibration = RAY_PATH_FINDER_GREEN_CB;
    cr_gr_calibration = RAY_PATH_FINDER_GREEN_CR;
    margin_gr_calibration = RAY_PATH_FINDER_GREEN_MARGIN;
  #endif
  #ifdef RAY_PATH_FINDER_GREEN_DRAW
    green_draw_calibration = RAY_PATH_FINDER_GREEN_DRAW;
  #endif

  #ifdef RAY_PATH_FINDER_ORANGE_LUM
    lum_or_calibration = RAY_PATH_FINDER_ORANGE_LUM;
    cb_or_calibration = RAY_PATH_FINDER_ORANGE_CB;
    cr_or_calibration = RAY_PATH_FINDER_ORANGE_CR;
    margin_or_calibration = RAY_PATH_FINDER_ORANGE_MARGIN;
  #endif
  #ifdef RAY_PATH_FINDER_ORANGE_DRAW
    orange_draw_calibration = RAY_PATH_FINDER_ORANGE_DRAW;
  #endif

  cv_add_to_device(&RAY_PATH_FINDER_CAMERA1, image_func, 0, 0);
}

void show_color_filter(struct image_t *img)
{
  uint8_t *buffer = img->buf;

  // Definitions of the colour green
  uint8_t lum_min_gr = lum_gr_calibration - margin_gr_calibration;
  uint8_t lum_max_gr = lum_gr_calibration + margin_gr_calibration;
  uint8_t cb_min_gr = cb_gr_calibration - margin_gr_calibration;
  uint8_t cb_max_gr = cb_gr_calibration + margin_gr_calibration;
  uint8_t cr_min_gr = cr_gr_calibration - margin_gr_calibration;
  uint8_t cr_max_gr = cr_gr_calibration + margin_gr_calibration;

  // Definitions of the colour orange
  uint8_t lum_min_or = lum_or_calibration - margin_or_calibration;
  uint8_t lum_max_or = lum_or_calibration + margin_or_calibration;
  uint8_t cb_min_or = cb_or_calibration - margin_or_calibration;
  uint8_t cb_max_or = cb_or_calibration + margin_or_calibration;
  uint8_t cr_min_or = cr_or_calibration - margin_or_calibration;
  uint8_t cr_max_or = cr_or_calibration + margin_or_calibration;

  // Loop through all pixels
  for (uint16_t y = 0; y < img->h; y++) {
    for (uint16_t x = 0; x < img->w; x++) {
      uint8_t *yp, *up, *vp;

      // Access the U, Y1, and V values directly
      if (x % 2 == 0) {
        // Even x
        up = &buffer[y * 2 * img->w + 2 * x];      // U
        yp = &buffer[y * 2 * img->w + 2 * x + 1];  // Y1
        vp = &buffer[y * 2 * img->w + 2 * x + 2];  // V
        //yp = &buffer[y * 2 * img->w + 2 * x + 3]; // Y2
      } else {
        // Uneven x
        up = &buffer[y * 2 * img->w + 2 * x - 2];  // U
        //yp = &buffer[y * 2 * img->w + 2 * x - 1]; // Y1
        vp = &buffer[y * 2 * img->w + 2 * x];      // V
        yp = &buffer[y * 2 * img->w + 2 * x + 1];  // Y2
      }

      if (green_draw_calibration) {
        if ((*yp >= lum_min_gr) && (*yp <= lum_max_gr) &&
            (*up >= cb_min_gr) && (*up <= cb_max_gr) &&
            (*vp >= cr_min_gr) && (*vp <= cr_max_gr)) {
          *yp = 255;
        }
      }
      if (orange_draw_calibration) {
        if ((*yp >= lum_min_or) && (*yp <= lum_max_or) &&
            (*up >= cb_min_or) && (*up <= cb_max_or) &&
            (*vp >= cr_min_or) && (*vp <= cr_max_or)) {
          *yp = 255;
        }
      }
    }
  }
}

void ray_paths_calibration_periodic(void)
{
  //fprintf(stderr, "Best Angle: %.2f degrees\n", best_angle_rad * 180.0f / M_PI);
  // pthread_mutex_lock(&mutex);
  // AbiSendMsgVISUAL_DETECTION(3, 0, 0, 0, 0, (int32_t) (-0.10f*best_angle_rad_instruction * 180.0f / M_PI), 0);
  // pthread_mutex_unlock(&mutex);
}

