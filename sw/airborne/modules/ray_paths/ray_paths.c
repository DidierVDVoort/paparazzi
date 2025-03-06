// Own header
#include "modules/ray_paths/ray_paths.h"
#include "modules/computer_vision/cv.h"
#include "modules/core/abi.h"
#include "std.h"
#include <stdint.h>

#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include "pthread.h"

static pthread_mutex_t mutex;

float best_angle = 0;

// Function
int16_t cost_function(struct image_t *img, float alpha, float entry_point_fraction);

struct image_t *random_draw1(struct image_t *img, uint8_t camera_id);
struct image_t *random_draw1(struct image_t *img, uint8_t camera_id __attribute__((unused)))
{
  float angles[] = {
    -60.f * (float)M_PI / 180.0f,
    -45.f * (float)M_PI / 180.0f,
    -30.f * (float)M_PI / 180.0f,
    -15.f * (float)M_PI / 180.0f,
    0,
    15.f * (float)M_PI / 180.0f,
    30.f * (float)M_PI / 180.0f,
    45.f * (float)M_PI / 180.0f,
    60.f * (float)M_PI / 180.0f
  };

  float entry_point_fractions[] = {0.35f, 0.41f, 0.44f, 0.47f, 0.5f, 0.53f, 0.56f, 0.59f, 0.65f};
  int16_t costs[9];

  int16_t min_cost = 32767;
  best_angle = 0;

  for (int i =0; i <9; i++){
    costs[i] = cost_function(img, angles[i], entry_point_fractions[i]);

    if (costs[i] < min_cost){
      min_cost = costs[i];
      best_angle = angles[i];
    }
  }
  fprintf(stderr, "[random_draw1] Max Cost: %d, Best Angle: %.2f degrees\n", min_cost, best_angle * 180.0f / M_PI);
  
  return img;
}

void ray_paths_init(void)
{
  #define COLOR_OBJECT_DETECTOR_FPS1 0 ///< Default FPS (zero means run at camera fps)
  // #ifdef COLOR_OBJECT_DETECTOR_CAMERA1
  cv_add_to_device(&COLOR_OBJECT_DETECTOR_CAMERA1, random_draw1, COLOR_OBJECT_DETECTOR_FPS1, 0);
}



int16_t cost_function(struct image_t *img, float alpha, float entry_point_fraction)
{
  uint8_t draw = 1;
  int16_t cost = 0;
  uint8_t *buffer = img->buf;

  // Definitions of the colour green
  uint8_t margin = 30;
  uint8_t lum = 86;
  uint8_t lum_min = lum - margin;
  uint8_t lum_max = lum + margin;
  uint8_t cb = 84;
  uint8_t cb_min = cb - margin;
  uint8_t cb_max = cb + margin;
  uint8_t cr = 122;
  uint8_t cr_min = cr - margin;
  uint8_t cr_max = cr + margin;

  // x is vertical, y is horizontal
  int16_t num_pixels_line = 0;
  // Compute the slope using the given angle
  float slope = tan(alpha);

  // Loop through half of x-values
  for (uint16_t x = 0; 2*x < img->w; x++) {
    uint8_t *yp, *up, *vp;

    for (uint16_t offset = 0; offset < 2; offset++) {
      // Compute the y-value using the slope
      num_pixels_line += 1;
      uint16_t y = (uint16_t) roundf(slope * x + img->h * entry_point_fraction) + offset - 1;
      if (y >= img->h) continue;  // Ensure y is within bounds

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
      
      uint16_t weight;
      if (x <= 30) {
          weight = 3;
      } else if (x > 30 && x <= 60) {
          weight = 2;
      } else {
          weight = 1;
      }

      // Increase cost when green is close to drone
      if ( (*yp >= lum_min) && (*yp <= lum_max) &&
      (*up >= cb_min ) && (*up <= cb_max ) &&
      (*vp >= cr_min ) && (*vp <= cr_max )) 
      {
        cost -= weight;
        if (draw) {
          *yp = 255;
        }
      } else {
        if (draw) {
          *yp = 0;
        }
      }
    }
  }

  if (alpha == 0){
    cost -= 50;
  }

  cost = round(cost * 3* (img->w/2) / num_pixels_line);
  return cost;
  
}

void ray_paths_periodic(void)
{
  pthread_mutex_lock(&mutex);
  AbiSendMsgVISUAL_DETECTION(3, 0, 0, 0, 0, (int32_t)best_angle, 0);
  pthread_mutex_unlock(&mutex);
}