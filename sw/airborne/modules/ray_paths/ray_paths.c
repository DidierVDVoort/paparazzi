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

float best_angle_rad = 0;

// Function
int16_t cost_function(struct image_t *img, float alpha, float entry_point_fraction, float best_angle_rad);
void draw_best_line(struct image_t *img, float alpha, float entry_point_fraction);

struct image_t *random_draw1(struct image_t *img, uint8_t camera_id);
struct image_t *random_draw1(struct image_t *img, uint8_t camera_id __attribute__((unused)))
{
  float angles[] = {
    72.65f * (float)M_PI / 180.0f,
    67.38f * (float)M_PI / 180.0f,
    57.99f * (float)M_PI / 180.0f,
    38.66f * (float)M_PI / 180.0f,
    0,
    -38.66f * (float)M_PI / 180.0f,
    -57.99f * (float)M_PI / 180.0f,
    -67.38f * (float)M_PI / 180.0f,
    -72.65f * (float)M_PI / 180.0f
  };

  float entry_point_fractions[] = {-0.115f, 0.03846f, 0.1923f, 0.34615f, 0.5f, 0.65385f, 0.8077f, 0.961538f, 1.115f};
  int16_t costs[9];
  int16_t min_cost = 32767;
  uint8_t best_index = 0;

  pthread_mutex_lock(&mutex);
  for (int i =0; i <9; i++){
    costs[i] = cost_function(img, angles[i], entry_point_fractions[i], best_angle_rad);
    if (costs[i] < min_cost){
      min_cost = costs[i];
      best_index = i;
    }
  }
  best_angle_rad = angles[best_index];
  pthread_mutex_unlock(&mutex);

  draw_best_line(img, best_angle_rad, entry_point_fractions[best_index]);
  
  fprintf(stderr, "[random_draw1] Min Cost: %d, Best Angle: %.2f degrees\n", min_cost, best_angle_rad * 180.0f / M_PI);
  
  return img;
}

void ray_paths_init(void)
{
  #define COLOR_OBJECT_DETECTOR_FPS1 1 ///< Default FPS (zero means run at camera fps)
  // #ifdef COLOR_OBJECT_DETECTOR_CAMERA1
  cv_add_to_device(&COLOR_OBJECT_DETECTOR_CAMERA1, random_draw1, COLOR_OBJECT_DETECTOR_FPS1, 0);
}



int16_t cost_function(struct image_t *img, float alpha, float entry_point_fraction, float best_angle_rad)
{
  uint8_t draw = 1;
  int16_t cost = 0;
  uint8_t *buffer = img->buf;

  // Definitions of the colour green
  uint8_t margin_gr = 30;
  uint8_t lum_gr = 86;
  uint8_t lum_min_gr = lum_gr - margin_gr;
  uint8_t lum_max_gr = lum_gr + margin_gr;
  uint8_t cb_gr = 84;
  uint8_t cb_min_gr = cb_gr - margin_gr;
  uint8_t cb_max_gr = cb_gr + margin_gr;
  uint8_t cr_gr = 122;
  uint8_t cr_min_gr = cr_gr - margin_gr;
  uint8_t cr_max_gr = cr_gr + margin_gr;

  // Definitions of the colour orange
  uint8_t margin_or = 40;
  uint8_t lum_or = 112;
  uint8_t lum_min_or = lum_or - margin_or;
  uint8_t lum_max_or = lum_or + margin_or;
  uint8_t cb_or = 82;
  uint8_t cb_min_or = cb_or - margin_or;
  uint8_t cb_max_or = cb_or + margin_or;
  uint8_t cr_or = 190;
  uint8_t cr_min_or = cr_or - margin_or;
  uint8_t cr_max_or = cr_or + margin_or;


  // x is vertical, y is horizontal
  int16_t num_pixels_line = 0;
  // Compute the slope using the given angle
  float slope = tan(alpha);

  // Loop through half of x-values
  for (uint16_t x = 0; x < 101; x++) {
    uint8_t *yp, *up, *vp;

    for (uint16_t offset = 0; offset < 11; offset++) {
      // Compute the y-value using the slope
      num_pixels_line += 1;
      uint16_t y = (uint16_t) roundf(slope * x + img->h * entry_point_fraction) + offset - 5;
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
      
      // if (draw) {
      //   *yp = 0;
      // }

      // Decrease cost when green is close to drone
      if ( (*yp >= lum_min_gr) && (*yp <= lum_max_gr) &&
      (*up >= cb_min_gr ) && (*up <= cb_max_gr ) &&
      (*vp >= cr_min_gr ) && (*vp <= cr_max_gr )) 
      {
        cost -= weight;
        // if (draw) {
        //   *yp = 255;
        // }
      } 
      
      // Increase cost when orange is near to drone
      if ( (*yp >= lum_min_or) && (*yp <= lum_max_or) &&
      (*up >= cb_min_or ) && (*up <= cb_max_or ) &&
      (*vp >= cr_min_or ) && (*vp <= cr_max_or )) 
      {
        cost += 10*weight;
        // if (draw) {
        //   *yp = 128;
        // }
      } 
    }
  }
  // Normalization
  cost = round(cost * 3* (img->w/2) / num_pixels_line);

  // Prefer going straight over turning
  if (alpha == 0){
    cost -= 10;
  }
  // Prefer inertia
  if (alpha == best_angle_rad){
    cost -= 30;
  }

  
  return cost;
  
}

void draw_best_line(struct image_t *img, float alpha, float entry_point_fraction)
{
  uint8_t *buffer = img->buf;

  // x is vertical, y is horizontal

  // Compute the slope using the given angle
  float slope = tan(alpha);

  // Loop through half of x-values
  for (uint16_t x = 0; x < 101; x++) {
    uint8_t *yp, *up, *vp;

    for (uint16_t offset = 0; offset < 11; offset++) {
      // Compute the y-value using the slope
      uint16_t y = (uint16_t) roundf(slope * x + img->h * entry_point_fraction) + offset - 5;
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

      *yp = 255;
    }
  }
}

void ray_paths_periodic(void)
{
  fprintf(stderr, "Best Angle: %.2f degrees\n", best_angle_rad * 180.0f / M_PI);
  pthread_mutex_lock(&mutex);
  AbiSendMsgVISUAL_DETECTION(3, 0, 0, 0, 0, (int32_t) (best_angle_rad * 180.0f / M_PI), 0);
  pthread_mutex_unlock(&mutex);
}