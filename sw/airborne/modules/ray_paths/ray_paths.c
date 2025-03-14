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

#ifndef RAY_PATH_FINDER_FPS1
#define RAY_PATH_FINDER_FPS1 0 ///< Default FPS (zero means run at camera fps)
#endif

// Filter Settings
uint8_t margin_gr = 0;
uint8_t lum_gr = 0;
uint8_t cb_gr = 0;
uint8_t cr_gr = 0;

uint8_t margin_or = 0;
uint8_t lum_or = 0;
uint8_t cb_or = 0;
uint8_t cr_or = 0;

uint8_t margin_pp = 0;
uint8_t lum_pp = 0;
uint8_t cb_pp = 0;
uint8_t cr_pp = 0;

uint8_t margin_br = 0;
uint8_t lum_br = 0;
uint8_t cb_br = 0;
uint8_t cr_br = 0;

bool green_draw = false;
bool orange_draw = false;
bool purple_draw = false;
bool brown_draw = false;

float best_angle_rad = 0;
float best_angle_rad_instruction = 0;

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
  }

  int16_t filtered_costs[9];  // Temporary array for filtered values

  for (int i = 0; i < 9; i++) {
      if (i == 0) {
          // Left boundary: 0.8 * itself
          filtered_costs[i] = (int16_t)(0.8 * costs[i]);
      } else if (i == 8) {
          // Right boundary: 0.8 * itself
          filtered_costs[i] = (int16_t)(0.8 * costs[i]);
      } else {
          // Middle values: 0.2 * left + 0.6 * itself + 0.2 * right
          filtered_costs[i] = (int16_t)(0.2 * costs[i - 1] + 0.6 * costs[i] + 0.2 * costs[i + 1]);
      }
  }
  
  // Copy filtered values back to costs
  for (int i = 0; i < 9; i++) {
      costs[i] = filtered_costs[i];
      if (costs[i] < min_cost){
        min_cost = costs[i];
        best_index = i;
      }
  }

  // uint8_t ratio;
  // ratio = round((cost[best_index] - cost[4])/cost[4])

  // if modules(ratio) > 20:
  //   best_angle_rad_instruction = angles[best_index]

  best_angle_rad = angles[best_index];
  best_angle_rad_instruction = 0;
  static float prev1 = -1, prev2 = -1;


  prev2 = prev1;
  prev1 = best_angle_rad;

  if (prev1 == prev2){
    best_angle_rad_instruction = best_angle_rad;
  }

  pthread_mutex_unlock(&mutex);

  draw_best_line(img, best_angle_rad, entry_point_fractions[best_index]);
  
  fprintf(stderr, "[random_draw1] Min Cost: %d, Best Angle: %.2f degrees\n", min_cost, best_angle_rad_instruction * 180.0f / M_PI);
  
  for (int i = 0; i < 9; i++) {
    fprintf(stderr, "%d", costs[i]);
    if (i < 8) { // Add a comma except for the last element
        fprintf(stderr, ", ");
    }
  }

fprintf(stderr, "]\n");  // Close the array and move to a new line

  return img;
}

void ray_paths_init(void)
{
  pthread_mutex_init(&mutex, NULL); // TODO: Check if this is necessary and work?

  #ifndef RAY_PATH_FINDER_FPS1
  #define RAY_PATH_FINDER_FPS1 0 ///< Default FPS (zero means run at camera fps)
  #endif
  
  #ifdef RAY_PATH_FINDER_GREEN_LUM
    lum_gr = RAY_PATH_FINDER_GREEN_LUM;
    cb_gr = RAY_PATH_FINDER_GREEN_CB;
    cr_gr = RAY_PATH_FINDER_GREEN_CR;
    margin_gr = RAY_PATH_FINDER_GREEN_MARGIN;
  #endif
  #ifdef RAY_PATH_FINDER_GREEN_DRAW
    green_draw = RAY_PATH_FINDER_GREEN_DRAW;
  #endif

  #ifdef RAY_PATH_FINDER_ORANGE_LUM
    lum_or = RAY_PATH_FINDER_ORANGE_LUM;
    cb_or = RAY_PATH_FINDER_ORANGE_CB;
    cr_or = RAY_PATH_FINDER_ORANGE_CR;
    margin_or = RAY_PATH_FINDER_ORANGE_MARGIN;
  #endif
  #ifdef RAY_PATH_FINDER_ORANGE_DRAW
    orange_draw = RAY_PATH_FINDER_ORANGE_DRAW;
  #endif

  #ifdef RAY_PATH_FINDER_PURPLE_LUM
  lum_pp = RAY_PATH_FINDER_PURPLE_LUM;
  cb_pp = RAY_PATH_FINDER_PURPLE_CB;
  cr_pp = RAY_PATH_FINDER_PURPLE_CR;
  margin_pp = RAY_PATH_FINDER_PURPLE_MARGIN;
#endif
#ifdef RAY_PATH_FINDER_PURPLE_DRAW
  purple_draw = RAY_PATH_FINDER_PURPLE_DRAW;
#endif

#ifdef RAY_PATH_FINDER_BROWN_LUM
lum_br = RAY_PATH_FINDER_BROWN_LUM;
cb_br = RAY_PATH_FINDER_BROWN_CB;
cr_br = RAY_PATH_FINDER_BROWN_CR;
margin_br = RAY_PATH_FINDER_BROWN_MARGIN;
#endif
#ifdef RAY_PATH_FINDER_BROWN_DRAW
brown_draw = RAY_PATH_FINDER_BROWN_DRAW;
#endif

  cv_add_to_device(&RAY_PATH_FINDER_CAMERA1, random_draw1, RAY_PATH_FINDER_FPS1, 0);
}



int16_t cost_function(struct image_t *img, float alpha, float entry_point_fraction, float best_angle_rad)
{
  int16_t cost = 0;
  uint8_t *buffer = img->buf;

  // Definitions of the colour green
  uint8_t lum_min_gr = lum_gr - margin_gr;
  uint8_t lum_max_gr = lum_gr + margin_gr;
  uint8_t cb_min_gr = cb_gr - margin_gr;
  uint8_t cb_max_gr = cb_gr + margin_gr;
  uint8_t cr_min_gr = cr_gr - margin_gr;
  uint8_t cr_max_gr = cr_gr + margin_gr;

  // Definitions of the colour orange
  uint8_t lum_min_or = lum_or - margin_or;
  uint8_t lum_max_or = lum_or + margin_or;
  uint8_t cb_min_or = cb_or - margin_or;
  uint8_t cb_max_or = cb_or + margin_or;
  uint8_t cr_min_or = cr_or - margin_or;
  uint8_t cr_max_or = cr_or + margin_or;

  // Definitions of the colour purple
  uint8_t lum_min_pp = lum_pp - margin_pp;
  uint8_t lum_max_pp = lum_pp + margin_pp;
  uint8_t cb_min_pp = cb_pp - margin_pp;
  uint8_t cb_max_pp = cb_pp + margin_pp;
  uint8_t cr_min_pp = cr_pp - margin_pp;
  uint8_t cr_max_pp = cr_pp + margin_pp;

      // Definitions of the colour brown
  uint8_t lum_min_br = lum_br - margin_br;
  uint8_t lum_max_br = lum_br + margin_br;
  uint8_t cb_min_br = cb_br - margin_br;
  uint8_t cb_max_br = cb_br + margin_br;
  uint8_t cr_min_br = cr_br - margin_br;
  uint8_t cr_max_br = cr_br + margin_br;


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
      if (x <= 20) {
          weight = 5;
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
      } 
      
      // Increase cost when orange is near to drone
      if ( (*yp >= lum_min_or) && (*yp <= lum_max_or) &&
      (*up >= cb_min_or ) && (*up <= cb_max_or ) &&
      (*vp >= cr_min_or ) && (*vp <= cr_max_or )) 
      {
        cost += 5*weight;
      } 

      // Increase cost when purple is near to drone
      if ( (*yp >= lum_min_pp) && (*yp <= lum_max_pp) &&
      (*up >= cb_min_pp ) && (*up <= cb_max_pp ) &&
      (*vp >= cr_min_pp ) && (*vp <= cr_max_pp )) 
      {
        cost += 2*weight;
      } 

      // Increase cost when brown is near to drone
      if ( (*yp >= lum_min_br) && (*yp <= lum_max_br) &&
      (*up >= cb_min_br ) && (*up <= cb_max_br ) &&
      (*vp >= cr_min_br ) && (*vp <= cr_max_br )) 
      {
        cost += 2*weight;
      } 
    }
  }
  // Normalization
  cost = round(cost*100/ num_pixels_line);


  // Prefer inertia
  if (alpha == best_angle_rad){
    cost -= 10;
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
  //fprintf(stderr, "Best Angle: %.2f degrees\n", best_angle_rad * 180.0f / M_PI);
  pthread_mutex_lock(&mutex);
  AbiSendMsgVISUAL_DETECTION(3, 0, 0, 0, 0, (int32_t) (-0.10f*best_angle_rad_instruction * 180.0f / M_PI), 0);
  pthread_mutex_unlock(&mutex);
}

