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

// Filter Settings
ColorSettings green = {0, 0, 0, 0, 0, 0, false};
ColorSettings orange = {0, 0, 0, 0, 0, 0, false};
ColorSettings purple = {0, 0, 0, 0, 0, 0, false};
ColorSettings brown = {0, 0, 0, 0, 0, 0, false};


float best_angle_rad = 0;
float best_angle_rad_instruction = 0;
int16_t turning_action = 0;
uint8_t best_index = 0;
int16_t turning_threshold = -15;

// Define cost function
int16_t cost_function(struct image_t *img, float alpha, float entry_point_fraction, float best_angle_rad);
void draw_best_line(struct image_t *img, float alpha, float entry_point_fraction);
int16_t compute_texture_score(uint8_t *buffer, int width, int height, int x, int y);

struct image_t *compute_ray_costs(struct image_t *img, uint8_t camera_id);
struct image_t *compute_ray_costs(struct image_t *img, uint8_t camera_id __attribute__((unused)))
{
  static const float angles[] = {
    72.65f, 67.38f, 57.99f, 38.66f, 0, -38.66f, -57.99f, -67.38f, -72.65f
  };

  static const float entry_point_fractions[] = {
    -0.115f, 0.03846f, 0.1923f, 0.34615f, 0.5f, 0.65385f, 0.8077f, 0.961538f, 1.115f
  };

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  pthread_mutex_lock(&mutex);
  int16_t costs[9]; //Initialise cost matrix for 9 rays
  int16_t min_cost = INT16_MAX;
  turning_action = 1;
  best_angle_rad_instruction = 0;

  // Loop over nine rays and determine the cost from cost function for all nine rays
  for (int i =0; i <9; i++){
    costs[i] = cost_function(img, angles[i] * (float)M_PI / 180.0f, entry_point_fractions[i], best_angle_rad);
  }

  int16_t filtered_costs[9];  // Initialise temporary cost matrix in which values from the filtered cost function are stored. 

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
      if (costs[i] < turning_threshold && turning_action == 1){
        turning_action = 0;
      }
      if (costs[i] < min_cost){
        min_cost = costs[i];
        best_index = i; // Best angle is determined based on ray that has the lowest cost
      }
  }

  best_angle_rad = angles[best_index] * (float)M_PI / 180.0f;
 
  uint16_t ratio = (costs[4] != 0) ? (uint16_t)(fabs((costs[best_index] - costs[4]) / (double)costs[4]) * 100.0) : 0;

  if (ratio > 5){
    best_angle_rad_instruction = best_angle_rad;
  } //only change steering angle if change in cost is larger than 30%
  float local_best_angle_rad = best_angle_rad;
  float local_entry_point_fraction = entry_point_fractions[best_index];
  pthread_mutex_unlock(&mutex);
  ///////////////////////////////////////////////////////////////////////////////////////////////////

  // draw_best_line(img, local_best_angle_rad, local_entry_point_fraction);
  
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

  cv_add_to_device(&RAY_PATH_FINDER_CAMERA1, compute_ray_costs, RAY_PATH_FINDER_FPS1, 0);
}

int16_t compute_texture_score(uint8_t *buffer, int width, int height, int x, int y){
  int window_size = 3;
  int half_window = window_size / 2;
  uint8_t min_y = 255, max_y = 0; // Initialize min/max values
  for (int dx = -half_window; dx <= half_window; dx++) {
    for (int dy = -half_window; dy <= half_window; dy++) {
        int nx = x + dx;
        int ny = y + dy;
        
        if (nx < 0 || nx >= width || ny < 0 || ny >= height) continue; // Boundary check
        
        uint8_t y_value = buffer[ny * width * 2 + 2 * nx + 1]; // Extract Y component
        
        if (y_value < min_y) min_y = y_value;
        if (y_value > max_y) max_y = y_value;
    }
}

return max_y - min_y; 
}

int16_t cost_function(struct image_t *img, float alpha, float entry_point_fraction, float best_angle_rad)
{
  int16_t num_pixels_line = 0;
  int16_t cost = 0;
  uint8_t *buffer = img->buf;
  int width = img->w, height = img->h;
  float slope = tan(alpha);

  uint8_t lum_min_gr = green.lum_min;
  uint8_t lum_max_gr = green.lum_max;
  uint8_t cb_min_gr = green.cb_min;
  uint8_t cb_max_gr = green.cb_max;
  uint8_t cr_min_gr = green.cr_min;
  uint8_t cr_max_gr = green.cr_max;

  uint8_t lum_min_or = orange.lum_min;
  uint8_t lum_max_or = orange.lum_max;
  uint8_t cb_min_or = orange.cb_min;
  uint8_t cb_max_or = orange.cb_max;
  uint8_t cr_min_or = orange.cr_min;
  uint8_t cr_max_or = orange.cr_max;

  uint8_t lum_min_pp = purple.lum_min;
  uint8_t lum_max_pp = purple.lum_max;
  uint8_t cb_min_pp = purple.cb_min;
  uint8_t cb_max_pp = purple.cb_max;
  uint8_t cr_min_pp = purple.cr_min;
  uint8_t cr_max_pp = purple.cr_max;

  uint8_t lum_min_br = brown.lum_min;
  uint8_t lum_max_br = brown.lum_max;
  uint8_t cb_min_br = brown.cb_min;
  uint8_t cb_max_br = brown.cb_max;
  uint8_t cr_min_br = brown.cr_min;
  uint8_t cr_max_br = brown.cr_max;


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
        // int texture_score = compute_texture_score(buffer, width, height, x, y);
        // if (texture_score < 20) cost -= weight;
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
    uint8_t *yp;

    for (uint16_t offset = 0; offset < 11; offset++) {
      // Compute the y-value using the slope
      uint16_t y = (uint16_t) roundf(slope * x + img->h * entry_point_fraction) + offset - 5;
      if (y >= img->h) continue;  // Ensure y is within bounds

      // Access the U, Y1, and V values directly
      if (x % 2 == 0) {
        // Even x
        yp = &buffer[y * 2 * img->w + 2 * x + 1];  // Y1
      } else {
        // Uneven x
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
  AbiSendMsgVISUAL_DETECTION(3, turning_action, 0, 0, 0, (int32_t) (-0.10f*best_angle_rad_instruction * 180.0f / M_PI), 0);
  pthread_mutex_unlock(&mutex);
}

