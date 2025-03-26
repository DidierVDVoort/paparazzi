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
ColorSettings white = {0, 0, 0, 0, 0, 0, false}; // Added white color settings

#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))
#define DEG_TO_RAD(deg) ((deg) * M_PI / 180.0f)

float best_angle_deg = 0;
float best_angle_deg_instruction = 0;
int16_t turning_action = 0;
uint8_t best_index = 0;
int16_t turning_threshold = -25;
int8_t ratio_setting = 20;
int16_t prev_costs[4][9] = {{0}};


// Define cost function
int16_t cost_function(struct image_t *img, float alpha, float entry_point_fraction, float best_angle_deg);
void compute_filtered_costs(const int16_t *costs, int16_t *filtered_costs);
void update_prev_costs(int16_t results_costs[9]);
void draw_best_line(struct image_t *img, float alpha, float entry_point_fraction);
int16_t compute_texture_score(uint8_t *buffer, int width, int height, int x, int y);

struct image_t *compute_ray_costs(struct image_t *img, uint8_t camera_id);
struct image_t *compute_ray_costs(struct image_t *img, uint8_t camera_id __attribute__((unused)))
{
  static const float angles[] = {72.65f, 67.38f, 57.99f, 38.66f, 0, -38.66f, -57.99f, -67.38f, -72.65f};
  static const float entry_point_fractions[] = {-0.115f, 0.03846f, 0.1923f, 0.34615f, 0.5f, 0.65385f, 0.8077f, 0.961538f, 1.115f};

  pthread_mutex_lock(&mutex);
  int16_t costs[9], filtered_costs[9], results_costs[9]; //Initialise cost matrix for 9 rays
  int16_t min_cost = INT16_MAX;
  turning_action = 1;
  best_angle_deg_instruction = 0;

  // Loop over nine rays and determine the cost from cost function for all nine rays
  for (uint_fast8_t i =0; i < ARRAY_SIZE(angles); i++){
    costs[i] = cost_function(img, DEG_TO_RAD(angles[i]), entry_point_fractions[i], best_angle_deg);
  }

  compute_filtered_costs(costs, filtered_costs);
  update_prev_costs(filtered_costs);

  int safe_path_count = 0;
  for (uint_fast8_t i = 0; i < ARRAY_SIZE(filtered_costs); i++) {
    results_costs[i] = (prev_costs[0][i] + prev_costs[1][i] + prev_costs[2][i] + prev_costs[3][i]) / 4;

    if (results_costs[i] < turning_threshold) {
        safe_path_count++;
    }
    if (results_costs[i] < min_cost) {
        min_cost = results_costs[i];
        best_index = i;
    }
  }
  turning_action = (safe_path_count >= 2) ? 0 : 1;

  // Print the results_costs array
  // fprintf(stderr, "Results Costs: ");
  // for (uint_fast8_t i = 0; i < ARRAY_SIZE(results_costs); i++) {
  //   fprintf(stderr, "%d ", results_costs[i]);
  // }
  // fprintf(stderr, "Results Costs angle=0: %d\n", results_costs[4]);
  // fprintf(stderr, "\n");

  best_angle_deg = angles[best_index];

  int16_t ratio = (results_costs[4] != 0) ? (int16_t)(fabs(((double)(results_costs[best_index] - results_costs[4]) / results_costs[4]) * 100.0)) : 0;
  if (ratio == 0 || ratio > ratio_setting) {
    best_angle_deg_instruction = best_angle_deg;
  }
  cost_instruction = round((results_costs[3] + results_costs[4] + results_costs[5])/3);
  

  pthread_mutex_unlock(&mutex);

  // fprintf(stderr, "Min Cost: %d, Ratio: %d Best Angle: %.2f degrees\n", min_cost, ratio, best_angle_deg_instruction);
  return img;
  }

  void compute_filtered_costs(const int16_t *costs, int16_t *filtered_costs) {
    for (uint_fast8_t i = 0; i < 9; i++) {
        if (i == 0) {
            filtered_costs[i] = (int16_t)(0.8 * costs[i] + 0.2 * costs[i + 1]);
        } else if (i == 8) {
            filtered_costs[i] = (int16_t)(0.8 * costs[i] + 0.2 * costs[i - 1]);
        } else {
            filtered_costs[i] = (int16_t)(0.2 * costs[i - 1] + 0.6 * costs[i] + 0.2 * costs[i + 1]);
        }
    }
}

void update_prev_costs(int16_t new_costs[9]){
  for (uint_fast8_t i=0; i < 9; i++){
    prev_costs[3][i] = prev_costs[2][i];
    prev_costs[2][i] = prev_costs[1][i];
    prev_costs[1][i] = prev_costs[0][i];
    prev_costs[0][i] = new_costs[i];
  }
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

  #ifdef RAY_PATH_FINDER_WHITE_LUM_MIN
    white.lum_min = RAY_PATH_FINDER_WHITE_LUM_MIN;
    white.lum_max = RAY_PATH_FINDER_WHITE_LUM_MAX;
    white.cb_min = RAY_PATH_FINDER_WHITE_CB_MIN;
    white.cb_max = RAY_PATH_FINDER_WHITE_CB_MAX;
    white.cr_min = RAY_PATH_FINDER_WHITE_CR_MIN;
    white.cr_max = RAY_PATH_FINDER_WHITE_CR_MAX;
    white.draw = RAY_PATH_FINDER_WHITE_DRAW;
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

int16_t cost_function(struct image_t *img, float alpha, float entry_point_fraction, float best_angle_deg)
{
  int16_t num_pixels_line = 0;
  int16_t cost = 0;
  uint8_t *buffer = img->buf;
  // int width = img->w;
  int height = img->h;
  float slope = tan(alpha);
  uint8_t half_thickness = 5;

  if (alpha == 0.f) {
    half_thickness = 10;
  }
  else {
    half_thickness = 5;
  }

  // Loop through half of x-values
  for (uint_fast8_t x = 0; x < 101; x++) {
    float base_y = slope * x + height * entry_point_fraction;

    for (int_fast8_t offset = -half_thickness; offset <= half_thickness; offset++) {
      int16_t y = (int16_t)(base_y + offset);
      if (y >= height) continue;

      num_pixels_line++;
      uint8_t *up, *vp, *yp;
      // Access the U, Y1, and V values directly
      if (x % 2 == 0) {
        // Even x
        up = &buffer[y * 2 * img->w + 2 * x];      // U
        yp = &buffer[y * 2 * img->w + 2 * x + 1];  // Y1
        vp = &buffer[y * 2 * img->w + 2 * x + 2];  // V

      } else {
        // Uneven x
        up = &buffer[y * 2 * img->w + 2 * x - 2];  // U
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
      if ( (*yp >= green.lum_min) && (*yp <= green.lum_max) &&
      (*up >= green.cb_min ) && (*up <= green.cb_max ) &&
      (*vp >= green.cr_min ) && (*vp <= green.cr_max ))
     
      {
        // int texture_score = compute_texture_score(buffer, width, height, x, y);
        // if (texture_score < 20) cost -= weight;
        cost -= weight;
      } 
      
      // Increase cost when orange is near to drone
      if ( (*yp >= orange.lum_min) && (*yp <= orange.lum_max) &&
      (*up >= orange.cb_min ) && (*up <= orange.cb_max) &&
      (*vp >= orange.cr_min) && (*vp <= orange.cr_max )) 
      {
        cost += 5*weight;
      } 

      // Increase cost when purple is near to drone
      if ( (*yp >= purple.lum_min) && (*yp <= purple.lum_max) &&
      (*up >= purple.cb_min ) && (*up <= purple.cb_max ) &&
      (*vp >= purple.cr_min ) && (*vp <= purple.cr_max )) 
      {
        cost += 2*weight;
      } 

      // Increase cost when brown is near to drone
      if ( (*yp >= brown.lum_min) && (*yp <= brown.lum_max) &&
      (*up >= brown.cb_min ) && (*up <= brown.cb_max ) &&
      (*vp >= brown.cr_min ) && (*vp <= brown.cr_max )) 
      {
        cost += 4*weight;
      } 

      // Increase cost when white is near to drone
      if ( (*yp >= white.lum_min) && (*yp <= white.lum_max) &&
      (*up >= white.cb_min ) && (*up <= white.cb_max ) &&
      (*vp >= white.cr_min ) && (*vp <= white.cr_max )) 
      {
        cost += 2*weight;
      }
    }
  }
  // Normalization
  if (num_pixels_line > 0) {
    cost = (int16_t)roundf(cost * 100.0f / num_pixels_line);
}


  // Prefer inertia
  if (alpha == best_angle_deg){
    cost -= 15;
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
  //fprintf(stderr, "Best Angle: %.2f degrees\n", best_angle_deg * 180.0f / M_PI);
  pthread_mutex_lock(&mutex);
  AbiSendMsgVISUAL_DETECTION(3, turning_action, cost_instruction, 0, 0, (int32_t) (-best_angle_deg_instruction), 0);
  pthread_mutex_unlock(&mutex);
}
