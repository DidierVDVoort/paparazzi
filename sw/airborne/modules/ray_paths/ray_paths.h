#ifndef RAY_PATHS_H
#define RAY_PATHS_H

#include <stdint.h>
#include <stdbool.h>

// Filter Settings
typedef struct {
  uint8_t lum_min;
  uint8_t lum_max;
  uint8_t cb_min;
  uint8_t cb_max;
  uint8_t cr_min;
  uint8_t cr_max;
  bool draw;
} ColorSettings;

extern ColorSettings green;
extern ColorSettings orange;
extern ColorSettings purple;
extern ColorSettings brown;
extern ColorSettings white; // Added white color settings

extern float best_angle_deg;
extern float best_angle_deg_instruction;
extern int16_t turning_threshold;
extern int8_t ratio_setting;
extern int16_t cost_instruction;

// Module functions
extern void ray_paths_init(void);
extern void ray_paths_periodic(void);

#endif /* RAY_PATHS_H */
