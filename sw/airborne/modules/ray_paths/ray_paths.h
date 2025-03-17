#ifndef RAY_PATHS_H
#define RAY_PATHS_H

#include <stdint.h>
#include <stdbool.h>

// Filter Settings
typedef struct {
  uint8_t margin;
  uint8_t lum;
  uint8_t cb;
  uint8_t cr;
  bool draw;
} ColorSettings;

extern ColorSettings green;
extern ColorSettings orange;
extern ColorSettings purple;
extern ColorSettings brown;

extern float best_angle_rad;
extern float best_angle_rad_instruction;

// Module functions
extern void ray_paths_init(void);
extern void ray_paths_periodic(void);

#endif /* RAY_PATHS_H */
