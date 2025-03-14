#ifndef RAY_PATHS_CALIBRATION_H
#define RAY_PATHS_CALIBRATION_H

#include <stdint.h>
#include <stdbool.h>

// Filter Settings
extern uint8_t margin_gr_calibration;
extern uint8_t lum_gr_calibration;
extern uint8_t cb_gr_calibration;
extern uint8_t cr_gr_calibration;

extern uint8_t margin_or_calibration;
extern uint8_t lum_or_calibration;
extern uint8_t cb_or_calibration;
extern uint8_t cr_or_calibration;

extern bool calibration_mode;
extern bool green_draw_calibration;
extern bool orange_draw_calibration;
extern bool purple_draw_calibration;
extern bool brown_draw_calibration;

// Module functions
extern void ray_paths_calibration_init(void);
extern void ray_paths_calibration_periodic(void);

#endif /* RAY_PATHS_CALIBRATION_H */
