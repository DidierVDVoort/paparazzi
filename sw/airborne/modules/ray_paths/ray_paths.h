#ifndef RAY_PATHS_H
#define RAY_PATHS_H

#include <stdint.h>
#include <stdbool.h>

// Filter Settings
extern uint8_t margin_gr;
extern uint8_t lum_gr;
extern uint8_t cb_gr;
extern uint8_t cr_gr;

extern uint8_t margin_or;
extern uint8_t lum_or;
extern uint8_t cb_or;
extern uint8_t cr_or;

extern uint8_t margin_pp;
extern uint8_t lum_pp;
extern uint8_t cb_pp;
extern uint8_t cr_pp;

extern uint8_t margin_br;
extern uint8_t lum_br;
extern uint8_t cb_br;
extern uint8_t cr_br;

extern bool green_draw;
extern bool orange_draw;
extern bool purple_draw;
extern bool brown_draw;

// Module functions
extern void ray_paths_init(void);
extern void ray_paths_periodic(void);

#endif /* RAY_PATHS_H */
