#ifndef SAFEST_BEARING_H
#define SAFEST_BEARING_H

#include <stdint.h>
#include <stdbool.h>

void entry(const float CNN_input[1][3][104][48], float CNN_output[1][2]);

// Module functions
extern void safest_bearing_init(void);
extern void safest_bearing_periodic(void);

#endif /* SAFEST_BEARING_H */

 
 