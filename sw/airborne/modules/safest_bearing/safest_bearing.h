#ifndef SAFEST_BEARING_H
#define SAFEST_BEARING_H

#include <stdint.h>
#include <stdbool.h>

// Module functions
extern void safest_bearing_init(void);
extern void safest_bearing_periodic(void);

extern float bearings_tensor[1][2];
#endif /* SAFEST_BEARING_H */

 
 