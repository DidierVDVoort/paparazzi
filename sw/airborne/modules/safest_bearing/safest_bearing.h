#ifndef SAFEST_BEARING_H
#define SAFEST_BEARING_H

#include <stdint.h>
#include <stdbool.h>

void entry(const float tensor_input_1[1][3][208][96], float tensor_49[1][2]);

// Module functions
extern void safest_bearing_init(void);
extern void safest_bearing_periodic(void);

#endif /* SAFEST_BEARING_H */

 
 