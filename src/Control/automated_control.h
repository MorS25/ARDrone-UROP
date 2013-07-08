#ifndef AUTOMATED_CONTROL_H
#define AUTOMATED_CONTROL_H

#include <config.h>

C_RESULT automated_init(void);
C_RESULT automated_update(void);
C_RESULT automated_shutdown(void);

void set_target_location(int32_t x, int32_t y, uint32_t maxX, uint32_t maxY, uint8_t sees_ball);

#endif
