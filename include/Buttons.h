
#ifndef BUTTONS_H   
#define BUTTONS_H

#include <stdint.h>
#include "BOARD.h"

#define BUTTONS_INIT() { \
    TRISD |= 0x00E0; \
    TRISF |= 0x0002; \
}

#define BUTTON_STATES() (((PORTD >> 4) & 0x0E) | ((PORTF >> 1) & 0x01))

#define SWITCH_STATES() ((PORTD >> 8) & 0x0F)

#endif 