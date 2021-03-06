/* LinkDirection.h */

#ifndef __link_direction_h__
#define __link_direction_h__

typedef enum {
    DIRECTION_LEFT     = 0,
    DIRECTION_RIGHT    = 1,
    DIRECTION_FORWARD  = 2,
    DIRECTION_BACKWARD = 3,
    DIRECTION_UP       = 4,
    DIRECTION_DOWN     = 5,
    DIRECTION_ZOOM_OUT = 6,
    DIRECTION_ZOOM_IN = 7,
    NUM_LINK_DIRECTIONS = 8
} LinkDirection;

#endif /* __link_direction_h__ */
