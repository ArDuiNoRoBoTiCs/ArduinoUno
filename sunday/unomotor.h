#ifndef __unomotor_h__
#define __unomotor_h__

#include "unopins.h"


void motorInit();

void high();

/**
 * Right motors forward.
 */
void forwardRight();

/**
 * Left motors forward.
 */
void forwardLeft();

/**
 * Left motors backward.
 */
void backLeft();

/**
 * Right motors backward.
 */
void backRight() ;

/**
 * Move car forward.
 */
void forward();

/**
 * Move car backward.
 */
void back();

/**
 * Turn car left.
 */
void left();

/**
 * Turn car right.
 */
void right();

/**
 * Stop the car.
 */
void stop();

#endif // __unomotor_h__s