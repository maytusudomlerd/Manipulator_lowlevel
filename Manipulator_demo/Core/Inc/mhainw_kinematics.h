/*
 * mhainw_kinematics.h
 *
 *  Created on: Feb 9, 2022
 *      Author: Maytus
 */

#ifndef INC_MHAINW_KINEMATICS_H_
#define INC_MHAINW_KINEMATICS_H_

#include "math.h"
#include "string.h"

#define DEGTORAD                     0.017453292

#define J1_POSITIVE_JOINTLIMIT 105 * DEGTORAD
#define J1_NEGATIVE_JOINTLIMIT -1 * J1_POSITIVE_JOINTLIMIT
#define J2_POSITIVE_JOINTLIMIT 160 * DEGTORAD
#define J2_NEGATIVE_JOINTLIMIT -1 * J2_POSITIVE_JOINTLIMIT
#define J3_UP_JOINTLIMIT 0
#define J3_DOWN_JOINTLIMIT -135.0
#define J4_POSITIVE_JOINTLIMIT 105 * DEGTORAD
#define J4_NEGATIVE_JOINTLIMIT -1 * J4_POSITIVE_JOINTLIMIT

extern float offset_x;
extern float offset_y;

void FPK(float *q,float *taskconfig);
void IPK(float *X, float gramma, float *jointconfig);
void IVK(float *q, float *dX, float *dq);
void chessboardtorobot(float xp, float yp, float chessboard_position, float *taskconfig);
void chessboardtemptochessboard(int target,float *x_position,float *y_position);
void pointinchessboardtomanipulator(int targetpoint,float chessboard_position,float *jointsetpoint);


#endif /* INC_MHAINW_KINEMATICS_H_ */
