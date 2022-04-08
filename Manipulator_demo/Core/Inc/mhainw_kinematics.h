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

void FPK(float *q,float *taskconfig);
void IPK(float *X, float gramma, float *jointconfig);
void IVK(float *q, float *dX, float *dq);
void chessboardtorobot(float xp, float yp, float chessboard_position, float *taskconfig);
void chessboardtemptochessboard(int target,float *x_position,float *y_position);
void pointinchessboardtomanipulator(int targetpoint,float *jointsetpoint);


#endif /* INC_MHAINW_KINEMATICS_H_ */
