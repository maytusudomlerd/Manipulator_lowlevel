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


#endif /* INC_MHAINW_KINEMATICS_H_ */
