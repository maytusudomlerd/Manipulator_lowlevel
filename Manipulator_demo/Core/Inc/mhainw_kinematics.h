/*
 * mhainw_kinematics.h
 *
 *  Created on: Feb 9, 2022
 *      Author: Maytus
 */

#ifndef INC_MHAINW_KINEMATICS_H_
#define INC_MHAINW_KINEMATICS_H_

#include "math.h"
#include "stdio.h"

void FPK(double *q,double *taskconfig);
void IPK(double *X, double gramma, double *jointconfig);



#endif /* INC_MHAINW_KINEMATICS_H_ */
