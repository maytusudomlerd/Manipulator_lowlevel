/*
 * kinematics.c
 *
 *  Created on: Feb 22, 2022
 *      Author: K. Peerasate ,U. Maytus
 */
#include "math.h"

double *FPK(double q[]){
	double eulShaped_idx_2;
	double p_tmp;
	double p_tmp_tmp;
	double p_tmp_tmp_tmp;
	double X[4]= {0};
	p_tmp_tmp_tmp = q[0] + q[1];
	p_tmp_tmp = p_tmp_tmp_tmp + q[3];
	p_tmp = sin(p_tmp_tmp);
	p_tmp_tmp = cos(p_tmp_tmp);
	eulShaped_idx_2 = atan2(p_tmp, p_tmp_tmp);
	if (sqrt(p_tmp_tmp * p_tmp_tmp + p_tmp * p_tmp) < 2.2204460492503131E-15) {
	eulShaped_idx_2 = 0.0;
	}
	X[0] = eulShaped_idx_2;
	X[1] = (260.0 * cos(p_tmp_tmp_tmp) + 310.0 * cos(q[0])) + 90.0 * p_tmp_tmp;
	X[2] = (260.0 * sin(p_tmp_tmp_tmp) + 310.0 * sin(q[0])) + 90.0 * p_tmp;
	X[3] = 224.95 - q[2];
	return X;
}
