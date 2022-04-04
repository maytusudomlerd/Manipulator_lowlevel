/*
 * kinematics.c
 *
 *  Created on: Feb 22, 2022
 *      Author: K. Peerasate ,U. Maytus
 */
#include "mhainw_kinematics.h"

void FPK(float *q,float *taskconfig){
	float eulShaped_idx_2;
	float p_tmp;
	float p_tmp_tmp;
	float p_tmp_tmp_tmp;
	p_tmp_tmp_tmp = q[0] + q[1];
	p_tmp_tmp = p_tmp_tmp_tmp + q[3];
	p_tmp = sin(p_tmp_tmp);
	p_tmp_tmp = cos(p_tmp_tmp);
	eulShaped_idx_2 = atan2(p_tmp, p_tmp_tmp);
	if (sqrt(p_tmp_tmp * p_tmp_tmp + p_tmp * p_tmp) < 2.2204460492503131e-15) {
	eulShaped_idx_2 = 0.0;
	}
	taskconfig[0] = eulShaped_idx_2;
	taskconfig[1] = (260.0 * cos(p_tmp_tmp_tmp) + 310.0 * cos(q[0])) + 90.0 * p_tmp_tmp;
	taskconfig[2] = (260.0 * sin(p_tmp_tmp_tmp) + 310.0 * sin(q[0])) + 90.0 * p_tmp;
	taskconfig[3] = 224.95 - q[2];
	//memcpy(taskconfig,X,strlen(X)+1);
}
void IPK(float *X, float gramma, float *jointconfig)
{
  float b_c[9];
  float p_0w[3];
  float c;
  float q1;
  float s;
  float s2;
  int i;
  c = cos(X[0]);
  s = sin(X[0]);
  b_c[0] = c;
  b_c[3] = -s;
  b_c[6] = 0.0;
  b_c[1] = s;
  b_c[4] = c;
  b_c[7] = 0.0;
  b_c[2] = 0.0;
  b_c[5] = 0.0;
  b_c[8] = 1.0;
  c = -((405.0 - (X[3] + 180.05)) + 180.05);
  for (i = 0; i < 3; i++) {
    p_0w[i] = X[i + 1] - ((b_c[i] * 90.0 + b_c[i + 3] * 0.0) + b_c[i + 6] * c);
  }
  s = ((p_0w[0] * p_0w[0] + p_0w[1] * p_0w[1]) - 163700.0) / 161200.0;
  s2 = gramma * sqrt(1.0 - s * s);
  c = 260.0 * s + 310.0;
  q1 = atan2(-260.0 * s2 * p_0w[0] + c * p_0w[1],
                     c * p_0w[0] + 260.0 * s2 * p_0w[1]);
  c = atan2(s2, s);
  jointconfig[0] = q1;
  jointconfig[1] = c;
  jointconfig[2] = 405.0 - (X[3] + 180.05);
  jointconfig[3] = X[0] - (q1 + c);
}

/*
 *  *q is pointer that point to now position in configuration space
 *  *dX is pointer that point to step position in task space
 *  *dq is point that point to variable that keep result of IVK
 */
void IVK(float *q, float *dX, float *dq)
{
  float b_dq_tmp;
  float c_dq_tmp;
  float d_dq_tmp;
  float dq_tmp;
  float dq_tmp_tmp;
  float e_dq_tmp;
  float f_dq_tmp;
  dq_tmp_tmp = q[0] + q[1];
  dq_tmp = cos(dq_tmp_tmp);
  dq_tmp_tmp = sin(dq_tmp_tmp);
  b_dq_tmp = sin(q[3]);
  c_dq_tmp = sin(q[1]);
  d_dq_tmp = sin(q[1] + q[3]);
  e_dq_tmp = cos(q[0]);
  f_dq_tmp = sin(q[0]);
  dq[0] = ((dX[1] * dq_tmp + dX[2] * dq_tmp_tmp) + 90.0 * dX[0] * b_dq_tmp) /
    (310.0 * c_dq_tmp);
  dq[1] = -(((((26.0 * dX[1] * dq_tmp + 2790.0 * dX[0] * d_dq_tmp) + 26.0 * dX[2]
               * dq_tmp_tmp) + 31.0 * dX[1] * e_dq_tmp) + 2340.0 * dX[0] *
             b_dq_tmp) + 31.0 * dX[2] * f_dq_tmp) / (8060.0 * c_dq_tmp);
  dq[2] = -dX[3];
  dq[3] = (((90.0 * dX[0] * d_dq_tmp + dX[1] * e_dq_tmp) + 260.0 * dX[0] *
            c_dq_tmp) + dX[2] * f_dq_tmp) / (260.0 * c_dq_tmp);

}
void chessboardtorobot(float xp, float yp, float chessboard_position, float chessboard_offset, float *taskconfig)
{
	//position in x
	//position in y
	//theta of chessboard
	//offset of chessboard
	//taskconfig output of of function {rz x y z}
  float b_p_tmp;
  float eulShaped_idx_2;
  float p_tmp;
  float X[4] = {0};
  p_tmp = cos(chessboard_position);
  b_p_tmp = sin(chessboard_position);
  eulShaped_idx_2 = atan2(b_p_tmp, p_tmp);
  if (sqrt(p_tmp * p_tmp + b_p_tmp * b_p_tmp) < 2.2204460492503131E-15) {
    eulShaped_idx_2 = 0.0;
  }
  X[0] = eulShaped_idx_2;
  X[1] = (chessboard_offset + xp * p_tmp) - yp * b_p_tmp;
  X[2] = yp * p_tmp + xp * b_p_tmp;
  X[3] = 100.0;
  memcpy(taskconfig,X,strlen(X)+1);
}


//void chessboardtorobot position(Encoder enc,uint8_t hpos,uint8_t vpos,,float jointsetpoint){
//	float temp_taskconfig[4] = {0};
//	static float hchessboard[2][2] = {{1,1},
//			{1,1}};
////	static float hchessboard[8][8] = {{1,2,3,4,5,6,7,8},
////							{1,2,3,4,5,6,7,8},
////							{1,2,3,4,5,6,7,8},
////							{1,2,3,4,5,6,7,8},
////							{1,2,3,4,5,6,7,8},
////							{1,2,3,4,5,6,7,8},
////						    {1,2,3,4,5,6,7,8},
////					   	    {1,2,3,4,5,6,7,8}};
////	static float vchessboard[8][8] = {{1,2,3,4,5,6,7,8},
////								{1,2,3,4,5,6,7,8},
////								{1,2,3,4,5,6,7,8},
////								{1,2,3,4,5,6,7,8},
////								{1,2,3,4,5,6,7,8},
////								{1,2,3,4,5,6,7,8},
////							    {1,2,3,4,5,6,7,8},
//						   	    {1,2,3,4,5,6,7,8}};
////	chessboardtorobot(hchessboard[hpos][vpos],vchessboard[hpos][vpos],chessboardposition,chessboard.offset,temp_taskconfig);
////	IPK(temp_taskconfig,-1,jointsetpoint);
//}


