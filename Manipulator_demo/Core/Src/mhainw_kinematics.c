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
  jointconfig[2] = -1*(405.0 - (X[3] + 180.05));
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


void chessboardtorobot(float xp, float yp, float chessboard_position, float *taskconfig)
{
	/*
	function that transform pose of chessboard frame to manipulator end-effector frame
	xp = position in x axis that robot want to go
	yp = position in y axis that robot want to go
	chessboard_position = theta of chessboard (rad)
	chessboar_offset = offset of chessboard from base robot frame
	taskconfig = output of of the function that in configuration space {rz x y z}
	 */
  float b_p_tmp;
  float eulShaped_idx_2;
  float p_tmp;
  static float chessboard_offset = 370.0; //360
  p_tmp = cos(chessboard_position);
  b_p_tmp = sin(chessboard_position);
  eulShaped_idx_2 = atan2(b_p_tmp, p_tmp);
  if (sqrt(p_tmp * p_tmp + b_p_tmp * b_p_tmp) < 2.2204460492503131E-15) {
    eulShaped_idx_2 = 0.0;
  }
  taskconfig[0] = eulShaped_idx_2;
  taskconfig[1] = ((chessboard_offset + xp * p_tmp) - yp * b_p_tmp) + offset_x;
  taskconfig[2] = (yp * p_tmp + xp * b_p_tmp) + offset_y;
  taskconfig[3] = 100;


//  memcpy(taskconfig,X,strlen(X)+1);
}

void chessboardtemptochessboard(int target,float *x_position,float *y_position){
	/*
		function that transform pose of point in chessboard frame to chessboard frame
		[  1  2   3   4   5   6   7   8  ]
		[  9  10  11  12  13  14  15  16 ]
		[  17 18  19  20  21  22  23  24 ]
		[  25 26  27  28  29  30  31  32 ]
		[  33 34  35  36  37  38  39  40 ]
		[  41 42  43  44  45  46  47  48 ]
		[  49 50  51  52  53  54  55  56 ]
		[  57 58  59  60  61  62  63  64 ]
		target = point on chessboard {0-64}
		x_position = position in x axis of point on chessboard frame
		y_position = position in y axis of point on chessboard frame
	 */
    float x,y;
    for(int i = 8; i <= 64; i+=8){
        if(target > i-8 && target <= i){
            x = (i/8) < 5 ? 5-(i/8) : 4-(i/8);
            y = target % 8 == 0 ? 8 : target % 8;
            y = y < 5 ? 5-y : 4-y;
            x = x*50 >= 0 ? x*50-25 : x*50+25;
            y = y*50 >= 0 ? y*50-25 : y*50+25;
        }
    }
    *x_position = x;
    *y_position = y;
}

//add chessboard enc
void pointinchessboardtomanipulator(int targetpoint,float chessboard_position,float *jointsetpoint){
	int can_reach[4] = {0};
	float i=-1.57;
	float positive_jointlimit[4] = {J1_POSITIVE_JOINTLIMIT, J2_POSITIVE_JOINTLIMIT, J3_UP_JOINTLIMIT, J4_POSITIVE_JOINTLIMIT};
	float negative_jointlimit[4] = {J1_NEGATIVE_JOINTLIMIT, J2_NEGATIVE_JOINTLIMIT, J3_DOWN_JOINTLIMIT, J4_NEGATIVE_JOINTLIMIT};
	float chessboard_x,chessboard_y;
	float temp_tasksetpoint[4] = {0};
	float temp_jointsetpoint[4] = {0};
	chessboardtemptochessboard(targetpoint,&chessboard_x,&chessboard_y);
	chessboardtorobot(chessboard_x,chessboard_y,chessboard_position,temp_tasksetpoint);
	IPK(temp_tasksetpoint,-1,temp_jointsetpoint);

	while(1){

		IPK(temp_tasksetpoint,-1,temp_jointsetpoint);
		for(int i=0;i<4;i++){
			if(temp_jointsetpoint[i] >= positive_jointlimit[i] || temp_jointsetpoint[i] <= negative_jointlimit[i]){
				can_reach[i] = 0;
			}
			else{
				can_reach[i] = 1;
			}
		}
		if(can_reach[0] == 1 && can_reach[1] == 1 && can_reach[2] == 1 && can_reach[3] == 1){
			break;
		} else{
			if(temp_tasksetpoint[0] > 0){
				temp_tasksetpoint[0] = i;
			}else{
				temp_tasksetpoint[0] = i;
			}
		}
		i += 1.57 ;

	}

	jointsetpoint[0] = temp_jointsetpoint[0];
	jointsetpoint[1] = temp_jointsetpoint[1];
	jointsetpoint[2] = temp_jointsetpoint[2];
	jointsetpoint[3] = temp_jointsetpoint[3];
}


