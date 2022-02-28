///*
// * mhainw_trajectory.c
// *
// *  Created on: Feb 28, 2022
// *      Author: K. Peerasate & U. Maytus
// */
//#include "math.h"
//
//typedef struct{
//	float T,dt;
//	float q;
//	float dq;
//	float ddq;
//	float Vmax;
//}Trajectory;
//
//void mhainw_trajectory_init(Trajectory *traj,float T,float dt,float q,float dq,float ddq,float Vmax){
//	traj->T = T;
//	traj->dt = dt;
//	traj->q = q;
//	traj->dq = dq;
//	traj->ddq = ddq;
//	traj->Vmax =Vmax;
//}
//
//void mhainw_trajectory_updatetraj(Trajectory *traj,float *q_i, float *q_f, float *dq_i, float *dq_f, float *ddq_i,
//                float *ddq_f, float *Tk, float *T)
//{
//  float b_c3_tmp;
//  float b_q_tmp;
//  float c2;
//  float c3;
//  float c3_tmp;
//  float c4;
//  float c4_tmp;
//  float c5;
//  float c5_tmp;
//  float q_tmp;
//  float Q = 0;
//  float Dq = 0;
//  float Ddq = 0;
//  float vmax = 0;
//  float T_pow3 = T * T * T;
//  float T_pow4 = T * T * T * T;
//  float T_pow5 = T * T * T * T * T;
//  float Tk_pow3 = Tk * Tk * Tk;
//  float Tk_pow4 = Tk * Tk * Tk * Tk;
//  float Tk_pow5 = Tk * Tk * Tk * Tk * Tk;
//
//  c2 = ddq_i / 2.0;
//  c5 = Tk * Tk;
//  c3_tmp = 3.0 * ddq_i * c5;
//  b_c3_tmp = ddq_f * c5;
//  c3 = -(((((20.0 * q_i - 20.0 * q_f) + 8.0 * Tk * dq_f) + 12.0 * Tk * dq_i) -
//          b_c3_tmp) +
//         c3_tmp) /
//       (2.0 * Tk_pow3);
//  c4_tmp = 14.0 * Tk * dq_f;
//  c4 = (((((30.0 * q_i - 30.0 * q_f) + c4_tmp) + 16.0 * Tk * dq_i) -
//         2.0 * ddq_f * c5) +
//        c3_tmp) /
//       (2.0 * Tk_pow4);
//  c5_tmp = ddq_i * c5;
//  c5 = -(((((12.0 * q_i - 12.0 * q_f) + 6.0 * Tk * dq_f) + 6.0 * Tk * dq_i) -
//          b_c3_tmp) +
//         c5_tmp) /
//       (2.0 * Tk_pow5);
//  q_tmp = T * T;
//  b_q_tmp = T_pow3;
//  c3_tmp = T_pow4;
//  traj->q = ((((q_i + dq_i * T) + c2 * q_tmp) + c3 * b_q_tmp) + c4 * c3_tmp) +
//       c5 * T_pow5;
//  traj->dq = (((dq_i + 2.0 * c2 * T) + 3.0 * c3 * q_tmp) + 4.0 * c4 * b_q_tmp) +
//        5.0 * c5 * c3_tmp;
//  traj->ddq = ((2.0 * c2 + 6.0 * c3 * T) + 12.0 * c4 * q_tmp) + 20.0 * c5 * b_q_tmp;
//  traj->Vmax =
//      (((((60.0 * q_f - 60.0 * q_i) - c4_tmp) - 14.0 * Tk * dq_i) + b_c3_tmp) -
//       c5_tmp) /
//      (32.0 * Tk);
//}
//
