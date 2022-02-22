/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * kalmanfilter.c
 *
 * Code generation for function 'kalmanfilter'
 *
 */

/* Include files */
#include "kalmanfilter.h"
#include "rt_nonfinite.h"
#include "rt_nonfinite.h"
#include <math.h>

/* Function Declarations */
static double rt_powd_snf(double u0, double u1);

/* Function Definitions */
static double rt_powd_snf(double u0, double u1)
{
  double d;
  double d1;
  double y;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = rtNaN;
  } else {
    d = fabs(u0);
    d1 = fabs(u1);
    if (rtIsInf(u1)) {
      if (d == 1.0) {
        y = 1.0;
      } else if (d > 1.0) {
        if (u1 > 0.0) {
          y = rtInf;
        } else {
          y = 0.0;
        }
      } else if (u1 > 0.0) {
        y = 0.0;
      } else {
        y = rtInf;
      }
    } else if (d1 == 0.0) {
      y = 1.0;
    } else if (d1 == 1.0) {
      if (u1 > 0.0) {
        y = u0;
      } else {
        y = 1.0 / u0;
      }
    } else if (u1 == 2.0) {
      y = u0 * u0;
    } else if ((u1 == 0.5) && (u0 >= 0.0)) {
      y = sqrt(u0);
    } else if ((u0 < 0.0) && (u1 > floor(u1))) {
      y = rtNaN;
    } else {
      y = pow(u0, u1);
    }
  }
  return y;
}

void kalmanfilter(double R, double Q, double dt, double theta, double x1,
                  double x2, double p11, double p12, double p21, double p22,
                  double *X1, double *X2, double *P11, double *P12, double *P21,
                  double *P22)
{
  double X1_tmp;
  double X1_tmp_tmp;
  double X2_tmp;
  double b_X1_tmp;
  double b_X2_tmp;
  double c_X1_tmp;
  double c_X2_tmp;
  double d_X1_tmp;
  double d_X2_tmp;
  double e_X1_tmp;
  X1_tmp = 4.0 * dt * p12;
  b_X1_tmp = 4.0 * dt * p21;
  c_X1_tmp = Q * rt_powd_snf(dt, 4.0);
  X1_tmp_tmp = dt * dt;
  d_X1_tmp = 4.0 * X1_tmp_tmp * p22;
  e_X1_tmp =
      ((((4.0 * R + 4.0 * p11) + X1_tmp) + b_X1_tmp) + c_X1_tmp) + d_X1_tmp;
  *X1 = ((((((4.0 * R * x1 + 4.0 * p11 * theta) + d_X1_tmp * theta) +
            4.0 * R * dt * x2) +
           X1_tmp * theta) +
          b_X1_tmp * theta) +
         c_X1_tmp * theta) /
        e_X1_tmp;
  X2_tmp = p22 * dt;
  b_X2_tmp = Q * rt_powd_snf(dt, 3.0);
  c_X2_tmp = b_X2_tmp / 2.0 + X2_tmp;
  d_X2_tmp = c_X2_tmp + p21;
  X2_tmp = (((R + p11) + dt * p21) + c_X1_tmp / 4.0) + dt * (p12 + X2_tmp);
  *X2 = x2 - d_X2_tmp * ((x1 - theta) + dt * x2) / X2_tmp;
  *P11 = R * ((((4.0 * p11 + X1_tmp) + b_X1_tmp) + c_X1_tmp) + d_X1_tmp) /
         e_X1_tmp;
  X1_tmp = b_X2_tmp + 2.0 * p22 * dt;
  *P12 = 2.0 * R * (X1_tmp + 2.0 * p12) / e_X1_tmp;
  *P21 = 2.0 * R * (X1_tmp + 2.0 * p21) / e_X1_tmp;
  *P22 = (p22 + Q * X1_tmp_tmp) - (c_X2_tmp + p12) * d_X2_tmp / X2_tmp;
}

/* End of code generation (kalmanfilter.c) */
