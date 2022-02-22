/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * kalmanfilter.h
 *
 * Code generation for function 'kalmanfilter'
 *
 */

#ifndef KALMANFILTER_H
#define KALMANFILTER_H

/* Include files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
extern void kalmanfilter(double R, double Q, double dt, double theta, double x1,
                         double x2, double p11, double p12, double p21,
                         double p22, double *X1, double *X2, double *P11,
                         double *P12, double *P21, double *P22);

#ifdef __cplusplus
}
#endif

#endif
/* End of code generation (kalmanfilter.h) */
