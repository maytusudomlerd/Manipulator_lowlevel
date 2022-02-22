/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * _coder_kalmanfilter_api.h
 *
 * Code generation for function 'kalmanfilter'
 *
 */

#ifndef _CODER_KALMANFILTER_API_H
#define _CODER_KALMANFILTER_API_H

/* Include files */
#include "emlrt.h"
#include "tmwtypes.h"
#include <string.h>

/* Variable Declarations */
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void kalmanfilter(real_T R, real_T Q, real_T dt, real_T theta, real_T x1,
                  real_T x2, real_T p11, real_T p12, real_T p21, real_T p22,
                  real_T *X1, real_T *X2, real_T *P11, real_T *P12, real_T *P21,
                  real_T *P22);

void kalmanfilter_api(const mxArray *const prhs[10], int32_T nlhs,
                      const mxArray *plhs[6]);

void kalmanfilter_atexit(void);

void kalmanfilter_initialize(void);

void kalmanfilter_terminate(void);

void kalmanfilter_xil_shutdown(void);

void kalmanfilter_xil_terminate(void);

#ifdef __cplusplus
}
#endif

#endif
/* End of code generation (_coder_kalmanfilter_api.h) */
