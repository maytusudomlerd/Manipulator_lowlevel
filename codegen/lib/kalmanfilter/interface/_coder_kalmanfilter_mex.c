/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * _coder_kalmanfilter_mex.c
 *
 * Code generation for function 'kalmanfilter'
 *
 */

/* Include files */
#include "_coder_kalmanfilter_mex.h"
#include "_coder_kalmanfilter_api.h"

/* Function Definitions */
void mexFunction(int32_T nlhs, mxArray *plhs[], int32_T nrhs,
                 const mxArray *prhs[])
{
  mexAtExit(&kalmanfilter_atexit);
  /* Module initialization. */
  kalmanfilter_initialize();
  /* Dispatch the entry-point. */
  unsafe_kalmanfilter_mexFunction(nlhs, plhs, nrhs, prhs);
  /* Module termination. */
  kalmanfilter_terminate();
}

emlrtCTX mexFunctionCreateRootTLS(void)
{
  emlrtCreateRootTLSR2021a(&emlrtRootTLSGlobal, &emlrtContextGlobal, NULL, 1,
                           NULL);
  return emlrtRootTLSGlobal;
}

void unsafe_kalmanfilter_mexFunction(int32_T nlhs, mxArray *plhs[6],
                                     int32_T nrhs, const mxArray *prhs[10])
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  const mxArray *outputs[6];
  int32_T b_nlhs;
  st.tls = emlrtRootTLSGlobal;
  /* Check for proper number of arguments. */
  if (nrhs != 10) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, 12, 10, 4,
                        12, "kalmanfilter");
  }
  if (nlhs > 6) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 12,
                        "kalmanfilter");
  }
  /* Call the function. */
  kalmanfilter_api(prhs, nlhs, outputs);
  /* Copy over outputs to the caller. */
  if (nlhs < 1) {
    b_nlhs = 1;
  } else {
    b_nlhs = nlhs;
  }
  emlrtReturnArrays(b_nlhs, &plhs[0], &outputs[0]);
}

/* End of code generation (_coder_kalmanfilter_mex.c) */
