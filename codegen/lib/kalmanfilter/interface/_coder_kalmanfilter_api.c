/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * _coder_kalmanfilter_api.c
 *
 * Code generation for function 'kalmanfilter'
 *
 */

/* Include files */
#include "_coder_kalmanfilter_api.h"
#include "_coder_kalmanfilter_mex.h"

/* Variable Definitions */
emlrtCTX emlrtRootTLSGlobal = NULL;

emlrtContext emlrtContextGlobal = {
    true,                                                 /* bFirstTime */
    false,                                                /* bInitialized */
    131611U,                                              /* fVersionInfo */
    NULL,                                                 /* fErrorFunction */
    "kalmanfilter",                                       /* fFunctionName */
    NULL,                                                 /* fRTCallStack */
    false,                                                /* bDebugMode */
    {2045744189U, 2170104910U, 2743257031U, 4284093946U}, /* fSigWrd */
    NULL                                                  /* fSigMem */
};

/* Function Declarations */
static real_T b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                 const emlrtMsgIdentifier *parentId);

static real_T c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                 const emlrtMsgIdentifier *msgId);

static real_T emlrt_marshallIn(const emlrtStack *sp, const mxArray *R,
                               const char_T *identifier);

static const mxArray *emlrt_marshallOut(const real_T u);

/* Function Definitions */
static real_T b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                 const emlrtMsgIdentifier *parentId)
{
  real_T y;
  y = c_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static real_T c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                 const emlrtMsgIdentifier *msgId)
{
  static const int32_T dims = 0;
  real_T ret;
  emlrtCheckBuiltInR2012b((emlrtCTX)sp, msgId, src, (const char_T *)"double",
                          false, 0U, (void *)&dims);
  ret = *(real_T *)emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

static real_T emlrt_marshallIn(const emlrtStack *sp, const mxArray *R,
                               const char_T *identifier)
{
  emlrtMsgIdentifier thisId;
  real_T y;
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = b_emlrt_marshallIn(sp, emlrtAlias(R), &thisId);
  emlrtDestroyArray(&R);
  return y;
}

static const mxArray *emlrt_marshallOut(const real_T u)
{
  const mxArray *m;
  const mxArray *y;
  y = NULL;
  m = emlrtCreateDoubleScalar(u);
  emlrtAssign(&y, m);
  return y;
}

void kalmanfilter_api(const mxArray *const prhs[10], int32_T nlhs,
                      const mxArray *plhs[6])
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  real_T P11;
  real_T P12;
  real_T P21;
  real_T P22;
  real_T Q;
  real_T R;
  real_T X1;
  real_T X2;
  real_T dt;
  real_T p11;
  real_T p12;
  real_T p21;
  real_T p22;
  real_T theta;
  real_T x1;
  real_T x2;
  st.tls = emlrtRootTLSGlobal;
  /* Marshall function inputs */
  R = emlrt_marshallIn(&st, emlrtAliasP(prhs[0]), "R");
  Q = emlrt_marshallIn(&st, emlrtAliasP(prhs[1]), "Q");
  dt = emlrt_marshallIn(&st, emlrtAliasP(prhs[2]), "dt");
  theta = emlrt_marshallIn(&st, emlrtAliasP(prhs[3]), "theta");
  x1 = emlrt_marshallIn(&st, emlrtAliasP(prhs[4]), "x1");
  x2 = emlrt_marshallIn(&st, emlrtAliasP(prhs[5]), "x2");
  p11 = emlrt_marshallIn(&st, emlrtAliasP(prhs[6]), "p11");
  p12 = emlrt_marshallIn(&st, emlrtAliasP(prhs[7]), "p12");
  p21 = emlrt_marshallIn(&st, emlrtAliasP(prhs[8]), "p21");
  p22 = emlrt_marshallIn(&st, emlrtAliasP(prhs[9]), "p22");
  /* Invoke the target function */
  kalmanfilter(R, Q, dt, theta, x1, x2, p11, p12, p21, p22, &X1, &X2, &P11,
               &P12, &P21, &P22);
  /* Marshall function outputs */
  plhs[0] = emlrt_marshallOut(X1);
  if (nlhs > 1) {
    plhs[1] = emlrt_marshallOut(X2);
  }
  if (nlhs > 2) {
    plhs[2] = emlrt_marshallOut(P11);
  }
  if (nlhs > 3) {
    plhs[3] = emlrt_marshallOut(P12);
  }
  if (nlhs > 4) {
    plhs[4] = emlrt_marshallOut(P21);
  }
  if (nlhs > 5) {
    plhs[5] = emlrt_marshallOut(P22);
  }
}

void kalmanfilter_atexit(void)
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  emlrtEnterRtStackR2012b(&st);
  emlrtLeaveRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
  kalmanfilter_xil_terminate();
  kalmanfilter_xil_shutdown();
  emlrtExitTimeCleanup(&emlrtContextGlobal);
}

void kalmanfilter_initialize(void)
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  emlrtClearAllocCountR2012b(&st, false, 0U, NULL);
  emlrtEnterRtStackR2012b(&st);
  emlrtFirstTimeR2012b(emlrtRootTLSGlobal);
}

void kalmanfilter_terminate(void)
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  st.tls = emlrtRootTLSGlobal;
  emlrtLeaveRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
}

/* End of code generation (_coder_kalmanfilter_api.c) */
