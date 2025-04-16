/*
 * simulink_experiment_debug_type1_types.h
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "simulink_experiment_debug_type1".
 *
 * Model version              : 16.0
 * Simulink Coder version : 9.8 (R2022b) 13-May-2022
 * C source code generated on : Wed Apr 16 13:01:58 2025
 *
 * Target selection: quarc_win64.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef RTW_HEADER_simulink_experiment_debug_type1_types_h_
#define RTW_HEADER_simulink_experiment_debug_type1_types_h_
#include "rtwtypes.h"
#ifndef struct_tag_lHMM3wSRZNH1UM8wYrn1a
#define struct_tag_lHMM3wSRZNH1UM8wYrn1a

struct tag_lHMM3wSRZNH1UM8wYrn1a
{
  real_T K[4];
  real_T L[8];
  real_T A[16];
  real_T B[4];
  real_T C[8];
  real_T x_hat[4];
  real_T prev_u;
};

#endif                                 /* struct_tag_lHMM3wSRZNH1UM8wYrn1a */

#ifndef typedef_studentControllerInterface_si_T
#define typedef_studentControllerInterface_si_T

typedef struct tag_lHMM3wSRZNH1UM8wYrn1a studentControllerInterface_si_T;

#endif                             /* typedef_studentControllerInterface_si_T */

/* Parameters (default storage) */
typedef struct P_simulink_experiment_debug_t_T_ P_simulink_experiment_debug_t_T;

/* Forward declaration for rtModel */
typedef struct tag_RTM_simulink_experiment_d_T RT_MODEL_simulink_experiment__T;

#endif                 /* RTW_HEADER_simulink_experiment_debug_type1_types_h_ */
