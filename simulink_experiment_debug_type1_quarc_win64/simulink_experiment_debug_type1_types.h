/*
 * simulink_experiment_debug_type1_types.h
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "simulink_experiment_debug_type1".
 *
 * Model version              : 16.5
 * Simulink Coder version : 9.8 (R2022b) 13-May-2022
 * C source code generated on : Wed Apr 16 13:58:36 2025
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
#ifndef struct_tag_4B5c9t1JtSUztS9hrJrCFG
#define struct_tag_4B5c9t1JtSUztS9hrJrCFG

struct tag_4B5c9t1JtSUztS9hrJrCFG
{
  int32_T isInitialized;
  boolean_T isSetupComplete;
  real_T pCumSum;
  real_T pCumSumRev[9];
  real_T pCumRevIndex;
  real_T pModValueRev;
};

#endif                                 /* struct_tag_4B5c9t1JtSUztS9hrJrCFG */

#ifndef typedef_g_dsp_internal_SlidingWindowA_T
#define typedef_g_dsp_internal_SlidingWindowA_T

typedef struct tag_4B5c9t1JtSUztS9hrJrCFG g_dsp_internal_SlidingWindowA_T;

#endif                             /* typedef_g_dsp_internal_SlidingWindowA_T */

#ifndef struct_tag_UBv5A1Kv8Iz5H6UPwIDAmH
#define struct_tag_UBv5A1Kv8Iz5H6UPwIDAmH

struct tag_UBv5A1Kv8Iz5H6UPwIDAmH
{
  real_T K[4];
  real_T prev_p_ball;
  real_T prev_theta;
  real_T prev_p_ball_ref;
  real_T prev_t;
};

#endif                                 /* struct_tag_UBv5A1Kv8Iz5H6UPwIDAmH */

#ifndef typedef_studentControllerInterface_si_T
#define typedef_studentControllerInterface_si_T

typedef struct tag_UBv5A1Kv8Iz5H6UPwIDAmH studentControllerInterface_si_T;

#endif                             /* typedef_studentControllerInterface_si_T */

#ifndef struct_tag_BlgwLpgj2bjudmbmVKWwDE
#define struct_tag_BlgwLpgj2bjudmbmVKWwDE

struct tag_BlgwLpgj2bjudmbmVKWwDE
{
  uint32_T f1[8];
};

#endif                                 /* struct_tag_BlgwLpgj2bjudmbmVKWwDE */

#ifndef typedef_cell_wrap_simulink_experiment_T
#define typedef_cell_wrap_simulink_experiment_T

typedef struct tag_BlgwLpgj2bjudmbmVKWwDE cell_wrap_simulink_experiment_T;

#endif                             /* typedef_cell_wrap_simulink_experiment_T */

#ifndef struct_tag_ps8Pv2DMyDxNpj6T0q1gJE
#define struct_tag_ps8Pv2DMyDxNpj6T0q1gJE

struct tag_ps8Pv2DMyDxNpj6T0q1gJE
{
  boolean_T matlabCodegenIsDeleted;
  int32_T isInitialized;
  boolean_T isSetupComplete;
  boolean_T TunablePropsChanged;
  cell_wrap_simulink_experiment_T inputVarSize;
  g_dsp_internal_SlidingWindowA_T *pStatistic;
  int32_T NumChannels;
  int32_T FrameLength;
  g_dsp_internal_SlidingWindowA_T _pobj0;
};

#endif                                 /* struct_tag_ps8Pv2DMyDxNpj6T0q1gJE */

#ifndef typedef_dsp_simulink_MovingAverage_si_T
#define typedef_dsp_simulink_MovingAverage_si_T

typedef struct tag_ps8Pv2DMyDxNpj6T0q1gJE dsp_simulink_MovingAverage_si_T;

#endif                             /* typedef_dsp_simulink_MovingAverage_si_T */

/* Parameters (default storage) */
typedef struct P_simulink_experiment_debug_t_T_ P_simulink_experiment_debug_t_T;

/* Forward declaration for rtModel */
typedef struct tag_RTM_simulink_experiment_d_T RT_MODEL_simulink_experiment__T;

#endif                 /* RTW_HEADER_simulink_experiment_debug_type1_types_h_ */
