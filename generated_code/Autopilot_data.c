/*
 * Autopilot_data.c
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "Autopilot".
 *
 * Model version              : 1.340
 * Simulink Coder version : 25.1 (R2025a) 21-Nov-2024
 * C source code generated on : Thu Aug 13 04:39:41 2026
 *
 * Target selection: grt.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "Autopilot.h"

/* Block parameters (default storage) */
P_Autopilot_T Autopilot_P = {
  /* Variable: K
   * Referenced by: '<S1>/Gain1'
   */
  { 0.020526339208197478, 2.2684617190769685, -7.4922949776130991,
    -2.1181780314975476 },

  /* Mask Parameter: PIDController1_D
   * Referenced by: '<S87>/Derivative Gain'
   */
  0.0,

  /* Mask Parameter: PIDController_D
   * Referenced by: '<S33>/Derivative Gain'
   */
  0.001,

  /* Mask Parameter: PIDController_I
   * Referenced by: '<S37>/Integral Gain'
   */
  0.0001,

  /* Mask Parameter: PIDController1_I
   * Referenced by: '<S91>/Integral Gain'
   */
  0.001,

  /* Mask Parameter: PIDController1_InitialCondition
   * Referenced by: '<S89>/Filter'
   */
  0.0,

  /* Mask Parameter: PIDController_InitialConditionF
   * Referenced by: '<S35>/Filter'
   */
  0.0,

  /* Mask Parameter: PIDController1_InitialConditi_l
   * Referenced by: '<S94>/Integrator'
   */
  0.0,

  /* Mask Parameter: PIDController_InitialConditio_j
   * Referenced by: '<S40>/Integrator'
   */
  0.0,

  /* Mask Parameter: PIDController1_LowerSaturationL
   * Referenced by:
   *   '<S101>/Saturation'
   *   '<S86>/DeadZone'
   */
  -0.73,

  /* Mask Parameter: PIDController_LowerSaturationLi
   * Referenced by:
   *   '<S47>/Saturation'
   *   '<S32>/DeadZone'
   */
  -0.1745,

  /* Mask Parameter: PIDController1_N
   * Referenced by: '<S97>/Filter Coefficient'
   */
  100.0,

  /* Mask Parameter: PIDController_N
   * Referenced by: '<S43>/Filter Coefficient'
   */
  115.919945582358,

  /* Mask Parameter: PIDController1_P
   * Referenced by: '<S99>/Proportional Gain'
   */
  0.01,

  /* Mask Parameter: PIDController_P
   * Referenced by: '<S45>/Proportional Gain'
   */
  0.01,

  /* Mask Parameter: PIDController1_UpperSaturationL
   * Referenced by:
   *   '<S101>/Saturation'
   *   '<S86>/DeadZone'
   */
  0.26,

  /* Mask Parameter: PIDController_UpperSaturationLi
   * Referenced by:
   *   '<S47>/Saturation'
   *   '<S32>/DeadZone'
   */
  0.2618,

  /* Expression: 0
   * Referenced by: '<S30>/Constant1'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<S84>/Constant1'
   */
  0.0,

  /* Expression: 184.999
   * Referenced by: '<S1>/velocity_trim2'
   */
  184.999,

  /* Expression: 0.736296
   * Referenced by: '<S1>/Throttle'
   */
  0.736296,

  /* Expression: 1
   * Referenced by: '<S1>/Saturation1'
   */
  1.0,

  /* Expression: 0
   * Referenced by: '<S1>/Saturation1'
   */
  0.0,

  /* Expression: -1.0
   * Referenced by: '<S1>/Gain2'
   */
  -1.0,

  /* Expression: 184.999
   * Referenced by: '<S1>/velocity_trim1'
   */
  184.999,

  /* Expression: 0.0026556289
   * Referenced by: '<S1>/alpha_trim'
   */
  0.0026556289,

  /* Expression: 4000
   * Referenced by: '<S1>/Constant'
   */
  4000.0,

  /* Computed Parameter: Integrator_gainval
   * Referenced by: '<S40>/Integrator'
   */
  0.008333,

  /* Computed Parameter: Filter_gainval
   * Referenced by: '<S35>/Filter'
   */
  0.008333,

  /* Expression: 0.0026556289
   * Referenced by: '<S1>/Thita_trim'
   */
  0.0026556289,

  /* Expression: -1
   * Referenced by: '<S1>/Gain'
   */
  -1.0,

  /* Expression: 1
   * Referenced by: '<S1>/Saturation'
   */
  1.0,

  /* Expression: -1
   * Referenced by: '<S1>/Saturation'
   */
  -1.0,

  /* Expression: 1
   * Referenced by: '<S1>/Saturation2'
   */
  1.0,

  /* Expression: -1
   * Referenced by: '<S1>/Saturation2'
   */
  -1.0,

  /* Expression: 57.2958
   * Referenced by: '<S1>/Multiply1'
   */
  57.2958,

  /* Expression: 0
   * Referenced by: '<S30>/Clamping_zero'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<S84>/ZeroGain'
   */
  0.0,

  /* Computed Parameter: Memory_InitialCondition
   * Referenced by: '<S84>/Memory'
   */
  false,

  /* Computed Parameter: Constant_Value_o
   * Referenced by: '<S30>/Constant'
   */
  1,

  /* Computed Parameter: Constant2_Value
   * Referenced by: '<S30>/Constant2'
   */
  -1,

  /* Computed Parameter: Constant3_Value
   * Referenced by: '<S30>/Constant3'
   */
  1,

  /* Computed Parameter: Constant4_Value
   * Referenced by: '<S30>/Constant4'
   */
  -1
};
