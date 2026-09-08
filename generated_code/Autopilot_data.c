/*
 * Autopilot_data.c
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "Autopilot".
 *
 * Model version              : 1.487
 * Simulink Coder version : 25.1 (R2025a) 21-Nov-2024
 * C source code generated on : Sun Sep  6 22:15:06 2026
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
  /* Mask Parameter: PIDController1_D
   * Referenced by: '<S92>/Derivative Gain'
   */
  0.0,

  /* Mask Parameter: PIDController_D
   * Referenced by: '<S38>/Derivative Gain'
   */
  0.01,

  /* Mask Parameter: PIDController_I
   * Referenced by: '<S42>/Integral Gain'
   */
  1.0E-5,

  /* Mask Parameter: PIDController1_I
   * Referenced by: '<S96>/Integral Gain'
   */
  0.001,

  /* Mask Parameter: PIDController1_InitialCondition
   * Referenced by: '<S94>/Filter'
   */
  0.0,

  /* Mask Parameter: PIDController_InitialConditionF
   * Referenced by: '<S40>/Filter'
   */
  0.0,

  /* Mask Parameter: PIDController1_InitialConditi_a
   * Referenced by: '<S99>/Integrator'
   */
  0.0,

  /* Mask Parameter: PIDController_InitialConditio_n
   * Referenced by: '<S45>/Integrator'
   */
  0.0,

  /* Mask Parameter: PIDController1_LowerSaturationL
   * Referenced by:
   *   '<S106>/Saturation'
   *   '<S91>/DeadZone'
   */
  -0.73,

  /* Mask Parameter: PIDController_LowerSaturationLi
   * Referenced by:
   *   '<S52>/Saturation'
   *   '<S37>/DeadZone'
   */
  -0.1745,

  /* Mask Parameter: PIDController1_N
   * Referenced by: '<S102>/Filter Coefficient'
   */
  100.0,

  /* Mask Parameter: PIDController_N
   * Referenced by: '<S48>/Filter Coefficient'
   */
  115.919945582358,

  /* Mask Parameter: PIDController1_P
   * Referenced by: '<S104>/Proportional Gain'
   */
  0.01,

  /* Mask Parameter: PIDController_P
   * Referenced by: '<S50>/Proportional Gain'
   */
  0.01,

  /* Mask Parameter: PIDController1_UpperSaturationL
   * Referenced by:
   *   '<S106>/Saturation'
   *   '<S91>/DeadZone'
   */
  0.26,

  /* Mask Parameter: PIDController_UpperSaturationLi
   * Referenced by:
   *   '<S52>/Saturation'
   *   '<S37>/DeadZone'
   */
  0.2618,

  /* Expression: 0
   * Referenced by: '<S35>/Constant1'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<S89>/Constant1'
   */
  0.0,

  /* Expression: 184.999
   * Referenced by: '<S1>/velocity_trim2'
   */
  184.999,

  /* Computed Parameter: Integrator_gainval
   * Referenced by: '<S99>/Integrator'
   */
  0.008333,

  /* Computed Parameter: Filter_gainval
   * Referenced by: '<S94>/Filter'
   */
  0.008333,

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

  /* Expression: -1
   * Referenced by: '<S1>/Gain1'
   */
  -1.0,

  /* Expression: 1
   * Referenced by: '<S1>/Aileron Saturation'
   */
  1.0,

  /* Expression: -1
   * Referenced by: '<S1>/Aileron Saturation'
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
   * Referenced by: '<S1>/Initial Altitude'
   */
  4000.0,

  /* Expression: -3.28084
   * Referenced by: '<S1>/EKF_Altitude_ft'
   */
  -3.28084,

  /* Expression: 4000
   * Referenced by: '<S1>/Constant'
   */
  4000.0,

  /* Computed Parameter: Integrator_gainval_j
   * Referenced by: '<S45>/Integrator'
   */
  0.008333,

  /* Computed Parameter: Filter_gainval_d
   * Referenced by: '<S40>/Filter'
   */
  0.008333,

  /* Expression: 0.0026556289
   * Referenced by: '<S1>/Thita_trim'
   */
  0.0026556289,

  /* Expression: -1
   * Referenced by: '<S1>/Gain3'
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

  /* Expression: 57.2958
   * Referenced by: '<S1>/Multiply1'
   */
  57.2958,

  /* Expression: 0
   * Referenced by: '<S35>/Clamping_zero'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<S89>/Clamping_zero'
   */
  0.0,

  /* Computed Parameter: Constant_Value_f
   * Referenced by: '<S35>/Constant'
   */
  1,

  /* Computed Parameter: Constant2_Value
   * Referenced by: '<S35>/Constant2'
   */
  -1,

  /* Computed Parameter: Constant3_Value
   * Referenced by: '<S35>/Constant3'
   */
  1,

  /* Computed Parameter: Constant4_Value
   * Referenced by: '<S35>/Constant4'
   */
  -1,

  /* Computed Parameter: Constant_Value_b
   * Referenced by: '<S89>/Constant'
   */
  1,

  /* Computed Parameter: Constant2_Value_n
   * Referenced by: '<S89>/Constant2'
   */
  -1,

  /* Computed Parameter: Constant3_Value_a
   * Referenced by: '<S89>/Constant3'
   */
  1,

  /* Computed Parameter: Constant4_Value_o
   * Referenced by: '<S89>/Constant4'
   */
  -1
};
