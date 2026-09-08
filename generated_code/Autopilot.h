/*
 * Autopilot.h
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

#ifndef Autopilot_h_
#define Autopilot_h_
#ifndef Autopilot_COMMON_INCLUDES_
#define Autopilot_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "rt_logging.h"
#include "rt_nonfinite.h"
#include "math.h"
#endif                                 /* Autopilot_COMMON_INCLUDES_ */

#include "Autopilot_types.h"
#include "rtGetNaN.h"
#include <float.h>
#include <string.h>
#include <stddef.h>

/* Macros for accessing real-time model data structure */
#ifndef rtmGetFinalTime
#define rtmGetFinalTime(rtm)           ((rtm)->Timing.tFinal)
#endif

#ifndef rtmGetRTWLogInfo
#define rtmGetRTWLogInfo(rtm)          ((rtm)->rtwLogInfo)
#endif

#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

#ifndef rtmGetStopRequested
#define rtmGetStopRequested(rtm)       ((rtm)->Timing.stopRequestedFlag)
#endif

#ifndef rtmSetStopRequested
#define rtmSetStopRequested(rtm, val)  ((rtm)->Timing.stopRequestedFlag = (val))
#endif

#ifndef rtmGetStopRequestedPtr
#define rtmGetStopRequestedPtr(rtm)    (&((rtm)->Timing.stopRequestedFlag))
#endif

#ifndef rtmGetT
#define rtmGetT(rtm)                   ((rtm)->Timing.taskTime0)
#endif

#ifndef rtmGetTFinal
#define rtmGetTFinal(rtm)              ((rtm)->Timing.tFinal)
#endif

#ifndef rtmGetTPtr
#define rtmGetTPtr(rtm)                (&(rtm)->Timing.taskTime0)
#endif

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real_T Integrator_DSTATE;            /* '<S99>/Integrator' */
  real_T Filter_DSTATE;                /* '<S94>/Filter' */
  real_T Integrator_DSTATE_k;          /* '<S45>/Integrator' */
  real_T Filter_DSTATE_e;              /* '<S40>/Filter' */
  real_T x[12];                        /* '<S1>/Navigation_EKF' */
  real_T P[144];                       /* '<S1>/Navigation_EKF' */
  int32_T waypoint_index;              /* '<S1>/L1_Guidance' */
  boolean_T initialized_not_empty;     /* '<S1>/Navigation_EKF' */
} DW_Autopilot_T;

/* External inputs (root inport signals with default storage) */
typedef struct {
  real_T alpha_meas;                   /* '<Root>/alpha_meas' */
  real_T U_meas;                       /* '<Root>/U_meas' */
  real_T U_meas_l;                     /* '<Root>/U_meas1' */
  real_T imu[6];                       /* '<Root>/imu' */
  real_T gps[3];                       /* '<Root>/gps' */
  real_T dt;                           /* '<Root>/dt' */
} ExtU_Autopilot_T;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real_T Throttle_cmd;                 /* '<Root>/Throttle_cmd' */
  real_T AileronCommand_DLQR;          /* '<Root>/Aileron Command_DLQR' */
  real_T ElevatorCommand;              /* '<Root>/Elevator Command' */
} ExtY_Autopilot_T;

/* Parameters (default storage) */
struct P_Autopilot_T_ {
  real_T PIDController1_D;             /* Mask Parameter: PIDController1_D
                                        * Referenced by: '<S92>/Derivative Gain'
                                        */
  real_T PIDController_D;              /* Mask Parameter: PIDController_D
                                        * Referenced by: '<S38>/Derivative Gain'
                                        */
  real_T PIDController_I;              /* Mask Parameter: PIDController_I
                                        * Referenced by: '<S42>/Integral Gain'
                                        */
  real_T PIDController1_I;             /* Mask Parameter: PIDController1_I
                                        * Referenced by: '<S96>/Integral Gain'
                                        */
  real_T PIDController1_InitialCondition;
                              /* Mask Parameter: PIDController1_InitialCondition
                               * Referenced by: '<S94>/Filter'
                               */
  real_T PIDController_InitialConditionF;
                              /* Mask Parameter: PIDController_InitialConditionF
                               * Referenced by: '<S40>/Filter'
                               */
  real_T PIDController1_InitialConditi_a;
                              /* Mask Parameter: PIDController1_InitialConditi_a
                               * Referenced by: '<S99>/Integrator'
                               */
  real_T PIDController_InitialConditio_n;
                              /* Mask Parameter: PIDController_InitialConditio_n
                               * Referenced by: '<S45>/Integrator'
                               */
  real_T PIDController1_LowerSaturationL;
                              /* Mask Parameter: PIDController1_LowerSaturationL
                               * Referenced by:
                               *   '<S106>/Saturation'
                               *   '<S91>/DeadZone'
                               */
  real_T PIDController_LowerSaturationLi;
                              /* Mask Parameter: PIDController_LowerSaturationLi
                               * Referenced by:
                               *   '<S52>/Saturation'
                               *   '<S37>/DeadZone'
                               */
  real_T PIDController1_N;             /* Mask Parameter: PIDController1_N
                                        * Referenced by: '<S102>/Filter Coefficient'
                                        */
  real_T PIDController_N;              /* Mask Parameter: PIDController_N
                                        * Referenced by: '<S48>/Filter Coefficient'
                                        */
  real_T PIDController1_P;             /* Mask Parameter: PIDController1_P
                                        * Referenced by: '<S104>/Proportional Gain'
                                        */
  real_T PIDController_P;              /* Mask Parameter: PIDController_P
                                        * Referenced by: '<S50>/Proportional Gain'
                                        */
  real_T PIDController1_UpperSaturationL;
                              /* Mask Parameter: PIDController1_UpperSaturationL
                               * Referenced by:
                               *   '<S106>/Saturation'
                               *   '<S91>/DeadZone'
                               */
  real_T PIDController_UpperSaturationLi;
                              /* Mask Parameter: PIDController_UpperSaturationLi
                               * Referenced by:
                               *   '<S52>/Saturation'
                               *   '<S37>/DeadZone'
                               */
  real_T Constant1_Value;              /* Expression: 0
                                        * Referenced by: '<S35>/Constant1'
                                        */
  real_T Constant1_Value_h;            /* Expression: 0
                                        * Referenced by: '<S89>/Constant1'
                                        */
  real_T velocity_trim2_Value;         /* Expression: 184.999
                                        * Referenced by: '<S1>/velocity_trim2'
                                        */
  real_T Integrator_gainval;           /* Computed Parameter: Integrator_gainval
                                        * Referenced by: '<S99>/Integrator'
                                        */
  real_T Filter_gainval;               /* Computed Parameter: Filter_gainval
                                        * Referenced by: '<S94>/Filter'
                                        */
  real_T Throttle_Value;               /* Expression: 0.736296
                                        * Referenced by: '<S1>/Throttle'
                                        */
  real_T Saturation1_UpperSat;         /* Expression: 1
                                        * Referenced by: '<S1>/Saturation1'
                                        */
  real_T Saturation1_LowerSat;         /* Expression: 0
                                        * Referenced by: '<S1>/Saturation1'
                                        */
  real_T Gain1_Gain;                   /* Expression: -1
                                        * Referenced by: '<S1>/Gain1'
                                        */
  real_T AileronSaturation_UpperSat;   /* Expression: 1
                                        * Referenced by: '<S1>/Aileron Saturation'
                                        */
  real_T AileronSaturation_LowerSat;   /* Expression: -1
                                        * Referenced by: '<S1>/Aileron Saturation'
                                        */
  real_T velocity_trim1_Value;         /* Expression: 184.999
                                        * Referenced by: '<S1>/velocity_trim1'
                                        */
  real_T alpha_trim_Value;             /* Expression: 0.0026556289
                                        * Referenced by: '<S1>/alpha_trim'
                                        */
  real_T InitialAltitude_Value;        /* Expression: 4000
                                        * Referenced by: '<S1>/Initial Altitude'
                                        */
  real_T EKF_Altitude_ft_Gain;         /* Expression: -3.28084
                                        * Referenced by: '<S1>/EKF_Altitude_ft'
                                        */
  real_T Constant_Value;               /* Expression: 4000
                                        * Referenced by: '<S1>/Constant'
                                        */
  real_T Integrator_gainval_j;       /* Computed Parameter: Integrator_gainval_j
                                      * Referenced by: '<S45>/Integrator'
                                      */
  real_T Filter_gainval_d;             /* Computed Parameter: Filter_gainval_d
                                        * Referenced by: '<S40>/Filter'
                                        */
  real_T Thita_trim_Value;             /* Expression: 0.0026556289
                                        * Referenced by: '<S1>/Thita_trim'
                                        */
  real_T Gain3_Gain;                   /* Expression: -1
                                        * Referenced by: '<S1>/Gain3'
                                        */
  real_T Saturation_UpperSat;          /* Expression: 1
                                        * Referenced by: '<S1>/Saturation'
                                        */
  real_T Saturation_LowerSat;          /* Expression: -1
                                        * Referenced by: '<S1>/Saturation'
                                        */
  real_T Multiply1_Gain;               /* Expression: 57.2958
                                        * Referenced by: '<S1>/Multiply1'
                                        */
  real_T Clamping_zero_Value;          /* Expression: 0
                                        * Referenced by: '<S35>/Clamping_zero'
                                        */
  real_T Clamping_zero_Value_d;        /* Expression: 0
                                        * Referenced by: '<S89>/Clamping_zero'
                                        */
  int8_T Constant_Value_f;             /* Computed Parameter: Constant_Value_f
                                        * Referenced by: '<S35>/Constant'
                                        */
  int8_T Constant2_Value;              /* Computed Parameter: Constant2_Value
                                        * Referenced by: '<S35>/Constant2'
                                        */
  int8_T Constant3_Value;              /* Computed Parameter: Constant3_Value
                                        * Referenced by: '<S35>/Constant3'
                                        */
  int8_T Constant4_Value;              /* Computed Parameter: Constant4_Value
                                        * Referenced by: '<S35>/Constant4'
                                        */
  int8_T Constant_Value_b;             /* Computed Parameter: Constant_Value_b
                                        * Referenced by: '<S89>/Constant'
                                        */
  int8_T Constant2_Value_n;            /* Computed Parameter: Constant2_Value_n
                                        * Referenced by: '<S89>/Constant2'
                                        */
  int8_T Constant3_Value_a;            /* Computed Parameter: Constant3_Value_a
                                        * Referenced by: '<S89>/Constant3'
                                        */
  int8_T Constant4_Value_o;            /* Computed Parameter: Constant4_Value_o
                                        * Referenced by: '<S89>/Constant4'
                                        */
};

/* Real-time Model Data Structure */
struct tag_RTM_Autopilot_T {
  const char_T *errorStatus;
  RTWLogInfo *rtwLogInfo;

  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    time_T taskTime0;
    uint32_T clockTick0;
    uint32_T clockTickH0;
    time_T stepSize0;
    time_T tFinal;
    boolean_T stopRequestedFlag;
  } Timing;
};

/* Block parameters (default storage) */
extern P_Autopilot_T Autopilot_P;

/* Block states (default storage) */
extern DW_Autopilot_T Autopilot_DW;

/* External inputs (root inport signals with default storage) */
extern ExtU_Autopilot_T Autopilot_U;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY_Autopilot_T Autopilot_Y;

/* Model entry point functions */
extern void Autopilot_initialize(void);
extern void Autopilot_step(void);
extern void Autopilot_terminate(void);

/* Real-time Model object */
extern RT_MODEL_Autopilot_T *const Autopilot_M;

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Note that this particular code originates from a subsystem build,
 * and has its own system numbers different from the parent model.
 * Refer to the system hierarchy for this subsystem below, and use the
 * MATLAB hilite_system command to trace the generated code back
 * to the parent model.  For example,
 *
 * hilite_system('Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy/Autopilot')    - opens subsystem Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy/Autopilot
 * hilite_system('Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy/Autopilot/Kp') - opens and selects block Kp
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy'
 * '<S1>'   : 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy/Autopilot'
 * '<S2>'   : 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy/Autopilot/GPS_Validity'
 * '<S3>'   : 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy/Autopilot/L1_Guidance'
 * '<S4>'   : 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy/Autopilot/MATLAB Function'
 * '<S5>'   : 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy/Autopilot/MATLAB Function1'
 * '<S6>'   : 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy/Autopilot/Navigation_EKF'
 * '<S7>'   : 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy/Autopilot/PID Controller'
 * '<S8>'   : 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy/Autopilot/PID Controller1'
 * '<S9>'   : 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy/Autopilot/PID Controller/Anti-windup'
 * '<S10>'  : 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy/Autopilot/PID Controller/D Gain'
 * '<S11>'  : 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy/Autopilot/PID Controller/External Derivative'
 * '<S12>'  : 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy/Autopilot/PID Controller/Filter'
 * '<S13>'  : 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy/Autopilot/PID Controller/Filter ICs'
 * '<S14>'  : 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy/Autopilot/PID Controller/I Gain'
 * '<S15>'  : 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy/Autopilot/PID Controller/Ideal P Gain'
 * '<S16>'  : 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy/Autopilot/PID Controller/Ideal P Gain Fdbk'
 * '<S17>'  : 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy/Autopilot/PID Controller/Integrator'
 * '<S18>'  : 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy/Autopilot/PID Controller/Integrator ICs'
 * '<S19>'  : 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy/Autopilot/PID Controller/N Copy'
 * '<S20>'  : 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy/Autopilot/PID Controller/N Gain'
 * '<S21>'  : 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy/Autopilot/PID Controller/P Copy'
 * '<S22>'  : 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy/Autopilot/PID Controller/Parallel P Gain'
 * '<S23>'  : 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy/Autopilot/PID Controller/Reset Signal'
 * '<S24>'  : 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy/Autopilot/PID Controller/Saturation'
 * '<S25>'  : 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy/Autopilot/PID Controller/Saturation Fdbk'
 * '<S26>'  : 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy/Autopilot/PID Controller/Sum'
 * '<S27>'  : 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy/Autopilot/PID Controller/Sum Fdbk'
 * '<S28>'  : 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy/Autopilot/PID Controller/Tracking Mode'
 * '<S29>'  : 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy/Autopilot/PID Controller/Tracking Mode Sum'
 * '<S30>'  : 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy/Autopilot/PID Controller/Tsamp - Integral'
 * '<S31>'  : 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy/Autopilot/PID Controller/Tsamp - Ngain'
 * '<S32>'  : 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy/Autopilot/PID Controller/postSat Signal'
 * '<S33>'  : 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy/Autopilot/PID Controller/preInt Signal'
 * '<S34>'  : 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy/Autopilot/PID Controller/preSat Signal'
 * '<S35>'  : 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy/Autopilot/PID Controller/Anti-windup/Disc. Clamping Parallel'
 * '<S36>'  : 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy/Autopilot/PID Controller/Anti-windup/Disc. Clamping Parallel/Dead Zone'
 * '<S37>'  : 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy/Autopilot/PID Controller/Anti-windup/Disc. Clamping Parallel/Dead Zone/Enabled'
 * '<S38>'  : 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy/Autopilot/PID Controller/D Gain/Internal Parameters'
 * '<S39>'  : 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy/Autopilot/PID Controller/External Derivative/Error'
 * '<S40>'  : 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy/Autopilot/PID Controller/Filter/Disc. Forward Euler Filter'
 * '<S41>'  : 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy/Autopilot/PID Controller/Filter ICs/Internal IC - Filter'
 * '<S42>'  : 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy/Autopilot/PID Controller/I Gain/Internal Parameters'
 * '<S43>'  : 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy/Autopilot/PID Controller/Ideal P Gain/Passthrough'
 * '<S44>'  : 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy/Autopilot/PID Controller/Ideal P Gain Fdbk/Disabled'
 * '<S45>'  : 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy/Autopilot/PID Controller/Integrator/Discrete'
 * '<S46>'  : 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy/Autopilot/PID Controller/Integrator ICs/Internal IC'
 * '<S47>'  : 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy/Autopilot/PID Controller/N Copy/Disabled'
 * '<S48>'  : 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy/Autopilot/PID Controller/N Gain/Internal Parameters'
 * '<S49>'  : 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy/Autopilot/PID Controller/P Copy/Disabled'
 * '<S50>'  : 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy/Autopilot/PID Controller/Parallel P Gain/Internal Parameters'
 * '<S51>'  : 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy/Autopilot/PID Controller/Reset Signal/Disabled'
 * '<S52>'  : 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy/Autopilot/PID Controller/Saturation/Enabled'
 * '<S53>'  : 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy/Autopilot/PID Controller/Saturation Fdbk/Disabled'
 * '<S54>'  : 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy/Autopilot/PID Controller/Sum/Sum_PID'
 * '<S55>'  : 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy/Autopilot/PID Controller/Sum Fdbk/Disabled'
 * '<S56>'  : 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy/Autopilot/PID Controller/Tracking Mode/Disabled'
 * '<S57>'  : 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy/Autopilot/PID Controller/Tracking Mode Sum/Passthrough'
 * '<S58>'  : 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy/Autopilot/PID Controller/Tsamp - Integral/TsSignalSpecification'
 * '<S59>'  : 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy/Autopilot/PID Controller/Tsamp - Ngain/Passthrough'
 * '<S60>'  : 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy/Autopilot/PID Controller/postSat Signal/Forward_Path'
 * '<S61>'  : 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy/Autopilot/PID Controller/preInt Signal/Internal PreInt'
 * '<S62>'  : 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy/Autopilot/PID Controller/preSat Signal/Forward_Path'
 * '<S63>'  : 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy/Autopilot/PID Controller1/Anti-windup'
 * '<S64>'  : 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy/Autopilot/PID Controller1/D Gain'
 * '<S65>'  : 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy/Autopilot/PID Controller1/External Derivative'
 * '<S66>'  : 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy/Autopilot/PID Controller1/Filter'
 * '<S67>'  : 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy/Autopilot/PID Controller1/Filter ICs'
 * '<S68>'  : 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy/Autopilot/PID Controller1/I Gain'
 * '<S69>'  : 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy/Autopilot/PID Controller1/Ideal P Gain'
 * '<S70>'  : 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy/Autopilot/PID Controller1/Ideal P Gain Fdbk'
 * '<S71>'  : 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy/Autopilot/PID Controller1/Integrator'
 * '<S72>'  : 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy/Autopilot/PID Controller1/Integrator ICs'
 * '<S73>'  : 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy/Autopilot/PID Controller1/N Copy'
 * '<S74>'  : 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy/Autopilot/PID Controller1/N Gain'
 * '<S75>'  : 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy/Autopilot/PID Controller1/P Copy'
 * '<S76>'  : 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy/Autopilot/PID Controller1/Parallel P Gain'
 * '<S77>'  : 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy/Autopilot/PID Controller1/Reset Signal'
 * '<S78>'  : 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy/Autopilot/PID Controller1/Saturation'
 * '<S79>'  : 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy/Autopilot/PID Controller1/Saturation Fdbk'
 * '<S80>'  : 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy/Autopilot/PID Controller1/Sum'
 * '<S81>'  : 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy/Autopilot/PID Controller1/Sum Fdbk'
 * '<S82>'  : 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy/Autopilot/PID Controller1/Tracking Mode'
 * '<S83>'  : 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy/Autopilot/PID Controller1/Tracking Mode Sum'
 * '<S84>'  : 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy/Autopilot/PID Controller1/Tsamp - Integral'
 * '<S85>'  : 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy/Autopilot/PID Controller1/Tsamp - Ngain'
 * '<S86>'  : 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy/Autopilot/PID Controller1/postSat Signal'
 * '<S87>'  : 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy/Autopilot/PID Controller1/preInt Signal'
 * '<S88>'  : 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy/Autopilot/PID Controller1/preSat Signal'
 * '<S89>'  : 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy/Autopilot/PID Controller1/Anti-windup/Disc. Clamping Parallel'
 * '<S90>'  : 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy/Autopilot/PID Controller1/Anti-windup/Disc. Clamping Parallel/Dead Zone'
 * '<S91>'  : 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy/Autopilot/PID Controller1/Anti-windup/Disc. Clamping Parallel/Dead Zone/Enabled'
 * '<S92>'  : 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy/Autopilot/PID Controller1/D Gain/Internal Parameters'
 * '<S93>'  : 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy/Autopilot/PID Controller1/External Derivative/Error'
 * '<S94>'  : 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy/Autopilot/PID Controller1/Filter/Disc. Forward Euler Filter'
 * '<S95>'  : 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy/Autopilot/PID Controller1/Filter ICs/Internal IC - Filter'
 * '<S96>'  : 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy/Autopilot/PID Controller1/I Gain/Internal Parameters'
 * '<S97>'  : 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy/Autopilot/PID Controller1/Ideal P Gain/Passthrough'
 * '<S98>'  : 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy/Autopilot/PID Controller1/Ideal P Gain Fdbk/Disabled'
 * '<S99>'  : 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy/Autopilot/PID Controller1/Integrator/Discrete'
 * '<S100>' : 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy/Autopilot/PID Controller1/Integrator ICs/Internal IC'
 * '<S101>' : 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy/Autopilot/PID Controller1/N Copy/Disabled'
 * '<S102>' : 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy/Autopilot/PID Controller1/N Gain/Internal Parameters'
 * '<S103>' : 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy/Autopilot/PID Controller1/P Copy/Disabled'
 * '<S104>' : 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy/Autopilot/PID Controller1/Parallel P Gain/Internal Parameters'
 * '<S105>' : 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy/Autopilot/PID Controller1/Reset Signal/Disabled'
 * '<S106>' : 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy/Autopilot/PID Controller1/Saturation/Enabled'
 * '<S107>' : 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy/Autopilot/PID Controller1/Saturation Fdbk/Disabled'
 * '<S108>' : 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy/Autopilot/PID Controller1/Sum/Sum_PID'
 * '<S109>' : 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy/Autopilot/PID Controller1/Sum Fdbk/Disabled'
 * '<S110>' : 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy/Autopilot/PID Controller1/Tracking Mode/Disabled'
 * '<S111>' : 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy/Autopilot/PID Controller1/Tracking Mode Sum/Passthrough'
 * '<S112>' : 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy/Autopilot/PID Controller1/Tsamp - Integral/TsSignalSpecification'
 * '<S113>' : 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy/Autopilot/PID Controller1/Tsamp - Ngain/Passthrough'
 * '<S114>' : 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy/Autopilot/PID Controller1/postSat Signal/Forward_Path'
 * '<S115>' : 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy/Autopilot/PID Controller1/preInt Signal/Internal PreInt'
 * '<S116>' : 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy/Autopilot/PID Controller1/preSat Signal/Forward_Path'
 */
#endif                                 /* Autopilot_h_ */
