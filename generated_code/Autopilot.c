/*
 * Autopilot.c
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
#include "rtwtypes.h"
#include "rt_nonfinite.h"
#include <math.h>
#include "Autopilot_private.h"
#include <string.h>

/* Block signals (default storage) */
B_Autopilot_T Autopilot_B;

/* Continuous states */
X_Autopilot_T Autopilot_X;

/* Disabled State Vector */
XDis_Autopilot_T Autopilot_XDis;

/* Block states (default storage) */
DW_Autopilot_T Autopilot_DW;

/* External inputs (root inport signals with default storage) */
ExtU_Autopilot_T Autopilot_U;

/* External outputs (root outports fed by signals with default storage) */
ExtY_Autopilot_T Autopilot_Y;

/* Real-time model */
static RT_MODEL_Autopilot_T Autopilot_M_;
RT_MODEL_Autopilot_T *const Autopilot_M = &Autopilot_M_;

/*
 * This function updates continuous states using the ODE4 fixed-step
 * solver algorithm
 */
static void rt_ertODEUpdateContinuousStates(RTWSolverInfo *si )
{
  time_T t = rtsiGetT(si);
  time_T tnew = rtsiGetSolverStopTime(si);
  time_T h = rtsiGetStepSize(si);
  real_T *x = rtsiGetContStates(si);
  ODE4_IntgData *id = (ODE4_IntgData *)rtsiGetSolverData(si);
  real_T *y = id->y;
  real_T *f0 = id->f[0];
  real_T *f1 = id->f[1];
  real_T *f2 = id->f[2];
  real_T *f3 = id->f[3];
  real_T temp;
  int_T i;
  int_T nXc = 2;
  rtsiSetSimTimeStep(si,MINOR_TIME_STEP);

  /* Save the state values at time t in y, we'll use x as ynew. */
  (void) memcpy(y, x,
                (uint_T)nXc*sizeof(real_T));

  /* Assumes that rtsiSetT and ModelOutputs are up-to-date */
  /* f0 = f(t,y) */
  rtsiSetdX(si, f0);
  Autopilot_derivatives();

  /* f1 = f(t + (h/2), y + (h/2)*f0) */
  temp = 0.5 * h;
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (temp*f0[i]);
  }

  rtsiSetT(si, t + temp);
  rtsiSetdX(si, f1);
  Autopilot_step();
  Autopilot_derivatives();

  /* f2 = f(t + (h/2), y + (h/2)*f1) */
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (temp*f1[i]);
  }

  rtsiSetdX(si, f2);
  Autopilot_step();
  Autopilot_derivatives();

  /* f3 = f(t + h, y + h*f2) */
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (h*f2[i]);
  }

  rtsiSetT(si, tnew);
  rtsiSetdX(si, f3);
  Autopilot_step();
  Autopilot_derivatives();

  /* tnew = t + h
     ynew = y + (h/6)*(f0 + 2*f1 + 2*f2 + 2*f3) */
  temp = h / 6.0;
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + temp*(f0[i] + 2.0*f1[i] + 2.0*f2[i] + f3[i]);
  }

  rtsiSetSimTimeStep(si,MAJOR_TIME_STEP);
}

/* Model step function */
void Autopilot_step(void)
{
  /* local block i/o variables */
  real_T rtb_FilterCoefficient;
  real_T rtb_Switch;
  real_T rtb_DeadZone;
  real_T rtb_Gain;
  real_T rtb_IntegralGain;
  real_T rtb_IntegralGain_p;
  real_T rtb_SignPreSat;
  real_T rtb_delta_alphadeg;
  real_T u0;
  int32_T tmp_0;
  int8_T tmp_1;
  int8_T tmp_2;
  boolean_T tmp;
  if (rtmIsMajorTimeStep(Autopilot_M)) {
    /* set solver stop time */
    if (!(Autopilot_M->Timing.clockTick0+1)) {
      rtsiSetSolverStopTime(&Autopilot_M->solverInfo,
                            ((Autopilot_M->Timing.clockTickH0 + 1) *
        Autopilot_M->Timing.stepSize0 * 4294967296.0));
    } else {
      rtsiSetSolverStopTime(&Autopilot_M->solverInfo,
                            ((Autopilot_M->Timing.clockTick0 + 1) *
        Autopilot_M->Timing.stepSize0 + Autopilot_M->Timing.clockTickH0 *
        Autopilot_M->Timing.stepSize0 * 4294967296.0));
    }
  }                                    /* end MajorTimeStep */

  /* Update absolute time of base rate at minor time step */
  if (rtmIsMinorTimeStep(Autopilot_M)) {
    Autopilot_M->Timing.t[0] = rtsiGetT(&Autopilot_M->solverInfo);
  }

  tmp = rtmIsMajorTimeStep(Autopilot_M);
  if (tmp) {
    /* Sum: '<S1>/Subtract3' incorporates:
     *  Constant: '<S1>/velocity_trim2'
     *  Inport: '<Root>/U'
     */
    rtb_IntegralGain = Autopilot_P.velocity_trim2_Value - Autopilot_U.U;

    /* Gain: '<S99>/Proportional Gain' */
    Autopilot_B.ProportionalGain = Autopilot_P.PIDController1_P *
      rtb_IntegralGain;

    /* Gain: '<S87>/Derivative Gain' */
    Autopilot_B.DerivativeGain = Autopilot_P.PIDController1_D * rtb_IntegralGain;
  }

  /* Gain: '<S97>/Filter Coefficient' incorporates:
   *  Integrator: '<S89>/Filter'
   *  Sum: '<S89>/SumD'
   */
  Autopilot_B.FilterCoefficient = (Autopilot_B.DerivativeGain -
    Autopilot_X.Filter_CSTATE) * Autopilot_P.PIDController1_N;

  /* Sum: '<S103>/Sum' incorporates:
   *  Integrator: '<S94>/Integrator'
   */
  rtb_SignPreSat = (Autopilot_B.ProportionalGain + Autopilot_X.Integrator_CSTATE)
    + Autopilot_B.FilterCoefficient;

  /* Saturate: '<S101>/Saturation' */
  if (rtb_SignPreSat > Autopilot_P.PIDController1_UpperSaturationL) {
    u0 = Autopilot_P.PIDController1_UpperSaturationL;
  } else if (rtb_SignPreSat < Autopilot_P.PIDController1_LowerSaturationL) {
    u0 = Autopilot_P.PIDController1_LowerSaturationL;
  } else {
    u0 = rtb_SignPreSat;
  }

  /* Sum: '<S1>/Sum' incorporates:
   *  Constant: '<S1>/Throttle'
   *  Saturate: '<S101>/Saturation'
   */
  u0 += Autopilot_P.Throttle_Value;

  /* Saturate: '<S1>/Saturation1' */
  if (u0 > Autopilot_P.Saturation1_UpperSat) {
    /* Outport: '<Root>/Throttle_cmd' */
    Autopilot_Y.Throttle_cmd = Autopilot_P.Saturation1_UpperSat;
  } else if (u0 < Autopilot_P.Saturation1_LowerSat) {
    /* Outport: '<Root>/Throttle_cmd' */
    Autopilot_Y.Throttle_cmd = Autopilot_P.Saturation1_LowerSat;
  } else {
    /* Outport: '<Root>/Throttle_cmd' */
    Autopilot_Y.Throttle_cmd = u0;
  }

  /* End of Saturate: '<S1>/Saturation1' */
  if (tmp) {
    /* Sum: '<S1>/Subtract' incorporates:
     *  Constant: '<S1>/alpha_trim'
     *  Inport: '<Root>/Angle of Attack (rad)'
     */
    rtb_delta_alphadeg = Autopilot_U.AngleofAttackrad -
      Autopilot_P.alpha_trim_Value;

    /* Sum: '<S1>/Sum1' incorporates:
     *  Constant: '<S1>/Constant'
     *  Inport: '<Root>/7 Altitude (ft)'
     */
    rtb_IntegralGain_p = Autopilot_P.Constant_Value - Autopilot_U.uAltitudeft;

    /* Gain: '<S43>/Filter Coefficient' incorporates:
     *  DiscreteIntegrator: '<S35>/Filter'
     *  Gain: '<S33>/Derivative Gain'
     *  Sum: '<S35>/SumD'
     */
    rtb_FilterCoefficient = (Autopilot_P.PIDController_D * rtb_IntegralGain_p -
      Autopilot_DW.Filter_DSTATE) * Autopilot_P.PIDController_N;

    /* Sum: '<S49>/Sum' incorporates:
     *  DiscreteIntegrator: '<S40>/Integrator'
     *  Gain: '<S45>/Proportional Gain'
     */
    rtb_DeadZone = (Autopilot_P.PIDController_P * rtb_IntegralGain_p +
                    Autopilot_DW.Integrator_DSTATE) + rtb_FilterCoefficient;

    /* Saturate: '<S47>/Saturation' */
    if (rtb_DeadZone > Autopilot_P.PIDController_UpperSaturationLi) {
      rtb_Gain = Autopilot_P.PIDController_UpperSaturationLi;
    } else if (rtb_DeadZone < Autopilot_P.PIDController_LowerSaturationLi) {
      rtb_Gain = Autopilot_P.PIDController_LowerSaturationLi;
    } else {
      rtb_Gain = rtb_DeadZone;
    }

    /* End of Saturate: '<S47>/Saturation' */

    /* Gain: '<S1>/Gain' incorporates:
     *  Constant: '<S1>/Thita_trim'
     *  Constant: '<S1>/velocity_trim1'
     *  Gain: '<S1>/Gain1'
     *  Inport: '<Root>/11 Actual theta'
     *  Inport: '<Root>/5 Pitch Rate'
     *  Inport: '<Root>/U'
     *  SignalConversion generated from: '<S1>/Gain1'
     *  Sum: '<S1>/Subtract1'
     *  Sum: '<S1>/Subtract2'
     *  Sum: '<S1>/Sum4'
     */
    rtb_Gain = ((((Autopilot_U.U - Autopilot_P.velocity_trim1_Value) *
                  Autopilot_P.K[0] + Autopilot_P.K[1] * rtb_delta_alphadeg) +
                 (Autopilot_U.u1Actualtheta - (rtb_Gain +
      Autopilot_P.Thita_trim_Value)) * Autopilot_P.K[2]) + Autopilot_P.K[3] *
                Autopilot_U.uPitchRate) * Autopilot_P.Gain_Gain;

    /* Gain: '<S1>/Gain2' incorporates:
     *  Inport: '<Root>/attitude//phi-rad'
     */
    u0 = Autopilot_P.Gain2_Gain * Autopilot_U.attitudephirad;

    /* Saturate: '<S1>/Saturation2' */
    if (u0 > Autopilot_P.Saturation2_UpperSat) {
      /* Outport: '<Root>/Aileron Command' */
      Autopilot_Y.AileronCommand = Autopilot_P.Saturation2_UpperSat;
    } else if (u0 < Autopilot_P.Saturation2_LowerSat) {
      /* Outport: '<Root>/Aileron Command' */
      Autopilot_Y.AileronCommand = Autopilot_P.Saturation2_LowerSat;
    } else {
      /* Outport: '<Root>/Aileron Command' */
      Autopilot_Y.AileronCommand = u0;
    }

    /* End of Saturate: '<S1>/Saturation2' */

    /* Saturate: '<S1>/Saturation' */
    if (rtb_Gain > Autopilot_P.Saturation_UpperSat) {
      /* Outport: '<Root>/Elevator Command' */
      Autopilot_Y.ElevatorCommand = Autopilot_P.Saturation_UpperSat;
    } else if (rtb_Gain < Autopilot_P.Saturation_LowerSat) {
      /* Outport: '<Root>/Elevator Command' */
      Autopilot_Y.ElevatorCommand = Autopilot_P.Saturation_LowerSat;
    } else {
      /* Outport: '<Root>/Elevator Command' */
      Autopilot_Y.ElevatorCommand = rtb_Gain;
    }

    /* End of Saturate: '<S1>/Saturation' */

    /* Gain: '<S1>/Multiply1' */
    rtb_delta_alphadeg *= Autopilot_P.Multiply1_Gain;

    /* DeadZone: '<S32>/DeadZone' */
    if (rtb_DeadZone > Autopilot_P.PIDController_UpperSaturationLi) {
      rtb_DeadZone -= Autopilot_P.PIDController_UpperSaturationLi;
    } else if (rtb_DeadZone >= Autopilot_P.PIDController_LowerSaturationLi) {
      rtb_DeadZone = 0.0;
    } else {
      rtb_DeadZone -= Autopilot_P.PIDController_LowerSaturationLi;
    }

    /* End of DeadZone: '<S32>/DeadZone' */

    /* Gain: '<S37>/Integral Gain' */
    rtb_IntegralGain_p *= Autopilot_P.PIDController_I;

    /* Switch: '<S30>/Switch1' incorporates:
     *  Constant: '<S30>/Clamping_zero'
     *  Constant: '<S30>/Constant'
     *  Constant: '<S30>/Constant2'
     *  RelationalOperator: '<S30>/fix for DT propagation issue'
     */
    if (rtb_DeadZone > Autopilot_P.Clamping_zero_Value) {
      tmp_1 = Autopilot_P.Constant_Value_o;
    } else {
      tmp_1 = Autopilot_P.Constant2_Value;
    }

    /* Switch: '<S30>/Switch2' incorporates:
     *  Constant: '<S30>/Clamping_zero'
     *  Constant: '<S30>/Constant3'
     *  Constant: '<S30>/Constant4'
     *  RelationalOperator: '<S30>/fix for DT propagation issue1'
     */
    if (rtb_IntegralGain_p > Autopilot_P.Clamping_zero_Value) {
      tmp_2 = Autopilot_P.Constant3_Value;
    } else {
      tmp_2 = Autopilot_P.Constant4_Value;
    }

    /* Switch: '<S30>/Switch' incorporates:
     *  Constant: '<S30>/Clamping_zero'
     *  Logic: '<S30>/AND3'
     *  RelationalOperator: '<S30>/Equal1'
     *  RelationalOperator: '<S30>/Relational Operator'
     *  Switch: '<S30>/Switch1'
     *  Switch: '<S30>/Switch2'
     */
    if ((Autopilot_P.Clamping_zero_Value != rtb_DeadZone) && (tmp_1 == tmp_2)) {
      /* Switch: '<S30>/Switch' incorporates:
       *  Constant: '<S30>/Constant1'
       */
      rtb_Switch = Autopilot_P.Constant1_Value;
    } else {
      /* Switch: '<S30>/Switch' */
      rtb_Switch = rtb_IntegralGain_p;
    }

    /* End of Switch: '<S30>/Switch' */
  }

  /* Gain: '<S84>/ZeroGain' */
  rtb_delta_alphadeg = Autopilot_P.ZeroGain_Gain * rtb_SignPreSat;

  /* DeadZone: '<S86>/DeadZone' */
  if (rtb_SignPreSat > Autopilot_P.PIDController1_UpperSaturationL) {
    rtb_SignPreSat -= Autopilot_P.PIDController1_UpperSaturationL;
  } else if (rtb_SignPreSat >= Autopilot_P.PIDController1_LowerSaturationL) {
    rtb_SignPreSat = 0.0;
  } else {
    rtb_SignPreSat -= Autopilot_P.PIDController1_LowerSaturationL;
  }

  /* End of DeadZone: '<S86>/DeadZone' */
  if (tmp) {
    /* Gain: '<S91>/Integral Gain' */
    rtb_IntegralGain *= Autopilot_P.PIDController1_I;

    /* Signum: '<S84>/SignPreIntegrator' */
    if (rtIsNaN(rtb_IntegralGain)) {
      /* DataTypeConversion: '<S84>/DataTypeConv2' */
      tmp_0 = 0;
    } else {
      if (rtb_IntegralGain < 0.0) {
        /* DataTypeConversion: '<S84>/DataTypeConv2' */
        u0 = -1.0;
      } else {
        /* DataTypeConversion: '<S84>/DataTypeConv2' */
        u0 = (rtb_IntegralGain > 0.0);
      }

      /* DataTypeConversion: '<S84>/DataTypeConv2' */
      tmp_0 = (int32_T)fmod(u0, 256.0);
    }

    /* End of Signum: '<S84>/SignPreIntegrator' */

    /* DataTypeConversion: '<S84>/DataTypeConv2' */
    if (tmp_0 < 0) {
      /* DataTypeConversion: '<S84>/DataTypeConv2' */
      Autopilot_B.DataTypeConv2 = (int8_T)-(int8_T)(uint8_T)-(real_T)tmp_0;
    } else {
      /* DataTypeConversion: '<S84>/DataTypeConv2' */
      Autopilot_B.DataTypeConv2 = (int8_T)tmp_0;
    }
  }

  /* Signum: '<S84>/SignPreSat' */
  if (rtIsNaN(rtb_SignPreSat)) {
    /* DataTypeConversion: '<S84>/DataTypeConv1' */
    tmp_0 = 0;
  } else {
    if (rtb_SignPreSat < 0.0) {
      /* DataTypeConversion: '<S84>/DataTypeConv1' */
      u0 = -1.0;
    } else {
      /* DataTypeConversion: '<S84>/DataTypeConv1' */
      u0 = (rtb_SignPreSat > 0.0);
    }

    /* DataTypeConversion: '<S84>/DataTypeConv1' */
    tmp_0 = (int32_T)fmod(u0, 256.0);
  }

  /* End of Signum: '<S84>/SignPreSat' */

  /* DataTypeConversion: '<S84>/DataTypeConv1' */
  if (tmp_0 < 0) {
    tmp_0 = (int8_T)-(int8_T)(uint8_T)-(real_T)tmp_0;
  }

  /* Logic: '<S84>/AND3' incorporates:
   *  DataTypeConversion: '<S84>/DataTypeConv1'
   *  RelationalOperator: '<S84>/Equal1'
   *  RelationalOperator: '<S84>/NotEqual'
   */
  Autopilot_B.AND3 = ((rtb_delta_alphadeg != rtb_SignPreSat) && (tmp_0 ==
    Autopilot_B.DataTypeConv2));
  if (tmp) {
    /* Switch: '<S84>/Switch' incorporates:
     *  Memory: '<S84>/Memory'
     */
    if (Autopilot_DW.Memory_PreviousInput) {
      /* Switch: '<S84>/Switch' incorporates:
       *  Constant: '<S84>/Constant1'
       */
      Autopilot_B.Switch = Autopilot_P.Constant1_Value_a;
    } else {
      /* Switch: '<S84>/Switch' */
      Autopilot_B.Switch = rtb_IntegralGain;
    }

    /* End of Switch: '<S84>/Switch' */
  }

  if (rtmIsMajorTimeStep(Autopilot_M)) {
    /* Matfile logging */
    rt_UpdateTXYLogVars(Autopilot_M->rtwLogInfo, (Autopilot_M->Timing.t));
  }                                    /* end MajorTimeStep */

  if (rtmIsMajorTimeStep(Autopilot_M)) {
    if (rtmIsMajorTimeStep(Autopilot_M)) {
      /* Update for DiscreteIntegrator: '<S40>/Integrator' */
      Autopilot_DW.Integrator_DSTATE += Autopilot_P.Integrator_gainval *
        rtb_Switch;

      /* Update for DiscreteIntegrator: '<S35>/Filter' */
      Autopilot_DW.Filter_DSTATE += Autopilot_P.Filter_gainval *
        rtb_FilterCoefficient;

      /* Update for Memory: '<S84>/Memory' */
      Autopilot_DW.Memory_PreviousInput = Autopilot_B.AND3;
    }
  }                                    /* end MajorTimeStep */

  if (rtmIsMajorTimeStep(Autopilot_M)) {
    /* signal main to stop simulation */
    {                                  /* Sample time: [0.0s, 0.0s] */
      if ((rtmGetTFinal(Autopilot_M)!=-1) &&
          !((rtmGetTFinal(Autopilot_M)-(((Autopilot_M->Timing.clockTick1+
               Autopilot_M->Timing.clockTickH1* 4294967296.0)) * 0.008333)) >
            (((Autopilot_M->Timing.clockTick1+Autopilot_M->Timing.clockTickH1*
               4294967296.0)) * 0.008333) * (DBL_EPSILON))) {
        rtmSetErrorStatus(Autopilot_M, "Simulation finished");
      }
    }

    rt_ertODEUpdateContinuousStates(&Autopilot_M->solverInfo);

    /* Update absolute time for base rate */
    /* The "clockTick0" counts the number of times the code of this task has
     * been executed. The absolute time is the multiplication of "clockTick0"
     * and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
     * overflow during the application lifespan selected.
     * Timer of this task consists of two 32 bit unsigned integers.
     * The two integers represent the low bits Timing.clockTick0 and the high bits
     * Timing.clockTickH0. When the low bit overflows to 0, the high bits increment.
     */
    if (!(++Autopilot_M->Timing.clockTick0)) {
      ++Autopilot_M->Timing.clockTickH0;
    }

    Autopilot_M->Timing.t[0] = rtsiGetSolverStopTime(&Autopilot_M->solverInfo);

    {
      /* Update absolute timer for sample time: [0.008333s, 0.0s] */
      /* The "clockTick1" counts the number of times the code of this task has
       * been executed. The resolution of this integer timer is 0.008333, which is the step size
       * of the task. Size of "clockTick1" ensures timer will not overflow during the
       * application lifespan selected.
       * Timer of this task consists of two 32 bit unsigned integers.
       * The two integers represent the low bits Timing.clockTick1 and the high bits
       * Timing.clockTickH1. When the low bit overflows to 0, the high bits increment.
       */
      Autopilot_M->Timing.clockTick1++;
      if (!Autopilot_M->Timing.clockTick1) {
        Autopilot_M->Timing.clockTickH1++;
      }
    }
  }                                    /* end MajorTimeStep */
}

/* Derivatives for root system: '<Root>' */
void Autopilot_derivatives(void)
{
  XDot_Autopilot_T *_rtXdot;
  _rtXdot = ((XDot_Autopilot_T *) Autopilot_M->derivs);

  /* Derivatives for Integrator: '<S94>/Integrator' */
  _rtXdot->Integrator_CSTATE = Autopilot_B.Switch;

  /* Derivatives for Integrator: '<S89>/Filter' */
  _rtXdot->Filter_CSTATE = Autopilot_B.FilterCoefficient;
}

/* Model initialize function */
void Autopilot_initialize(void)
{
  /* Registration code */

  /* initialize real-time model */
  (void) memset((void *)Autopilot_M, 0,
                sizeof(RT_MODEL_Autopilot_T));

  {
    /* Setup solver object */
    rtsiSetSimTimeStepPtr(&Autopilot_M->solverInfo,
                          &Autopilot_M->Timing.simTimeStep);
    rtsiSetTPtr(&Autopilot_M->solverInfo, &rtmGetTPtr(Autopilot_M));
    rtsiSetStepSizePtr(&Autopilot_M->solverInfo, &Autopilot_M->Timing.stepSize0);
    rtsiSetdXPtr(&Autopilot_M->solverInfo, &Autopilot_M->derivs);
    rtsiSetContStatesPtr(&Autopilot_M->solverInfo, (real_T **)
                         &Autopilot_M->contStates);
    rtsiSetNumContStatesPtr(&Autopilot_M->solverInfo,
      &Autopilot_M->Sizes.numContStates);
    rtsiSetNumPeriodicContStatesPtr(&Autopilot_M->solverInfo,
      &Autopilot_M->Sizes.numPeriodicContStates);
    rtsiSetPeriodicContStateIndicesPtr(&Autopilot_M->solverInfo,
      &Autopilot_M->periodicContStateIndices);
    rtsiSetPeriodicContStateRangesPtr(&Autopilot_M->solverInfo,
      &Autopilot_M->periodicContStateRanges);
    rtsiSetContStateDisabledPtr(&Autopilot_M->solverInfo, (boolean_T**)
      &Autopilot_M->contStateDisabled);
    rtsiSetErrorStatusPtr(&Autopilot_M->solverInfo, (&rtmGetErrorStatus
      (Autopilot_M)));
    rtsiSetRTModelPtr(&Autopilot_M->solverInfo, Autopilot_M);
  }

  rtsiSetSimTimeStep(&Autopilot_M->solverInfo, MAJOR_TIME_STEP);
  rtsiSetIsMinorTimeStepWithModeChange(&Autopilot_M->solverInfo, false);
  rtsiSetIsContModeFrozen(&Autopilot_M->solverInfo, false);
  Autopilot_M->intgData.y = Autopilot_M->odeY;
  Autopilot_M->intgData.f[0] = Autopilot_M->odeF[0];
  Autopilot_M->intgData.f[1] = Autopilot_M->odeF[1];
  Autopilot_M->intgData.f[2] = Autopilot_M->odeF[2];
  Autopilot_M->intgData.f[3] = Autopilot_M->odeF[3];
  Autopilot_M->contStates = ((X_Autopilot_T *) &Autopilot_X);
  Autopilot_M->contStateDisabled = ((XDis_Autopilot_T *) &Autopilot_XDis);
  Autopilot_M->Timing.tStart = (0.0);
  rtsiSetSolverData(&Autopilot_M->solverInfo, (void *)&Autopilot_M->intgData);
  rtsiSetSolverName(&Autopilot_M->solverInfo,"ode4");
  rtmSetTPtr(Autopilot_M, &Autopilot_M->Timing.tArray[0]);
  rtmSetTFinal(Autopilot_M, 79.996800000000007);
  Autopilot_M->Timing.stepSize0 = 0.008333;

  /* Setup for data logging */
  {
    static RTWLogInfo rt_DataLoggingInfo;
    rt_DataLoggingInfo.loggingInterval = (NULL);
    Autopilot_M->rtwLogInfo = &rt_DataLoggingInfo;
  }

  /* Setup for data logging */
  {
    rtliSetLogXSignalInfo(Autopilot_M->rtwLogInfo, (NULL));
    rtliSetLogXSignalPtrs(Autopilot_M->rtwLogInfo, (NULL));
    rtliSetLogT(Autopilot_M->rtwLogInfo, "tout");
    rtliSetLogX(Autopilot_M->rtwLogInfo, "");
    rtliSetLogXFinal(Autopilot_M->rtwLogInfo, "");
    rtliSetLogVarNameModifier(Autopilot_M->rtwLogInfo, "rt_");
    rtliSetLogFormat(Autopilot_M->rtwLogInfo, 4);
    rtliSetLogMaxRows(Autopilot_M->rtwLogInfo, 0);
    rtliSetLogDecimation(Autopilot_M->rtwLogInfo, 1);
    rtliSetLogY(Autopilot_M->rtwLogInfo, "");
    rtliSetLogYSignalInfo(Autopilot_M->rtwLogInfo, (NULL));
    rtliSetLogYSignalPtrs(Autopilot_M->rtwLogInfo, (NULL));
  }

  /* block I/O */
  (void) memset(((void *) &Autopilot_B), 0,
                sizeof(B_Autopilot_T));

  /* states (continuous) */
  {
    (void) memset((void *)&Autopilot_X, 0,
                  sizeof(X_Autopilot_T));
  }

  /* disabled states */
  {
    (void) memset((void *)&Autopilot_XDis, 0,
                  sizeof(XDis_Autopilot_T));
  }

  /* states (dwork) */
  (void) memset((void *)&Autopilot_DW, 0,
                sizeof(DW_Autopilot_T));

  /* external inputs */
  (void)memset(&Autopilot_U, 0, sizeof(ExtU_Autopilot_T));

  /* external outputs */
  (void)memset(&Autopilot_Y, 0, sizeof(ExtY_Autopilot_T));

  /* Matfile logging */
  rt_StartDataLoggingWithStartTime(Autopilot_M->rtwLogInfo, 0.0, rtmGetTFinal
    (Autopilot_M), Autopilot_M->Timing.stepSize0, (&rtmGetErrorStatus
    (Autopilot_M)));

  /* InitializeConditions for Integrator: '<S94>/Integrator' */
  Autopilot_X.Integrator_CSTATE = Autopilot_P.PIDController1_InitialConditi_l;

  /* InitializeConditions for Integrator: '<S89>/Filter' */
  Autopilot_X.Filter_CSTATE = Autopilot_P.PIDController1_InitialCondition;

  /* InitializeConditions for DiscreteIntegrator: '<S40>/Integrator' */
  Autopilot_DW.Integrator_DSTATE = Autopilot_P.PIDController_InitialConditio_j;

  /* InitializeConditions for DiscreteIntegrator: '<S35>/Filter' */
  Autopilot_DW.Filter_DSTATE = Autopilot_P.PIDController_InitialConditionF;

  /* InitializeConditions for Memory: '<S84>/Memory' */
  Autopilot_DW.Memory_PreviousInput = Autopilot_P.Memory_InitialCondition;
}

/* Model terminate function */
void Autopilot_terminate(void)
{
  /* (no terminate code required) */
}
