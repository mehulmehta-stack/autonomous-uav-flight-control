/*
 * Autopilot.c
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
#include "rtwtypes.h"
#include <string.h>
#include <math.h>
#include <emmintrin.h>
#include "rt_nonfinite.h"
#include "Autopilot_private.h"
#include "rt_defines.h"

/* Block states (default storage) */
DW_Autopilot_T Autopilot_DW;

/* External inputs (root inport signals with default storage) */
ExtU_Autopilot_T Autopilot_U;

/* External outputs (root outports fed by signals with default storage) */
ExtY_Autopilot_T Autopilot_Y;

/* Real-time model */
static RT_MODEL_Autopilot_T Autopilot_M_;
RT_MODEL_Autopilot_T *const Autopilot_M = &Autopilot_M_;

/* Forward declaration for local functions */
static void Autopilot_mod(const real_T x[3], real_T r[3]);
static void Autopilot_mrdiv_m(real_T A[72], const real_T B[36]);

/* Function for MATLAB Function: '<S1>/Navigation_EKF' */
static void Autopilot_mod(const real_T x[3], real_T r[3])
{
  int32_T k;
  for (k = 0; k < 3; k++) {
    real_T q;
    real_T x_0;
    x_0 = x[k];
    if (rtIsNaN(x_0) || rtIsInf(x_0)) {
      q = (rtNaN);
    } else {
      q = fabs(x_0 / 6.2831853071795862);
      if (fabs(q - floor(q + 0.5)) > 2.2204460492503131E-16 * q) {
        q = fmod(x_0, 6.2831853071795862);
      } else {
        q = 0.0;
      }

      if (q == 0.0) {
        q = 0.0;
      } else if (q < 0.0) {
        q += 6.2831853071795862;
      }
    }

    r[k] = q;
  }
}

/* Function for MATLAB Function: '<S1>/Navigation_EKF' */
static void Autopilot_mrdiv_m(real_T A[72], const real_T B[36])
{
  __m128d tmp;
  real_T b_A[36];
  real_T smax;
  int32_T b_ix;
  int32_T c_k;
  int32_T d_j;
  int32_T ix;
  int32_T iy;
  int32_T jj;
  int32_T vectorUB;
  int8_T ipiv[6];
  memcpy(&b_A[0], &B[0], 36U * sizeof(real_T));
  for (vectorUB = 0; vectorUB < 6; vectorUB++) {
    ipiv[vectorUB] = (int8_T)(vectorUB + 1);
  }

  for (d_j = 0; d_j < 5; d_j++) {
    jj = d_j * 7;
    iy = 7 - d_j;
    b_ix = 0;
    smax = fabs(b_A[jj]);
    for (c_k = 2; c_k < iy; c_k++) {
      real_T s;
      s = fabs(b_A[(jj + c_k) - 1]);
      if (s > smax) {
        b_ix = c_k - 1;
        smax = s;
      }
    }

    if (b_A[jj + b_ix] != 0.0) {
      if (b_ix != 0) {
        iy = d_j + b_ix;
        ipiv[d_j] = (int8_T)(iy + 1);
        for (ix = 0; ix < 6; ix++) {
          b_ix = ix * 6 + d_j;
          smax = b_A[b_ix];
          b_A[b_ix] = b_A[iy];
          b_A[iy] = smax;
          iy += 6;
        }
      }

      iy = (jj - d_j) + 6;
      c_k = (((((iy - jj) - 1) / 2) << 1) + jj) + 2;
      vectorUB = c_k - 2;
      for (b_ix = jj + 2; b_ix <= vectorUB; b_ix += 2) {
        tmp = _mm_loadu_pd(&b_A[b_ix - 1]);
        _mm_storeu_pd(&b_A[b_ix - 1], _mm_div_pd(tmp, _mm_set1_pd(b_A[jj])));
      }

      for (b_ix = c_k; b_ix <= iy; b_ix++) {
        b_A[b_ix - 1] /= b_A[jj];
      }
    }

    iy = 4 - d_j;
    b_ix = jj + 8;
    for (c_k = 0; c_k <= iy; c_k++) {
      smax = b_A[(c_k * 6 + jj) + 6];
      if (smax != 0.0) {
        ix = (b_ix - d_j) + 4;
        for (vectorUB = b_ix; vectorUB <= ix; vectorUB++) {
          b_A[vectorUB - 1] += b_A[((jj + vectorUB) - b_ix) + 1] * -smax;
        }
      }

      b_ix += 6;
    }
  }

  for (d_j = 0; d_j < 6; d_j++) {
    jj = 12 * d_j;
    iy = 6 * d_j;
    for (b_ix = 0; b_ix < d_j; b_ix++) {
      ix = 12 * b_ix;
      smax = b_A[b_ix + iy];
      if (smax != 0.0) {
        for (c_k = 0; c_k < 12; c_k++) {
          vectorUB = c_k + jj;
          A[vectorUB] -= A[c_k + ix] * smax;
        }
      }
    }

    smax = 1.0 / b_A[d_j + iy];
    for (iy = 0; iy <= 10; iy += 2) {
      vectorUB = iy + jj;
      tmp = _mm_loadu_pd(&A[vectorUB]);
      _mm_storeu_pd(&A[vectorUB], _mm_mul_pd(tmp, _mm_set1_pd(smax)));
    }
  }

  for (d_j = 5; d_j >= 0; d_j--) {
    jj = 12 * d_j;
    iy = 6 * d_j - 1;
    for (b_ix = d_j + 2; b_ix < 7; b_ix++) {
      ix = (b_ix - 1) * 12;
      smax = b_A[b_ix + iy];
      if (smax != 0.0) {
        for (c_k = 0; c_k < 12; c_k++) {
          vectorUB = c_k + jj;
          A[vectorUB] -= A[c_k + ix] * smax;
        }
      }
    }
  }

  for (d_j = 4; d_j >= 0; d_j--) {
    int8_T ipiv_0;
    ipiv_0 = ipiv[d_j];
    if (d_j + 1 != ipiv_0) {
      for (iy = 0; iy < 12; iy++) {
        b_ix = 12 * d_j + iy;
        smax = A[b_ix];
        vectorUB = (ipiv_0 - 1) * 12 + iy;
        A[b_ix] = A[vectorUB];
        A[vectorUB] = smax;
      }
    }
  }
}

real_T rt_atan2d_snf(real_T u0, real_T u1)
{
  real_T y;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = (rtNaN);
  } else if (rtIsInf(u0) && rtIsInf(u1)) {
    int32_T tmp;
    int32_T tmp_0;
    if (u0 > 0.0) {
      tmp = 1;
    } else {
      tmp = -1;
    }

    if (u1 > 0.0) {
      tmp_0 = 1;
    } else {
      tmp_0 = -1;
    }

    y = atan2(tmp, tmp_0);
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      y = RT_PI / 2.0;
    } else if (u0 < 0.0) {
      y = -(RT_PI / 2.0);
    } else {
      y = 0.0;
    }
  } else {
    y = atan2(u0, u1);
  }

  return y;
}

/* Model step function */
void Autopilot_step(void)
{
  real_T Fd[144];
  real_T P_pred[144];
  real_T Q[144];
  real_T K[72];
  real_T b_K[36];
  real_T x_pred[12];
  real_T z[6];
  real_T z_pred[6];
  real_T acceleration_ned[3];
  real_T cp;
  real_T ct;
  real_T cy;
  real_T sp;
  real_T st;
  real_T sy;
  real_T tan_theta;
  int32_T r1;
  int32_T r2;
  int32_T r3;
  int32_T rtemp;
  static const real_T b[3] = { 0.0, 0.0, 9.80665 };

  __m128d tmp_3;
  __m128d tmp_4;
  __m128d tmp_5;
  real_T Fd_0[144];
  real_T P_pred_0[72];
  real_T z_pred_tmp[72];
  real_T A_tmp_0[36];
  real_T ct_0[9];
  real_T z_0[6];
  real_T x_pred_0[3];
  real_T tmp_6[2];
  real_T K_now_idx_1;
  real_T K_now_idx_2;
  real_T Q_tmp;
  real_T ct_tmp;
  real_T ct_tmp_0;
  real_T ct_tmp_1;
  real_T ct_tmp_2;
  real_T ct_tmp_3;
  real_T ct_tmp_4;
  real_T ct_tmp_5;
  real_T ct_tmp_6;
  real_T ct_tmp_7;
  real_T rtb_DeadZone;
  real_T rtb_DigitalClock;
  real_T rtb_FilterCoefficient;
  real_T rtb_IntegralGain;
  real_T tmp;
  real_T tmp_0;
  real_T x_pred_tmp;
  real_T x_pred_tmp_tmp;
  int32_T b_K_tmp;
  int32_T b_K_tmp_0;
  int32_T b_K_tmp_1;
  int8_T A_tmp[36];
  int8_T tmp_1;
  int8_T tmp_2;
  boolean_T tmp_7;
  static const real_T b_0[3] = { 0.0, 0.0, 9.80665 };

  static const real_T v[12] = { 1.0E-6, 1.0E-6, 1.0E-6, 0.0016, 0.0016, 0.0016,
    2.7415567780803767E-7, 2.7415567780803767E-7, 2.7415567780803767E-7, 2.5E-5,
    2.5E-5, 2.5E-5 };

  static const int8_T a[72] = { 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
    0, 0, 0, 1 };

  static const int8_T c[72] = { 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 1 };

  static const int8_T d[36] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1 };

  static const int8_T b_a[36] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  static const real_T b_b[36] = { 4.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 4.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 9.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 4.0E-6, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 4.0E-6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 4.0E-6 };

  static const real_T c_b[9] = { 4.0E-6, 0.0, 0.0, 0.0, 4.0E-6, 0.0, 0.0, 0.0,
    4.0E-6 };

  static const int8_T e[144] = { 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1 };

  static const uint8_T c_0[3] = { 100U, 185U, 220U };

  static const real_T b_1[6] = { 2.2724, 2.2368, 2.2149, 0.6695, 0.5453, 0.5334
  };

  static const real_T b_2[12] = { 0.0176, 0.0213, 0.0198, 1.581, 2.2732, 2.352,
    -7.7995, -7.5064, -7.1666, -2.415, -2.1186, -2.0173 };

  /* Sum: '<S1>/Subtract3' incorporates:
   *  Constant: '<S1>/velocity_trim2'
   *  Inport: '<Root>/U_meas1'
   */
  rtb_IntegralGain = Autopilot_P.velocity_trim2_Value - Autopilot_U.U_meas_l;

  /* Gain: '<S102>/Filter Coefficient' incorporates:
   *  DiscreteIntegrator: '<S94>/Filter'
   *  Gain: '<S92>/Derivative Gain'
   *  Sum: '<S94>/SumD'
   */
  rtb_FilterCoefficient = (Autopilot_P.PIDController1_D * rtb_IntegralGain -
    Autopilot_DW.Filter_DSTATE) * Autopilot_P.PIDController1_N;

  /* Sum: '<S108>/Sum' incorporates:
   *  DiscreteIntegrator: '<S99>/Integrator'
   *  Gain: '<S104>/Proportional Gain'
   */
  rtb_DeadZone = (Autopilot_P.PIDController1_P * rtb_IntegralGain +
                  Autopilot_DW.Integrator_DSTATE) + rtb_FilterCoefficient;

  /* Saturate: '<S106>/Saturation' */
  if (rtb_DeadZone > Autopilot_P.PIDController1_UpperSaturationL) {
    ct = Autopilot_P.PIDController1_UpperSaturationL;
  } else if (rtb_DeadZone < Autopilot_P.PIDController1_LowerSaturationL) {
    ct = Autopilot_P.PIDController1_LowerSaturationL;
  } else {
    ct = rtb_DeadZone;
  }

  /* Sum: '<S1>/Sum2' incorporates:
   *  Constant: '<S1>/Throttle'
   *  Saturate: '<S106>/Saturation'
   */
  rtb_DigitalClock = ct + Autopilot_P.Throttle_Value;

  /* Saturate: '<S1>/Saturation1' */
  if (rtb_DigitalClock > Autopilot_P.Saturation1_UpperSat) {
    /* Outport: '<Root>/Throttle_cmd' */
    Autopilot_Y.Throttle_cmd = Autopilot_P.Saturation1_UpperSat;
  } else if (rtb_DigitalClock < Autopilot_P.Saturation1_LowerSat) {
    /* Outport: '<Root>/Throttle_cmd' */
    Autopilot_Y.Throttle_cmd = Autopilot_P.Saturation1_LowerSat;
  } else {
    /* Outport: '<Root>/Throttle_cmd' */
    Autopilot_Y.Throttle_cmd = rtb_DigitalClock;
  }

  /* End of Saturate: '<S1>/Saturation1' */

  /* DigitalClock: '<S1>/Digital Clock' */
  rtb_DigitalClock = Autopilot_M->Timing.taskTime0;

  /* MATLAB Function: '<S1>/Navigation_EKF' incorporates:
   *  Inport: '<Root>/dt'
   *  Inport: '<Root>/gps'
   *  Inport: '<Root>/imu'
   *  MATLAB Function: '<S1>/GPS_Validity'
   */
  if (!Autopilot_DW.initialized_not_empty) {
    Autopilot_DW.x[0] = Autopilot_U.gps[0];
    Autopilot_DW.x[1] = Autopilot_U.gps[1];
    Autopilot_DW.x[2] = Autopilot_U.gps[2];
    Autopilot_DW.x[3] = -39.880822458921287;
    Autopilot_DW.x[4] = -39.880822458921273;
    Autopilot_DW.x[5] = 0.0;
    Autopilot_DW.x[6] = 0.0;
    Autopilot_DW.x[7] = 0.0026556;
    Autopilot_DW.x[8] = 3.9269908169872414;
    Autopilot_DW.x[9] = Autopilot_U.imu[3];
    Autopilot_DW.x[10] = Autopilot_U.imu[4];
    Autopilot_DW.x[11] = Autopilot_U.imu[5];
    Autopilot_DW.initialized_not_empty = true;
  }

  cp = cos(Autopilot_DW.x[6]);
  sp = sin(Autopilot_DW.x[6]);
  ct = cos(Autopilot_DW.x[7]);
  st = sin(Autopilot_DW.x[7]);
  cy = cos(Autopilot_DW.x[8]);
  sy = sin(Autopilot_DW.x[8]);
  K_now_idx_1 = ct * cy;
  ct_0[0] = K_now_idx_1;
  K_now_idx_2 = sp * st;
  ct_tmp_1 = cp * sy;
  ct_tmp_7 = K_now_idx_2 * cy - ct_tmp_1;
  ct_0[3] = ct_tmp_7;
  ct_tmp = cp * st;
  ct_tmp_0 = ct_tmp * cy + sp * sy;
  ct_0[6] = ct_tmp_0;
  ct_0[1] = ct * sy;
  ct_tmp_4 = cp * cy;
  ct_0[4] = K_now_idx_2 * sy + ct_tmp_4;
  ct_tmp_5 = sp * cy;
  ct_tmp_6 = ct_tmp * sy - ct_tmp_5;
  ct_0[7] = ct_tmp_6;
  ct_0[2] = -st;
  ct_tmp_2 = sp * ct;
  ct_0[5] = ct_tmp_2;
  ct_tmp_3 = cp * ct;
  ct_0[8] = ct_tmp_3;
  tmp = Autopilot_U.imu[0];
  tan_theta = Autopilot_U.imu[1];
  tmp_0 = Autopilot_U.imu[2];
  for (r2 = 0; r2 <= 0; r2 += 2) {
    tmp_3 = _mm_loadu_pd(&ct_0[r2 + 3]);
    tmp_4 = _mm_loadu_pd(&ct_0[r2]);
    tmp_5 = _mm_loadu_pd(&ct_0[r2 + 6]);
    _mm_storeu_pd(&acceleration_ned[r2], _mm_add_pd(_mm_add_pd(_mm_add_pd
      (_mm_mul_pd(tmp_3, _mm_set1_pd(tan_theta)), _mm_mul_pd(tmp_4, _mm_set1_pd
      (tmp))), _mm_mul_pd(tmp_5, _mm_set1_pd(tmp_0))), _mm_loadu_pd(&b[r2])));
  }

  for (r2 = 2; r2 < 3; r2++) {
    acceleration_ned[r2] = ((ct_0[r2 + 3] * tan_theta + ct_0[r2] * tmp) +
      ct_0[r2 + 6] * tmp_0) + b_0[r2];
  }

  tan_theta = tan(Autopilot_DW.x[7]);
  tmp = cp * tan_theta;
  tan_theta *= sp;
  x_pred[6] = (tan_theta * Autopilot_U.imu[4] + Autopilot_U.imu[3]) + tmp *
    Autopilot_U.imu[5];
  tmp_0 = cp * Autopilot_U.imu[4] - sp * Autopilot_U.imu[5];
  x_pred[7] = tmp_0;
  x_pred_tmp_tmp = cp * Autopilot_U.imu[5];
  x_pred_tmp = sp * Autopilot_U.imu[4] + x_pred_tmp_tmp;
  x_pred[8] = x_pred_tmp / ct;
  x_pred[0] = Autopilot_DW.x[3];
  x_pred[1] = Autopilot_DW.x[4];
  x_pred[2] = Autopilot_DW.x[5];
  x_pred[3] = acceleration_ned[0];
  x_pred[4] = acceleration_ned[1];
  x_pred[5] = acceleration_ned[2];
  x_pred[9] = 0.0;
  x_pred[10] = 0.0;
  x_pred[11] = 0.0;
  for (r2 = 0; r2 <= 10; r2 += 2) {
    tmp_3 = _mm_loadu_pd(&x_pred[r2]);
    tmp_4 = _mm_loadu_pd(&Autopilot_DW.x[r2]);
    _mm_storeu_pd(&x_pred[r2], _mm_add_pd(_mm_mul_pd(tmp_3, _mm_set1_pd
      (Autopilot_U.dt)), tmp_4));
  }

  tmp_3 = _mm_set1_pd(3.1415926535897931);
  tmp_4 = _mm_add_pd(_mm_loadu_pd(&x_pred[6]), tmp_3);
  _mm_storeu_pd(&x_pred_0[0], tmp_4);
  x_pred_0[2] = x_pred[8] + 3.1415926535897931;
  Autopilot_mod(x_pred_0, acceleration_ned);
  tmp_4 = _mm_sub_pd(_mm_loadu_pd(&acceleration_ned[0]), tmp_3);
  _mm_storeu_pd(&x_pred[6], tmp_4);
  x_pred[8] = acceleration_ned[2] - 3.1415926535897931;
  memset(&Q[0], 0, 144U * sizeof(real_T));
  Q[36] = 1.0;
  Q[49] = 1.0;
  Q[62] = 1.0;
  Q_tmp = -sp * st;
  Q[75] = (Q_tmp * cy + ct_tmp_1) * Autopilot_U.imu[2] + ct_tmp_0 *
    Autopilot_U.imu[1];
  Q[87] = (ct_tmp_2 * cy * Autopilot_U.imu[1] + -st * cy * Autopilot_U.imu[0]) +
    ct_tmp_3 * cy * Autopilot_U.imu[2];
  Q_tmp = Q_tmp * sy - ct_tmp_4;
  Q[99] = (-cp * st * sy + ct_tmp_5) * Autopilot_U.imu[2] + (-ct * sy *
    Autopilot_U.imu[0] + Q_tmp * Autopilot_U.imu[1]);
  Q[76] = ct_tmp_6 * Autopilot_U.imu[1] + Q_tmp * Autopilot_U.imu[2];
  Q[88] = (ct_tmp_2 * sy * Autopilot_U.imu[1] + -st * sy * Autopilot_U.imu[0]) +
    ct_tmp_3 * sy * Autopilot_U.imu[2];
  Q[100] = (ct_tmp_7 * Autopilot_U.imu[1] + K_now_idx_1 * Autopilot_U.imu[0]) +
    ct_tmp_0 * Autopilot_U.imu[2];
  Q[77] = ct_tmp_3 * Autopilot_U.imu[1] - ct_tmp_2 * Autopilot_U.imu[2];
  Q[89] = (-ct * Autopilot_U.imu[0] - K_now_idx_2 * Autopilot_U.imu[1]) - ct_tmp
    * Autopilot_U.imu[2];
  Q[78] = tmp * Autopilot_U.imu[4] - tan_theta * Autopilot_U.imu[5];
  Q_tmp = 1.0 / (ct * ct) * x_pred_tmp;
  Q[90] = Q_tmp;
  Q[79] = -sp * Autopilot_U.imu[4] - x_pred_tmp_tmp;
  Q[80] = tmp_0 / ct;
  Q[92] = Q_tmp / ct;
  memset(&Fd[0], 0, 144U * sizeof(real_T));
  for (r1 = 0; r1 < 12; r1++) {
    Fd[r1 + 12 * r1] = 1.0;
  }

  for (r2 = 0; r2 <= 142; r2 += 2) {
    tmp_4 = _mm_loadu_pd(&Q[r2]);
    tmp_5 = _mm_loadu_pd(&Fd[r2]);
    _mm_storeu_pd(&Fd[r2], _mm_add_pd(_mm_mul_pd(tmp_4, _mm_set1_pd
      (Autopilot_U.dt)), tmp_5));
    _mm_storeu_pd(&Q[r2], _mm_set1_pd(0.0));
  }

  for (r1 = 0; r1 < 12; r1++) {
    Q[r1 + 12 * r1] = v[r1];
    memset(&Fd_0[r1 * 12], 0, 12U * sizeof(real_T));
    for (r2 = 0; r2 < 12; r2++) {
      ct = Autopilot_DW.P[12 * r1 + r2];
      for (r3 = 0; r3 <= 10; r3 += 2) {
        tmp_4 = _mm_loadu_pd(&Fd[12 * r2 + r3]);
        b_K_tmp = 12 * r1 + r3;
        tmp_5 = _mm_loadu_pd(&Fd_0[b_K_tmp]);
        _mm_storeu_pd(&Fd_0[b_K_tmp], _mm_add_pd(_mm_mul_pd(tmp_4, _mm_set1_pd
          (ct)), tmp_5));
      }
    }
  }

  for (r2 = 0; r2 < 12; r2++) {
    for (r3 = 0; r3 < 12; r3++) {
      ct = 0.0;
      for (b_K_tmp = 0; b_K_tmp < 12; b_K_tmp++) {
        ct += Fd_0[12 * b_K_tmp + r2] * Fd[12 * b_K_tmp + r3];
      }

      r1 = 12 * r3 + r2;
      P_pred[r1] = Q[r1] + ct;
    }
  }

  for (r2 = 0; r2 < 12; r2++) {
    for (r3 = 0; r3 < 12; r3++) {
      b_K_tmp = 12 * r2 + r3;
      Q[b_K_tmp] = (P_pred[12 * r3 + r2] + P_pred[b_K_tmp]) * 0.5;
    }
  }

  memcpy(&P_pred[0], &Q[0], 144U * sizeof(real_T));
  z[0] = Autopilot_U.gps[0];
  z[1] = Autopilot_U.gps[1];
  z[2] = Autopilot_U.gps[2];
  z[3] = Autopilot_U.imu[3];
  z[4] = Autopilot_U.imu[4];
  z[5] = Autopilot_U.imu[5];
  for (r2 = 0; r2 < 6; r2++) {
    z_pred[r2] = 0.0;
  }

  for (r2 = 0; r2 < 12; r2++) {
    ct = x_pred[r2];
    for (r3 = 0; r3 < 6; r3++) {
      z_pred[r3] += (real_T)(&a[0])[6 * r2 + r3] * ct;
    }
  }

  if ((rtb_DigitalClock >= 25.0) && (rtb_DigitalClock < 35.0)) {
    for (r2 = 0; r2 < 36; r2++) {
      b_K[r2] = d[r2];
      A_tmp[r2] = b_a[r2];
    }

    for (r2 = 0; r2 < 12; r2++) {
      cp = 0.0;
      sp = 0.0;
      rtb_DigitalClock = 0.0;
      for (r3 = 0; r3 < 12; r3++) {
        ct = P_pred[12 * r2 + r3];
        _mm_storeu_pd(&tmp_6[0], _mm_add_pd(_mm_mul_pd(_mm_set_pd(A_tmp[3 * r3 +
          1], A_tmp[3 * r3]), _mm_set1_pd(ct)), _mm_set_pd(sp, cp)));
        cp = tmp_6[0];
        sp = tmp_6[1];
        rtb_DigitalClock += (real_T)A_tmp[3 * r3 + 2] * ct;
      }

      A_tmp_0[3 * r2 + 2] = rtb_DigitalClock;
      A_tmp_0[3 * r2 + 1] = sp;
      A_tmp_0[3 * r2] = cp;
    }

    for (r2 = 0; r2 < 3; r2++) {
      for (r3 = 0; r3 < 3; r3++) {
        ct = 0.0;
        for (b_K_tmp = 0; b_K_tmp < 12; b_K_tmp++) {
          ct += A_tmp_0[3 * b_K_tmp + r2] * b_K[12 * r3 + b_K_tmp];
        }

        r1 = 3 * r3 + r2;
        ct_0[r1] = c_b[r1] + ct;
      }
    }

    for (r2 = 0; r2 < 12; r2++) {
      A_tmp_0[r2] = 0.0;
      A_tmp_0[r2 + 12] = 0.0;
      A_tmp_0[r2 + 24] = 0.0;
    }

    for (r2 = 0; r2 < 3; r2++) {
      for (r3 = 0; r3 < 12; r3++) {
        b_K_tmp = (int32_T)b_K[12 * r2 + r3];
        for (r1 = 0; r1 <= 10; r1 += 2) {
          tmp_4 = _mm_loadu_pd(&P_pred[12 * r3 + r1]);
          rtemp = 12 * r2 + r1;
          tmp_5 = _mm_loadu_pd(&A_tmp_0[rtemp]);
          _mm_storeu_pd(&A_tmp_0[rtemp], _mm_add_pd(_mm_mul_pd(tmp_4,
            _mm_set1_pd(b_K_tmp)), tmp_5));
        }
      }
    }

    r1 = 0;
    r2 = 1;
    r3 = 2;
    rtb_DigitalClock = fabs(ct_0[0]);
    cp = fabs(ct_0[1]);
    if (cp > rtb_DigitalClock) {
      rtb_DigitalClock = cp;
      r1 = 1;
      r2 = 0;
    }

    if (fabs(ct_0[2]) > rtb_DigitalClock) {
      r1 = 2;
      r2 = 1;
      r3 = 0;
    }

    ct_0[r2] /= ct_0[r1];
    ct_0[r3] /= ct_0[r1];
    ct_0[r2 + 3] -= ct_0[r1 + 3] * ct_0[r2];
    ct_0[r3 + 3] -= ct_0[r1 + 3] * ct_0[r3];
    ct_0[r2 + 6] -= ct_0[r1 + 6] * ct_0[r2];
    ct_0[r3 + 6] -= ct_0[r1 + 6] * ct_0[r3];
    if (fabs(ct_0[r3 + 3]) > fabs(ct_0[r2 + 3])) {
      rtemp = r2;
      r2 = r3;
      r3 = rtemp;
    }

    ct_0[r3 + 3] /= ct_0[r2 + 3];
    ct_0[r3 + 6] -= ct_0[r3 + 3] * ct_0[r2 + 6];
    for (rtemp = 0; rtemp < 12; rtemp++) {
      b_K_tmp = 12 * r1 + rtemp;
      b_K[b_K_tmp] = A_tmp_0[rtemp] / ct_0[r1];
      b_K_tmp_0 = 12 * r2 + rtemp;
      b_K[b_K_tmp_0] = A_tmp_0[rtemp + 12] - ct_0[r1 + 3] * b_K[b_K_tmp];
      b_K_tmp_1 = 12 * r3 + rtemp;
      b_K[b_K_tmp_1] = A_tmp_0[rtemp + 24] - ct_0[r1 + 6] * b_K[b_K_tmp];
      b_K[b_K_tmp_0] /= ct_0[r2 + 3];
      b_K[b_K_tmp_1] -= ct_0[r2 + 6] * b_K[b_K_tmp_0];
      b_K[b_K_tmp_1] /= ct_0[r3 + 6];
      b_K[b_K_tmp_0] -= ct_0[r3 + 3] * b_K[b_K_tmp_1];
      b_K[b_K_tmp] -= b_K[b_K_tmp_1] * ct_0[r3];
      b_K[b_K_tmp] -= b_K[b_K_tmp_0] * ct_0[r2];
    }

    tmp_4 = _mm_sub_pd(_mm_loadu_pd(&Autopilot_U.imu[3]), _mm_loadu_pd(&z_pred[3]));
    _mm_storeu_pd(&tmp_6[0], tmp_4);
    cp = tmp_6[0];
    sp = tmp_6[1];
    rtb_DigitalClock = Autopilot_U.imu[5] - z_pred[5];
    for (r2 = 0; r2 < 12; r2++) {
      ct = b_K[r2 + 12];
      st = b_K[r2];
      cy = b_K[r2 + 24];
      Autopilot_DW.x[r2] = ((ct * sp + st * cp) + cy * rtb_DigitalClock) +
        x_pred[r2];
      for (r3 = 0; r3 < 12; r3++) {
        r1 = 12 * r3 + r2;
        Q[r1] = (real_T)e[r1] - (((real_T)A_tmp[3 * r3 + 1] * ct + (real_T)
          A_tmp[3 * r3] * st) + (real_T)A_tmp[3 * r3 + 2] * cy);
        Fd[r3 + 12 * r2] = 0.0;
      }
    }

    for (r2 = 0; r2 < 12; r2++) {
      for (r3 = 0; r3 < 12; r3++) {
        ct = P_pred[12 * r2 + r3];
        for (b_K_tmp = 0; b_K_tmp <= 10; b_K_tmp += 2) {
          tmp_4 = _mm_loadu_pd(&Q[12 * r3 + b_K_tmp]);
          r1 = 12 * r2 + b_K_tmp;
          tmp_5 = _mm_loadu_pd(&Fd[r1]);
          _mm_storeu_pd(&Fd[r1], _mm_add_pd(_mm_mul_pd(tmp_4, _mm_set1_pd(ct)),
            tmp_5));
        }
      }

      A_tmp_0[r2] = 0.0;
      A_tmp_0[r2 + 12] = 0.0;
      A_tmp_0[r2 + 24] = 0.0;
    }

    for (r2 = 0; r2 < 3; r2++) {
      for (r3 = 0; r3 < 3; r3++) {
        ct = c_b[3 * r2 + r3];
        for (b_K_tmp = 0; b_K_tmp <= 10; b_K_tmp += 2) {
          tmp_4 = _mm_loadu_pd(&b_K[12 * r3 + b_K_tmp]);
          r1 = 12 * r2 + b_K_tmp;
          tmp_5 = _mm_loadu_pd(&A_tmp_0[r1]);
          _mm_storeu_pd(&A_tmp_0[r1], _mm_add_pd(_mm_mul_pd(tmp_4, _mm_set1_pd
            (ct)), tmp_5));
        }
      }
    }

    for (r2 = 0; r2 < 12; r2++) {
      memset(&P_pred[r2 * 12], 0, 12U * sizeof(real_T));
      for (r3 = 0; r3 < 12; r3++) {
        ct = Q[12 * r3 + r2];
        for (b_K_tmp = 0; b_K_tmp <= 10; b_K_tmp += 2) {
          tmp_4 = _mm_loadu_pd(&Fd[12 * r3 + b_K_tmp]);
          r1 = 12 * r2 + b_K_tmp;
          tmp_5 = _mm_loadu_pd(&P_pred[r1]);
          _mm_storeu_pd(&P_pred[r1], _mm_add_pd(_mm_mul_pd(tmp_4, _mm_set1_pd(ct)),
            tmp_5));
        }

        Fd_0[r3 + 12 * r2] = 0.0;
      }

      for (r3 = 0; r3 < 3; r3++) {
        ct = b_K[12 * r3 + r2];
        for (b_K_tmp = 0; b_K_tmp <= 10; b_K_tmp += 2) {
          tmp_4 = _mm_loadu_pd(&A_tmp_0[12 * r3 + b_K_tmp]);
          r1 = 12 * r2 + b_K_tmp;
          tmp_5 = _mm_loadu_pd(&Fd_0[r1]);
          _mm_storeu_pd(&Fd_0[r1], _mm_add_pd(_mm_mul_pd(tmp_4, _mm_set1_pd(ct)),
            tmp_5));
        }
      }
    }

    for (r2 = 0; r2 <= 142; r2 += 2) {
      tmp_4 = _mm_loadu_pd(&P_pred[r2]);
      tmp_5 = _mm_loadu_pd(&Fd_0[r2]);
      _mm_storeu_pd(&Autopilot_DW.P[r2], _mm_add_pd(tmp_4, tmp_5));
    }
  } else {
    for (r2 = 0; r2 < 72; r2++) {
      K[r2] = c[r2];
    }

    for (r2 = 0; r2 < 12; r2++) {
      for (r3 = 0; r3 < 6; r3++) {
        z_pred_tmp[r3 + 6 * r2] = 0.0;
      }

      for (r3 = 0; r3 < 12; r3++) {
        ct = P_pred[12 * r2 + r3];
        for (b_K_tmp = 0; b_K_tmp < 6; b_K_tmp++) {
          r1 = 6 * r2 + b_K_tmp;
          z_pred_tmp[r1] += (real_T)(&a[0])[6 * r3 + b_K_tmp] * ct;
        }
      }
    }

    for (r2 = 0; r2 < 6; r2++) {
      for (r3 = 0; r3 < 6; r3++) {
        ct = 0.0;
        for (b_K_tmp = 0; b_K_tmp < 12; b_K_tmp++) {
          ct += z_pred_tmp[6 * b_K_tmp + r2] * K[12 * r3 + b_K_tmp];
        }

        b_K_tmp = 6 * r3 + r2;
        b_K[b_K_tmp] = b_b[b_K_tmp] + ct;
      }

      memset(&P_pred_0[r2 * 12], 0, 12U * sizeof(real_T));
      for (r3 = 0; r3 < 12; r3++) {
        b_K_tmp = (int32_T)K[12 * r2 + r3];
        for (r1 = 0; r1 <= 10; r1 += 2) {
          tmp_4 = _mm_loadu_pd(&P_pred[12 * r3 + r1]);
          rtemp = 12 * r2 + r1;
          tmp_5 = _mm_loadu_pd(&P_pred_0[rtemp]);
          _mm_storeu_pd(&P_pred_0[rtemp], _mm_add_pd(_mm_mul_pd(tmp_4,
            _mm_set1_pd(b_K_tmp)), tmp_5));
        }
      }
    }

    memcpy(&K[0], &P_pred_0[0], 72U * sizeof(real_T));
    Autopilot_mrdiv_m(K, b_K);
    for (r2 = 0; r2 <= 4; r2 += 2) {
      tmp_4 = _mm_loadu_pd(&z[r2]);
      tmp_5 = _mm_loadu_pd(&z_pred[r2]);
      _mm_storeu_pd(&z_0[r2], _mm_sub_pd(tmp_4, tmp_5));
    }

    for (r2 = 0; r2 < 12; r2++) {
      ct = 0.0;
      for (r3 = 0; r3 < 6; r3++) {
        ct += K[12 * r3 + r2] * z_0[r3];
      }

      Autopilot_DW.x[r2] = x_pred[r2] + ct;
      for (r3 = 0; r3 < 12; r3++) {
        ct = 0.0;
        for (b_K_tmp = 0; b_K_tmp < 6; b_K_tmp++) {
          ct += K[12 * b_K_tmp + r2] * (real_T)(&a[0])[6 * r3 + b_K_tmp];
        }

        r1 = 12 * r3 + r2;
        Q[r1] = (real_T)e[r1] - ct;
        Fd[r3 + 12 * r2] = 0.0;
      }
    }

    for (r2 = 0; r2 < 12; r2++) {
      for (r3 = 0; r3 < 12; r3++) {
        ct = P_pred[12 * r2 + r3];
        for (b_K_tmp = 0; b_K_tmp <= 10; b_K_tmp += 2) {
          tmp_4 = _mm_loadu_pd(&Q[12 * r3 + b_K_tmp]);
          r1 = 12 * r2 + b_K_tmp;
          tmp_5 = _mm_loadu_pd(&Fd[r1]);
          _mm_storeu_pd(&Fd[r1], _mm_add_pd(_mm_mul_pd(tmp_4, _mm_set1_pd(ct)),
            tmp_5));
        }
      }
    }

    for (r2 = 0; r2 < 6; r2++) {
      memset(&z_pred_tmp[r2 * 12], 0, 12U * sizeof(real_T));
      for (r3 = 0; r3 < 6; r3++) {
        ct = b_b[6 * r2 + r3];
        for (b_K_tmp = 0; b_K_tmp <= 10; b_K_tmp += 2) {
          tmp_4 = _mm_loadu_pd(&K[12 * r3 + b_K_tmp]);
          r1 = 12 * r2 + b_K_tmp;
          tmp_5 = _mm_loadu_pd(&z_pred_tmp[r1]);
          _mm_storeu_pd(&z_pred_tmp[r1], _mm_add_pd(_mm_mul_pd(tmp_4,
            _mm_set1_pd(ct)), tmp_5));
        }
      }
    }

    for (r2 = 0; r2 < 12; r2++) {
      memset(&P_pred[r2 * 12], 0, 12U * sizeof(real_T));
      for (r3 = 0; r3 < 12; r3++) {
        ct = Q[12 * r3 + r2];
        for (b_K_tmp = 0; b_K_tmp <= 10; b_K_tmp += 2) {
          tmp_4 = _mm_loadu_pd(&Fd[12 * r3 + b_K_tmp]);
          r1 = 12 * r2 + b_K_tmp;
          tmp_5 = _mm_loadu_pd(&P_pred[r1]);
          _mm_storeu_pd(&P_pred[r1], _mm_add_pd(_mm_mul_pd(tmp_4, _mm_set1_pd(ct)),
            tmp_5));
        }

        Fd_0[r3 + 12 * r2] = 0.0;
      }

      for (r3 = 0; r3 < 6; r3++) {
        ct = K[12 * r3 + r2];
        for (b_K_tmp = 0; b_K_tmp <= 10; b_K_tmp += 2) {
          tmp_4 = _mm_loadu_pd(&z_pred_tmp[12 * r3 + b_K_tmp]);
          r1 = 12 * r2 + b_K_tmp;
          tmp_5 = _mm_loadu_pd(&Fd_0[r1]);
          _mm_storeu_pd(&Fd_0[r1], _mm_add_pd(_mm_mul_pd(tmp_4, _mm_set1_pd(ct)),
            tmp_5));
        }
      }
    }

    for (r2 = 0; r2 <= 142; r2 += 2) {
      tmp_4 = _mm_loadu_pd(&P_pred[r2]);
      tmp_5 = _mm_loadu_pd(&Fd_0[r2]);
      _mm_storeu_pd(&Autopilot_DW.P[r2], _mm_add_pd(tmp_4, tmp_5));
    }
  }

  for (r2 = 0; r2 < 12; r2++) {
    for (r3 = 0; r3 < 12; r3++) {
      b_K_tmp = 12 * r2 + r3;
      Q[b_K_tmp] = (Autopilot_DW.P[12 * r3 + r2] + Autopilot_DW.P[b_K_tmp]) *
        0.5;
    }
  }

  memcpy(&Autopilot_DW.P[0], &Q[0], 144U * sizeof(real_T));
  tmp_4 = _mm_add_pd(_mm_loadu_pd(&Autopilot_DW.x[6]), tmp_3);
  _mm_storeu_pd(&acceleration_ned[0], tmp_4);
  acceleration_ned[2] = Autopilot_DW.x[8] + 3.1415926535897931;
  Autopilot_mod(acceleration_ned, x_pred_0);
  tmp_3 = _mm_sub_pd(_mm_loadu_pd(&x_pred_0[0]), tmp_3);
  _mm_storeu_pd(&Autopilot_DW.x[6], tmp_3);
  Autopilot_DW.x[8] = x_pred_0[2] - 3.1415926535897931;
  memcpy(&x_pred[0], &Autopilot_DW.x[0], 12U * sizeof(real_T));

  /* End of MATLAB Function: '<S1>/Navigation_EKF' */

  /* MATLAB Function: '<S1>/L1_Guidance' */
  rtb_DigitalClock = sqrt(x_pred[3] * x_pred[3] + x_pred[4] * x_pred[4]);
  if (rtb_DigitalClock < 1.0E-6) {
    cp = 0.0;
  } else {
    tmp_3 = _mm_div_pd(_mm_loadu_pd(&x_pred[3]), _mm_set1_pd(rtb_DigitalClock));
    _mm_storeu_pd(&tmp_6[0], tmp_3);
    cp = tmp_6[0];
    sp = tmp_6[1];
    if (Autopilot_DW.waypoint_index == 1) {
      r1 = 0;
      r2 = 0;
      rtemp = -4500;
      r3 = -4500;
    } else if (Autopilot_DW.waypoint_index == 2) {
      r1 = -4500;
      r2 = -4500;
      rtemp = 0;
      r3 = -9000;
    } else if (Autopilot_DW.waypoint_index == 3) {
      r1 = 0;
      r2 = -9000;
      rtemp = 4500;
      r3 = -4500;
    } else {
      r1 = 4500;
      r2 = -4500;
      rtemp = 0;
      r3 = 0;
    }

    ct = x_pred[0] - (real_T)rtemp;
    st = x_pred[1] - (real_T)r3;
    if (sqrt(ct * ct + st * st) < 350.0) {
      if (Autopilot_DW.waypoint_index <= 2147483646) {
        Autopilot_DW.waypoint_index++;
      }

      if (Autopilot_DW.waypoint_index > 4) {
        Autopilot_DW.waypoint_index = 1;
      }

      if (Autopilot_DW.waypoint_index == 1) {
        r1 = 0;
        r2 = 0;
        rtemp = -4500;
        r3 = -4500;
      } else if (Autopilot_DW.waypoint_index == 2) {
        r1 = -4500;
        r2 = -4500;
        rtemp = 0;
        r3 = -9000;
      } else if (Autopilot_DW.waypoint_index == 3) {
        r1 = 0;
        r2 = -9000;
        rtemp = 4500;
        r3 = -4500;
      } else {
        r1 = 4500;
        r2 = -4500;
        rtemp = 0;
        r3 = 0;
      }
    }

    rtemp -= r1;
    r3 -= r2;
    st = sqrt(rtemp * rtemp + r3 * r3);
    if (st < 1.0E-6) {
      cp = 0.0;
    } else {
      cy = (real_T)rtemp / st;
      ct = (real_T)r3 / st;
      sy = (x_pred[0] - (real_T)r1) * cy + (x_pred[1] - (real_T)r2) * ct;
      if (sy < 0.0) {
        tan_theta = 0.0;
      } else if (sy > st) {
        tan_theta = st;
      } else {
        tan_theta = sy;
      }

      sy = tan_theta + 350.0;
      if (tan_theta + 350.0 > st) {
        sy = st;
      }

      tmp_3 = _mm_sub_pd(_mm_add_pd(_mm_mul_pd(_mm_set1_pd(sy), _mm_set_pd(ct,
        cy)), _mm_set_pd(r2, r1)), _mm_loadu_pd(&x_pred[0]));
      _mm_storeu_pd(&tmp_6[0], tmp_3);
      cy = sqrt(tmp_6[0] * tmp_6[0] + tmp_6[1] * tmp_6[1]);
      if (cy < 1.0E-6) {
        cp = 0.0;
      } else {
        st = tmp_6[0] / cy;
        ct = tmp_6[1] / cy;
        cp = atan(2.0 * rtb_DigitalClock * rtb_DigitalClock / 350.0 * sin
                  (rt_atan2d_snf(cp * ct - sp * st, cp * st + sp * ct)) /
                  9.80665);
        if (cp > 0.29670597283903605) {
          cp = 0.29670597283903605;
        } else if (cp < -0.29670597283903605) {
          cp = -0.29670597283903605;
        }
      }
    }
  }

  /* MATLAB Function: '<S1>/MATLAB Function1' incorporates:
   *  Inport: '<Root>/U_meas1'
   *  MATLAB Function: '<S1>/MATLAB Function'
   */
  tmp_7 = rtIsNaN(Autopilot_U.U_meas_l);
  if (tmp_7) {
    sy = (rtNaN);
    K_now_idx_1 = (rtNaN);
  } else if (Autopilot_U.U_meas_l > 220.0) {
    sp = (Autopilot_U.U_meas_l - 220.0) / 35.0;
    sy = sp * -0.021900000000000031 + 2.2149;
    K_now_idx_1 = sp * -0.011900000000000022 + 0.5334;
  } else if (Autopilot_U.U_meas_l < 100.0) {
    sp = (Autopilot_U.U_meas_l - 100.0) / 85.0;
    sy = sp * -0.035600000000000076 + 2.2724;
    K_now_idx_1 = sp * -0.12419999999999998 + 0.6695;
  } else {
    r3 = 0;
    if (Autopilot_U.U_meas_l >= 185.0) {
      r3 = 1;
    }

    rtb_DigitalClock = (Autopilot_U.U_meas_l - (real_T)c_0[r3]) / (real_T)
      (c_0[r3 + 1] - c_0[r3]);
    if (rtb_DigitalClock == 0.0) {
      sy = b_1[r3];
    } else if (rtb_DigitalClock == 1.0) {
      sy = b_1[r3 + 1];
    } else if (b_1[r3 + 1] == b_1[r3]) {
      sy = b_1[r3];
    } else {
      sy = (1.0 - rtb_DigitalClock) * b_1[r3] + b_1[r3 + 1] * rtb_DigitalClock;
    }

    r3 = 0;
    if (Autopilot_U.U_meas_l >= 185.0) {
      r3 = 1;
    }

    rtb_DigitalClock = (Autopilot_U.U_meas_l - (real_T)c_0[r3]) / (real_T)
      (c_0[r3 + 1] - c_0[r3]);
    if (rtb_DigitalClock == 0.0) {
      K_now_idx_1 = b_1[r3 + 3];
    } else if (rtb_DigitalClock == 1.0) {
      K_now_idx_1 = b_1[r3 + 4];
    } else if (b_1[r3 + 3] == b_1[r3 + 4]) {
      K_now_idx_1 = b_1[r3 + 3];
    } else {
      K_now_idx_1 = (1.0 - rtb_DigitalClock) * b_1[r3 + 3] + b_1[r3 + 4] *
        rtb_DigitalClock;
    }
  }

  /* Gain: '<S1>/Gain1' incorporates:
   *  MATLAB Function: '<S1>/L1_Guidance'
   *  MATLAB Function: '<S1>/MATLAB Function1'
   *  SignalConversion generated from: '<S5>/ SFunction '
   *  Sum: '<S1>/Sum'
   */
  rtb_DigitalClock = ((x_pred[6] - cp) * sy + K_now_idx_1 * x_pred[9]) *
    Autopilot_P.Gain1_Gain;

  /* Saturate: '<S1>/Aileron Saturation' */
  if (rtb_DigitalClock > Autopilot_P.AileronSaturation_UpperSat) {
    /* Outport: '<Root>/Aileron Command_DLQR' */
    Autopilot_Y.AileronCommand_DLQR = Autopilot_P.AileronSaturation_UpperSat;
  } else if (rtb_DigitalClock < Autopilot_P.AileronSaturation_LowerSat) {
    /* Outport: '<Root>/Aileron Command_DLQR' */
    Autopilot_Y.AileronCommand_DLQR = Autopilot_P.AileronSaturation_LowerSat;
  } else {
    /* Outport: '<Root>/Aileron Command_DLQR' */
    Autopilot_Y.AileronCommand_DLQR = rtb_DigitalClock;
  }

  /* End of Saturate: '<S1>/Aileron Saturation' */

  /* Sum: '<S1>/Subtract' incorporates:
   *  Constant: '<S1>/alpha_trim'
   *  Inport: '<Root>/alpha_meas'
   */
  cp = Autopilot_U.alpha_meas - Autopilot_P.alpha_trim_Value;

  /* Sum: '<S1>/Sum1' incorporates:
   *  Constant: '<S1>/Constant'
   *  Constant: '<S1>/Initial Altitude'
   *  Gain: '<S1>/EKF_Altitude_ft'
   *  Sum: '<S1>/Sum3'
   */
  sp = Autopilot_P.Constant_Value - (Autopilot_P.EKF_Altitude_ft_Gain * x_pred[2]
    + Autopilot_P.InitialAltitude_Value);

  /* Gain: '<S48>/Filter Coefficient' incorporates:
   *  DiscreteIntegrator: '<S40>/Filter'
   *  Gain: '<S38>/Derivative Gain'
   *  Sum: '<S40>/SumD'
   */
  ct = (Autopilot_P.PIDController_D * sp - Autopilot_DW.Filter_DSTATE_e) *
    Autopilot_P.PIDController_N;

  /* Sum: '<S54>/Sum' incorporates:
   *  DiscreteIntegrator: '<S45>/Integrator'
   *  Gain: '<S50>/Proportional Gain'
   */
  st = (Autopilot_P.PIDController_P * sp + Autopilot_DW.Integrator_DSTATE_k) +
    ct;

  /* Saturate: '<S52>/Saturation' */
  if (st > Autopilot_P.PIDController_UpperSaturationLi) {
    cy = Autopilot_P.PIDController_UpperSaturationLi;
  } else if (st < Autopilot_P.PIDController_LowerSaturationLi) {
    cy = Autopilot_P.PIDController_LowerSaturationLi;
  } else {
    cy = st;
  }

  /* End of Saturate: '<S52>/Saturation' */

  /* MATLAB Function: '<S1>/MATLAB Function' incorporates:
   *  Inport: '<Root>/U_meas1'
   */
  if (tmp_7) {
    sy = (rtNaN);
    K_now_idx_1 = (rtNaN);
    K_now_idx_2 = (rtNaN);
    rtb_DigitalClock = (rtNaN);
  } else if (Autopilot_U.U_meas_l > 220.0) {
    tmp_3 = _mm_set_pd(2.352, 0.0198);
    tmp_4 = _mm_div_pd(_mm_sub_pd(_mm_set1_pd(Autopilot_U.U_meas_l), _mm_set1_pd
      (220.0)), _mm_set1_pd(35.0));
    _mm_storeu_pd(&tmp_6[0], _mm_add_pd(_mm_mul_pd(tmp_4, _mm_sub_pd(tmp_3,
      _mm_set_pd(2.2732, 0.0213))), tmp_3));
    sy = tmp_6[0];
    K_now_idx_1 = tmp_6[1];
    tmp_3 = _mm_set_pd(-2.0173, -7.1666);
    _mm_storeu_pd(&tmp_6[0], _mm_add_pd(_mm_mul_pd(tmp_4, _mm_sub_pd(tmp_3,
      _mm_set_pd(-2.1186, -7.5064))), tmp_3));
    K_now_idx_2 = tmp_6[0];
    rtb_DigitalClock = tmp_6[1];
  } else if (Autopilot_U.U_meas_l < 100.0) {
    tmp_3 = _mm_set_pd(1.581, 0.0176);
    tmp_4 = _mm_div_pd(_mm_sub_pd(_mm_set1_pd(Autopilot_U.U_meas_l), _mm_set1_pd
      (100.0)), _mm_set1_pd(85.0));
    _mm_storeu_pd(&tmp_6[0], _mm_add_pd(_mm_mul_pd(tmp_4, _mm_sub_pd(_mm_set_pd
      (2.2732, 0.0213), tmp_3)), tmp_3));
    sy = tmp_6[0];
    K_now_idx_1 = tmp_6[1];
    tmp_3 = _mm_set_pd(-2.415, -7.7995);
    _mm_storeu_pd(&tmp_6[0], _mm_add_pd(_mm_mul_pd(tmp_4, _mm_sub_pd(_mm_set_pd(
      -2.1186, -7.5064), tmp_3)), tmp_3));
    K_now_idx_2 = tmp_6[0];
    rtb_DigitalClock = tmp_6[1];
  } else {
    r3 = 0;
    if (Autopilot_U.U_meas_l >= 185.0) {
      r3 = 1;
    }

    rtb_DigitalClock = (Autopilot_U.U_meas_l - (real_T)c_0[r3]) / (real_T)
      (c_0[r3 + 1] - c_0[r3]);
    if (rtb_DigitalClock == 0.0) {
      sy = b_2[r3];
    } else if (rtb_DigitalClock == 1.0) {
      sy = b_2[r3 + 1];
    } else if (b_2[r3 + 1] == b_2[r3]) {
      sy = b_2[r3];
    } else {
      sy = (1.0 - rtb_DigitalClock) * b_2[r3] + b_2[r3 + 1] * rtb_DigitalClock;
    }

    r3 = 0;
    if (Autopilot_U.U_meas_l >= 185.0) {
      r3 = 1;
    }

    rtb_DigitalClock = (Autopilot_U.U_meas_l - (real_T)c_0[r3]) / (real_T)
      (c_0[r3 + 1] - c_0[r3]);
    if (rtb_DigitalClock == 0.0) {
      K_now_idx_1 = b_2[r3 + 3];
    } else if (rtb_DigitalClock == 1.0) {
      K_now_idx_1 = b_2[r3 + 4];
    } else if (b_2[r3 + 3] == b_2[r3 + 4]) {
      K_now_idx_1 = b_2[r3 + 3];
    } else {
      K_now_idx_1 = (1.0 - rtb_DigitalClock) * b_2[r3 + 3] + b_2[r3 + 4] *
        rtb_DigitalClock;
    }

    r3 = 0;
    if (Autopilot_U.U_meas_l >= 185.0) {
      r3 = 1;
    }

    rtb_DigitalClock = (Autopilot_U.U_meas_l - (real_T)c_0[r3]) / (real_T)
      (c_0[r3 + 1] - c_0[r3]);
    if (rtb_DigitalClock == 0.0) {
      K_now_idx_2 = b_2[r3 + 6];
    } else if (rtb_DigitalClock == 1.0) {
      K_now_idx_2 = b_2[r3 + 7];
    } else if (b_2[r3 + 6] == b_2[r3 + 7]) {
      K_now_idx_2 = b_2[r3 + 6];
    } else {
      K_now_idx_2 = (1.0 - rtb_DigitalClock) * b_2[r3 + 6] + b_2[r3 + 7] *
        rtb_DigitalClock;
    }

    r3 = 0;
    if (Autopilot_U.U_meas_l >= 185.0) {
      r3 = 1;
    }

    rtb_DigitalClock = (Autopilot_U.U_meas_l - (real_T)c_0[r3]) / (real_T)
      (c_0[r3 + 1] - c_0[r3]);
    if (rtb_DigitalClock == 0.0) {
      rtb_DigitalClock = b_2[r3 + 9];
    } else if (rtb_DigitalClock == 1.0) {
      rtb_DigitalClock = b_2[r3 + 10];
    } else if (b_2[r3 + 9] == b_2[r3 + 10]) {
      rtb_DigitalClock = b_2[r3 + 9];
    } else {
      rtb_DigitalClock = (1.0 - rtb_DigitalClock) * b_2[r3 + 9] + b_2[r3 + 10] *
        rtb_DigitalClock;
    }
  }

  /* Gain: '<S1>/Gain3' incorporates:
   *  Constant: '<S1>/Thita_trim'
   *  Constant: '<S1>/velocity_trim1'
   *  Inport: '<Root>/U_meas'
   *  MATLAB Function: '<S1>/MATLAB Function'
   *  SignalConversion generated from: '<S4>/ SFunction '
   *  Sum: '<S1>/Subtract1'
   *  Sum: '<S1>/Subtract2'
   *  Sum: '<S1>/Sum4'
   */
  rtb_DigitalClock = ((((Autopilot_U.U_meas - Autopilot_P.velocity_trim1_Value) *
                        sy + K_now_idx_1 * cp) + (x_pred[7] - (cy +
    Autopilot_P.Thita_trim_Value)) * K_now_idx_2) + rtb_DigitalClock * x_pred[10])
    * Autopilot_P.Gain3_Gain;

  /* Saturate: '<S1>/Saturation' */
  if (rtb_DigitalClock > Autopilot_P.Saturation_UpperSat) {
    /* Outport: '<Root>/Elevator Command' */
    Autopilot_Y.ElevatorCommand = Autopilot_P.Saturation_UpperSat;
  } else if (rtb_DigitalClock < Autopilot_P.Saturation_LowerSat) {
    /* Outport: '<Root>/Elevator Command' */
    Autopilot_Y.ElevatorCommand = Autopilot_P.Saturation_LowerSat;
  } else {
    /* Outport: '<Root>/Elevator Command' */
    Autopilot_Y.ElevatorCommand = rtb_DigitalClock;
  }

  /* End of Saturate: '<S1>/Saturation' */

  /* Gain: '<S1>/Multiply1' */
  cy = Autopilot_P.Multiply1_Gain * cp;

  /* DeadZone: '<S37>/DeadZone' */
  if (st > Autopilot_P.PIDController_UpperSaturationLi) {
    st -= Autopilot_P.PIDController_UpperSaturationLi;
  } else if (st >= Autopilot_P.PIDController_LowerSaturationLi) {
    st = 0.0;
  } else {
    st -= Autopilot_P.PIDController_LowerSaturationLi;
  }

  /* End of DeadZone: '<S37>/DeadZone' */

  /* Gain: '<S42>/Integral Gain' */
  sp *= Autopilot_P.PIDController_I;

  /* DeadZone: '<S91>/DeadZone' */
  if (rtb_DeadZone > Autopilot_P.PIDController1_UpperSaturationL) {
    rtb_DeadZone -= Autopilot_P.PIDController1_UpperSaturationL;
  } else if (rtb_DeadZone >= Autopilot_P.PIDController1_LowerSaturationL) {
    rtb_DeadZone = 0.0;
  } else {
    rtb_DeadZone -= Autopilot_P.PIDController1_LowerSaturationL;
  }

  /* End of DeadZone: '<S91>/DeadZone' */

  /* Gain: '<S96>/Integral Gain' */
  rtb_IntegralGain *= Autopilot_P.PIDController1_I;

  /* Switch: '<S89>/Switch1' incorporates:
   *  Constant: '<S89>/Clamping_zero'
   *  Constant: '<S89>/Constant'
   *  Constant: '<S89>/Constant2'
   *  RelationalOperator: '<S89>/fix for DT propagation issue'
   */
  if (rtb_DeadZone > Autopilot_P.Clamping_zero_Value_d) {
    tmp_1 = Autopilot_P.Constant_Value_b;
  } else {
    tmp_1 = Autopilot_P.Constant2_Value_n;
  }

  /* Switch: '<S89>/Switch2' incorporates:
   *  Constant: '<S89>/Clamping_zero'
   *  Constant: '<S89>/Constant3'
   *  Constant: '<S89>/Constant4'
   *  RelationalOperator: '<S89>/fix for DT propagation issue1'
   */
  if (rtb_IntegralGain > Autopilot_P.Clamping_zero_Value_d) {
    tmp_2 = Autopilot_P.Constant3_Value_a;
  } else {
    tmp_2 = Autopilot_P.Constant4_Value_o;
  }

  /* Switch: '<S89>/Switch' incorporates:
   *  Constant: '<S89>/Clamping_zero'
   *  Constant: '<S89>/Constant1'
   *  Logic: '<S89>/AND3'
   *  RelationalOperator: '<S89>/Equal1'
   *  RelationalOperator: '<S89>/Relational Operator'
   *  Switch: '<S89>/Switch1'
   *  Switch: '<S89>/Switch2'
   */
  if ((Autopilot_P.Clamping_zero_Value_d != rtb_DeadZone) && (tmp_1 == tmp_2)) {
    rtb_IntegralGain = Autopilot_P.Constant1_Value_h;
  }

  /* Update for DiscreteIntegrator: '<S99>/Integrator' incorporates:
   *  Switch: '<S89>/Switch'
   */
  Autopilot_DW.Integrator_DSTATE += Autopilot_P.Integrator_gainval *
    rtb_IntegralGain;

  /* Update for DiscreteIntegrator: '<S94>/Filter' */
  Autopilot_DW.Filter_DSTATE += Autopilot_P.Filter_gainval *
    rtb_FilterCoefficient;

  /* Switch: '<S35>/Switch1' incorporates:
   *  Constant: '<S35>/Clamping_zero'
   *  Constant: '<S35>/Constant'
   *  Constant: '<S35>/Constant2'
   *  RelationalOperator: '<S35>/fix for DT propagation issue'
   */
  if (st > Autopilot_P.Clamping_zero_Value) {
    tmp_1 = Autopilot_P.Constant_Value_f;
  } else {
    tmp_1 = Autopilot_P.Constant2_Value;
  }

  /* Switch: '<S35>/Switch2' incorporates:
   *  Constant: '<S35>/Clamping_zero'
   *  Constant: '<S35>/Constant3'
   *  Constant: '<S35>/Constant4'
   *  RelationalOperator: '<S35>/fix for DT propagation issue1'
   */
  if (sp > Autopilot_P.Clamping_zero_Value) {
    tmp_2 = Autopilot_P.Constant3_Value;
  } else {
    tmp_2 = Autopilot_P.Constant4_Value;
  }

  /* Switch: '<S35>/Switch' incorporates:
   *  Constant: '<S35>/Clamping_zero'
   *  Constant: '<S35>/Constant1'
   *  Logic: '<S35>/AND3'
   *  RelationalOperator: '<S35>/Equal1'
   *  RelationalOperator: '<S35>/Relational Operator'
   *  Switch: '<S35>/Switch1'
   *  Switch: '<S35>/Switch2'
   */
  if ((Autopilot_P.Clamping_zero_Value != st) && (tmp_1 == tmp_2)) {
    sp = Autopilot_P.Constant1_Value;
  }

  /* Update for DiscreteIntegrator: '<S45>/Integrator' incorporates:
   *  Switch: '<S35>/Switch'
   */
  Autopilot_DW.Integrator_DSTATE_k += Autopilot_P.Integrator_gainval_j * sp;

  /* Update for DiscreteIntegrator: '<S40>/Filter' */
  Autopilot_DW.Filter_DSTATE_e += Autopilot_P.Filter_gainval_d * ct;

  /* Matfile logging */
  rt_UpdateTXYLogVars(Autopilot_M->rtwLogInfo, (&Autopilot_M->Timing.taskTime0));

  /* signal main to stop simulation */
  {                                    /* Sample time: [0.008333s, 0.0s] */
    if ((rtmGetTFinal(Autopilot_M)!=-1) &&
        !((rtmGetTFinal(Autopilot_M)-Autopilot_M->Timing.taskTime0) >
          Autopilot_M->Timing.taskTime0 * (DBL_EPSILON))) {
      rtmSetErrorStatus(Autopilot_M, "Simulation finished");
    }
  }

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

  Autopilot_M->Timing.taskTime0 = Autopilot_M->Timing.clockTick0 *
    Autopilot_M->Timing.stepSize0 + Autopilot_M->Timing.clockTickH0 *
    Autopilot_M->Timing.stepSize0 * 4294967296.0;
}

/* Model initialize function */
void Autopilot_initialize(void)
{
  /* Registration code */

  /* initialize real-time model */
  (void) memset((void *)Autopilot_M, 0,
                sizeof(RT_MODEL_Autopilot_T));
  rtmSetTFinal(Autopilot_M, 599.992666);
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

  {
    int32_T i;
    static const real_T v[12] = { 25.0, 25.0, 25.0, 4.0, 4.0, 4.0,
      0.0076154354946677142, 0.0076154354946677142, 0.019495514866349348, 0.0001,
      0.0001, 0.0001 };

    /* InitializeConditions for DiscreteIntegrator: '<S99>/Integrator' */
    Autopilot_DW.Integrator_DSTATE = Autopilot_P.PIDController1_InitialConditi_a;

    /* InitializeConditions for DiscreteIntegrator: '<S94>/Filter' */
    Autopilot_DW.Filter_DSTATE = Autopilot_P.PIDController1_InitialCondition;

    /* InitializeConditions for DiscreteIntegrator: '<S45>/Integrator' */
    Autopilot_DW.Integrator_DSTATE_k =
      Autopilot_P.PIDController_InitialConditio_n;

    /* InitializeConditions for DiscreteIntegrator: '<S40>/Filter' */
    Autopilot_DW.Filter_DSTATE_e = Autopilot_P.PIDController_InitialConditionF;

    /* SystemInitialize for MATLAB Function: '<S1>/Navigation_EKF' */
    Autopilot_DW.initialized_not_empty = false;
    memset(&Autopilot_DW.P[0], 0, 144U * sizeof(real_T));
    for (i = 0; i < 12; i++) {
      Autopilot_DW.P[i + 12 * i] = v[i];
    }

    /* End of SystemInitialize for MATLAB Function: '<S1>/Navigation_EKF' */

    /* SystemInitialize for MATLAB Function: '<S1>/L1_Guidance' */
    Autopilot_DW.waypoint_index = 1;
  }
}

/* Model terminate function */
void Autopilot_terminate(void)
{
  /* (no terminate code required) */
}
