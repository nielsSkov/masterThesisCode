//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: EKF_initialize.cpp
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 22-Mar-2018 10:04:22
//

// Include Files
#include "EKF.h"
#include "EKF_initialize.h"
#include "EKF_data.h"

// Named Constants
#define b_m_p                          (0.201)
#define b_m_c                          (5.273086)
#define b_l                            (0.32349999999999995)
#define b_g                            (9.81)
#define b_K_t                          (0.093400000000000011)
#define b_r_pulley                     (0.028)

// Function Definitions

//
// Arguments    : void
// Return Type  : void
//
void EKF_initialize()
{
  int i0;
  static const double dv0[4] = { 3.021216, 0.0, 0.0, 0.004 };

  static const double dv1[4] = { 1.9368, 0.0, 0.0, 0.0004 };

  for (i0 = 0; i0 < 4; i0++) {
    B_coulumb[i0] = dv0[i0];
    B_viscous_kal[i0] = dv1[i0];
  }

  r_pulley = b_r_pulley;
  K_t = b_K_t;
  g = b_g;
  l = b_l;
  m_c = b_m_c;
  m_p = b_m_p;
  P_correction_EKF_not_empty_init();
}

//
// File trailer for EKF_initialize.cpp
//
// [EOF]
//
