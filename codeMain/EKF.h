//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: EKF.h
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 22-Mar-2018 10:04:22
//
#ifndef EKF_H
#define EKF_H

// Include Files
#include <stddef.h>
#include <stdlib.h>
//#include "rtwtypes.h"
//#include "EKF_types.h"

// Function Declarations
extern void EKF(const double y_meas[2], double Ia, double t_s, const double
                x_init[4], double x_est_correction[4]);
extern void P_correction_EKF_not_empty_init();

#endif

//
// File trailer for EKF.h
//
// [EOF]
//
