/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * run_mlc.cpp
 *
 * Code generation for function 'run_mlc'
 *
 */

/* Include files */
#include "run_mlc.h"
#include "run_mlc_data.h"

/* Function Definitions */
double run_mlc(const double state[4])
{
  double Ia;
  int i;
  double current_mlc_state[4];
  bool exitg1;
  int idx;

  /*  for MATLAB Coder to work */
  Ia = 1.1;

  /*  get best action in current state */
  for (i = 0; i < 4; i++) {
    current_mlc_state[i] = 0.0;
  }

  /*  convert x(1) to MLC_state(1) */
  i = 0;
  exitg1 = false;
  while ((!exitg1) && (i <= (int)num_buck[0] - 1)) {
    if (1.0 + (double)i == num_buck[0]) {
      current_mlc_state[0] = 1.0 + (double)i;
      i++;
    } else if (state[0] < x_buck_value[i]) {
      current_mlc_state[0] = 1.0 + (double)i;
      exitg1 = true;
    } else {
      i++;
    }
  }

  /*  convert x(2) to MLC_state(2) */
  i = 0;
  exitg1 = false;
  while ((!exitg1) && (i <= (int)num_buck[1] - 1)) {
    if (1.0 + (double)i == num_buck[1]) {
      current_mlc_state[1] = 1.0 + (double)i;
      i++;
    } else if (state[1] < theta_buck_value[i]) {
      current_mlc_state[1] = 1.0 + (double)i;
      exitg1 = true;
    } else {
      i++;
    }
  }

  /*  convert x(3) to MLC_state(3) */
  i = 0;
  exitg1 = false;
  while ((!exitg1) && (i <= (int)num_buck[2] - 1)) {
    if (1.0 + (double)i == num_buck[2]) {
      current_mlc_state[2] = 1.0 + (double)i;
      i++;
    } else if (state[2] < x_dot_buck_value[i]) {
      current_mlc_state[2] = 1.0 + (double)i;
      exitg1 = true;
    } else {
      i++;
    }
  }

  /*  convert x(4) to MLC_state(4) */
  i = 0;
  exitg1 = false;
  while ((!exitg1) && (i <= (int)num_buck[3] - 1)) {
    if (1.0 + (double)i == num_buck[3]) {
      current_mlc_state[3] = 1.0 + (double)i;
      i++;
    } else if (state[3] < theta_dot_buck_value[i]) {
      current_mlc_state[3] = 1.0 + (double)i;
      exitg1 = true;
    } else {
      i++;
    }
  }

  /* value_action_table(mlc_state(1),mlc_state(2),mlc_state(3),mlc_state(4),:) */
  idx = 1;
  if (value_action_table[((((int)current_mlc_state[0] + (((int)
           current_mlc_state[1] - 1) << 2)) + 40 * ((int)current_mlc_state[2] -
         1)) + 160 * ((int)current_mlc_state[3] - 1)) - 1] < value_action_table
      [((((int)current_mlc_state[0] + (((int)current_mlc_state[1] - 1) << 2)) +
         40 * ((int)current_mlc_state[2] - 1)) + 160 * ((int)current_mlc_state[3]
        - 1)) + 639]) {
    idx = 2;
  }

  /*  convert MLC action to actuation */
  for (i = 0; i < (int)num_buck[4]; i++) {
    if (idx == 1.0 + (double)i) {
      Ia = current_buck_value[i];
    }
  }

  return Ia;
}

/* End of code generation (run_mlc.cpp) */
