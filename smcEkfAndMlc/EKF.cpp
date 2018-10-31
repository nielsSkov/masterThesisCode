//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: EKF.cpp
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 22-Mar-2018 10:04:22
//

// Include Files
#include <cmath>
#include <string.h>
#include "EKF.h"
#include "inv.h"
#include "eye.h"
#include "EKF_data.h"

// Variable Definitions
static double x_est_correction_EKF[4];
static double P_correction_EKF[16];
static bool P_correction_EKF_not_empty;

// Function Definitions

//
// function [x_est_correction] = EKF(y_meas,Ia,t_s,x_init)
// Arguments    : const double y_meas[2]
//                double Ia
//                double t_s
//                const double x_init[4]
//                double x_est_correction[4]
// Return Type  : void
//
void EKF(const double y_meas[2], double Ia, double t_s, const double x_init[4],
         double x_est_correction[4])
{
  int i;
  double t;
  double x;
  double b_x;
  double c_x;
  double d_x;
  double e_x;
  double y;
  double f_x;
  double g_x;
  double f1[4];
  double b_y_meas[2];
  double dv2[2];
  double dv3[4];
  int i1;
  double dv4[2];
  int i2;
  double x_est_pred[4];
  double F[16];
  static const signed char iv0[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0,
    0, 1 };

  double F_B_c;
  double F_B_p;
  double dF_B_theta;
  double a;
  double b_a;
  double N2;
  double c_a;
  double h_x;
  double i_x;
  double j_x;
  double d_a;
  double k_x;
  double l_x;
  double e_a;
  double r;
  double f_a[8];
  double dv5[8];
  double dv6[8];
  static const signed char iv1[8] = { 0, 0, 0, 0, 1, 0, 0, 1 };

  signed char I[16];
  double b_F[16];
  double P_pred[16];
  static const double Q[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
    50, 0.0, 0.0, 0.0, 0.0, 50 };

  double b_y[4];
  static const signed char g_a[8] = { 1, 0, 0, 1, 0, 0, 0, 0 };

  static const signed char R[4] = { 5, 0, 0, 10 };

  static const signed char b[8] = { 1, 0, 0, 0, 0, 1, 0, 0 };

  double K[8];
  double b_I[16];
  double c_I[16];

  // 'EKF:5' if isempty(P_correction_EKF)
  if (!P_correction_EKF_not_empty) {
    // 'EKF:6' P_correction_EKF = eye(4);
    eye(P_correction_EKF);
    P_correction_EKF_not_empty = true;

    // 'EKF:7' x_est_correction_EKF = x_init;
    for (i = 0; i < 4; i++) {
      x_est_correction_EKF[i] = x_init[i];
    }
  }
  
	float B_v_pos=1.9368;
	float B_v_neg=1.4219;
  if(((x_est_correction_EKF[2])>0)||(((x_est_correction_EKF[2])==0)&&(Ia>0))){
        B_viscous_kal[0] = B_v_pos;
      }
    else if(((x_est_correction_EKF[2])<0)||(((x_est_correction_EKF[2])==0)&&(Ia<0))){
        B_viscous_kal[0] = B_v_neg;
      }
    else{
        B_viscous_kal[0] = 0;
      }
  
  // 'EKF:10' k_tanh = 250;
  // 'EKF:11' m_b = m_p;
  // 'EKF:12' R = [50 0;0 50];
  // 'EKF:13' Q = [1 0 0 0;0 1 0 0;0 0 0.5 0;0 0 0 2];
  // 'EKF:15' Hk = [1 0 0 0;
  // 'EKF:16'           0 1 0 0];
  //  System equation
  // 'EKF:19' f_1 = -[inv(m_c+m_b*sin(x_est_correction_EKF(2))^2) cos(x_est_correction_EKF(2))/(l*(m_c+m_b*sin(x_est_correction_EKF(2))^2)); 
  // 'EKF:20'             cos(x_est_correction_EKF(2))/(l*(m_c+m_b*sin(x_est_correction_EKF(2))^2)) (m_c+m_b)/(m_b*l^2*(m_c+m_b*sin(x_est_correction_EKF(2))^2))]; 
  t = std::sin(x_est_correction_EKF[1]);
  x = std::sin(x_est_correction_EKF[1]);
  b_x = std::sin(x_est_correction_EKF[1]);
  c_x = std::sin(x_est_correction_EKF[1]);

  // +B_coulumb(1,1)*tanh(k_tanh*x_est_correction_EKF(3))
  // 'EKF:21' f_B = [B_viscous_kal(1,1)*x_est_correction_EKF(3);%+B_coulumb(1,1)*tanh(k_tanh*x_est_correction_EKF(3)) 
  // 'EKF:22'            B_viscous_kal(2,2)*x_est_correction_EKF(4)+B_coulumb(2,2)*tanh(k_tanh*x_est_correction_EKF(4))]; 
  // 'EKF:23' f_2 = -[(m_b*l*sin(x_est_correction_EKF(2))*x_est_correction_EKF(4)-g*m_b*sin(x_est_correction_EKF(2))*cos(x_est_correction_EKF(2)))/(m_c+m_b*sin(x_est_correction_EKF(2))^2); 
  // 'EKF:24'             (m_b*l*sin(x_est_correction_EKF(2))*cos(x_est_correction_EKF(2))-g*sin(x_est_correction_EKF(2))*(m_c+m_b))/(l*(m_c+m_b*sin(x_est_correction_EKF(2))^2))]; 
  d_x = std::sin(x_est_correction_EKF[1]);
  e_x = std::sin(x_est_correction_EKF[1]);

  // 'EKF:25' g_1 = K_t/r_pulley*[0;0;inv(m_c+m_b*sin(x_est_correction_EKF(2))^2);cos(x_est_correction_EKF(2))/(l*(m_c+m_b*sin(x_est_correction_EKF(2))^2))]; 
  y = K_t / r_pulley;
  f_x = std::sin(x_est_correction_EKF[1]);
  g_x = std::sin(x_est_correction_EKF[1]);

  //  Time update (prediction step)
  // 'EKF:28' f = [x_est_correction_EKF(3);x_est_correction_EKF(4);f_1*f_B+f_2]+g_1*Ia; 
  // 'EKF:29' x_est_pred = x_est_correction_EKF+t_s*f;
  f1[0] = -inv(m_c + m_p * (t * t));
  f1[2] = -(std::cos(x_est_correction_EKF[1]) / (l * (m_c + m_p * (x * x))));
  f1[1] = -(std::cos(x_est_correction_EKF[1]) / (l * (m_c + m_p * (b_x * b_x))));
  f1[3] = -((m_c + m_p) / (m_p * (l * l) * (m_c + m_p * (c_x * c_x))));
  b_y_meas[0] = B_viscous_kal[0] * x_est_correction_EKF[2];
  b_y_meas[1] = B_viscous_kal[3] * x_est_correction_EKF[3] + B_coulumb[3] * std::
    tanh(250.0 * x_est_correction_EKF[3]);
  dv2[0] = -((m_p * l * std::sin(x_est_correction_EKF[1]) *
              x_est_correction_EKF[3] - g * m_p * std::sin(x_est_correction_EKF
    [1]) * std::cos(x_est_correction_EKF[1])) / (m_c + m_p * (d_x * d_x)));
  dv2[1] = -((m_p * l * std::sin(x_est_correction_EKF[1]) * std::cos
              (x_est_correction_EKF[1]) - g * std::sin(x_est_correction_EKF[1]) *
              (m_c + m_p)) / (l * (m_c + m_p * (e_x * e_x))));
  dv3[0] = x_est_correction_EKF[2];
  dv3[1] = x_est_correction_EKF[3];
  for (i1 = 0; i1 < 2; i1++) {
    dv4[i1] = 0.0;
    for (i2 = 0; i2 < 2; i2++) {
      dv4[i1] += f1[i1 + (i2 << 1)] * b_y_meas[i2];
    }

    dv3[i1 + 2] = dv4[i1] + dv2[i1];
  }

  f1[0] = 0.0;
  f1[1] = 0.0;
  f1[2] = y * inv(m_c + m_p * (f_x * f_x)) * Ia;
  f1[3] = y * (std::cos(x_est_correction_EKF[1]) / (l * (m_c + m_p * (g_x * g_x))))
    * Ia;
  for (i1 = 0; i1 < 4; i1++) {
    x_est_pred[i1] = x_est_correction_EKF[i1] + t_s * (dv3[i1] + f1[i1]);
  }

  // ----- Jacobian start ------
  // 'EKF:32' F = eye(4);
  for (i1 = 0; i1 < 16; i1++) {
    F[i1] = iv0[i1];
  }

  // 'EKF:33' F(1:2,1:4) = [0 0 1 0;0 0 0 1];
  // 'EKF:34' F_B_c = B_viscous_kal(1,1)*x_est_pred(3);
  F_B_c = B_viscous_kal[0] * x_est_pred[2];

  // +B_coulumb(1,1)*tanh(k_tanh*x_est_pred(3))
  // 'EKF:35' F_B_p = B_viscous_kal(2,2)*x_est_pred(4)+B_coulumb(2,2)*tanh(k_tanh*x_est_pred(4)); 
  F_B_p = B_viscous_kal[3] * x_est_pred[3] + B_coulumb[3] * std::tanh(250.0 *
    x_est_pred[3]);

  // 'EKF:37' dF_B_x = B_viscous_kal(1,1);
  // +B_coulumb(1,1)*k_tanh*sech(x_est_pred(3))^2
  // 'EKF:38' dF_B_theta = B_viscous_kal(2,2)+B_coulumb(2,2)*k_tanh*sech(x_est_pred(4))^2; 
  t = 1.0 / std::cosh(x_est_pred[3]);
  dF_B_theta = B_viscous_kal[3] + B_coulumb[3] * 250.0 * (t * t);

  // 'EKF:40' N1 = (-m_b*sin(2*x_est_pred(2)))/((m_c+m_b*sin(x_est_pred(2))^2)^2); 
  t = std::sin(x_est_pred[1]);
  a = m_c + m_p * (t * t);

  // 'EKF:41' N2 = (-sin(x_est_pred(2))*(l*(m_c+m_b*sin(x_est_pred(2))^2))-l*m_b*sin(2*x_est_pred(2))*cos(x_est_pred(2)))/((l*(m_c+m_b*sin(x_est_pred(2))^2))^2); 
  t = std::sin(x_est_pred[1]);
  x = std::sin(x_est_pred[1]);
  b_a = l * (m_c + m_p * (x * x));
  N2 = (-std::sin(x_est_pred[1]) * (l * (m_c + m_p * (t * t))) - l * m_p * std::
        sin(2.0 * x_est_pred[1]) * std::cos(x_est_pred[1])) / (b_a * b_a);

  // 'EKF:42' N3 = (-(m_c+m_b)*m_b^2*l^2*sin(2*x_est_pred(2)))/((m_b*l^2*(m_c+m_b*sin(x_est_pred(2))^2))^2); 
  t = std::sin(x_est_pred[1]);
  b_a = m_p * (l * l) * (m_c + m_p * (t * t));

  // 'EKF:44' f1 = [inv(m_c+m_b*sin(x_est_pred(2))^2) cos(x_est_pred(2))/(l*(m_c+m_b*sin(x_est_pred(2))^2)); 
  // 'EKF:45'           cos(x_est_pred(2))/(l*(m_c+m_b*sin(x_est_pred(2))^2)) (m_c+m_b)/(m_b*l^2*(m_c+m_b*sin(x_est_pred(2))^2))]; 
  t = std::sin(x_est_pred[1]);
  x = std::sin(x_est_pred[1]);
  b_x = std::sin(x_est_pred[1]);
  c_x = std::sin(x_est_pred[1]);

  // 'EKF:47' df1 = -[0,N1*F_B_c+N2*F_B_p,f1(1,1)*dF_B_x,f1(1,2)*dF_B_theta;
  // 'EKF:48'            0,N2*F_B_c+N3*F_B_p,f1(2,1)*dF_B_x,f1(2,2)*dF_B_theta]; 
  // 'EKF:50' df2_theta = [((m_b*l*cos(x_est_pred(2))*x_est_pred(4)^2-g*m_b*(cos(x_est_pred(2))^2-sin(x_est_pred(2))^2))*(m_c+m_b*sin(x_est_pred(2))^2)-(m_b*l*sin(x_est_pred(2))*x_est_pred(4)^2-g*m_b*sin(x_est_pred(2))*cos(x_est_pred(2)))*m_b*sin(2*x_est_pred(2)))/((m_c+m_b*sin(x_est_pred(2))^2)^2); 
  // 'EKF:51'                  ((m_b*l*x_est_pred(4)^2*(cos(x_est_pred(2))^2-sin(x_est_pred(2))^2)-g*cos(x_est_pred(2))*(m_c+m_b))*(l*(m_c+m_b*sin(x_est_pred(2))^2))-(m_b*l*x_est_pred(4)^2*sin(x_est_pred(2))*cos(x_est_pred(2))-g*sin(x_est_pred(2))*(m_c+m_b))*(l*m_b*sin(2*x_est_pred(2))))/((l*(m_c+m_b*sin(x_est_pred(2))^2))^2)]; 
  d_x = std::cos(x_est_pred[1]);
  e_x = std::sin(x_est_pred[1]);
  f_x = std::sin(x_est_pred[1]);
  g_x = std::sin(x_est_pred[1]);
  c_a = m_c + m_p * (g_x * g_x);
  g_x = std::cos(x_est_pred[1]);
  h_x = std::sin(x_est_pred[1]);
  i_x = std::sin(x_est_pred[1]);
  j_x = std::sin(x_est_pred[1]);
  d_a = l * (m_c + m_p * (j_x * j_x));

  // 'EKF:53' df2_theta_dot = [(2*m_b*l*sin(x_est_pred(2))*x_est_pred(4))/(m_c+m_b*sin(x_est_pred(2))^2); 
  // 'EKF:54'                      (2*m_b*l*sin(x_est_pred(2))*cos(x_est_pred(2))*x_est_pred(4))/(l*(m_c+m_b*sin(x_est_pred(2))^2))]; 
  j_x = std::sin(x_est_pred[1]);
  k_x = std::sin(x_est_pred[1]);

  // 'EKF:55' df2 = -[0 df2_theta(1) 0 df2_theta_dot(1);
  // 'EKF:56'            0 df2_theta(2) 0 df2_theta_dot(2)];
  // 'EKF:58' dg_theta = K_t/r_pulley*[(-m_b*sin(2*x_est_pred(2)))/((m_c+m_b*sin(x_est_pred(2))^2)^2); 
  // 'EKF:59'                              (-sin(x_est_pred(2))*(l*(m_c+m_b*sin(x_est_pred(2))^2))-cos(x_est_pred(2))*l*m_b*sin(2*x_est_pred(2)))/((l*(m_c+m_b*sin(x_est_pred(2))^2))^2)]; 
  y = K_t / r_pulley;
  l_x = std::sin(x_est_pred[1]);
  e_a = m_c + m_p * (l_x * l_x);
  l_x = std::sin(x_est_pred[1]);
  r = std::sin(x_est_pred[1]);
  r = l * (m_c + m_p * (r * r));

  // 'EKF:60' dg1 = [0 dg_theta(1) 0 0;
  // 'EKF:61'            0 dg_theta(2) 0 0]*Ia;
  // 'EKF:63' F(3:4,1:4) = df1+df2+dg1;
  f_a[0] = -0.0;
  f_a[2] = -(-m_p * std::sin(2.0 * x_est_pred[1]) / (a * a) * F_B_c + N2 * F_B_p);
  f_a[4] = -(inv(m_c + m_p * (t * t)) * B_viscous_kal[0]);
  f_a[6] = -(std::cos(x_est_pred[1]) / (l * (m_c + m_p * (x * x))) * dF_B_theta);
  f_a[1] = -0.0;
  f_a[3] = -(N2 * F_B_c + -(m_c + m_p) * (m_p * m_p) * (l * l) * std::sin(2.0 *
              x_est_pred[1]) / (b_a * b_a) * F_B_p);
  f_a[5] = -(std::cos(x_est_pred[1]) / (l * (m_c + m_p * (b_x * b_x))) *
             B_viscous_kal[0]);
  f_a[7] = -((m_c + m_p) / (m_p * (l * l) * (m_c + m_p * (c_x * c_x))) *
             dF_B_theta);
  dv5[0] = -0.0;
  dv5[2] = -(((m_p * l * std::cos(x_est_pred[1]) * (x_est_pred[3] * x_est_pred[3])
               - g * m_p * (d_x * d_x - e_x * e_x)) * (m_c + m_p * (f_x * f_x))
              - (m_p * l * std::sin(x_est_pred[1]) * (x_est_pred[3] *
    x_est_pred[3]) - g * m_p * std::sin(x_est_pred[1]) * std::cos(x_est_pred[1]))
              * m_p * std::sin(2.0 * x_est_pred[1])) / (c_a * c_a));
  dv5[4] = -0.0;
  dv5[6] = -(2.0 * m_p * l * std::sin(x_est_pred[1]) * x_est_pred[3] / (m_c +
              m_p * (j_x * j_x)));
  dv5[1] = -0.0;
  dv5[3] = -(((m_p * l * (x_est_pred[3] * x_est_pred[3]) * (g_x * g_x - h_x *
    h_x) - g * std::cos(x_est_pred[1]) * (m_c + m_p)) * (l * (m_c + m_p * (i_x *
    i_x))) - (m_p * l * (x_est_pred[3] * x_est_pred[3]) * std::sin(x_est_pred[1])
              * std::cos(x_est_pred[1]) - g * std::sin(x_est_pred[1]) * (m_c +
    m_p)) * (l * m_p * std::sin(2.0 * x_est_pred[1]))) / (d_a * d_a));
  dv5[5] = -0.0;
  dv5[7] = -(2.0 * m_p * l * std::sin(x_est_pred[1]) * std::cos(x_est_pred[1]) *
             x_est_pred[3] / (l * (m_c + m_p * (k_x * k_x))));
  dv6[0] = 0.0;
  dv6[2] = y * (-m_p * std::sin(2.0 * x_est_pred[1]) / (e_a * e_a)) * Ia;
  dv6[4] = 0.0;
  dv6[6] = 0.0;
  dv6[1] = 0.0;
  dv6[3] = y * ((-std::sin(x_est_pred[1]) * (l * (m_c + m_p * (l_x * l_x))) -
                 std::cos(x_est_pred[1]) * l * m_p * std::sin(2.0 * x_est_pred[1]))
                / (r * r)) * Ia;
  dv6[5] = 0.0;
  dv6[7] = 0.0;
  for (i1 = 0; i1 < 4; i1++) {
    for (i2 = 0; i2 < 2; i2++) {
      F[i2 + (i1 << 2)] = iv1[i2 + (i1 << 1)];
      F[(i2 + (i1 << 2)) + 2] = (f_a[i2 + (i1 << 1)] + dv5[i2 + (i1 << 1)]) +
        dv6[i2 + (i1 << 1)];
    }
  }

  // ------- Jacobian end -------
  // 'EKF:66' F = eye(4)+t_s*F;
  for (i1 = 0; i1 < 16; i1++) {
    I[i1] = 0;
  }

  for (i = 0; i < 4; i++) {
    I[i + (i << 2)] = 1;
  }

  for (i1 = 0; i1 < 16; i1++) {
    F[i1] = (double)I[i1] + t_s * F[i1];
  }

  // 'EKF:67' P_pred = F*P_correction_EKF*F'+Q;
  for (i1 = 0; i1 < 4; i1++) {
    for (i2 = 0; i2 < 4; i2++) {
      b_F[i1 + (i2 << 2)] = 0.0;
      for (i = 0; i < 4; i++) {
        b_F[i1 + (i2 << 2)] += F[i1 + (i << 2)] * P_correction_EKF[i + (i2 << 2)];
      }
    }

    for (i2 = 0; i2 < 4; i2++) {
      r = 0.0;
      for (i = 0; i < 4; i++) {
        r += b_F[i1 + (i << 2)] * F[i2 + (i << 2)];
      }

      P_pred[i1 + (i2 << 2)] = r + Q[i1 + (i2 << 2)];
    }
  }

  //  Measurement update (correction step)
  // 'EKF:70' y_err = y_meas-Hk*x_est_pred;
  // 'EKF:71' K = P_pred*Hk'*inv(Hk*P_pred*Hk'+R);
  for (i1 = 0; i1 < 2; i1++) {
    for (i2 = 0; i2 < 4; i2++) {
      f_a[i1 + (i2 << 1)] = 0.0;
      for (i = 0; i < 4; i++) {
        f_a[i1 + (i2 << 1)] += (double)g_a[i1 + (i << 1)] * P_pred[i + (i2 << 2)];
      }
    }

    for (i2 = 0; i2 < 2; i2++) {
      r = 0.0;
      for (i = 0; i < 4; i++) {
        r += f_a[i1 + (i << 1)] * (double)b[i + (i2 << 2)];
      }

      f1[i1 + (i2 << 1)] = r + (double)R[i1 + (i2 << 1)];
    }
  }

  if (std::abs(f1[1]) > std::abs(f1[0])) {
    r = f1[0] / f1[1];
    t = 1.0 / (r * f1[3] - f1[2]);
    b_y[0] = f1[3] / f1[1] * t;
    b_y[1] = -t;
    b_y[2] = -f1[2] / f1[1] * t;
    b_y[3] = r * t;
  } else {
    r = f1[1] / f1[0];
    t = 1.0 / (f1[3] - r * f1[2]);
    b_y[0] = f1[3] / f1[0] * t;
    b_y[1] = -r * t;
    b_y[2] = -f1[2] / f1[0] * t;
    b_y[3] = t;
  }

  for (i1 = 0; i1 < 4; i1++) {
    for (i2 = 0; i2 < 2; i2++) {
      f_a[i1 + (i2 << 2)] = 0.0;
      for (i = 0; i < 4; i++) {
        f_a[i1 + (i2 << 2)] += P_pred[i1 + (i << 2)] * (double)b[i + (i2 << 2)];
      }
    }

    for (i2 = 0; i2 < 2; i2++) {
      K[i1 + (i2 << 2)] = 0.0;
      for (i = 0; i < 2; i++) {
        K[i1 + (i2 << 2)] += f_a[i1 + (i << 2)] * b_y[i + (i2 << 1)];
      }
    }
  }

  // 'EKF:72' x_est_correction = x_est_pred+K*y_err;
  for (i1 = 0; i1 < 2; i1++) {
    r = 0.0;
    for (i2 = 0; i2 < 4; i2++) {
      r += (double)g_a[i1 + (i2 << 1)] * x_est_pred[i2];
    }

    b_y_meas[i1] = y_meas[i1] - r;
  }

  // 'EKF:73' x_est_correction_EKF = x_est_correction;
  for (i = 0; i < 4; i++) {
    r = 0.0;
    for (i1 = 0; i1 < 2; i1++) {
      r += K[i + (i1 << 2)] * b_y_meas[i1];
    }

    r += x_est_pred[i];
    x_est_correction_EKF[i] = r;
    x_est_correction[i] = r;
  }

  // 'EKF:74' P_correction_EKF = (eye(4)-K*Hk)*P_pred*(eye(4)-K*Hk)'+K*R*K';
  for (i1 = 0; i1 < 16; i1++) {
    I[i1] = 0;
  }

  for (i = 0; i < 4; i++) {
    I[i + (i << 2)] = 1;
  }

  memset(&F[0], 0, sizeof(double) << 4);
  for (i = 0; i < 4; i++) {
    F[i + (i << 2)] = 1.0;
    for (i1 = 0; i1 < 4; i1++) {
      r = 0.0;
      for (i2 = 0; i2 < 2; i2++) {
        r += K[i + (i2 << 2)] * (double)g_a[i2 + (i1 << 1)];
      }

      c_I[i + (i1 << 2)] = (double)I[i + (i1 << 2)] - r;
    }

    for (i1 = 0; i1 < 4; i1++) {
      b_I[i + (i1 << 2)] = 0.0;
      for (i2 = 0; i2 < 4; i2++) {
        b_I[i + (i1 << 2)] += c_I[i + (i2 << 2)] * P_pred[i2 + (i1 << 2)];
      }
    }
  }

  for (i1 = 0; i1 < 4; i1++) {
    for (i2 = 0; i2 < 4; i2++) {
      r = 0.0;
      for (i = 0; i < 2; i++) {
        r += K[i2 + (i << 2)] * (double)g_a[i + (i1 << 1)];
      }

      b_F[i1 + (i2 << 2)] = F[i2 + (i1 << 2)] - r;
    }

    for (i2 = 0; i2 < 2; i2++) {
      f_a[i1 + (i2 << 2)] = 0.0;
      for (i = 0; i < 2; i++) {
        f_a[i1 + (i2 << 2)] += K[i1 + (i << 2)] * (double)R[i + (i2 << 1)];
      }
    }
  }

  for (i1 = 0; i1 < 4; i1++) {
    for (i2 = 0; i2 < 4; i2++) {
      c_I[i1 + (i2 << 2)] = 0.0;
      for (i = 0; i < 4; i++) {
        c_I[i1 + (i2 << 2)] += b_I[i1 + (i << 2)] * b_F[i + (i2 << 2)];
      }

      F[i1 + (i2 << 2)] = 0.0;
      for (i = 0; i < 2; i++) {
        F[i1 + (i2 << 2)] += f_a[i1 + (i << 2)] * K[i2 + (i << 2)];
      }
    }
  }

  for (i1 = 0; i1 < 4; i1++) {
    for (i2 = 0; i2 < 4; i2++) {
      P_correction_EKF[i2 + (i1 << 2)] = c_I[i2 + (i1 << 2)] + F[i2 + (i1 << 2)];
    }
  }
}

//
// Arguments    : void
// Return Type  : void
//
void P_correction_EKF_not_empty_init()
{
  P_correction_EKF_not_empty = false;
}

//
// File trailer for EKF.cpp
//
// [EOF]
//
