////////////////////////////////////////////////////////////////////////////
///////BEFORE UPLOAD////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
//
// (1) Remember to add libraries containing the below .h files:
//       Sketch - Import Library - Add Library
//
// (2) Remember
//     Tools - Board - Teensy 3.6 (Programming Port)
//     Tools - Port - COM# (Teensy 3.6 (Programming Port))
//     check that  "Serial.begin(115200)" below agree with baud on terminal
//
////////////////////////////////////////////////////////////////////////////

#include <Joint_teensy.h> // main file

// EKF libraries
#include "EKF.h"
#include "EKF_initialize.h"

// Definitions
#define SAMPLINGTIME 0.00667 // s
#define ENABLEPENDUL 11      // 50
#define ENABLESLED   12      // 48

///////////////////////////////////////////////////////////
///////COLOUMB FRICTION LOOKUP/////////////////////////////
///////////////////////////////////////////////////////////

//cart position lookup
float position[] =
  { 0.05, 0.06, 0.07, 0.08, 0.09, 0.10, 0.11, 0.12, 0.13, 0.14,
    0.15, 0.16, 0.17, 0.18, 0.19, 0.20, 0.21, 0.22, 0.23, 0.24,
    0.25, 0.26, 0.27, 0.28, 0.29, 0.30, 0.31, 0.32, 0.33, 0.34,
    0.35, 0.36, 0.37, 0.38, 0.39, 0.40, 0.41, 0.42, 0.43, 0.44,
    0.45, 0.46, 0.47, 0.48, 0.49, 0.50, 0.51, 0.52, 0.53, 0.54,
    0.55, 0.56, 0.57, 0.58, 0.59, 0.60, 0.61, 0.62, 0.63, 0.64,
    0.65, 0.66, 0.67, 0.68, 0.69, 0.70, 0.71, 0.72             };

//coloumb friction coefficient 
//at each position for positive velocity
//
//first attempt
//float coloumbP[] =
//  { 2.0599, 2.0059, 2.1820, 2.2164, 2.2342, 2.2546, 2.2178, 2.2195,
//    2.4184, 2.4506, 2.7560, 2.8778, 2.8485, 2.8498, 3.0238, 3.2066,
//    3.1093, 3.1399, 2.8875, 2.7890, 2.6054, 1.9920, 1.9811, 1.9713,
//    1.9721, 1.9877, 2.1605, 2.2211, 2.2249, 2.1049, 2.3816, 2.4523,
//    2.6783, 2.4560, 2.9049, 2.9076, 3.1950, 3.2866, 3.1905, 3.2095,
//    3.2441, 3.1898, 2.6971, 2.8141, 2.4427, 2.2702, 2.1833, 2.2546,
//    2.2863, 2.6159, 2.5990, 2.6953, 2.8589, 3.3110, 3.2057, 3.5125,
//    3.2856, 3.3000, 2.9808, 2.9562, 3.0018, 2.7540, 2.7749, 2.7897,
//    3.0878, 2.8514, 2.9757, 3.0784                                 };
//second attempt
//float coloumbP[] =
//  { 3.1844, 3.1915, 3.2685, 3.3231, 3.3499, 3.3759, 3.3758, 3.3837, 
//    3.4614, 3.5903, 3.7829, 3.9178, 3.9412, 3.9615, 4.0873, 4.2049, 
//    4.1929, 4.1282, 4.0035, 3.8601, 3.6049, 3.2372, 3.0708, 3.0673, 
//    3.0813, 3.1243, 3.2161, 3.2854, 3.2797, 3.2943, 3.4298, 3.5779, 
//    3.6637, 3.7209, 3.8860, 4.0593, 4.2394, 4.3231, 4.2648, 4.2116, 
//    4.2039, 4.0877, 3.8838, 3.7583, 3.5531, 3.3929, 3.3598, 3.3695, 
//    3.4444, 3.5858, 3.6693, 3.7477, 3.9482, 4.2188, 4.3765, 4.4690, 
//    4.4364, 4.2977, 4.0892, 4.0089, 3.9927, 3.8960, 3.8626, 3.9456, 
//    4.0244, 4.0068, 4.0124, 4.0512                                 };
//third attempt
float coloumbP[] =
  { 2.0599, 2.0662, 2.1381, 2.1714, 2.1948, 2.2462, 2.2906, 2.3641,
    2.4541, 2.5419, 2.6354, 2.7466, 2.8576, 2.9494, 3.0025, 3.0104,
    3.0008, 2.9567, 2.8189, 2.6462, 2.4794, 2.3163, 2.1865, 2.0951,
    2.0488, 2.0721, 2.0967, 2.1510, 2.2220, 2.3104, 2.3658, 2.4560,
    2.5617, 2.7070, 2.8373, 2.9427, 3.0279, 3.1276, 3.1694, 3.1450,
    3.0841, 2.9708, 2.8381, 2.6918, 2.5490, 2.4321, 2.4032, 2.3836,
    2.4188, 2.5065, 2.6558, 2.8007, 2.9618, 3.0691, 3.1635, 3.2093,
    3.2171, 3.1765, 3.1086, 3.0108, 2.9393, 2.9058, 2.8899, 2.8902,
    2.8996, 2.9615, 2.9700, 3.0784                                 };

//coloumb friction coefficient 
//at each position for negative velocity
//
//first attempt
//float coloumbN[] =
//  { 5.6263, 5.6912, 5.0210, 4.7789, 4.5369, 4.5226, 4.3667, 3.6637,
//    3.3560, 3.3377, 2.9555, 2.8196, 2.6894, 2.6508, 2.7522, 2.7978,
//    3.1890, 2.9839, 3.2756, 3.5558, 3.5269, 4.3655, 4.5228, 4.4821,
//    4.2200, 4.0933, 3.4397, 3.3830, 3.0173, 3.0530, 2.8399, 2.5662,
//    2.3793, 2.6272, 1.9711, 2.0432, 1.8917, 1.6675, 1.9137, 1.8585,
//    1.9262, 2.4228, 2.2988, 2.6986, 2.7103, 2.9014, 2.9634, 2.7753,
//    2.7122, 2.4782, 2.5381, 2.0629, 2.1222, 2.3033, 2.3858, 2.7245,
//    2.2818, 2.5743, 2.4700, 2.5944, 2.8320, 2.8006, 2.7077, 2.7269,
//    2.5346, 2.8446, 2.7306, 2.5191                                 };
//second attempt
//float coloumbN[] =
//  { 6.6085, 6.4970, 6.1523, 5.8337, 5.6311, 5.4858, 5.2078, 4.7583, 
//    4.4747, 4.3296, 4.1328, 3.9255, 3.7735, 3.7425, 3.8210, 3.9691, 
//    4.1320, 4.1849, 4.3378, 4.5378, 4.7859, 5.2481, 5.5282, 5.4814, 
//    5.3031, 5.0250, 4.6441, 4.3614, 4.1560, 4.0319, 3.8748, 3.6434, 
//    3.5108, 3.4368, 3.2144, 3.0541, 2.9193, 2.8297, 2.9040, 2.9620, 
//    3.1132, 3.3680, 3.5274, 3.7263, 3.8801, 3.9791, 3.9705, 3.8634, 
//    3.7359, 3.6128, 3.4879, 3.2808, 3.2330, 3.3469, 3.5011, 3.5922, 
//    3.5100, 3.5486, 3.6085, 3.7025, 3.8501, 3.8784, 3.8333, 3.7636, 
//    3.7292, 3.8079, 3.7853, 3.6189                                 };
//third attempt
float coloumbN[] =
  { 5.6263, 5.4987, 5.1417, 4.9257, 4.6487, 4.3322, 4.0786, 3.8225,
    3.5715, 3.3112, 3.0779, 2.9407, 2.8604, 2.8355, 2.8457, 2.9103,
    3.0284, 3.1653, 3.3837, 3.6218, 3.8165, 3.9902, 4.1001, 4.1013,
    4.0578, 3.8796, 3.6658, 3.4336, 3.1961, 2.9678, 2.8269, 2.6447,
    2.4937, 2.3306, 2.1720, 2.0713, 1.9899, 1.9119, 1.9537, 2.0036,
    2.1147, 2.2597, 2.4044, 2.5582, 2.6746, 2.7244, 2.7459, 2.7200,
    2.6334, 2.5232, 2.4298, 2.3751, 2.3673, 2.3480, 2.3565, 2.4114,
    2.4775, 2.5494, 2.6026, 2.6170, 2.6656, 2.6750, 2.7167, 2.7325,
    2.7000, 2.6817, 2.7051, 2.5191                                 };

///////////////////////////////////////////////////////////

// Create joint objects
Joint  sled( 1, SAMPLINGTIME );
Joint pend1( 2, SAMPLINGTIME );
Joint pend2( 3, SAMPLINGTIME );

// Global variables
float setOutSled  = 0;
float setOutPend1 = 0;
float velSled     = 0;
float velPend1    = 0;
float velPend2    = 0;
float posSled;
float posPend1;
float posPend2;
int   setOut      = 0;

float t_last      = 0;
float x1_last     = 0;
float x2_last     = 0;
float u_last      = 0;
float x1_old      = 0;
float catchAngle  = 0.02;
int   slideOn     = 0;

//ring-buffer and offset for FIR filter
int   offsetFIR      = 0;
int   offsetFIR2     = 0;
float ringBuffFIR[]  = { 0, 0, 0, 0, 0};
float ringBuffFIR2[] = { 0, 0, 0, 0, 0};

unsigned long current_time = 0;
unsigned long last_time    = 0;
unsigned long loop_time    = 0;
unsigned long time_now     = 0;

// For initializing the EKF
double x_est_correction[4];
double x_init[4];
float  setOutSledNoComp = 0;
bool   first_run        = true;

///////////////////////////////////////////////////////////
///////SYSTEM SETUP////////////////////////////////////////
///////////////////////////////////////////////////////////
void setup()
{
  //Set Baud Rate
  Serial.begin(115200); //Serial.print is too slow @9600
 
  //Setup Data Bus
  pinMode(25, INPUT);
  pinMode(26, INPUT);
  pinMode(27, INPUT);
  pinMode(28, INPUT);
  pinMode(29, INPUT);
  pinMode(30, INPUT);
  pinMode(31, INPUT);
  pinMode(32, INPUT);

  //Initialize Ioints
  sled.init();
  pend1.init();
  pend2.init();

  //Reset Positions
  sled.resetPos();
  pend1.resetPos();
  pend2.resetPos();

  //Set DAC Resolution
  analogWriteResolution(12);
  analogReadResolution(12);

  //Set output to zero
  sled.setOutput(0,1);
  pend1.setOutput(0,1);
  
  //------------------------------------------------------//
  //---FOR SAFTY REASONS, DO NOT CHANGE THE THESE LINES---//
  //------------------------------------------------------//
  //                                                      //
  //Enable Motor Driver Outputs                           //
  digitalWrite( ENABLEPENDUL, LOW    );                   //
  digitalWrite( ENABLESLED,   LOW    );                   //
  pinMode(      ENABLEPENDUL, OUTPUT );                   //
  pinMode(      ENABLESLED,   OUTPUT );                   //
  digitalWrite( ENABLEPENDUL, LOW    );                   //
  digitalWrite( ENABLESLED,   LOW    );                   //
  //                                                      //
  //------------------------------------------------------//

  //set zero cart position at midle of rail for friction lookup
  for( int i = 0; i < 68; i++ ){ position[i] -= 0.38; }
  
  current_time = micros();
  EKF_initialize();
}

void loop()
{
  ///////MODEL PARAMETERS////////////////////////////////////
  float r_pulley  = 0.028;
  float K_t       = 0.0934;
  float l         = 0.3348;      //0.3235;
  float m_b       = 0.201;
  float g         = 9.82;
  float k_tanh    = 250;

  //cart friction and mass
  float m_c       =  6.280;
  float B_v_c     = 10.414;
  //float B_c_pos  = 3.0212 - 0.5;
  //float B_c_neg  = 2.7464;
  //float B_v_pos  = 1.936;
  //float B_v_neg  = 1.422;

  //pendulum frictions
  float B_v_p    = 0.0004;
  float F_c_p    = 0.004;
  
  /////////////////////////////////////////////////////////
  //////SERIAL INTERFACE///////////////////////////////////
  /////////////////////////////////////////////////////////
  String input;
  if( Serial.available() > 0 )
  {
    input = Serial.readStringUntil('\n');
    
    ///////Stop All////////////////////////////////////////
    if( input == "0" )
    {
      setOut   = 0;
      
      //reset EKF
      P_correction_EKF_not_empty_init();
      
      //tell python script to stop logging
      Serial.print("\n");
      Serial.print("stopLogging");
    }
    ///////Swing-Up and Sliding Mode///////////////////////
    else if( input == "1" )
    {
      setOut = 1;
      time_now = micros();
      Serial.print("\n");
      Serial.print("startLogging");      
    }
    ///////Cart Mass and Friction Estimation///////////////
    else if( input == "5" )
    {
      setOut = 5;
      time_now = micros();
      Serial.print("\n");
      Serial.print("startLogging");
    } 
    ///////Pendulum (1 & 2) Friction Estimation////////////
    else if( input == "6" )
    {
      setOut = 6;
      time_now = micros();
      Serial.print("\n");
      Serial.print("startLogging");
    }
    else if( input == "f" )
    {
      setOut = 7;
      time_now = micros();
      Serial.print("\n");
      Serial.print("startLogging");      
    }
    ///////Reset Variables/////////////////////////////////
    else if( input == "r" )
    {
      pend1.resetPos();
      pend2.resetPos();
      sled.resetPos();
      
      //reset EKF
      P_correction_EKF_not_empty_init();
      
      Serial.println("\nInput = r");
    } 
    ///////Input Error/////////////////////////////////////
    else
    {
      Serial.println("\nERROR: Unknown Command!");
    }
    //<<
  } //<<<<SERIAL INTERFACE <END<
    //<<

  ///////READ POSITIONS////////////////////////////////////
  posSled  =  sled.readPos();
  posPend1 =  pend1.readPos();
  posPend1 = (posPend1 + PI);  //+PI for zero in upright
  posPend2 = -pend2.readPos(); //<--| negative-signs to get
  posPend2 = (posPend2 + PI);  //   | counter-clockwise
  velSled  =  sled.readVel();  //   | positive direction
  velPend1 =  pend1.readVel(); //   | convention for
  velPend2 = -pend2.readVel(); //<--| 2nd pendulum
  
  unsigned long time_stamp = micros();

  ///////PRINT FOR OPPERATOR///////////////////////////////
  Serial.print("\n");
  if( setOut == 0 )
  {
    //time converted from micro sec to sec
    float tSec = float(float(time_stamp)/1000000);  // [s]
    
    //print states for operator (system not controlled)
    int deci = 2;
    Serial.print( tSec,     deci );
    Serial.print( ", "           );
    Serial.print( posPend1, deci );
    Serial.print( ", "           );
    Serial.print( posSled,  deci );
    Serial.print( ", "           );
    Serial.print( velPend1, deci );
    Serial.print( ", "           );
    Serial.print( velSled,  deci );  
  } 

  /////////////////////////////////////////////////////////
  ///////FIR FILTER////////////////////////////////////////
  /////////////////////////////////////////////////////////

  float x1_FIR = posPend1;

  //time difference for nummerical diff
  float t_delta = float(float(time_stamp-t_last)/1000000); // [s]
  
  float x3_FIR = (x1_FIR - x1_last)/t_delta;

  //----implementation of FIR filter (with ring buffer)----

  //set window size
  float  M_FIR = 5;
  //filter coefficients
  float h[]  = { 1/M_FIR, 1/M_FIR, 1/M_FIR, 1/M_FIR, 1/M_FIR};

  //update new mesurement in ring-buffer
  ringBuffFIR[offsetFIR] = x3_FIR;
  
  //initializing variable to be updated with filtered value
  x3_FIR = 0;

  //loop through ring-buffer applying filter
  for( int j = offsetFIR; j < M_FIR+offsetFIR; j++ )
  {
    int ringDex = j % int(M_FIR);

    x3_FIR += ringBuffFIR[ringDex]*h[ringDex];
  }
  
  //move ring-buffer offset one step
  if( offsetFIR++ == M_FIR-1 ){ offsetFIR = 0; }
  //NOTE: it checks first then increments

  //-----------------position FIR---------------------------

  float x2_FIR = posSled-0.38; //<--rail center as zero

  float x4_FIR = (x2_FIR - x2_last)/t_delta;

  //----implementation of FIR filter (with ring buffer)----

  //update new mesurement in ring-buffer
  ringBuffFIR2[offsetFIR2] = x4_FIR;
  
  //initializing variable to be updated with filtered value
  x4_FIR = 0;

  //loop through ring-buffer applying filter
  for( int j = offsetFIR2; j < M_FIR+offsetFIR2; j++ )
  {
    int ringDex2 = j % int(M_FIR);

    x4_FIR += ringBuffFIR2[ringDex2]*h[ringDex2];
  }
  
  //move ring-buffer offset one step
  if( offsetFIR2++ == M_FIR-1 ){ offsetFIR2 = 0; }
  //NOTE: it checks first then increments

  x1_last = x1_FIR;
  x2_last = x2_FIR;
  t_last  = time_stamp;

  ///////INITIALIZE AND UPDATE EKF///////////////////////
  double y_meas[2];
  y_meas[0] = posSled;
  y_meas[1] = posPend1;

  if( first_run )
  {
    x_init[0] = posSled;
    x_init[1] = posPend1;
    x_init[2] = 0;
    x_init[3] = 0;
    first_run = false;
  }

  EKF(y_meas, setOutSledNoComp, SAMPLINGTIME, x_init, x_est_correction);

  //change coordinate convention
  float x1 = x_est_correction[1];
  float x2 = x_est_correction[0]-0.38; //<--rail center as zero
  float x3 = x_est_correction[3];
  float x4 = x_est_correction[2];
  
  //update currently used variables if filter is in use
  float current_x1;
  float current_x2;
  float current_x3; 
  float current_x4;
  //
  if(      slideOn == 1 ){ current_x1 = x1;
                           current_x2 = x2;
                           current_x3 = x3;
                           current_x4 = x4;     }
  else if( slideOn == 0 ){ current_x1 = x1_FIR;
                           current_x2 = x2_FIR;
                           current_x3 = x3_FIR;
                           current_x4 = x4_FIR; }

  //creating wrapped vertion of angle for sliding mode
  float x1Wrap = float(fmod( float(x1 + PI), float(2*PI) ));
  if( x1Wrap < 0 )
  {
    x1Wrap = float(x1Wrap + float(2*PI));
  }
  x1Wrap = float(x1Wrap - PI);
  
  /////////////////////////////////////////////////////////
  ///////STOP ALL//////////////////////////////////////////
  /////////////////////////////////////////////////////////
  if( setOut == 0 )
  {
    digitalWrite(ENABLESLED, LOW);
    setOutSled  = 0;
    setOutPend1 = 0;
  }
  /////////////////////////////////////////////////////////
  ///////SWING-UP AND SLIDING MODE/////////////////////////
  /////////////////////////////////////////////////////////
  else if( setOut == 1 )
  {
    //enable cart motor output 
    digitalWrite(ENABLESLED, HIGH);
    
    float tSec = float (time_stamp-time_now)/1000000;

    //>>>>>>>>>>>>>>>>>>>>>>>>>>>><<<<<<<<<<<<<<<<<<<<<<<<<//
    //>>>>>SLIDING MODE PARAMETERS<<<<<<<<<<<<<<<<<<<<<<<<<//
    //>>>>><<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<//

    //initialize sat
    float sat = 0;

    //reduced order linear system gains
    float k_s[] = { 7.3918 ,  -1.3414 ,  -5.5344 };
    
    //variable change for readability
    float k1 = k_s[0], k2 = k_s[1], k3 = k_s[2];

    //sliding mode beta gains
    float rho     = 6.2; //7.135;
    float beta_0  = .1;
    float beta    = rho + beta_0;
    
    //slope of sat-function
    float epsilon = 0.03;
    
    //initialize control and output variable
    setOutSled = 0;
    float u    = 0;

    float B_c_c = estimateCartFriction( position,   coloumbP,   coloumbN,
                                        current_x2, current_x4, setOutSled );
    
    
    /////////////////////////////////////////////////////////
    ///////CATCH - SLIDING MODE//////////////////////////////
    /////////////////////////////////////////////////////////
    //
    if( abs(x1Wrap) < catchAngle )
    {
      slideOn = 1;

      //set wider catch angle to stay in sliding mode after wing-up sequence 
      catchAngle = 0.2;
      
      //inverse of function on output
      float g_b_inv = m_c + m_b - m_b*cos(x1Wrap)*cos(x1Wrap);
      
      //sliding manifold
      float s = x4 - k2*(x3 - (x4*cos(x1Wrap))/l) + k1*x1Wrap + k3*x2;
      
      //saturation function
      float    satS = s / epsilon;
      if(      satS >  1 ){ satS =  1;  }
      else if( satS < -1 ){ satS = -1;  }

      //gain for optional extra x-control
      float k_lin[] = { 10.5460 , 15.8190 };

      //optional linear position (x) controller
      float lin_u = float( -k_lin[1]*x2 -k_lin[1]*x4 );
      
      //final control
      float u = - satS*beta*g_b_inv;// + lin_u;
      
      //calculating required current to obtain control, u
      setOutSled = u*r_pulley/K_t;

      //option to set current limit peak
      if(0)
      {
        float i_peak_limit = 6;
        if( abs(setOutSled) > i_peak_limit )
        {
          if( setOutSled > 0 )
          {
            setOutSled = i_peak_limit;
          }
          else if( setOutSled < i_peak_limit )
          {
            setOutSled = -i_peak_limit;
          }
        }
      }
      //<<
    } //<<<<CATCH - SLIDING MODE <END<
      //<<
    /////////////////////////////////////////////////////////
    ///////SWING-UP//////////////////////////////////////////
    /////////////////////////////////////////////////////////
    else //if(0)
    {
      slideOn = 0;
      
      //set narrow catch angle to provide
      //best handover to sliding mode
      catchAngle = 0.01;
      
      //energy control gain
      float k = 200;

      //sign-function
      float sgnIn = cos(x1_FIR)*x3_FIR;
      float sgn   = 1;
      //
      if(      sgnIn >= 0 ){ sgn =  1; }
      else if( sgnIn <  0 ){ sgn = -1; }
      
      //calculate maximum acceleration of cart
      float i_max = 4.58+.5;
      float u_max = i_max*K_t/r_pulley;
      float a_max = u_max/(m_c+m_b);
      
      //extra energy offset from equilibrium to get fast catch
      float E_off = -.03; // -.007;
      
      //energy error
      float E_delta = .5*m_b*l*l*x3*x3 + m_b*g*l*(cos(x1_FIR) - 1) + E_off;
      
      //energy control law (acceleration of cart)
      float a_c = -k*E_delta*sgn;
      
      //saturation
      if(      a_c >  a_max ){ a_c =  a_max; }
      else if( a_c < -a_max ){ a_c = -a_max; } //a_c = a_c otherwise
      
      //estimation of needed actuation to achieve cart acceleration, a_c
      float theta_acc_est = ( m_c + m_b )*( -B_v_p*x3_FIR -tanh(k_tanh*x3_FIR)*F_c_p + m_b*g*l*sin(x1_FIR) )/( l*l*m_b*(m_c + m_b - m_b*cos(x1_FIR)*cos(x1_FIR)) ) + ( cos(x1_FIR)*(u_last - m_b*l*sin(x1_FIR)*x3_FIR*x3_FIR) )/( l*(m_c + m_b - m_b*cos(x1_FIR)*cos(x1_FIR)) );
      
      //gain for x-control
      float k_lin[] = { 10.5460, 15.8190 };
      
      //linear control of cart position
      float lin_u =  -k_lin[1]*x2 -k_lin[1]*x4;
      
      //final control out put, energy control with position control
      u = ( m_c + m_b )*a_c + m_b*l*sin(x1_FIR)*x3_FIR*x3_FIR -m_b*l*cos(x1_FIR)*theta_acc_est + lin_u;
      
      //calculated needed armature current, i_a, to achieve control, u
      setOutSled = u*r_pulley/K_t;
      
      //store final control for estimation of theta_acc in next loop
      u_last = u;
      
      //<<
    } //<<<<SWING-UP <END<
      //<<
   
    //update currently used variables 
    if( slideOn == 0 ){ current_x2 = x2_FIR;
                        current_x4 = x4_FIR; }
    else              { current_x2 = x2;
                        current_x4 = x4;     }

    B_c_c        = estimateCartFriction( position,   coloumbP,   coloumbN,
                                         current_x2, current_x4, setOutSled );
    
    float frictionComp = cartFrictionCompensation( B_c_c,    B_v_c, current_x4,
                                                   r_pulley, K_t               );

    setOutSledNoComp = setOutSled;
    setOutSled       = setOutSled + frictionComp;

    //choose to print readable data (0) to print for data collection (1)
    int collectData = 1;
    
    //choose nr of decimals printed after decimal point
    int deci = 5;

    //print mesurements depending on choice above
    printToTerminal( collectData, deci,   tSec,
                     x1,          x2,       x3,     x4,
                     x1_FIR,      x2_FIR,   x3_FIR, x4_FIR,
                     x1Wrap,      setOutSledNoComp, setOutSled, B_c_c,
                     posPend1,    posPend2,         velPend1,   velPend2,
                     posSled,     velSled                                );
    //<< 
  } //<<<<SWING-UP AND SLIDING MODE <END<
    //<<
    //
  /////////////////////////////////////////////////////////
  ///////CART FRICTION AND MASS ESTIMATION////////////////
  /////////////////////////////////////////////////////////
  else if( setOut == 5 )
  {
    digitalWrite(ENABLESLED, HIGH);
    setOutPend1 = 0;
    
    float tSec = float(float(time_stamp-time_now)/1000000);  // [s]
    
    int   testTime   = 20;    // [s]
    float railOffset = 0.60;  //<--position on rail under test [m]
    float railRange  = 0.002; //<--range margin [m] of movement 
                              //   on each side of railOffset
    //Note:
    //security end-stop activates
    //  @ 0.04 m and below
    //  @ 0.73 m and above
    
    //rail offsets (from left side of rail) under test:
    //  0.05, 0.06, ..., 0.72  (total of 68 tests)

    if( posSled < railOffset+railRange && setOutSled >= 0 )
    {
      setOutSled = 4;
    }
    else if( posSled > railOffset-railRange && tSec < testTime )
    {
      setOutSled = -4;
    }
    else if( tSec < testTime )
    {
      setOutSled = 4;
    }
    else if( tSec > testTime )
    {
      setOutSled = 0;
      digitalWrite(ENABLESLED, LOW);
      if( velSled == 0 )
      {
        setOut    = 0; //no longer moving, return to complete stop,
      }                //where logging is also stoped
    }
    
    //choose to print readable data (0) to print for data collection (1)
    int collectData = 1;
    
    //choose nr of decimals printed after decimal point
    int deci = 2;

    float B_c_c = 0; //to use print function (otherwise not declared here)

    //print mesurements depending on choice above
    printToTerminal( collectData, deci,   tSec,
                     x1,          x2,       x3,     x4,
                     x1_FIR,      x2_FIR,   x3_FIR, x4_FIR,
                     x1Wrap,      setOutSledNoComp, setOutSled, B_c_c,
                     posPend1,    posPend2,         velPend1,   velPend2,
                     posSled,     velSled                                );
    //<<
  } //<<<<CART FRICTION AND MASS ESTIMATION <END<
    //<<
  /////////////////////////////////////////////////////////
  ///////PENDULUM FRICTION ESTIMATION//////////////////////
  /////////////////////////////////////////////////////////
  else if( setOut == 6 )
  {
    //choose to print readable data (0) to print for data collection (1)
    int collectData = 1;
    
    //choose nr of decimals printed after decimal point
    int deci = 5;

    float B_c_c = 0; //to use print function (otherwise not declared here)

    float tSec = float (time_stamp-time_now)/1000000;
    
    //select pendulum to test
    int pend1test = 0;
    int pend2test = 1;
    
    //when pendulum under test falls from 
    //upright equilibrium, print data to log 
    if( pend1test && posPend1 > 0.01 )
    {
      printToTerminal( collectData, deci,     tSec,
                       x1,          x2,       x3,     x4,
                       x1_FIR,      x2_FIR,   x3_FIR, x4_FIR,
                       x1Wrap,      setOutSledNoComp, setOutSled, B_c_c,
                       posPend1,    posPend2,         velPend1,   velPend2,
                       posSled,     velSled                                );
    }
    else if( pend2test && posPend2 > 0.01 )
    {
      printToTerminal( collectData, deci,     tSec,
                       x1,          x2,       x3,     x4,
                       x1_FIR,      x2_FIR,   x3_FIR, x4_FIR,
                       x1Wrap,      setOutSledNoComp, setOutSled, B_c_c,
                       posPend1,    posPend2,         velPend1,   velPend2,
                       posSled,     velSled                                );
    }
    //<<
  } //<<<<PENDULUM FRICTION ESTIMATION <END<
    //<<
    //  
  //////////////////////////////////////////////////////////
  ///////RUN CART FRICTION COMPENSATION ALONE///////////////
  //////////////////////////////////////////////////////////
  else if( setOut == 7 )
  {
    slideOn = 0;
    //enable cart motor output 
    digitalWrite(ENABLESLED, HIGH);
   
    float tSec = float (time_stamp-time_now)/1000000;
    
    float B_c_c = estimateCartFriction( position,   coloumbP,   coloumbN,
                                        current_x2, current_x4, setOutSled );
    
    float frictionComp = cartFrictionCompensation( B_c_c,    B_v_c, current_x4,
                                                   r_pulley, K_t               );

    setOutSled = frictionComp; 
    
    //choose to print readable data (0) to print for data collection (1)
    int collectData = 1;
    
    //choose nr of decimals printed after decimal point
    int deci = 5;

    //print mesurements depending on choice above
    printToTerminal( collectData, deci,   tSec,
                     x1,          x2,       x3,     x4,
                     x1_FIR,      x2_FIR,   x3_FIR, x4_FIR,
                     x1Wrap,      setOutSledNoComp, setOutSled, B_c_c,
                     posPend1,    posPend2,         velPend1,   velPend2,
                     posSled,     velSled                                );
    //<<
  } //<<<<FRICTION COMPENSATION ONLY <END<
    //<<
    //  
  
  //////////////////////////////////////////////////////////
  ///////SET OUTPUTS////////////////////////////////////////
  //////////////////////////////////////////////////////////

  sled.setOutput(setOutSled,1);
  //sled.setOutput(setOutSledNoComp,1);

  while( current_time + SAMPLINGTIME * 1000000 > micros() )
  {
    //burn time to ensure consistant sample time
  }
  
  if( micros() - current_time > SAMPLINGTIME * 1000000 + 2000 +1000000 )
  {
    Serial.println("Loop Time not uphold");
    digitalWrite(ENABLESLED, LOW);
    delay(5000);
    
    setOutSled = 0;
    setOut     = 0;
  }
  current_time = micros();
  //<<
} //<<<<MAIN-LOOP <END<
  //<<

//linear interpolation function for friction lookup
float interpolate(float z0, float y0, float z1, float y1, float z)
{
    //if at the edge of the segment, return edge value
    if (z <= z0) { return y0; }
    if (z >= z1) { return y1; }

    //interpolation @x between (x0,x1) and (y0,y1)
    return y0 + ( ( z - z0 )/( z1 - z0 ) )*( y1 - y0 );
}

//find coloumb friction coefficient in lookup and interpolate
float interpolateFrictionLookup( float position[], float coloumb[], float z )
{
    //saturate if out position is out of rail range
    if( z < position[0]  ){ return coloumb[0];  }
    if( z > position[67] ){ return coloumb[67]; }
    
    //find cart position in lookup
    for( int i = 0; i < (68-1); i++)
    {
        if( (position[i] <= z) && (position[i+1] >= z) )
        {
           // Serial.print( ", "                );
           // Serial.print( colomb[i],            5 );
            //                  z0             y0        
            return interpolate( position[i],   coloumb[i], 
                                position[i+1], coloumb[i+1], z );
        }   //                  z1             y1
    }
}

//estimate friction based on direction of cart
float estimateCartFriction( float position[],
                            float coloumbP[],
                            float coloumbN[],
                            float x2,
                            float x4,
                            float u          )
{
  float B_c_c = 0;
  if( (x4 > 0) || ((x4 == 0) && (u > 0)) )
  {
    B_c_c = interpolateFrictionLookup( position, coloumbP, x2 );
  }
  else if( (x4 < 0) || ((x4 == 0) && (u < 0)) )
  {
    B_c_c = interpolateFrictionLookup( position, coloumbN, x2 );
  }
  return B_c_c; 
}

//calculate friction compensation based on current friction and velocity
float cartFrictionCompensation( float B_c_c, float B_v_c, float x4,
                                float r,     float k_tau           )
{
  //dead-band sign function
  float sgn_x4 = x4;
  if(      sgn_x4 >  0.01 ){ sgn_x4 =  1; }
  else if( sgn_x4 < -0.01 ){ sgn_x4 = -1; }
  else{                     sgn_x4 =  0; }   
  
  //dead-band x4
  float x4_0 = x4;
  if( (x4_0 < .1) && (x4_0 > -.1 )){ x4_0 = 0; }
  
  float frictionComp = r / k_tau *( (sgn_x4*(B_c_c)) + x4_0*B_v_c*0 );
  
  return frictionComp;
}

void printToTerminal( int   collectData, int   deci, float tSec,
                      float x1,          float x2,     
                      float x3,          float x4,
                      float x1_FIR,      float x2_FIR,
                      float x3_FIR,      float x4_FIR,
                      float x1Wrap,      float setOutSledNoComp,
                      float setOutSled,  float B_c_c,
                      float posPend1,    float posPend2,
                      float velPend1,    float velPend2,
                      float posSled,     float velSled             )
{
  if( collectData )
  {
    //printing for data collection
    Serial.print( tSec,             deci );
    Serial.print( ", "                   );
    Serial.print( x1,               deci );
    Serial.print( ", "                   );
    Serial.print( x1Wrap,           deci );
    Serial.print( ", "                   );
    Serial.print( x1_FIR,           deci );
    Serial.print( ", "                   );
    Serial.print( x2,               deci );
    Serial.print( ", "                   );
    Serial.print( x2_FIR,           deci );
    Serial.print( ", "                   );
    Serial.print( x3,               deci );
    Serial.print( ", "                   );
    Serial.print( x3_FIR,           deci );
    Serial.print( ", "                   );
    Serial.print( x4,               deci );
    Serial.print( ", "                   );
    Serial.print( x4_FIR,           deci );
    Serial.print( ", "                   );
    Serial.print( setOutSledNoComp, deci );
    Serial.print( ", "                   );
    Serial.print( setOutSled,       deci );
    Serial.print( ", "                   );
    Serial.print( B_c_c,            deci );
  }
  else
  {
    Serial.print( tSec,     deci );
    Serial.print( ", "           );
    Serial.print( posPend1, deci );
    Serial.print( ", "           );
    Serial.print( posSled,  deci );
    Serial.print( ", "           );
    Serial.print( velPend1, deci );
    Serial.print( ", "           );
    Serial.print( velSled,  deci );
  }
}
