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

//EKF libraries
#include "EKF.h"
#include "EKF_initialize.h"

//definitions
#define SAMPLINGTIME 0.00667 // s
#define ENABLEPENDUL 11      // 50
#define ENABLESLED   12      // 48

//introducing shorthand for scientific e-notation
#define eN3  pow( 10,  -3 )
#define eN6  pow( 10,  -6 )
#define eN9  pow( 10,  -9 )
#define eN12 pow( 10, -12 )
#define e3   pow( 10,   3 )
#define e6   pow( 10,   6 )
#define e9   pow( 10,   9 )
#define e12  pow( 10,  12 )

#define railOffset 0.38

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
//
//coloumb friction coefficient 
//at each position for positive velocity
//
float coloumbP[] =
  { 2.5089, 2.4779, 2.5412, 2.5587, 2.5882, 2.6186, 2.6376, 2.6989,
    2.8233, 2.9654, 3.1410, 3.2437, 3.2481, 3.2897, 3.4364, 3.5252,
    3.5073, 3.4635, 3.3174, 3.1777, 2.9318, 2.5645, 2.4085, 2.3895,
    2.3946, 2.4522, 2.5371, 2.5996, 2.5777, 2.6028, 2.7608, 2.8936,
    2.9561, 3.0029, 3.2153, 3.4121, 3.5772, 3.6474, 3.5797, 3.5428,
    3.5870, 3.4824, 3.2406, 3.1043, 2.9090, 2.7204, 2.6684, 2.6877,
    2.7343, 2.8770, 2.9953, 3.0911, 3.3058, 3.5883, 3.7440, 3.8397,
    3.8122, 3.7122, 3.4670, 3.3471, 3.3368, 3.2350, 3.1918, 3.2628,
    3.3465, 3.3398, 3.3890, 3.4460                                 };
//
//coloumb friction coefficient 
//at each position for negative velocity
//
float coloumbN[] =
  { 6.0457, 6.0008, 5.4575, 5.0818, 4.9099, 4.8706, 4.6863, 4.1072,
    3.8124, 3.7340, 3.3521, 3.1361, 3.0851, 3.0505, 3.1449, 3.1818,
    3.4730, 3.4172, 3.6360, 3.9225, 4.0484, 4.6924, 4.9115, 4.8782,
    4.6373, 4.4396, 3.8567, 3.7538, 3.4382, 3.4088, 3.2508, 2.9450,
    2.8026, 2.9261, 2.4537, 2.4430, 2.2954, 2.0959, 2.2386, 2.1855,
    2.3199, 2.7089, 2.7662, 3.0776, 3.1715, 3.3254, 3.3598, 3.2184,
    3.0543, 2.8371, 2.8494, 2.5146, 2.5082, 2.6631, 2.8188, 3.0751,
    2.7527, 3.0189, 2.8425, 2.9274, 3.1565, 3.1543, 3.0928, 3.0650,
    2.9287, 3.1714, 3.1068, 2.8821                                 };
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////

//create joint objects
Joint  cart( 1, SAMPLINGTIME );
Joint pend1( 2, SAMPLINGTIME );
Joint pend2( 3, SAMPLINGTIME );

struct firStru
{
  float xIn;          //   set window size here,
  float buff[10];     //<--must be same size as
  int   offset;                               //
};                                              //
                                                 //
///////Global Variables////////////////////////////////////
                                                 //
//ring-buffer and offset for FIR filter        //
int   offsetFIR_x3 = 0;                     //
int   offsetFIR_x4 = 0;                  //
int   offsetFIR_p2 = 0;               //
float buffFIR_x3[10];              //<--here
float buffFIR_x4[10];          //<--here
float buffFIR_p2[10];      //<--and here

int logStop = 0;

float t_last       = 0;
float x1_last      = 0;
float x2_last      = 0;
float x3_last      = 0;
float x4_last      = 0;
float p2_last      = 0;
float p2_FIR_last  = 0;

float setOut_ia    = 0;
float setOutPend1  = 0;

float velCart      = 0;
float velPend1     = 0;
float velPend2     = 0;
float posCart      = 0;
float posPend1     = 0;
float posPend2     = 0;

int   setOut       = 0;

float u_last       = 0;    //used by Kalman and swing so far
float catchAngle   = 0.02;
int   slideOn      = false;
int   slideTwin    = false;

int  twinActive      = 0;
bool twinCatching       = false;
float catchAngleTwin = 0.1;

unsigned long current_time = 0;
unsigned long last_time    = 0;
unsigned long loop_time    = 0;
unsigned long time_now     = 0;

//for EKF initialization
double x_est_correction[4];
double x_init[4];
float  setOut_ia_noComp = 0;
bool   first_run        = true;

//for Kalman initialization
float xEstK[6*1] = { 0 };
//
float Pk[6*6] =
{ 14.185841171390,   0.030200335462, 0.001829079170,  101.695493886294,    0.988096632836,   0.200091103028,
   0.030200335462,  14.519946416777, 0.002114160236,    0.705198836483,  111.070129716470,   0.228222661121,
   0.001829079170,   0.002114160236, 2.798279396029,    0.002494930421,    0.006615239723,   6.788717407965,
 101.695493886294,   0.705198836483, 0.002494930421, 2327.959125138203,    7.104051276564,   0.628709554973,
   0.988096632836, 111.070129716470, 0.006615239723,    7.104051276564, 2383.358258809027,   0.994982551030,
   0.200091103028,   0.228222661121, 6.788717407965,    0.628709554973,    0.994982551030, 500.159420342470 };

bool  firstRunK   = true;

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
  cart.init();
  pend1.init();
  pend2.init();

  //Reset Positions
  cart.resetPos();
  pend1.resetPos();
  pend2.resetPos();

  //Set DAC Resolution
  analogWriteResolution(12);
  analogReadResolution(12);

  //Set output to zero
  cart.setOutput(0,1);
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
  for( int i = 0; i < 68; i++ ){ position[i] -= railOffset; 
                                 coloumbN[i] += 0.32;
                                 coloumbP[i] -= 1.25; }
  
  current_time = micros();
  EKF_initialize();
}

void loop()
{
  ///////MODEL PARAMETERS/////////////////////////////////////////////////////
  //
  //                        addjustment from
  //         measured           estimate    pendulum 1
  float m1 = .100+.075+.026     +.02250;  //mass             [kg]
  float l1 = .3235              -.00658;  //length           [m]
  //
  //                        addjustment from
  //          measured          estimate    pendulum 2
  float m2 = .100+.075+.026+.05 +.0132;   //mass             [kg]
  float l2 = .2                 -.0000;   //length           [m]
  //
  //         estimated
  float M  = 6.28;                        //mass of cart     [kg]
  //
  //         gravity in Denmark
  float g  = 9.82;                        //gravitational
  //                                        acceleration     [m s^-2]
  //
  //         measured
  float r  = 0.028;                       //radius of pulley [m]
  //
  //                                        cart
  float b_c_v  = 0;                       //viscous friction [N m^-1 s]
  //
  //              estimated                 pendulum 1
  float b_p1_c = .0041;                   //coulomb friction [N m]
  float b_p1_v = .0005;                   //viscous friction [N m s]
  //
  //              estimated                 pendulum 2 
  float b_p2_c = .0057;                   //coulomb friction [N m]
  float b_p2_v = .0001;                   //viscous friction [N m s]
  //
  //             for steap slope of tanh
  float k_tanh = 250;                     //tanh constant    [1]
  //
  //             from datasheet             motor
  float k_tau  = .0934;                   //torque constant  [N m A^-1]
  //
  //notation for cart pendulum system with only one pendulum
  //
  float l     = l1;                       //length           [m]
  float m     = m1;                       //mass             [kg]
  float b_p_c = b_p1_c;                   //coloumb friction [N m]
  float b_p_v = b_p1_v;                   //viscous friction [N m s]
  //
  ////////////////////////////////////////////////////////////////////////////
  
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
      twinActive = 0;      
      slideTwin = false;
      
      //reset EKF
      P_correction_EKF_not_empty_init();
      first_run = true;
    
      //reset Kalman
      firstRunK = true;
    }
    ///////Swing-Up and Sliding Mode///////////////////////
    else if( input == "1" )
    {
      setOut = 1;
      slideTwin = false;
      time_now = micros();
      Serial.print("\n");
      Serial.print("startLogging");      
    }
    ///////Swing-Up and Catch Twin with LQR////////////////
    else if( input == "2" )
    {
      setOut = 2;
      twinActive = 1;
      slideTwin = false;
      time_now = micros();
      Serial.print("\n");
      Serial.print("startLogging");      
    }
    ///////Swing-Up and Catch Twini with Sliding Mode//////
    else if( input == "3" )
    {
      setOut = 2;
      twinActive = 1;
      slideTwin = true;
      time_now = micros();
      Serial.print("\n");
      Serial.print("startLogging");      
    }
    ///////Cart Mass and Friction Estimation///////////////
    else if( input == "5" )
    {
      setOut = 5;
      slideTwin = false;
      time_now = micros();
      Serial.print("\n");
      Serial.print("startLogging");
    } 
    ///////Pendulum (1 & 2) Friction Estimation////////////
    else if( input == "6" )
    {
      setOut = 6;
      slideTwin = false;
      time_now = micros();
      Serial.print("\n");
      Serial.print("startLogging");
    }
    ///////Friction Compensation Only//////////////////////
    else if( input == "f" )
    {
      setOut = 7;
      slideTwin = false;
      time_now = micros();
      Serial.print("\n");
      Serial.print("startLogging");      
    }
    ///////Reset Variables/////////////////////////////////
    else if( input == "r" )
    {
      pend1.resetPos();
      pend2.resetPos();
     // cart.resetPos();
      
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
  posCart  =  cart.readPos();
  posCart  =  posCart+.77;    //<--to be able to reset cart @ right rail edge
  posPend1 =  pend1.readPos();
  posPend1 = ( posPend1 + (float)PI -.00314 ); //+PI for zero in upright
  posPend2 =  pend2.readPos();     //offsets   //    | negative-sign to get
  posPend2 = (-posPend2 + (float)PI +.00314 ); //<-- | counter-clockwise
  velCart  =  cart.readVel();
  velPend1 =  pend1.readVel(); //   | positive direction
  velPend2 =  pend2.readVel(); //   | convention for
  velPend2 = -velPend2;        //<--| 2nd pendulum

  unsigned long time_stamp = micros();
 
  ///////PRINT FOR OPPERATOR///////////////////////////////
  Serial.print("\n");
  if( setOut == 0 )
  {
    //tell python script to stop logging
    if( logStop == 1 )
    {
      Serial.print("stopLogging");
      Serial.print("\n");
      logStop = 0;        //<--to only print once
    }

    //time converted from micro sec to sec
    float tSec = float(float(time_stamp)/1000000);  // [s]
    
    //print states for operator (system not controlled)
    int deci = 2;
    Serial.print( tSec,     deci );
    Serial.print( ", "           );
    Serial.print( posPend1, 5 );
    Serial.print( ", "           );
    Serial.print( posPend2, 5 );
    Serial.print( ", "           );
    Serial.print( posCart,  deci );
    Serial.print( ", "           );
   // Serial.print( velPend1, deci );
   // Serial.print( ", "           );
   // Serial.print( velPend2, deci );
   // Serial.print( ", "           );
   // Serial.print( velCart,  deci );  
  } 

  /////////////////////////////////////////////////////////
  ///////FIR (MA) FILTER INIT//////////////////////////////
  /////////////////////////////////////////////////////////

  //time difference for nummerical diff
  float t_delta = float(float(time_stamp-t_last)/1000000); // [s]

  //mesurements used in FIR filters
  float x1_FIR = posPend1;
  float x2_FIR = posCart-railOffset; //<--rail center as zero
  float p2     = posPend2;
  
  //nummerical diff for FIR filters
  float x3_FIR = (x1_FIR - x1_last)/t_delta;
  float x4_FIR = (x2_FIR - x2_last)/t_delta;
  float p2_FIR = (p2     - p2_last)/t_delta;  

  //set window size for all FIR filters
  float  N_FIR = (sizeof(buffFIR_x3)/sizeof(buffFIR_x3[0]));
  
  //filter coefficients
  float h[(int)N_FIR];
  for( int i = 0; i< N_FIR; i++ )
  {
    h[i]  = 1/N_FIR;
  }

  /////////////////////////////////////////////////////////
  ///////FIR (MA) FILTERS//////////////////////////////////
  /////////////////////////////////////////////////////////

  //struct for FIR (MA) filter of x3 (p1 angle velocity)
  struct firStru sx3 =
  {
    x3_FIR,
    { 0 },
    offsetFIR_x3
  };
  memcpy( sx3.buff, buffFIR_x3, sizeof(buffFIR_x3) );
  
  //calculate angular velocity through FIR (MA) filter
  sx3 = FIR( sx3, h, (int)N_FIR );

  //assigning output of filter
          x3_FIR       = sx3.xIn;
          offsetFIR_x3 = sx3.offset;
  memcpy( buffFIR_x3,    sx3.buff,   sizeof(buffFIR_x3) );

  //-------------------------------------------------------

  //struct for FIR (MA) filter of x4 (cart velocity)
  struct firStru sx4 =
  {
    x4_FIR,
    { 0 },
    offsetFIR_x4
  };
  memcpy( sx4.buff, buffFIR_x4, sizeof(buffFIR_x4) );
  
  //calculate angular velocity through FIR (MA) filter
  sx4 = FIR( sx4, h, (int)N_FIR );

  //assigning output of filter
          x4_FIR       = sx4.xIn;
          offsetFIR_x4 = sx4.offset;
  memcpy( buffFIR_x4,    sx4.buff,   sizeof(buffFIR_x4) );

  //-------------------------------------------------------

  //struct for FIR (MA) filter of p2 angle velocity
  struct firStru sp2 =
  {
    p2_FIR,
    { 0 },
    offsetFIR_p2
  };
  memcpy( sp2.buff, buffFIR_p2, sizeof(buffFIR_p2) );
  
  //calculate angular velocity through FIR (MA) filter
  sp2 = FIR( sp2, h, (int)N_FIR );

  //assigning output of filter
          p2_FIR       = sp2.xIn;
          offsetFIR_p2 = sp2.offset;
  memcpy( buffFIR_p2,    sp2.buff,   sizeof(buffFIR_p2) );

  /////////////////////////////////////////////////////////
  ///////EXPONENTIAL SMOOTHING FILTER//////////////////////
  /////////////////////////////////////////////////////////
  
  float alpha = .5;
  x3_FIR      = alpha*x3_FIR + ( 1 - alpha )*x3_last;
  x4_FIR      = alpha*x4_FIR + ( 1 - alpha )*x4_last;
  p2_FIR      = alpha*p2_FIR + ( 1 - alpha )*p2_FIR_last;
  
  x1_last     = x1_FIR;
  x2_last     = x2_FIR;
  x3_last     = x3_FIR;
  x4_last     = x4_FIR;
  p2_last     = p2;
  p2_FIR_last = p2_FIR;
  t_last      = time_stamp;
 
  //time converted from micro sec to sec
  float tSec = float(float(time_stamp)/1000000);  // [s]
 
 
  //creating wrapped vertion of theta1 for catch
  float x1Wrap = float(fmod( float(posPend1 + (float)PI), float(2*PI) ));
  if( x1Wrap < 0 )
  {
    x1Wrap = float(x1Wrap + float(2*PI));
  }
  x1Wrap = float(x1Wrap - (float)PI);
  
  //creating wrapped vertion of theta2 for catch
  float x2Wrap = float(fmod( float(posPend2 + (float)PI), float(2*PI) ));
  if( x2Wrap < 0 )
  {
    x2Wrap = float(x2Wrap + float(2*PI));
  }
  x2Wrap = float(x2Wrap - (float)PI);

  
  //choose states depending on which system (twin or not) is active
  if( twinActive )
  {
    if( twinCatching )
    {
      ///////KALMAN FILTER/////////////////////////////////
      // 
      //            theta1_dot theta2_dot x_dot
      kalmanFilter( x3_FIR,    p2_FIR,    x4_FIR );
    }
  }
  else
  {
    ///////EKF/////////////////////////////////////////////
    double y_meas[2];
    y_meas[0] = posCart;
    y_meas[1] = posPend1;

    if( first_run )
    {
      x_init[0] = posCart;
      x_init[1] = posPend1;
      x_init[2] = 0;
      x_init[3] = 0;
      first_run = false;
    }

    EKF(y_meas, setOut_ia_noComp, SAMPLINGTIME, x_init, x_est_correction);
    
    float x1EKF = x_est_correction[1]; 
    //
    //creating wrapped vertion of angle for sliding mode
    x1Wrap = float(fmod( float(x1EKF + (float)PI), float(2*PI) ));
    if( x1Wrap < 0 )
    {
      x1Wrap = float(x1Wrap + float(2*PI));
    }
    x1Wrap = float(x1Wrap - (float)PI);
  }  
 
  /////////////////////////////////////////////////////////
  ///////STOP ALL//////////////////////////////////////////
  /////////////////////////////////////////////////////////
  if( setOut == 0 )
  {
    digitalWrite(ENABLESLED, LOW);
    setOut_ia  = 0;
    setOutPend1 = 0;
  }
  /////////////////////////////////////////////////////////
  ///////SWING-UP AND SLIDING MODE/////////////////////////
  /////////////////////////////////////////////////////////
  else if( setOut == 1 )
  {
    float x1, x2, x3, x4;
    
    //enable cart motor output 
    digitalWrite(ENABLESLED, HIGH);
    
    float tSec = float (time_stamp-time_now)/1000000;

    //>>>>>>>>>>>>>>>>>>>>>>>>>>>><<<<<<<<<<<<<<<<<<<<<<<<<//
    //>>>>>SLIDING MODE PARAMETERS<<<<<<<<<<<<<<<<<<<<<<<<<//
    //>>>>><<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<//

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

    /////////////////////////////////////////////////////////
    //////EDGE OF RAIL SECURITY//////////////////////////////
    /////////////////////////////////////////////////////////
    //
    float sgnCart = posCart-railOffset;
    if(      sgnCart > 0 ){ sgnCart =  1; }
    else if( sgnCart < 0 ){ sgnCart = -1; }
    else                  { sgnCart =  0; }
    //
    //  cart is close to edge   &&  velocity away from center is large
    if( abs(posCart-railOffset) > .08  &&  sgnCart*velCart > 1 )
    {
      Serial.println("Secure Edge Active");
    
      Serial.println(sgnCart);

      setOut_ia = -sgnCart*5; //<--breaking if cart runs away
    }
    /////////////////////////////////////////////////////////
    ///////CATCH - SLIDING MODE//////////////////////////////
    /////////////////////////////////////////////////////////
    //
    else if( abs(x1Wrap) < catchAngle )
    {
      x1 = x1Wrap;
      x2 = x_est_correction[0]-railOffset; //<--rail center as zero
      x3 = x_est_correction[3];
      x4 = x_est_correction[2];
      
      slideOn = true;
     
      //set wider catch angle to stay in sliding mode after wing-up sequence 
      catchAngle = 0.2;
      
      //inverse of function on output
      float g_b_inv = M + m - m*cos(x1)*cos(x1);
      
      //sliding manifold
      float s = x4 - k2*(x3 - (x4*cos(x1))/l) + k1*x1 + k3*x2;
      
      //saturation function
      float    satS = s / epsilon;
      if(      satS >  1 ){ satS =  1;  }
      else if( satS < -1 ){ satS = -1;  }

      //gain for optional extra x-control
      float k_lin[] = { 10.5460 , 15.8190 };

      //optional linear position (x) controller
      float lin_u = float( -k_lin[0]*x2 -k_lin[1]*x4 );
      
      //final control
      float u = - satS*beta*g_b_inv;// + lin_u;
      
      //calculating required current to obtain control, u
      setOut_ia = u*r/k_tau;

      //<<
    } //<<<<CATCH - SLIDING MODE <END<
      //<<
    /////////////////////////////////////////////////////////
    ///////SWING-UP//////////////////////////////////////////
    /////////////////////////////////////////////////////////
    else //if( tSec >= 5.0) //if(0)
    {
      x1 = x1_FIR;
      x2 = x2_FIR;
      x3 = x3_FIR;
      x4 = x4_FIR;
     
      Serial.print(tSec);
       
      slideOn = false;
      
      //set narrow catch angle to provide
      //best handover to sliding mode
      catchAngle = 0.02;
      
      //energy control gain
      float k = 200;

      //sign-function
      float sgnIn = cos(x1)*x3;
      float sgn   = 1;
      //
      if(      sgnIn >= 0 ){ sgn =  1; }
      else if( sgnIn <  0 ){ sgn = -1; }
      
      //calculate maximum acceleration of cart
      float i_max = 4.58+.5+.2;
      float u_max = i_max*k_tau/r;
      float a_max = u_max/(M + m);
      
      //extra energy offset from equilibrium to get fast catch
      float E_off = -.026;// -.03; // -.007;
      
      //energy error
      float E_delta = .5*m*l*l*x3*x3 + m*g*l*(cos(x1) - 1) + E_off;
      
      //energy control law (acceleration of cart)
      float a_c = -k*E_delta*sgn;
      
      //saturation
      if(      a_c >  a_max ){ a_c =  a_max; }
      else if( a_c < -a_max ){ a_c = -a_max; } //a_c = a_c otherwise
      
      //estimation of needed actuation to achieve cart acceleration, a_c
      float theta_acc_est = ( M + m )*
                            ( -b_p_v*x3 -tanh(k_tanh*x3)*b_p_c + m*g*l*sin(x1) )/
                            ( l*l*m*(M + m - m*cos(x1)*cos(x1)) )
                          + ( cos(x1)*(u_last - m*l*sin(x1)*x3*x3) )/
                            ( l*(M + m - m*cos(x1)*cos(x1)) );
      
      //gain for x-control
      float k_lin[] = { 10.5460, 15.8190 };
      
      //linear control of cart position
      float lin_u =  -k_lin[0]*x2 -k_lin[1]*x4;
      
      //final control output, energy control with position control
      float u = ( M + m )*a_c + m*l*sin(x1)*x3*x3
                              - m*l*cos(x1)*theta_acc_est + lin_u;
      
      //option to limit peak current
      if(0)
      {
        float i_peakLimit = 8;
        float u_peakLimit = i_peakLimit*k_tau/r;
        
        if( abs(u) > u_peakLimit )
        {
          if(      u > 0 ){ u =  u_peakLimit; }
          else if( u < 0 ){ u = -u_peakLimit; }
        }
      }
      
      //calculated needed armature current, i_a, to achieve control, u
      setOut_ia = u*r/k_tau;
      
      //store final control for estimation of theta_acc in next loop
      u_last = u;
      
      //<<
    } //<<<<SWING-UP <END<
      //<<
      
    float b_c_c  = estimateCartFriction( position, coloumbP, coloumbN,
                                         x2,       x4,       setOut_ia );
   
    float frictionComp = cartFrictionCompensation( b_c_c, b_c_v, x4,
                                                   r,     k_tau     );
    setOut_ia_noComp = setOut_ia;
    setOut_ia        = setOut_ia + frictionComp;

    //<< 
  } //<<<<SWING-UP AND SLIDING MODE <END<
    //<<
    //
  /////////////////////////////////////////////////////////
  ///////TWIN SWING-UP AND CATCH///////////////////////////
  /////////////////////////////////////////////////////////
  else if( setOut == 2 )
  {
    float x1, x2, x3, x4, x5, x6;
    
    //enable cart motor output 
    digitalWrite(ENABLESLED, HIGH);
    
    float tSec = float (time_stamp-time_now)/1000000;
    int deci = 5;
    Serial.print( tSec,               deci );
    Serial.print( ", "                     );
    Serial.print( posPend1,           deci );
    Serial.print( ", "                     );
    Serial.print( posPend2,           deci );
    Serial.print( ", "                     );
    Serial.print( posCart-railOffset, deci );
    Serial.print( ", "                     );
    Serial.print( velPend1,           deci );
    Serial.print( ", "                     );
    Serial.print( velPend2,           deci );
    Serial.print( ", "                     );
    Serial.print( velCart,            deci );
    Serial.print( ", "                     );
    Serial.print( u_last,             deci );
    Serial.print( ", "                     );
    
    /////////////////////////////////////////////////////////
    //////EDGE OF RAIL SECURITY//////////////////////////////
    /////////////////////////////////////////////////////////
    //
    float sgnCart = posCart-railOffset;
    if(      sgnCart > 0 ){ sgnCart =  1; }
    else if( sgnCart < 0 ){ sgnCart = -1; }
    else                  { sgnCart =  0; }
    //
    //  cart is close to edge   &&  velocity away from center is large
    if( abs(posCart-railOffset) > .08  &&  sgnCart*velCart > 1.5 )
    {
      Serial.println("Secure Edge Active");
      
      setOut_ia = -sgnCart*5; //<--breaking if cart runs away
    }
    /////////////////////////////////////////////////////////
    ///////TWIN CATCH WITH SLIDING MODE//////////////////////
    /////////////////////////////////////////////////////////
    //
    else if( slideTwin && (abs(x1Wrap)+abs(x2Wrap)) < catchAngleTwin  )
    {
      x1 = xEstK[0];
      x2 = xEstK[1];
      x3 = xEstK[2];
      x4 = xEstK[3];
      x5 = xEstK[4];
      x6 = xEstK[5];
      
      int deci = 5;
      Serial.print( x1, deci );
      Serial.print( ", "     );
      Serial.print( x2, deci );
      Serial.print( ", "     );
      Serial.print( x3, deci );
      Serial.print( ", "     );
      Serial.print( x4, deci );
      Serial.print( ", "     );
      Serial.print( x5, deci );
      Serial.print( ", "     );
      Serial.print( x6, deci );
      logStop = 1;
      
      twinCatching = true;
      
      //set wider catch angle to stay in stabilization after wing-up sequence 
      catchAngleTwin = 2;
      
      //reduced order linear system gains
      //float k_s[] = { -12.0798, 10.5851, -7.8671, -31.6698, 0.0789 }; //works well in sim
      float k_s[] = { -21.3136, 17.3192, -13.2417, -53.8390, 0.4732 };//also works well in sim
      //float k_s[] = { -9.5609, 8.6073, -6.3968, -25.6343,  0.0388 };
      //float k_s[] = { -16.1846, 13.6424, -10.2666, -41.5516, 0.2135 };
      
      //variable change for readability
      float k1 = k_s[0], k2 = k_s[1], k3 = k_s[2], k4 = k_s[3], k5 = k_s[4];
      
      float rho   = 2;
      float beta0 = .1;
      float beta  = rho + beta0;
      
      //inverse of function on output
      float g_b_inv = M + m1 + m2 - m1*cos(x1)*cos(x1) - m2*cos(x2)*cos(x2);
      
      //sliding manifold
      float  s = x6 + k1*x1 + k2*x2 + k5*x3
               - k3*( x4 + x5 - x6*( cos(x1)/l1 + cos(x2)/l2 ) )             
               + k4*( l1*x4 + l2*x5 - x6*(cos(x1) + cos(x2))   );
      
      //saturation function
      float epsilon = .03;
      float    satS = s / epsilon;
      if(      satS >  1 ){ satS =  1;  }
      else if( satS < -1 ){ satS = -1;  }

      //final control
      float u = - satS*beta*g_b_inv;
      
      u_last = u;
      
      //calculating required current to obtain control, u
      setOut_ia = u*r/k_tau;
    }
    /////////////////////////////////////////////////////////
    ///////TWIN CATCH WITH LQR///////////////////////////////
    /////////////////////////////////////////////////////////
    //
    else if( ~slideTwin && (abs(x1Wrap)+abs(x2Wrap)) < catchAngleTwin  )
    {
      x1 = xEstK[0];
      x2 = xEstK[1];
      x3 = xEstK[2];
      x4 = xEstK[3];
      x5 = xEstK[4];
      x6 = xEstK[5];
      
      int deci = 5;
      Serial.print( x1, deci );
      Serial.print( ", "     );
      Serial.print( x2, deci );
      Serial.print( ", "     );
      Serial.print( x3, deci );
      Serial.print( ", "     );
      Serial.print( x4, deci );
      Serial.print( ", "     );
      Serial.print( x5, deci );
      Serial.print( ", "     );
      Serial.print( x6, deci );
      logStop = 1;
      
      twinCatching = true;
      
      //set wider catch angle to stay in stabilization after wing-up sequence 
      catchAngleTwin = 0.8;
      
      //LQR gain vectors
      float kLQR[] = { -5058.01, 4037.40, 296.63, -892.48, 553.70, 256.29 };
       
      //LQR
      float u = -kLQR[0]*x1 -kLQR[1]*x2 -kLQR[2]*x3
                -kLQR[3]*x4 -kLQR[4]*x5 -kLQR[5]*x6;
      
      //option to set current limit peak
      if(0)
      {
        float i_peakLimit = 8;
        float u_peakLimit = i_peakLimit*k_tau/r;
        
        if( abs(u) > u_peakLimit )
        {
          if(      u > 0 ){ u =  u_peakLimit; }
          else if( u < 0 ){ u = -u_peakLimit; }
        }
      }
      
      //calculating required current to obtain control, u
      setOut_ia = u*r/k_tau;
      
      u_last = u;

      //<<
    } //<<<<TWIN CATCH <END<
      //<<
    /////////////////////////////////////////////////////////
    ///////TWIN SWING-UP/////////////////////////////////////
    /////////////////////////////////////////////////////////
    else //if(0)
    {
      x1 = x1_FIR; //theta1
      x2 = p2;     //theta2
      x3 = x2_FIR; //x
      x4 = x3_FIR; //theta1_dot
      x5 = p2_FIR; //theta2_dot
      x6 = x4_FIR; //x_dot
      
      int deci = 5;
      Serial.print( x1, deci );
      Serial.print( ", "     );
      Serial.print( x2, deci );
      Serial.print( ", "     );
      Serial.print( x3, deci );
      Serial.print( ", "     );
      Serial.print( x4, deci );
      Serial.print( ", "     );
      Serial.print( x5, deci );
      Serial.print( ", "     );
      Serial.print( x6, deci );
      logStop = 1;
      
      twinCatching = false;
      
      //set narrow catch angle to provide
      //best handover to sliding mode
      catchAngleTwin = 0.06;

      //energy control gains
      float k1 = 19;
      float k2 = 12.78;
       
      //extra energy offset from equilibrium to get fast catch
      float E_off1 = -.067577;//-.177;//-.175;//.022;
      float E_off2 = -.007;//.022;
      
      //inertias
      float J1 = m1*(l1*l1);
      float J2 = m2*(l2*l2);
      
      //energy error
      float E_delta1 = .5*J1*(x4*x4) + m1*g*l1*(cos(x1) - 1) + E_off1;
      float E_delta2 = .5*J2*(x5*x5) + m2*g*l2*(cos(x2) - 1) + E_off2;

      float G = k1*m1*l1*E_delta1*cos(x1)*x4 + k2*m2*l2*E_delta2*cos(x2)*x5;

      //energy control law (acceleration of cart)
      float a_c = -G;
      
      //calculate maximum acceleration of cart
      float i_max = 4.58+.5; //+5;
      float u_max = i_max*k_tau/r;
      float a_max = u_max/(M + m1 + m2);
      
      //saturation
      if(      a_c >  a_max ){ a_c =  a_max; }
      else if( a_c < -a_max ){ a_c = -a_max; } //a_c = -G otherwise

      //estimation of needed actuation to achieve cart acceleration, a_c
      float theta1_acc_est =
        -(b_p1_v*l2*m1*x4 + b_p1_v*l2*m2*x4 + M*b_p1_c*l2*tanh(k_tanh*x4)
        + b_p1_c*l2*m1*tanh(k_tanh*x4) + b_p1_c*l2*m2*tanh(k_tanh*x4)
        + M*b_p1_v*l2*x4 - b_p1_v*l2*m2*x4*cos(x2)*cos(x2)
        - g*l1*l2*m1*m1*sin(x1) - b_p1_c*l2*m2*tanh(k_tanh*x4)*cos(x2)*cos(x2)
        - l1*l2*m1*u_last*cos(x1) + l1*l1*l2*m1*m1*x4*x4*cos(x1)*sin(x1)
        + b_p2_c*l1*m1*tanh(k_tanh*x5)*cos(x1)*cos(x2)
        - M*g*l1*l2*m1*sin(x1) - g*l1*l2*m1*m2*sin(x1)
        + b_p2_v*l1*m1*x5*cos(x1)*cos(x2)
        + l1*l2*l2*m1*m2*x5*x5*cos(x1)*sin(x2)
        + g*l1*l2*m1*m2*cos(x2)*cos(x2)*sin(x1)
        - g*l1*l2*m1*m2*cos(x1)*cos(x2)*sin(x2))/
        (l1*l1*l2*m1*(M + m1 + m2 - m1*cos(x1)*cos(x1) - m2*cos(x2)*cos(x2)));
    
      float theta2_acc_est =
        -(b_p2_v*l1*m1*x5 + b_p2_v*l1*m2*x5 + M*b_p2_c*l1*tanh(k_tanh*x5)
        + b_p2_c*l1*m1*tanh(k_tanh*x5) + b_p2_c*l1*m2*tanh(k_tanh*x5)
        + M*b_p2_v*l1*x5 - b_p2_v*l1*m1*x5*cos(x1)*cos(x1)
        - g*l1*l2*m2*m2*sin(x2) - b_p2_c*l1*m1*tanh(k_tanh*x5)*cos(x1)*cos(x1)
        - l1*l2*m2*u_last*cos(x2) + l1*l2*l2*m2*m2*x5*x5*cos(x2)*sin(x2)
        + b_p1_c*l2*m2*tanh(k_tanh*x4)*cos(x1)*cos(x2)
        - M*g*l1*l2*m2*sin(x2) - g*l1*l2*m1*m2*sin(x2)
        + b_p1_v*l2*m2*x4*cos(x1)*cos(x2)
        + l1*l1*l2*m1*m2*x4*x4*cos(x2)*sin(x1)
        + g*l1*l2*m1*m2*cos(x1)*cos(x1)*sin(x2)
        - g*l1*l2*m1*m2*cos(x1)*cos(x2)*sin(x1))
        /(l1*l2*l2*m2*(M + m1 + m2 - m1*cos(x1)*cos(x1) - m2*cos(x2)*cos(x2)));  

      //gain for x-control
      float k_lin[] = { 10.5460, 15.8190 };
      
      //linear control of cart position
      float lin_u =  -k_lin[0]*x3 -k_lin[1]*x6;
      
      //final control output, energy control with position control
      float u = (M+m1+m2)*a_c
              + m1*l1*sin(x1)*(x4*x4) - m1*l1*cos(x1)*theta1_acc_est
              + m2*l2*sin(x2)*(x5*x5) - m2*l2*cos(x2)*theta2_acc_est
              + lin_u;
      
      //setting max output in one direction for the
      //first 0.1 s to start the swing-up procecure
      if( tSec < 0.1 ){  u = u_max;  }
      
      //calculating needed armature current, i_a, to achieve control, u
      setOut_ia = u*r/k_tau;
      
      u_last = u;
      
      //<<
    } //<<<<TWIN SWING-UP <END<
      //<<
    
    float b_c_c = estimateCartFriction( position, coloumbP, coloumbN,
                                        x3,       x6,       setOut_ia );
     
    float frictionComp = cartFrictionCompensation( b_c_c, b_c_v, x6,
                                                   r,     k_tau     );
    setOut_ia_noComp = setOut_ia;
    setOut_ia        = setOut_ia + frictionComp;

    //<< 
  } //<<<<TWIN SWING-UP AND CATCH <END<
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
    
    int   testTime  = 20;    // [s]
    float railOff   = 0.05;  //<--position on rail under test [m]
    float railRange = 0.002; //<--range margin [m] of movement 
                              //   on each side of railOff
    //Note:
    //security end-stop activates
    //  @ 0.04 m and below
    //  @ 0.73 m and above
    
    //rail offsets (from left side of rail) under test:
    //  0.05, 0.06, ..., 0.72  (total of 68 tests)

    if( posCart < railOff+railRange && setOut_ia >= 0 )
    {
      setOut_ia = 4;
    }
    else if( posCart > railOff-railRange && tSec < testTime )
    {
      setOut_ia = -4;
    }
    else if( tSec < testTime )
    {
      setOut_ia = 4;
    }
    else if( tSec > testTime )
    {
      setOut_ia = 0;
      digitalWrite(ENABLESLED, LOW);
      if( velCart == 0 )
      {
        logStop   = 1;
        setOut    = 0; //no longer moving, return to complete stop,
      }                //where logging is also stoped
    }
    
    //choose nr of decimals printed after decimal point
    int deci = 5;

    Serial.print( tSec,      deci );
    Serial.print( ", "            );
    Serial.print( setOut_ia, deci );
    Serial.print( ", "             );
    Serial.print( posCart,    deci );
    Serial.print( ", "             );
    Serial.print( velCart,    deci );
    //<<
  } //<<<<CART FRICTION AND MASS ESTIMATION <END<
    //<<
  /////////////////////////////////////////////////////////
  ///////PENDULUM FRICTION ESTIMATION//////////////////////
  /////////////////////////////////////////////////////////
  else if( setOut == 6 )
  {
    float tSec = float (time_stamp-time_now)/1000000;
    
    //select pendulum to test
    int pend1test = 0;
    int pend2test = 1;
    
    //when pendulum under test falls from 
    //upright equilibrium, print data to log 
    if( pend1test && posPend1 > 0.01 )
    {
      int deci = 5;
      Serial.print( tSec,     deci );
      Serial.print( ", "           );
      Serial.print( posPend1, deci );
      Serial.print( ", "           );
      Serial.print( velPend1, deci );
    }
    else if( pend2test && posPend2 > 0.01 )
    {
      int deci = 5;
      Serial.print( tSec,     deci );
      Serial.print( ", "           );
      Serial.print( posPend2, deci );
      Serial.print( ", "           );
      Serial.print( velPend2, deci );
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
    slideOn = false;
    //enable cart motor output 
    digitalWrite(ENABLESLED, HIGH);
   
    float tSec = float (time_stamp-time_now)/1000000;
    
    float b_c_c = estimateCartFriction( position, coloumbP, coloumbN,
                                        x2_FIR,   x4_FIR,   setOut_ia );
    
    float frictionComp = cartFrictionCompensation( b_c_c, b_c_v, x4_FIR,
                                                   r,     k_tau         );

    setOut_ia = frictionComp; 
    
    //<<
  } //<<<<FRICTION COMPENSATION ONLY <END<
    //<<
    //  
  
  //////////////////////////////////////////////////////////
  ///////SET OUTPUTS////////////////////////////////////////
  //////////////////////////////////////////////////////////

  cart.setOutput( setOut_ia, 1 );

  while( current_time + SAMPLINGTIME * 1000000 > micros() )
  {
    //burn time to ensure consistant sample time
  }
  
  if( micros() - current_time > SAMPLINGTIME * 1000000 + 2000 +1000000 )
  {
    Serial.println("Loop Time not uphold");
    digitalWrite(ENABLESLED, LOW);
    delay(5000);
    
    setOut_ia = 0;
    setOut    = 0;
  }
  current_time = micros();
  //<<
} //<<<<MAIN-LOOP <END<
  //<<

///////////////////////////////////////////////////////////
//
// matrix sizes
// c1: cols in matrix 1   r1: rows in matrix 1 
// c2: cols in matrix 2   r2: rows in matrix 2
//
// matrix array-vector interpretation
// A[row][col] => A[ row *c1+ col ] = A[ vector element ]
//
// for matrix declarations as vectors
// float A[r1][c1] => float A[r1*c1]
//
///////////////////////////////////////////////////////////

void inv3x3Matrix( float invM[3*3], float m[3*3] )
{
  int c = 3;
  
  float det  = m[0 *c+ 0] * (m[1 *c+ 1]*m[2 *c+ 2] - m[1 *c+ 2] * m[2 *c+ 1]);
        det -= m[0 *c+ 1] * (m[1 *c+ 0]*m[2 *c+ 2] - m[1 *c+ 2] * m[2 *c+ 0]);
        det += m[0 *c+ 2] * (m[1 *c+ 0]*m[2 *c+ 1] - m[1 *c+ 1] * m[2 *c+ 0]);

  float invDet = 1.0 / (det);

  invM[0 *c+ 0] = (invDet) * (m[1 *c+ 1] * m[2 *c+ 2] - m[1 *c+ 2] * m[2 *c+ 1]);
  invM[1 *c+ 0] = (invDet) * (m[1 *c+ 2] * m[2 *c+ 0] - m[1 *c+ 0] * m[2 *c+ 2]);
  invM[2 *c+ 0] = (invDet) * (m[1 *c+ 0] * m[2 *c+ 1] - m[1 *c+ 1] * m[2 *c+ 0]);

  invM[0 *c+ 1] = (invDet) * (m[0 *c+ 2] * m[2 *c+ 1] - m[0 *c+ 1] * m[2 *c+ 2]);
  invM[1 *c+ 1] = (invDet) * (m[0 *c+ 0] * m[2 *c+ 2] - m[0 *c+ 2] * m[2 *c+ 0]);
  invM[2 *c+ 1] = (invDet) * (m[0 *c+ 1] * m[2 *c+ 0] - m[0 *c+ 0] * m[2 *c+ 1]);

  invM[0 *c+ 2] = (invDet) * (m[0 *c+ 1] * m[1 *c+ 2] - m[0 *c+ 2] * m[1 *c+ 1]);
  invM[1 *c+ 2] = (invDet) * (m[0 *c+ 2] * m[1 *c+ 0] - m[0 *c+ 0] * m[1 *c+ 2]);
  invM[2 *c+ 2] = (invDet) * (m[0 *c+ 0] * m[1 *c+ 1] - m[0 *c+ 1] * m[1 *c+ 0]);
}

void matrixTranspose( float matrixT[], float matrix[],
                                       int r1, int c1 ) 
{
  int i, j;
  for (   i = 0; i < r1; i++ )
  { 
    for ( j = 0; j < c1; j++ )

      matrixT[j *r1+ i] = matrix[i *c1+ j]; 
  }
}

void matrixAdd( float add[], bool substract,
                float matrix1[],  float matrix2[],
                int   r1, int c1, int r2, int c2  )
{
  int i, j;

  //check if matrix dimentions match, if not: exit subroutine
  if( c1 != c2 || r1 != r2 )
  {
    Serial.print( "matrixAdd: Matrix Dimention Mismatch!" );
    logStop = 1;
    setOut  = 0; //stop logging
  }

  for(   i = 0; i < r1; ++i )
  {
    for( j = 0; j < c1; ++j )
    {
      if( substract )
      {
        add[i *c1+ j] = matrix1[i *c1+ j] - matrix2[i *c2+ j];
      }
      else
      {
        add[i *c1+ j] = matrix1[i *c1+ j] + matrix2[i *c2+ j];
      }
    }
  }
}

void matrixMult( float mult[],
                 float matrix1[],  float matrix2[],
                 int   r1, int c1, int r2, int c2  )
{
  int i, j, k;

  //check if matrix dimentions match, if not: exit subroutine
  if( c1 != r2 )
  {
    Serial.print( "matrixMult: Matrix Dimention Mismatch!" );
    logStop = 1;
    setOut  = 0; //stop logging
  }

  for(     i = 0; i < r1; ++i )
  {
    for(   j = 0; j < c2; ++j )
    {
      mult[i *c2+ j] = 0;
      
      for( k = 0; k < c1; ++k )
      {
        mult[i *c2+ j] += matrix1[i *c1+ k]*matrix2[k *c2+ j];
      }
    }
  }
}

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
                            float ia          )
{
  float b_c_c = 0;
  if( (x4 > 0) || ((x4 == 0) && (ia > 0)) )
  {
    b_c_c = interpolateFrictionLookup( position, coloumbP, x2 );
  }
  else if( (x4 < 0) || ((x4 == 0) && (ia < 0)) )
  {
    b_c_c = interpolateFrictionLookup( position, coloumbN, x2 );
  }
  return b_c_c; 
}

//calculate friction compensation based on current friction and velocity
float cartFrictionCompensation( float b_c_c, float b_c_v, float x4,
                                float r,     float k_tau           )
{
  //dead-band sign function
  float sgn_x4 = x4;
  if(      sgn_x4 >  0.012 ){ sgn_x4 =  1; }
  else if( sgn_x4 < -0.012 ){ sgn_x4 = -1; }
  else{                       sgn_x4 =  0; }   
  
  //saturation function
  float   epsilon = 0.03;
  float    sat_x4 = x4 / epsilon;
  if(      sat_x4 >  1 ){ sat_x4 =  1;  }
  else if( sat_x4 < -1 ){ sat_x4 = -1;  }
  
  //dead-band x4
  float x4_0 = x4;
  if( (x4_0 < .1) && (x4_0 > -.1 )){ x4_0 = 0; }
  
  float frictionComp = r / k_tau *( (sat_x4*(b_c_c)) + x4_0*b_c_v*0 );
  
  return frictionComp;
}

struct firStru FIR( struct firStru x, float h[], int N )
{
  //update new mesurement in ring-buffer
  x.buff[x.offset] = x.xIn;

  //initializing variable to be updated with filtered value
  x.xIn = 0;

  //loop through ring-buffer applying filter
  for( int j = x.offset; j < N+x.offset; j++ )
  {
    int ringdex = j % N;

    x.xIn += x.buff[ringdex]*h[ringdex];
  }
  
  //move ring-buffer offset one step
  if( x.offset++ == N-1 ){ x.offset = 0; }
  //NOTE: it checks first, then increments

  return x;  //<--struct is returned
}

///////////////////////////////////////////////////////
///////KALMAN FILTER///////////////////////////////////
///////////////////////////////////////////////////////
//
void kalmanFilter( float x3_FIR, float p2_FIR, float x4_FIR )
{
  //creating wrapped vertion of measured theta1 for KF
  float pos1Wrap = float(fmod( float(posPend1 + (float)PI), float(2*PI) ));
  if( pos1Wrap < 0 )
  {
    pos1Wrap = float(pos1Wrap + float(2*PI));
  }
  pos1Wrap = float(pos1Wrap - (float)PI);
  
  //creating wrapped vertion of measured theta2 for KF
  float pos2Wrap = float(fmod( float(posPend2 + (float)PI), float(2*PI) ));
  if( pos2Wrap < 0 )
  {
    pos2Wrap = float(pos2Wrap + float(2*PI));
  }
  pos2Wrap = float(pos2Wrap - (float)PI);
 
  //>>
  //>>>>---initialization--------------------------------
  //>>

  //variables for matrix sizes
  int c1 = 0, r1 = 0, c2 = 0, r2 = 0;
  // 
  // c1: cols in matrix 1   r1: rows in matrix 1 
  // c2: cols in matrix 2   r2: rows in matrix 2
  //
  // matrix array-vector interpretation
  // A[row][col] => A[ row *c1+ col ] = A[ vector element ]
  //
  // for matrix declarations as vectors
  // float A[r1][c1] => float A[r1*c1]

  r1 = 6, c1 = 6;
  float Q[r1*c1]  = { 0 };
  //
  //setting diagonal elements of Q
  Q[0 *c1+ 0] = 1;   Q[1 *c1+ 1] = 1;   Q[2 *c1+ 2] = 1; 
  Q[3 *c1+ 3] = 100; Q[4 *c1+ 4] = 100; Q[5 *c1+ 5] = 10; 

  float R[3*3] = { 100,   0,  0,
                     0, 100,  0,
                     0,   0, 10  };
  
//  Q[0 *c1+ 0] = 1;    Q[1 *c1+ 1] = 1;    Q[2 *c1+ 2] = 1; 
//  Q[3 *c1+ 3] = 1000; Q[4 *c1+ 4] = 1000; Q[5 *c1+ 5] = 10000; 
//
//  float R[3*3] = { 10,  0,   0,
//                    0, 10,   0,
//                    0,  0, 100  };
  if( firstRunK )
  {
    //initialize estimated states
    xEstK[0] = pos1Wrap;
    xEstK[1] = pos2Wrap;
    xEstK[2] = posCart-railOffset;
    xEstK[3] = x3_FIR;
    xEstK[4] = p2_FIR;
    xEstK[5] = x4_FIR;
  
    float PkTmp[6*6] =
{ 14.185841171390,   0.030200335462, 0.001829079170,  101.695493886294,    0.988096632836,   0.200091103028,
   0.030200335462,  14.519946416777, 0.002114160236,    0.705198836483,  111.070129716470,   0.228222661121,
   0.001829079170,   0.002114160236, 2.798279396029,    0.002494930421,    0.006615239723,   6.788717407965,
 101.695493886294,   0.705198836483, 0.002494930421, 2327.959125138203,    7.104051276564,   0.628709554973,
   0.988096632836, 111.070129716470, 0.006615239723,    7.104051276564, 2383.358258809027,   0.994982551030,
   0.200091103028,   0.228222661121, 6.788717407965,    0.628709554973,    0.994982551030, 500.159420342470 };
    for( int i = 0; i < 6*6-1; i++ ){ Pk[i] = PkTmp[i]; }
  
    firstRunK = false;
  }
  
  float y[3*1] = { pos1Wrap, pos2Wrap, posCart-railOffset };
  
  //defining discrete state space linearized around x = [ 0 0 0 0 0 0 ]';
  float A[6*6] =
  { 1.000713    , 27.51247*eN6, 0,   6.66766*eN3,   -.16497*eN6, 0         ,
   38.817329*eN6,  1.00113    , 0,   -.12727*eN6,   6.66319*eN3, 0         ,
    7.771088*eN6,  8.72165*eN6, 1, -25.47930*eN9, -52.29775*eN9, 6.6700*eN3,
    0.213968    ,  8.24961*eN3, 0,    .99929,     -49.46724*eN6, 0         ,
   11.639376*eN3,  0.34023    , 0, -38.16238*eN6,    .99795    , 0         ,
    2.330160*eN3,  2.61518*eN3, 0,  -7.63997*eN6, -15.68148*eN6, 1         };
  
  float B[6*1] = { 11.17302*eN6,
                   17.69227*eN6,
                    3.54193*eN6,
                    3.35023*eN3,
                    5.30502*eN3,
                    1.06204*eN3 };
  
  float C[3*6] =
  { 1.00035    , 13.75623*eN6, 0,   3.33383*eN3, -82.486645*eN9, 0          ,
   19.40866*eN6,  1.00056    , 0, -63.63579*eN9,   3.331598*eN3, 0          ,
    3.88554*eN6,  4.36082*eN6, 1, -12.73965*eN9, -26.148875*eN9, 3.3350*eN3};
  
  //>>
  //>>>>---prediction------------------------------------
  //>>

  /////////////////////////////////////
  //>> xPred = A*xEstK + B*u_last  <<//
  /////////////////////////////////////

  //>> A*xEstK
  float A_X_xEstK[6*1] = { 0 };
  //
  matrixMult( A_X_xEstK, A, xEstK,  6, 6, 6, 1 );

  //>> B*u_last
  float B_X_uLast[6*1] = { 0 };
  //
  float uLast[1*1] = { u_last }; //for type compatibility with matrixMult
  //
  matrixMult( B_X_uLast, B, uLast, 6, 1, 1, 1 );
  
  //>> xPred = A*xEstK + B*u_last
  float xPred[6*1] = { 0 };
  //
  matrixAdd( xPred, false, A_X_xEstK, B_X_uLast, 6, 1, 6, 1 );

  ///////////////////////////
  //>> Pk = A*Pk*A' + Q  <<//
  ///////////////////////////
  
  //>> A*Pk
  float A_X_Pk[6*6] = { 0 };
  //
  matrixMult( A_X_Pk, A, Pk, 6, 6, 6, 6 );
  
  //>> A'
  float AT[6*6] = { 0 };
  //
  matrixTranspose( AT, A, 6, 6 );
  
  //>> A*Pk*A'
  float A_X_Pk_X_AT[6*6] = { 0 };
  //
  matrixMult( A_X_Pk_X_AT, A_X_Pk, AT, 6, 6, 6, 6 );
  
  //>> Pk = A*Pk*A' + Q
  matrixAdd( Pk, false, A_X_Pk_X_AT, Q, 6, 6, 6, 6 );
  //old Pk was overwritten
  
  //>>
  //>>>>---update----------------------------------------
  //>>

  //////////////////////////////////////
  //>> K = Pk*C'*inv( C*Pk*C' + R ) <<//
  //////////////////////////////////////
  
  //>> C'
  float CT[6*3] = { 0 };
  //
  matrixTranspose( CT, C, 3, 6 );
  
  //>> Pk*C'
  float Pk_X_CT[6*3] = { 0 };
  //
  matrixMult( Pk_X_CT, Pk, CT, 6, 6, 6, 3 );
  
  //>> C*Pk
  float C_X_Pk[3*6] = { 0 };
  //
  matrixMult( C_X_Pk, C, Pk, 3, 6, 6, 6 );
  
  //>> C*Pk*C'
  float C_X_Pk_X_CT[3*3] = { 0 };
  //
  matrixMult( C_X_Pk_X_CT, C_X_Pk, CT, 3, 6, 6, 3 );
  
  //>> C*Pk*C' + R
  float C_X_Pk_X_CT_ADD_R[3*3] = { 0 };
  //
  matrixAdd( C_X_Pk_X_CT_ADD_R, false, C_X_Pk_X_CT, R, 3, 3, 3, 3 );
  
  //>> inv( C*Pk*C' + R )
  float inv_C_X_Pk_X_CT_ADD_R[3*3] = { 0 };
  //
  inv3x3Matrix( inv_C_X_Pk_X_CT_ADD_R, C_X_Pk_X_CT_ADD_R );
  
  //>> K = Pk*C'*inv( C*Pk*C' + R )
  float K[6*3]  = { 0 };
  //
  matrixMult( K, Pk_X_CT, inv_C_X_Pk_X_CT_ADD_R, 6, 3, 3, 3 );
  
  /////////////////////////////////////////
  //>> xEstK = xPred + K*(y - C*xPred) <<//
  /////////////////////////////////////////
  
  //>> C*xPred
  float C_X_xPred[3*1] = { 0 };
  //
  matrixMult( C_X_xPred, C, xPred, 3, 6, 6, 1 );
  
  //>> (y - C*xPred)
  float y_SUB_C_X_xPred[3*1] = { 0 };
  //
  matrixAdd( y_SUB_C_X_xPred, true, y, C_X_xPred, 3, 1, 3, 1 );
  
  //>> K*(y - C*xPred)
  float K_X_y_SUB_C_X_xPred[6*1] = { 0 };
  //
  matrixMult( K_X_y_SUB_C_X_xPred, K, y_SUB_C_X_xPred, 6, 3, 3, 1 );
  
  //>> xEstK = xPred + K*(y - C*xPred)
  matrixAdd( xEstK, false, xPred, K_X_y_SUB_C_X_xPred, 6, 1, 6, 1 );
  //xEstK was updated
  
  ///////////////////////////
  //>> Pk = (I - K*C)*Pk <<//
  ///////////////////////////

  //>> I
  float I[6*6]  = {0};
  r1 = 6, c1 = 6;
  I[0 *c1+ 0] = I[1 *c1+ 1] = I[2 *c1+ 2] = 1;
  I[3 *c1+ 3] = I[4 *c1+ 4] = I[5 *c1+ 5] = 1;

  //>> K*C
  float K_X_C[6*6] = { 0 };
  //
  matrixMult( K_X_C, K, C, 6, 3, 3, 6 );

  //>> (I - K*C)
  float I_NADD_K_X_C[6*6] = { 0 };
  //
  matrixAdd( I_NADD_K_X_C, 1, I, K_X_C, 6, 6, 6, 6 );

  //>> Pk = (I - K*C)*Pk
  float P[6*6] = { 0 };
  //
  matrixMult( P, I_NADD_K_X_C, Pk, 6, 6, 6, 6 );
  //
  for( int i = 0; i < 6*6; i++ ){ Pk[i] = P[i]; }
  //Pk was updated
  
  //>>
} //>>>>---Kalman filter end-----------------------------
  //>>
