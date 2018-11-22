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
int   logSwitch   = 0;

//ring-buffer and offset for FIR filter
int   offsetFIR      = 0;
//int   offsetFIR2     = 0;
float ringBuffFIR[]  = { 0, 0, 0, 0, 0};//, 0, 0, 0, 0, 0 };//, 0, 0, 0, 0, 0, 0 };
//float ringBuffFIR2[] = { 0, 0, 0, 0, 0};//, 0, 0, 0, 0, 0 };//, 0, 0, 0, 0, 0, 0 };

unsigned long current_time = 0;
unsigned long last_time    = 0;
unsigned long loop_time    = 0;
unsigned long time_now     = 0;

float I_error = 0;

// For initializing the EKF
double x_est_correction[4];
double x_init[4];
float  setOutSledNoComp = 0;
bool   first_run        = true;

// Init control
bool wait_for_position = false;
#define init_ref 0.4
#define start_angle 0.15
#define start_pos 0.1
#define stop_angle 0.6

float cart_ref = init_ref; // initial reference
float cart_ref_goal = init_ref;

unsigned long start_3 = 0;
int i = 0;

// MLC init variables
int MLC_loop_count = 0;


////////////////////////////////
///////SYSTEM SETUP/////////////
////////////////////////////////
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

  current_time = micros();
  EKF_initialize();
}

void loop()
{

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
      cart_ref = init_ref;
      
      //reset EKF
      P_correction_EKF_not_empty_init();
      
      //reset control such that it
      //waits for the correct position
      //wait_for_position = false;

      //cart_ref_goal     = init_ref;
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
      Serial.print("startLogging");
    } 
    ///////Pendulum (1 & 2) Friction Estimation////////////
    else if( input == "6" )
    {
      setOut = 6;
      time_now = micros();
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
      
      //reset control such that it
      //waits for the correct position
      //wait_for_position = false;
      
      //cart_ref          = init_ref;
      //cart_ref_goal     = init_ref;
      
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
  posSled  = sled.readPos();
  posPend1 = pend1.readPos();
  posPend1 = (posPend1 + PI);  //+PI for zero in upright
  posPend2 = -pend2.readPos(); //<--| negative-signs to get
  posPend2 = (posPend2 + PI);  //   | counter-clockwise
  velSled  = sled.readVel();   //   | positive direction
  velPend1 = pend1.readVel();  //   | convention for
  velPend2 = -pend2.readVel(); //<--| 2nd pendulum
  
  unsigned long time_stamp = micros();

  ///////'LOG STOP' AND PRINT FOR OPPERATOR////////////////
  Serial.print("\n");
  if( setOut == 0 )
  {
    if( logSwitch == 1 )
    {
      //tell python script to stop logging
      Serial.println("stopLogging");
      
      //only print stopLogging once
      logSwitch = 0;
    }

    //time converted from micro sec to sec
    float tSec = float(float(time_stamp)/1000000);  // [s]
    
    //print states for operator (system not controlled)
    int deci = 5;
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

  ///////CART REFFERENCE///////////////////////////////////
//  if( fabs(cart_ref_goal - cart_ref) > 0.001 )
//  {
//    if( (cart_ref_goal - cart_ref) > 0 )
//    {
//      cart_ref += 0.1 / 150;
//    } 
//    else
//    {
//      cart_ref -= 0.1 / 150;
//    }
//  } 
//  else
//  {
//    cart_ref = cart_ref_goal;
//  }

  ///////MODEL PARAMETERS//////////////////////////////////
  float r_pulley = 0.028;
  float K_t      = 0.0934;
  float B_c_pos  = 3.0212 - 0.5;
  float B_c_neg  = 2.7464;


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
  float h[]  = { 1/M_FIR, 1/M_FIR, 1/M_FIR, 1/M_FIR, 1/M_FIR};//, 1/M_FIR, 1/M_FIR, 1/M_FIR, 1/M_FIR, 1/M_FIR};//, 1/M_FIR, 1/M_FIR, 1/M_FIR, 1/M_FIR, 1/M_FIR, 1/M_FIR };

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

//  //-----------------position FIR---------------------------
//
//    float x2 = posSled-0.38; //<--rail center as zero
//
//    float x4 = (x2 - x2_last)/t_delta;
//    Serial.print( x4,   5 );
//    Serial.print( ", "    );
//
//    //----implementation of FIR filter (with ring buffer)----
//
//    //set window size
//    //M_FIR = 16;
//    //filter coefficients
//    //float h[]  = { 1/M_FIR, 1/M_FIR, 1/M_FIR, 1/M_FIR, 1/M_FIR, 1/M_FIR, 1/M_FIR, 1/M_FIR, 1/M_FIR, 1/M_FIR, 1/M_FIR, 1/M_FIR, 1/M_FIR, 1/M_FIR, 1/M_FIR, 1/M_FIR };
//
//    //update new mesurement in ring-buffer
//    ringBuffFIR2[offsetFIR2] = x4;
//    
//    //initializing variable to be updated with filtered value
//    x4 = 0;
//
//    //loop through ring-buffer applying filter
//    for( int j = offsetFIR2; j < M_FIR+offsetFIR2; j++ )
//    {
//      int ringDex2 = j % int(M_FIR);
//
//      x4 += ringBuffFIR2[ringDex2]*h[ringDex2];
//    }
//    
//    //move ring-buffer offset one step
//    if( offsetFIR2++ == M_FIR-1 ){ offsetFIR2 = 0; }
//    //NOTE: it checks first then increments

    x1_last = x1_FIR;
    //x2_last = x2;
    t_last  = time_stamp;
 
  /////////////////////////////////////////////////////////
  ///////STOP ALL//////////////////////////////////////////
  /////////////////////////////////////////////////////////
  if( setOut == 0 )
  {
    digitalWrite(ENABLESLED, LOW);
    setOutSled  = 0;
    setOutPend1 = 0;

    I_error     = 0; //reset integrator
  }
  /////////////////////////////////////////////////////////
  ///////SWING-UP AND SLIDING MODE/////////////////////////
  /////////////////////////////////////////////////////////
  else if( setOut == 1 )
  {
    double y_meas[2];
    y_meas[0] = posSled;
    y_meas[1] = posPend1; // Has to be in radians

    if( first_run )
    {
      x_init[0] = posSled;
      x_init[1] = posPend1; // Has to be in radians
      x_init[2] = 0;
      x_init[3] = 0;
      first_run = false;
    }

    //initialize EKF
    EKF(y_meas, setOutSledNoComp, SAMPLINGTIME, x_init, x_est_correction);

    //Model Parameters
    float r_pulley = 0.028;
    float K_t      = 0.0934;
    float B_c_neg  = 2.7464;
    float B_v_pos  = 1.936;
    float B_v_neg  = 1.422;
    float B_v_c    = B_v_pos;
    float l        = 0.3348;      //0.3235;
    float m_c      = 5.273+1.103;
    float m_b      = 0.201;
    float g        = 9.81;
    float B_v_p    = 0.0004;
    float F_c_p    = 0.004;
    float k_tanh   = 250;
    
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

    //enable cart motor output    
    digitalWrite(ENABLESLED, HIGH);
    
    //change coordinate convention
    float x1 = x_est_correction[1];
    float x2 = x_est_correction[0]-0.38; //<--rail center as zero
    float x3 = x_est_correction[3];
    float x4 = x_est_correction[2];
    
    //creating wrapped vertion of angle for sliding mode
    float x1Wrap = float(fmod( float(x1 + PI), float(2*PI) ));
    if( x1Wrap < 0 )
    {
      x1Wrap = float(x1Wrap + float(2*PI));
    }
    x1Wrap = float(x1Wrap - PI);
    x1 = x1Wrap;
    
    //time converted from micro sec to sec
    float tSec = float(float(time_stamp)/1000000);  // [s] 
    

    //set friction based on velocity direction
    if( (x4 > 0) || ((x4 == 0) && (setOutSled > 0)) )
    {
      B_v_c = B_v_pos;
    }
    else if( (x4 < 0) || ((x4 == 0) && (setOutSled < 0)) )
    {
      B_v_c = B_v_neg;
    }
 
    //printing for data collection
    float deci = 5; 
    Serial.print( tSec,   deci );
    Serial.print( ", "         );
    Serial.print( x1,     deci );
    Serial.print( ", "         );
    Serial.print( x2,     deci );
    Serial.print( ", "         );
    Serial.print( x3,     deci );
    Serial.print( ", "         );
    Serial.print( x4,     deci );
    Serial.print( ", "         );
    Serial.print( x3_FIR, deci );

    /////////////////////////////////////////////////////////
    ///////CATCH - SLIDING MODE//////////////////////////////
    /////////////////////////////////////////////////////////
    //
    if(  abs(x1Wrap) < catchAngle  )
    {
      //set wider catch angle to stay in sliding mode after wing-up sequence 
      catchAngle = 0.2;
      
      //inverse of function on output
      float g_b_inv = m_c + m_b - m_b*cos(x1)*cos(x1);
      
      //sliding manifold
      float s = x4 - k2*(x3 - (x4*cos(x1))/l) + k1*x1 + k3*x2;
      
      //saturation function
      float z = s / epsilon;
      if (z > 1)
      {
        sat = 1;
      }
      else if (z < -1)
      {
        sat = -1;
      }
      else
      {
        sat = z;
      }

      //gain for optional extra x-control
      float k_lin[] = { 10.5460 , 15.8190 };

      //optional linear position (x) controller
      float lin_u = float( -k_lin[1]*x2 -k_lin[1]*x4 );
      
      //final control
      float u = - sat*beta*g_b_inv;// + lin_u;
      
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
      //set narrow catch angle to provide
      //best handover to sliding mode
      catchAngle = 0.01;
      
      //energy control gain
      float k = 110+90; //200;

      //sign-function
      float sgnIn = cos(x1_FIR)*x3_FIR;
      float sgn   = 1;
      //
      if( sgnIn >= 0 )
      {
        sgn = 1;
      }
      else if( sgnIn < 0 )
      {
        sgn = -1;
      }
      
      //calculate maximum acceleration of cart
      float i_max = 4.58+1;
      float u_max = i_max*K_t/r_pulley;
      float a_max = u_max/(m_c+m_b);
      
      //extra energy offset from equilibrium to get fast catch
      float E_off = -.02-.01; // -.016;
      
      //energy error
      float E_delta = .5*m_b*l*l*x3*x3 + m_b*g*l*(cos(x1_FIR) - 1) + E_off;
      
      //energy control law (acceleration of cart)
      float a_c = -k*E_delta*sgn;
      
      if( a_c > a_max )
      {
        a_c = a_max;
      }
      else if( a_c < -a_max )
      {
        a_c = -a_max;
      }
      
      //estimation of needed actuation to achieve cart acceleration, a_c
      float theta_acc_est = ( m_c + m_b )*( -B_v_p*x3_FIR -tanh(k_tanh*x3_FIR)*F_c_p + m_b*g*l*sin(x1_FIR) )/( l*l*m_b*(m_c + m_b - m_b*cos(x1_FIR)*cos(x1_FIR)) ) + ( cos(x1_FIR)*(u_last - m_b*l*sin(x1_FIR)*x3_FIR*x3_FIR) )/( l*(m_c + m_b - m_b*cos(x1_FIR)*cos(x1_FIR)) );
      
      //gain for x-control
      float k_lin[] = { 10.5460 , 15.8190 };
      
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
    
    //reduced notation for friction compensation
    //x4 = x_est_correction[2];
    
    //friction compensation
    setOutSledNoComp = setOutSled;
    if ((x4 > 0) || ((x4 == 0) && (setOutSled > 0)))
    {
      setOutSled = setOutSled + r_pulley / K_t * (B_c_neg);
    }
    else if ((x4 < 0) || ((x4 == 0) && (setOutSled < 0)))
    {
      setOutSled = setOutSled + r_pulley / K_t * (B_c_neg) * (-1);
    }
    else
    {
      setOutSled = 0;
    } 
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
        logSwitch = 1; //when time is up and the cart is
        setOut    = 0; //no longer moving, return to complete stop,
      }                //where logging is also stoped
    }
    
    //print data to log 
    int deci = 5;
    Serial.print( tSec,       deci );
    Serial.print( ", "             );
    Serial.print( setOutSled, deci );
    Serial.print( ", "             );
    Serial.print( posSled,    deci );
    Serial.print( ", "             );
    Serial.print( velSled,    deci );
    //<<
  } //<<<<CART FRICTION AND MASS ESTIMATION <END<
    //<<
  /////////////////////////////////////////////////////////
  ///////PENDULUM FRICTION ESTIMATION//////////////////////
  /////////////////////////////////////////////////////////
  else if( setOut == 6 )
  {
    float tSec = float (time_stamp-time_now)/1000000;
    
    //select number of logged decimals
    int deci      = 5;
    
    //select pendulum to test
    int pend1test = 0;
    int pend2test = 1;
    
    //when pendulum under test falls from 
    //upright equilibrium, print data to log 
    if( pend1test && posPend1 > 0.01 )
    {
      Serial.print( tSec,     deci );
      Serial.print( ", "           );
      Serial.print( posPend1, deci );
      Serial.print( ", "           );
      Serial.print( velPend1, deci );
      Serial.print( "\n"           );
    }
    else if( pend2test && posPend2 > 0.01 )
    {
      Serial.print( tSec,     deci );
      Serial.print( ", "           );
      Serial.print( posPend2, deci );
      Serial.print( ", "           );
      Serial.print( velPend2, deci );
      Serial.print( "\n"           );
    }
    //<<
  } //<<<<PENDULUM FRICTION ESTIMATION <END<
    //<<
  
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

//sign-function
int sign( float angle )
{
  if( angle > 0 )
  {
    return 1;
  } 
  else if( angle < 0 )
  {
    return -1;
  } 
  else
  {
    return 0;
  }
}
