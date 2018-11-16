// before upload:

// (1) remember to add libraries containing the below three .h files:
//    Sketch - Import Library - Add Library
// the libraries will be placed in
//  Documents\Arduino\libraries
// NB: when a file is changed the corresponding library has to been added again

// (2) Remember
// Tools - Board - Teensy 3.6 (Programming Port)
// Tools - Port - COM# (Teensy 3.6 (Programming Port))
// check that  "Serial.begin(115200)" below agree with baud on terminal

// (3) remember: max voltage 50, max current 5

#include <Joint_teensy.h> // main file
//#include <Utility.h> // sat and sign function
//#include <looptime.h> // set time in loop

// EKF libraries
#include "EKF.h"
#include "EKF_initialize.h"

// MLC libraries
#include "run_mlc.h"
#include "run_mlc_initialize.h"

//#include <math.h>

// Definitions
#define SAMPLINGTIME 0.00667 // s
#define ENABLEPENDUL 11//50
#define ENABLESLED 12//48

// Create joint objects
Joint sled(1, SAMPLINGTIME);
Joint pend1(2, SAMPLINGTIME);
Joint pend2(3, SAMPLINGTIME);

// Global variables
float setOutSled = 0;
float setOutPend1 = 0;
float velSled = 0;
float velPend1 = 0;
float velPend2 = 0;
float posSled;
float posPend1;
float posPend2;
int   setOut = 0;

float t_last  = 0;
float x1_last = 0;
float u_last  = 0;
float x1_old  = 0;
float catchAngle = 0.01;

unsigned long current_time = 0;
unsigned long last_time = 0;
unsigned long  loop_time = 0;
unsigned long time_now = 0;

int logSwitch = 0;

float I_error = 0;

// For initializing the EKF
bool first_run = true;
double x_est_correction[4];
double x_init[4];
float setOutSledNoComp = 0;

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


// Setup system
void setup() {
  ////////////////////////
  // Setup Serial
  ///////////////////////
  //Serial.begin(9600);
  Serial.begin(115200); // to be able to do Serial.print and still have time in loop
  ////////////////////////
  // Setup pin modes
  ////////////////////////
  // Data bus
  pinMode(25, INPUT);
  pinMode(26, INPUT);
  pinMode(27, INPUT);
  pinMode(28, INPUT);
  pinMode(29, INPUT);
  pinMode(30, INPUT);
  pinMode(31, INPUT);
  pinMode(32, INPUT);

  ///////////////////////
  // Setup pendulum system
  //////////////////////
  // Initialize joints
  sled.init();
  pend1.init();
  pend2.init();
  // Reset positions
  sled.resetPos();
  pend1.resetPos();
  pend2.resetPos();
  // Set DAC resolution
  analogWriteResolution(12);
  analogReadResolution(12);
  // Set output to zero
  sled.setOutput(0,1);
  pend1.setOutput(0,1);
  // setup Enable motor driver outputs (FOR SAFTY REASONS, DO NOT CHANGE THE BELOW 6 LINES)
  digitalWrite(ENABLEPENDUL, LOW);
  digitalWrite(ENABLESLED, LOW);
  pinMode(ENABLEPENDUL, OUTPUT);
  pinMode(ENABLESLED, OUTPUT);
  digitalWrite(ENABLEPENDUL, LOW);
  digitalWrite(ENABLESLED, LOW);

  current_time = micros();
  EKF_initialize();
  run_mlc_initialize();

}

void loop() {
  ////////////////////////////////////////////
  // Input from user on serial interface
  ////////////////////////////////////////////

  //int input;
  String input;
  if (Serial.available() > 0) {
    //input = Serial.read();
    input = Serial.readStringUntil('\n');
    // stop all
    if (input == "0") {
      // stop motor
      setOut = 0;
      cart_ref = init_ref;
      P_correction_EKF_not_empty_init(); // Reset EKF
      wait_for_position = false; // Reset control such that it waits for the correct position
      cart_ref_goal = init_ref;
      //Serial.println("Input = 0");
    } 
    else if (input == "1") {
      // Sliding mode control with constant beta
      setOut = 1;
      time_now = micros();
      Serial.print("\n");
      Serial.print("startLogging");      
      //Serial.println("Input = 1");
    } 
    else if (input == "2") {
      // Sliding mode control with constant beta
      setOut = 2;
      //Serial.println("Input = 2");
    } 
    else if (input == "3") {
      // Sliding mode control with varying beta
      setOut = 3;
      //Serial.println("Input = 3");
    } 
    else if (input == "4") {
      // Machine learning control
      setOut = 4;
      i = 0;
      //Serial.println("Input = 4");
    } 
    else if (input == "5") {
      // Cart Mass and Friction Estimation
      setOut = 5;
      time_now = micros();
      Serial.print("startLogging");
      //Serial.println("Input = 5");
    } 
    else if (input == "6") {
      // Pendulum (1 and 2) Friction Estimation
      setOut = 6;
      time_now = micros();
      Serial.print("startLogging");
      //Serial.println("Input = 6");
    } 
    else if (input == "r") {
      // Reset variables
      pend1.resetPos();
      pend2.resetPos();
      sled.resetPos();

      P_correction_EKF_not_empty_init(); // Reset EKF
      wait_for_position = false; // Reset control such that it waits for the correct position
      cart_ref = init_ref;
      cart_ref_goal = init_ref;

      Serial.println("Input = r");
    } 
    else if (input.startsWith("ref=")) {
      // change reference
      int ref_pos = input.indexOf('=');
      String ref_string = input.substring(ref_pos + 1);
      cart_ref_goal = ref_string.toFloat();
      //Serial.print("Cart reference changed to ref=");
      //Serial.println(ref_string);
    } 
    else {
      Serial.println("ERROR: Wrong command selected.");
    }
    /*digitalWrite(ENABLESLED,HIGH);
     int tmp = input-'0';
     Serial.println(-tmp*100);
     setOutSled = -tmp*100;*/
  }

  ////////////////////////////////////////////
  // Read positions
  ////////////////////////////////////////////

  posSled  = sled.readPos();
  posPend1 = pend1.readPos();
  posPend1 = (posPend1 + PI);  //+PI for zero in upright
  posPend2 = -pend2.readPos(); //<--| negative-signs to get
  posPend2 = (posPend2 + PI);  //   | counter-clockwise
  velSled  = sled.readVel();   //   | positive direction
  velPend1 = pend1.readVel();  //   | convention for
  velPend2 = -pend2.readVel(); //<--| 2nd pendulum
  
  unsigned long time_stamp = micros();

  Serial.print("\n");
  if( setOut == 0 )
  {
    if( logSwitch == 1 )
    {
      Serial.println("stopLogging");
      logSwitch = 0;
    }

    float tSec = float(float(time_stamp)/1000000);  // [s]
    
    Serial.print( tSec       ,2 );
    Serial.print( ", "          );
    Serial.print( posPend1   ,2 );
    Serial.print( ", "          );
    Serial.print( posSled    ,2 );
    Serial.print( ", "          );
    Serial.print( velPend1   ,2 );
    Serial.print( ", "          );
    Serial.print( velSled    ,2 );
  } 

  ////////////////////////////////////////////
  // Apply control
  ////////////////////////////////////////////

  if (fabs(cart_ref_goal - cart_ref) > 0.001) {
    if ((cart_ref_goal - cart_ref) > 0) {
      cart_ref += 0.1 / 150;
    } 
    else {
      cart_ref -= 0.1 / 150;
    }
  } 
  else {
    cart_ref = cart_ref_goal;
  }
  // ---------------Model parameters------------------------//
  float r_pulley = 0.028;
  float K_t = 0.0934;
  float B_c_pos = 3.0212 - 0.5;
  float B_c_neg = 2.7464;
  //float B_v_pos = 1.936;
  /*float B_v_neg = 1.422;
   float B_v_c = B_v_pos;
   float l = 0.3235;
   float m_c = 5.273;
   float m_b = 0.201;
   float g = 9.81;
   float B_v_p = 0.0004;
   float F_c_p = 0.004;
   float k_tanh = 250;*/

///////////CONTROL CODE/////////////////////////////////////////////

  // stop all
  if (setOut == 0) {
    //-----------FORCE TEST--------------
    //  setOutSled = 0;
    //  digitalWrite(ENABLESLED, HIGH);
    //  setOutSled = 1.3;
    //-----------------------------------
    digitalWrite(ENABLESLED, LOW);
    setOutSled = 0;
    setOutPend1 = 0;
    I_error = 0; // Reset integrator
  }
  // activate Sliding mode control
  else if (setOut == 1)
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

    // EKF(y_meas,Ia,t_s,x_init)
    EKF(y_meas, setOutSledNoComp, SAMPLINGTIME, x_init, x_est_correction);

    // Should only print state estimates if the EKF is active
    //Serial.print("EKF: ");
//    Serial.print(x_est_correction[0], 5);
//    Serial.print(",");
//    Serial.print(x_est_correction[1], 5);
//    Serial.print(",");
//    Serial.print(x_est_correction[2], 5);
//    Serial.print(",");
//    Serial.print(x_est_correction[3], 5);
//    Serial.print(",");
 
//  //angle and velocity reading from second pendulum
//  Serial.print("pendul 2: ");
//  Serial.print(posPend2,2);   //position is 
//  Serial.print("  ");
//  Serial.println(velPend2,2);

    // ---------------Model parameters------------------------//
    float r_pulley = 0.028;
    float K_t = 0.0934;
    //float B_c_pos = 3.0212 - 0.5;
    float B_c_neg = 2.7464;
    float B_v_pos = 1.936;
    float B_v_neg = 1.422;
    float B_v_c = B_v_pos;
    float l = 0.3348; //0.3235;
    float m_c = 5.273+1.103;
    float m_b = 0.201;
    float g = 9.81;
    float B_v_p = 0.0004;
    float F_c_p = 0.004;
    float k_tanh = 250;
    
    // ------------------sliding mode parameters-----------------//
    float sat = 0;

    float k_s[] = { 7.3918 ,  -1.3414 ,  -5.5344 };

    float k1 = k_s[0], k2 = k_s[1], k3 = k_s[2];

    float rho     = 6.2;//7.135;
    float beta_0  = .1;
    float beta    = rho + beta_0;
    float epsilon = 0.03;
    
    // ---------------------Friction determiner ------------------//
    if (((x_est_correction[2]) > 0) || (((x_est_correction[2]) == 0) && (setOutSled > 0))) {
      B_v_c = B_v_pos;
    }
    else if (((x_est_correction[2]) < 0) || (((x_est_correction[2]) == 0) && (setOutSled < 0))) {
      B_v_c = B_v_neg;
    }

    // Insert control here
    setOutSled = 0;
    float u = 0;
    digitalWrite(ENABLESLED, HIGH);
    
    //  CHANGE OF COOREDINATE CONVENSION
    float x1 = x_est_correction[1];
    float x2 = x_est_correction[0]-0.38;
    float x3 = x_est_correction[3];
    float x4 = x_est_correction[2];
   
    Serial.print(x1, 2);
    Serial.print(", ");
    Serial.print(x2, 2);
    Serial.print(", ");
    Serial.print(x3, 2);
    Serial.print(", ");
    Serial.print(x4, 2);
    
    ///////CATCH//////////////////////////////////////////////////////////////
    if( ( abs(posPend1) < catchAngle ) || ( abs(posPend1) > 2*PI-catchAngle ) )
    {
      // Sliding mode start
      //Serial.println("\n\nslide");
      
      catchAngle = 0.2;
      
      //change angle reff if approaching on far side
      if( abs(x1) > 2*PI-catchAngle )
      {
        x1_old = x1;
        x1 = x1-2*PI;
      }
      else
      {
        x1_old = 0;
      }

      float g_b_inv = m_c + m_b - m_b*cos(x1)*cos(x1);
      float s       = x4 - k2*(x3 - (x4*cos(x1))/l) + k1*x1 + k3*x2;
      
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

      float k_lin[] = { 10.5460 , 15.8190 };

      float lin_u = float( -k_lin[1]*x2 -k_lin[1]*x4 );
      
      float u = - sat*beta*g_b_inv;// + lin_u;
      
      setOutSled = u*r_pulley/K_t;

      //set current limit peak to i_a = +-6
//      if( abs(setOutSled) > 6 )
//      {
//        if( setOutSled > 0 )
//        {
//          setOutSled = 6;
//        }
//        else if( setOutSled < 6 )
//        {
//          setOutSled = -6;
//        }
//      }

      if( x1_old )
      {
        x1 = x1_old;
      }

      // Sliding mode end

    } 
    /////////SWING-UP////////////////////////////////////////////////////////
    else //if(0)
    {
      //Serial.println("\n\nswing");
      
      catchAngle = 0.01;

      float t_delta = (time_stamp-t_last)/1000000;
      
      x1 = posPend1;
      //nummerical diff
      x3 = (x1 - x1_last)/t_delta;
      
      float k = 110;//200;

      float sgnIn = cos(x1)*x3;
      
      float sgn = 1;
      
      if( sgnIn >= 0 )
      {
        sgn = 1;
      }
      else if( sgnIn < 0 )
      {
        sgn = -1;
      }

      float i_max = 4.58;
      float u_max = i_max*K_t/r_pulley;
      float a_max = u_max/(m_c+m_b) -.1;
      
      float E_delta = .5*m_b*l*l*x3*x3 + m_b*g*l*(cos(x1) - 1) -.02;// -.016;
      
      float a_c = -k*E_delta*sgn;
      
      if( a_c > a_max )
      {
        a_c = a_max;
      }
      else if( a_c < -a_max )
      {
        a_c = -a_max;
      }
      
      float theta_acc_est = ( m_c + m_b )*( -B_v_p*x3 -tanh(k_tanh*x3)*F_c_p + m_b*g*l*sin(x1) )/( l*l*m_b*(m_c + m_b - m_b*cos(x1)*cos(x1)) ) + ( cos(x1)*(u_last - m_b*l*sin(x1)*x3*x3) )/( l*(m_c + m_b - m_b*cos(x1)*cos(x1)) );
      
      float k_lin[] = { 10.5460 , 15.8190 };
      
      float lin_u =  -k_lin[1]*x2 -k_lin[1]*x4;
      
      u = ( m_c + m_b )*a_c + m_b*l*sin(x1)*x3*x3 -m_b*l*cos(x1)*theta_acc_est + lin_u;
      
      setOutSled = u*r_pulley/K_t;
      
      u_last = u;
      
      x1_last = x1;
      t_last  = time_stamp;
      
    }
    
    // ---------------------Compensation ------------------//
    setOutSledNoComp = setOutSled;
    if (((x_est_correction[2]) > 0) || (((x_est_correction[2]) == 0) && (setOutSled > 0))) {
      setOutSled = setOutSled + r_pulley / K_t * (B_c_neg); //tanh(250*x_est_correction[2]);
    }
    else if (((x_est_correction[2]) < 0) || (((x_est_correction[2]) == 0) && (setOutSled < 0))) {
      setOutSled = setOutSled + r_pulley / K_t * (B_c_neg) * (-1); //tanh(250*x_est_correction[2]);
    }
    else {
      setOutSled = 0;
    }
  }
  ////////PREVIOUS GROUPS///////////////////////////////////////////////////
  else if (setOut == 2) {

    if (posSled > cart_ref - start_pos && posSled < cart_ref + start_pos && posPend1 > -start_angle && posPend1 < start_angle) { // Check angle, 0.2 rad = 11.5
      wait_for_position = true;
    }
    else if (wait_for_position == false) {
      Serial.println(" ");
      Serial.println("Move the cart to the middle of the rail and lift the pendulum approximately straight up in the clockwise direction");
      Serial.println(" ");
    }
    if (wait_for_position == true) { // Pendulum should now be pointing straight up and start balancing around the middle of the rail

      double y_meas[2];
      y_meas[0] = posSled;
      y_meas[1] = posPend1; // Has to be in radians

      if (first_run) {
        x_init[0] = posSled;
        x_init[1] = posPend1; // Has to be in radians
        x_init[2] = 0;
        x_init[3] = 0;
        first_run = false;
      }

      // EKF(y_meas,Ia,t_s,x_init)
      EKF(y_meas, setOutSledNoComp, SAMPLINGTIME, x_init, x_est_correction);

      // Should only print state estimates if the EKF is active
      //Serial.print("EKF: ");
//      Serial.print(x_est_correction[0], 5);
//      Serial.print(",");
//      Serial.print(x_est_correction[1], 5);
//      Serial.print(",");
//      Serial.print(x_est_correction[2], 5);
//      Serial.print(",");
//      Serial.print(x_est_correction[3], 5);
//      Serial.print(",");


      // ---------------Model parameters------------------------//
      float r_pulley = 0.028;
      float K_t = 0.0934;
      //float B_c_pos = 3.0212;
      float B_c_neg = 2.7464;
      float B_v_pos = 1.936;
      float B_v_neg = 1.422;
      float B_v_c = B_v_pos;
      float l = 0.3235;
      float m_c = 5.273;
      float m_b = 0.201;
      /*float g = 9.81;
       float B_v_p = 0.0004;
       float F_c_p = 0.004;
       float k_tanh = 250;*/

      // ------------------sliding mode parameters-----------------//
      float sat = 0;
      //For u=-inv(gamma)(beta_0*sat(s)):
      //float k1 = -4.59 - 25, k2 = 15.8 + 13, k3 = -1.46 - 0.5;
      float k1 = -30, k2 = 29, k3 = -2.2;
      float beta_0 = 10;
      float epsilon = 0.035;

      // ---------------------Friction determiner ------------------//
      if (((x_est_correction[2]) > 0) || (((x_est_correction[2]) == 0) && (setOutSled > 0))) {
        B_v_c = B_v_pos;
      }
      else if (((x_est_correction[2]) < 0) || (((x_est_correction[2]) == 0) && (setOutSled < 0))) {
        B_v_c = B_v_neg;
      }

      // Insert control here
      setOutSled = 0;
      digitalWrite(ENABLESLED, HIGH);
      if (posPend1 < stop_angle && posPend1 > -stop_angle) { //0.3 rad = 17 deg
        /*
            Sliding mode control here
         variables to use, cart_ref, x_est_correction,
         */
        // Sliding mode start
        float ga4 = K_t / r_pulley * cos(x_est_correction[1]) / (l * (m_c + m_b * sin(x_est_correction[1]) * sin(x_est_correction[1])));
        float gamma = ga4;
        float s = x_est_correction[3] + k1 * (x_est_correction[0] - cart_ref) + k2 * x_est_correction[1] + k3 * (x_est_correction[2] * cos(x_est_correction[1]) / l - x_est_correction[3]);
        float z = s / epsilon;
        if (z > 1) {
          sat = 1;
        } 
        else if (z < -1) {
          sat = -1;
        } 
        else {
          sat = z;
        }
        setOutSled = -1 / (gamma) * ((beta_0) * sat);
        // Sliding mode end

      } 
      else { // Safety trigger, if the angle gets too large, it cannot catch the pendulum again
        setOut = 0;
        setOutSled = 0;
        P_correction_EKF_not_empty_init(); // Reset EKF
        wait_for_position = false; // Reset control such that it waits for the correct position
        cart_ref = init_ref;
        cart_ref_goal = init_ref;
        first_run = true;
        Serial.println("Safety angle exceeded - Turning off controllers");
      }

      // ---------------------Compensation ------------------//
      setOutSledNoComp = setOutSled;
      if (((x_est_correction[2]) > 0) || (((x_est_correction[2]) == 0) && (setOutSled > 0))) {
        setOutSled = setOutSled + r_pulley / K_t * (B_c_neg); //tanh(250*x_est_correction[2]);
      }
      else if (((x_est_correction[2]) < 0) || (((x_est_correction[2]) == 0) && (setOutSled < 0))) {
        setOutSled = setOutSled + r_pulley / K_t * (B_c_neg) * (-1); //tanh(250*x_est_correction[2]);
      }
      else {
        setOutSled = 0;
      }
    }
  }
  // activate Sliding mode control
  else if (setOut == 3) {

    if (posSled > cart_ref - start_pos && posSled < cart_ref + start_pos && posPend1 > -start_angle && posPend1 < start_angle) { // Check angle, 0.2 rad = 11.5
      wait_for_position = true;
    }
    else if (wait_for_position == false) {
      Serial.println(" ");
      Serial.println("Move the cart to the middle of the rail and lift the pendulum approximately straight up in the clockwise direction");
      Serial.println(" ");
    }
    if (wait_for_position == true) { // Pendulum should now be pointing straight up and start balancing around the middle of the rail

      double y_meas[2];
      y_meas[0] = posSled;
      y_meas[1] = posPend1; // Has to be in radians

      if (first_run) {
        x_init[0] = posSled;
        x_init[1] = posPend1; // Has to be in radians
        x_init[2] = 0;
        x_init[3] = 0;
        first_run = false;
      }

      // EKF(y_meas,Ia,t_s,x_init)
      EKF(y_meas, setOutSledNoComp, SAMPLINGTIME, x_init, x_est_correction);

      // Should only print state estimates if the EKF is active
      //Serial.print("EKF: ");
//      Serial.print(x_est_correction[0], 5);
//      Serial.print(",");
//      Serial.print(x_est_correction[1], 5);
//      Serial.print(",");
//      Serial.print(x_est_correction[2], 5);
//      Serial.print(",");
//      Serial.print(x_est_correction[3], 5);
//      Serial.print(",");

      // ---------------Model parameters------------------------//
      float r_pulley = 0.028;
      float K_t = 0.0934;
      //float B_c_pos = 3.0212 - 0.5;
      float B_c_neg = 2.7464;
      float B_v_pos = 1.936;
      float B_v_neg = 1.422;
      float B_v_c = B_v_pos;
      float l = 0.3235;
      float m_c = 5.273;
      float m_b = 0.201;
      float g = 9.81;
      float B_v_p = 0.0004;
      float F_c_p = 0.004;
      float k_tanh = 250;

      // ------------------sliding mode parameters-----------------//
      float sat = 0;
      // For u = -inv(gamma)(omega+beta_0)*sat(s), with new manifold differentiation
      //float k1 = -4.59 - 25, k2 = 15.8 + 13, k3 = -1.46 - 0.5;
      float k1 = -30, k2 = 29, k3 = -2.2;
      float beta_0 = 9;
      float epsilon = 0.055;


      // ---------------------Friction determiner ------------------//
      if (((x_est_correction[2]) > 0) || (((x_est_correction[2]) == 0) && (setOutSled > 0))) {
        B_v_c = B_v_pos;
      }
      else if (((x_est_correction[2]) < 0) || (((x_est_correction[2]) == 0) && (setOutSled < 0))) {
        B_v_c = B_v_neg;
      }

      // Insert control here
      setOutSled = 0;
      digitalWrite(ENABLESLED, HIGH);
      if (posPend1 < stop_angle && posPend1 > -stop_angle) { //0.3 rad = 17 deg
        /*
            Sliding mode control here
         variables to use, cart_ref, x_est_correction,
         */
        // Sliding mode start
        float F4 = -(cos(x_est_correction[1]) * (B_v_c * x_est_correction[2] + m_b * l * sin(x_est_correction[1]) * x_est_correction[3] * x_est_correction[3])) / (l * (m_c + m_b * (sin(x_est_correction[1])) * (sin(x_est_correction[1])))) - ((m_c + m_b) * (B_v_p * x_est_correction[3] + F_c_p * sign(k_tanh * x_est_correction[3]) - m_b * l * g * sin(x_est_correction[1]))) / (m_b * l * l * (m_c + m_b * (sin(x_est_correction[1])) * (sin(x_est_correction[1]))));
        float ga4 = K_t / r_pulley * cos(x_est_correction[1]) / (l * (m_c + m_b * sin(x_est_correction[1]) * sin(x_est_correction[1])));
        float fc2 = (cos(x_est_correction[1]) * (B_v_p * x_est_correction[3] + F_c_p * sign(k_tanh * x_est_correction[3]) - m_b * l * g * sin(x_est_correction[1]))) / (l * (m_c + m_b * (sin(x_est_correction[1])) * (sin(x_est_correction[1]))));
        float omega = F4 + k1 * x_est_correction[2] + k2 * x_est_correction[3] + k3 * (-x_est_correction[2] * x_est_correction[3] * sin(x_est_correction[1]) / l + fc2 / (l) * (1 + m_c / m_b + cos(x_est_correction[1]) * cos(x_est_correction[1])) / cos(x_est_correction[1]));
        float gamma = ga4;
        float s = x_est_correction[3] + k1 * (x_est_correction[0] - cart_ref) + k2 * x_est_correction[1] + k3 * (x_est_correction[2] * cos(x_est_correction[1]) / l - x_est_correction[3]);
        float z = s / epsilon;
        if (z > 1) {
          sat = 1;
        } 
        else if (z < -1) {
          sat = -1;
        } 
        else {
          sat = z;
        }
        setOutSled = -1 / (gamma) * ((fabs(omega) / 3 + beta_0) * sat); // Output set from the sliding mode control

        // Sliding mode end
      } 
      else { // Safety trigger, if the angle gets too large, it cannot catch the pendulum again
        setOut = 0;
        setOutSled = 0;
        P_correction_EKF_not_empty_init(); // Reset EKF
        wait_for_position = false; // Reset control such that it waits for the correct position
        cart_ref = init_ref;
        cart_ref_goal = init_ref;
        first_run = true;
        Serial.println("Safety angle exceeded - Turning off controllers");
      }

      // ---------------------Compensation ------------------//
      setOutSledNoComp = setOutSled;
      if (((x_est_correction[2]) > 0) || (((x_est_correction[2]) == 0) && (setOutSled > 0))) {
        setOutSled = setOutSled + r_pulley / K_t * (B_c_neg); //tanh(250*x_est_correction[2]);
      }
      else if (((x_est_correction[2]) < 0) || (((x_est_correction[2]) == 0) && (setOutSled < 0))) {
        setOutSled = setOutSled + r_pulley / K_t * (B_c_neg) * (-1); //tanh(250*x_est_correction[2]);
      }
      else {
        setOutSled = 0;
      }
    }
  }
  // activate Machine Learning control
  else if (setOut == 4) {
    digitalWrite(ENABLESLED, HIGH);
    if (posSled > cart_ref - start_pos && posSled < cart_ref + start_pos && posPend1 > -start_angle && posPend1 < start_angle && wait_for_position==false) { // Check angle, 0.2 rad = 11.5
      wait_for_position = true;
      MLC_loop_count = 1;
    }
    else if (wait_for_position == false) {
      Serial.println(" ");
      Serial.println("Move the cart to the middle of the rail and lift the pendulum approximately straight up in the clockwise direction");
      Serial.println(" ");
    }
    if (wait_for_position == true) { // Pendulum should now be pointing straight up and start balancing around the middle of the rail

      if (posPend1 < stop_angle && posPend1 > -stop_angle) { //0.3 rad = 17 deg
        /*
          Machine learning based control should be put into this case
         */

        // Kalman filter
        double y_meas[2];
        y_meas[0] = posSled;
        y_meas[1] = posPend1; // Has to be in radians

        if (first_run) {
          x_init[0] = posSled;
          x_init[1] = posPend1; // Has to be in radians
          x_init[2] = 0;
          x_init[3] = 0;
          first_run = false;
        }

        // ---------------------Inverse compensation ------------------//
        if (((x_est_correction[2]) > 0) || (((x_est_correction[2]) == 0) && (setOutSled > 0))) {
          setOutSledNoComp = setOutSled - r_pulley / K_t * (B_c_pos); //tanh(250*x_est_correction[2]);
        }
        else if (((x_est_correction[2]) < 0) || (((x_est_correction[2]) == 0) && (setOutSled < 0))) {
          setOutSledNoComp = setOutSled - r_pulley / K_t * (B_c_neg) * (-1); //tanh(250*x_est_correction[2]);
        }
        else {
          setOutSledNoComp = 0;
        }
        // EKF(y_meas,Ia,t_s,x_init)
        EKF(y_meas, setOutSledNoComp, SAMPLINGTIME, x_init, x_est_correction);

        // adjust sampling frequency to run at lower for MLC
        if (MLC_loop_count == 3){
          //x_est_correction[0]=x_est_correction[0]-0.4;
          setOutSled = run_mlc(x_est_correction);
          //x_est_correction[0]=x_est_correction[0]+0.4;
          MLC_loop_count = 1;
        }
        else {
          MLC_loop_count++;
        } 
      } 
      else { // Safety trigger, if the angle gets too large, it cannot catch the pendulum again
        setOut = 0;
        setOutSled = 0;
        cart_ref = init_ref;
        cart_ref_goal = init_ref;
        P_correction_EKF_not_empty_init(); // Reset EKF
        wait_for_position = false;
        Serial.println("Safety angle exceeded - Turning off controllers");
      }
    }
  }
  /////////////////////////////////////////
  ////CART FRICTION AND MASS ESTIMATION////
  /////////////////////////////////////////
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
    
    int deci = 5;
    Serial.print( tSec,       deci );
    Serial.print( ", "             );
    Serial.print( setOutSled, deci );
    Serial.print( ", "             );
    Serial.print( posSled,    deci );
    Serial.print( ", "             );
    Serial.print( velSled,    deci );
  }
  ////////////////////////////////////
  ////PENDULUM FRICTION ESTIMATION////
  ////////////////////////////////////
  else if( setOut == 6 )
  {
    float tSec = float (time_stamp-time_now)/1000000;
    
    int deci      = 5;
    int pend1test = 0;
    int pend2test = 1;
    
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
  }
   
  ////////////////////////////////////////////
  // Set outputs
  ////////////////////////////////////////////

  sled.setOutput(setOutSled,1);
  //sled.setOutput(setOutSledNoComp,1);

//  Serial.print(posSled, 5);
//  Serial.print(",");
//  Serial.print(posPend1, 5);
//  Serial.print(",");
//  Serial.println(time_stamp, DEC);
//  Serial.print(",");
//  Serial.print(setOutSledNoComp, 5); // setOutSledNoComp control without compensation
//  Serial.print(",");
//  Serial.println(setOutSled, 5); // setOutSledNoComp control without compensation

  //if(posSled<0.05||posSled>0.72){
  //  setOut=0;}

  while (current_time + SAMPLINGTIME * 1000000 > micros()) { // Ensures correct sampling time
    // burn time
  }
  //Serial.println(micros()-current_time); //loop time
  if (micros() - current_time > SAMPLINGTIME * 1000000 + 2000 +1000000 ) {
    Serial.println("Loop Time not uphold");
    digitalWrite(ENABLESLED, LOW);
    delay(5000);
    setOutSled = 0;
    setOut = 0;
  }
  current_time = micros();
}

int sign(float angle) {
  if (angle > 0) {
    return 1;
  } 
  else if (angle < 0) {
    return -1;
  } 
  else {
    return 0;
  }

}

