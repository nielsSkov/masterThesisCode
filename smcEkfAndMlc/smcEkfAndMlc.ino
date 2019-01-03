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
//float coloumbP[] =
//  { 2.0599, 2.0662, 2.1381, 2.1714, 2.1948, 2.2462, 2.2906, 2.3641,
//    2.4541, 2.5419, 2.6354, 2.7466, 2.8576, 2.9494, 3.0025, 3.0104,
//    3.0008, 2.9567, 2.8189, 2.6462, 2.4794, 2.3163, 2.1865, 2.0951,
//    2.0488, 2.0721, 2.0967, 2.1510, 2.2220, 2.3104, 2.3658, 2.4560,
//    2.5617, 2.7070, 2.8373, 2.9427, 3.0279, 3.1276, 3.1694, 3.1450,
//    3.0841, 2.9708, 2.8381, 2.6918, 2.5490, 2.4321, 2.4032, 2.3836,
//    2.4188, 2.5065, 2.6558, 2.8007, 2.9618, 3.0691, 3.1635, 3.2093,
//    3.2171, 3.1765, 3.1086, 3.0108, 2.9393, 2.9058, 2.8899, 2.8902,
//    2.8996, 2.9615, 2.9700, 3.0784                                 };
//fourth attempt
//float coloumbP[] =
//  { 2.5089, 2.4863, 2.5210, 2.5631, 2.5894, 2.6127, 2.6584, 2.7284,
//    2.8329, 2.9804, 3.1066, 3.1965, 3.2661, 3.3395, 3.4087, 3.4750,
//    3.4930, 3.4163, 3.3189, 3.1266, 2.8755, 2.6637, 2.4754, 2.4012,
//    2.4196, 2.4655, 2.5260, 2.5595, 2.6007, 2.6651, 2.7505, 2.8580,
//    2.9512, 3.0791, 3.2105, 3.3952, 3.5322, 3.5814, 3.5944, 3.5811,
//    3.5155, 3.4185, 3.2882, 3.0785, 2.9111, 2.7864, 2.7020, 2.7022,
//    2.7788, 2.8666, 2.9840, 3.1490, 3.3364, 3.5304, 3.7119, 3.7837,
//    3.7740, 3.6458, 3.5251, 3.3996, 3.2945, 3.2618, 3.2475, 3.2676,
//    3.3051, 3.3647, 3.3851, 3.4460                                 };
//fourth attempt (a lot less smoothing)
//float coloumbP[] =
//  { 2.5089, 2.4360, 2.5843, 2.5293, 2.6041, 2.6122, 2.6421, 2.6728,
//    2.8433, 2.9377, 3.1647, 3.2575, 3.2460, 3.2609, 3.4466, 3.5619,
//    3.4723, 3.5184, 3.2847, 3.1919, 2.9892, 2.4870, 2.4113, 2.3882,
//    2.3820, 2.4536, 2.5322, 2.6208, 2.5802, 2.5511, 2.8028, 2.8654,
//    3.0138, 2.9110, 3.2739, 3.3821, 3.6021, 3.6725, 3.5723, 3.5177,
//    3.6073, 3.5415, 3.1618, 3.1645, 2.8854, 2.7091, 2.6446, 2.7139,
//    2.6852, 2.9086, 2.9872, 3.0896, 3.2542, 3.6685, 3.6848, 3.9229,
//    3.7519, 3.8016, 3.4059, 3.3272, 3.3840, 3.1988, 3.1939, 3.2335,
//    3.4026, 3.2925, 3.4084, 3.4460                                 };
//fourth attempt (medium smoothing)
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
//float coloumbN[] =
//  { 5.6263, 5.4987, 5.1417, 4.9257, 4.6487, 4.3322, 4.0786, 3.8225,
//    3.5715, 3.3112, 3.0779, 2.9407, 2.8604, 2.8355, 2.8457, 2.9103,
//    3.0284, 3.1653, 3.3837, 3.6218, 3.8165, 3.9902, 4.1001, 4.1013,
//    4.0578, 3.8796, 3.6658, 3.4336, 3.1961, 2.9678, 2.8269, 2.6447,
//    2.4937, 2.3306, 2.1720, 2.0713, 1.9899, 1.9119, 1.9537, 2.0036,
//    2.1147, 2.2597, 2.4044, 2.5582, 2.6746, 2.7244, 2.7459, 2.7200,
//    2.6334, 2.5232, 2.4298, 2.3751, 2.3673, 2.3480, 2.3565, 2.4114,
//    2.4775, 2.5494, 2.6026, 2.6170, 2.6656, 2.6750, 2.7167, 2.7325,
//    2.7000, 2.6817, 2.7051, 2.5191                                 };
//fourth attempt
//float coloumbN[] =
//  { 6.0457, 5.8981, 5.5088, 5.2078, 4.9781, 4.7767, 4.5121, 4.2287,
//    3.9130, 3.6207, 3.4146, 3.2296, 3.1161, 3.1008, 3.1556, 3.2596,
//    3.3579, 3.5195, 3.6744, 3.8990, 4.2394, 4.5244, 4.7341, 4.7770,
//    4.6029, 4.3123, 4.0221, 3.7270, 3.5362, 3.3636, 3.1773, 3.0404,
//    2.8838, 2.7131, 2.6023, 2.4179, 2.2847, 2.2331, 2.1898, 2.2770,
//    2.4242, 2.5996, 2.8329, 3.0099, 3.1690, 3.2641, 3.2670, 3.1867,
//    3.0541, 2.9050, 2.7357, 2.6472, 2.6123, 2.6892, 2.8068, 2.8735,
//    2.9326, 2.8925, 2.9317, 3.0037, 3.0525, 3.1113, 3.0919, 3.0538,
//    3.0692, 3.0503, 3.0857, 2.8821                                 };
//fourth attempt (a lot less smoothing)
//float coloumbN[] =
//  { 6.0457, 6.0008, 5.4575, 5.0818, 4.9099, 4.8706, 4.6863, 4.1072,
//    3.8124, 3.7340, 3.3521, 3.1361, 3.0851, 3.0505, 3.1449, 3.1818,
//    3.4730, 3.4172, 3.6360, 3.9225, 4.0484, 4.6924, 4.9115, 4.8782,
//    4.6373, 4.4396, 3.8567, 3.7538, 3.4382, 3.4088, 3.2508, 2.9450,
//    2.8026, 2.9261, 2.4537, 2.4430, 2.2954, 2.0959, 2.2386, 2.1855,
//    2.3199, 2.7089, 2.7662, 3.0776, 3.1715, 3.3254, 3.3598, 3.2184,
//    3.0543, 2.8371, 2.8494, 2.5146, 2.5082, 2.6631, 2.8188, 3.0751,
//    2.7527, 3.0189, 2.8425, 2.9274, 3.1565, 3.1543, 3.0928, 3.0650,
//    2.9287, 3.1714, 3.1068, 2.8821                                 };
//fourth attempt (medium smoothing)
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

// Create joint objects
Joint  sled( 1, SAMPLINGTIME );
Joint pend1( 2, SAMPLINGTIME );
Joint pend2( 3, SAMPLINGTIME );


struct firStru
{
  float xIn;         //   set window size here
  float buff[5];     //<--must be same size as
  int   offset;                               //
};                                             //
                                                //
///////Global Variables////////////////////////////////////
                                               //
//ring-buffer and offset for FIR filter     //
int   offsetFIR_x3 = 0;                  //
int   offsetFIR_x4 = 0;               //
float buffFIR_x3[5];              //<--here
float buffFIR_x4[5];          //<--and here

int logStop = 0;

float t_last       = 0;
float x1_last      = 0;
float x2_last      = 0;
float x3_last      = 0;
float x4_last      = 0;

float setOutSled   = 0;
float setOutPend1  = 0;
float velSled      = 0;
float velPend1     = 0;
float velPend2     = 0;
float posSled;
float posPend1;
float posPend2;
int   setOut       = 0;

float u_last       = 0;
float x1_old       = 0;
float catchAngle   = 0.02;
int   slideOn      = 0;

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
  for( int i = 0; i < 68; i++ ){ position[i] -= 0.38; 
                                 coloumbN[i] += 0.32;
                                 coloumbP[i] -= 1.25; }
  
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
    ///////Kalman Filter Test////////////
    else if( input == "8" )
    {
      setOut = 8;
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
    Serial.print( posPend1, deci );
    Serial.print( ", "           );
    Serial.print( posPend2, deci );
    Serial.print( ", "           );
    Serial.print( posSled,  deci );
    Serial.print( ", "           );
    Serial.print( velPend1, deci );
    Serial.print( ", "           );
    Serial.print( velPend2, deci );
    Serial.print( ", "           );
    Serial.print( velSled,  deci );  
  } 

  /////////////////////////////////////////////////////////
  ///////FIR (MA) FILTER INIT//////////////////////////////
  /////////////////////////////////////////////////////////

  //time difference for nummerical diff
  float t_delta = float(float(time_stamp-t_last)/1000000); // [s]

  //mesurements used in FIR filters
  float x1_FIR = posPend1;
  float x2_FIR = posSled-0.38; //<--rail center as zero
  
  //nummerical diff for FIR filters
  float x3_FIR = (x1_FIR - x1_last)/t_delta;
  float x4_FIR = (x2_FIR - x2_last)/t_delta;
  
  //set window size for all FIR filters
  float  N_FIR = (sizeof(buffFIR_x3)/sizeof(buffFIR_x3[0]));  //<--same size as for x4
  
  //filter coefficients
  float h[(int)N_FIR];
  for( int i = 0; i< N_FIR; i++ )
  {
    h[i]  = 1/N_FIR;
  }

  /////////////////////////////////////////////////////////
  ///////FIR (MA) FILTERS//////////////////////////////////
  /////////////////////////////////////////////////////////

  //struct for FIR (MA) filter of x3 (angle velocity)
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

  /////////////////////////////////////////////////////////
  ///////EXPONENTIAL SMOOTHING FILTER//////////////////////
  /////////////////////////////////////////////////////////
  
  float alpha = .8;
  x3_FIR      = alpha*x3_FIR + ( 1 - alpha )*x3_last;
  x4_FIR      = alpha*x4_FIR + ( 1 - alpha )*x4_last;
  
  x1_last = x1_FIR;
  x2_last = x2_FIR;
  x3_last = x3_FIR;
  x4_last = x4_FIR;
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
    //////EDGE OF RAIL SECURITY//////////////////////////////
    /////////////////////////////////////////////////////////
    //
    float sgnCart = posSled-.38;
    if(      sgnCart > 0 ){ sgnCart =  1; }
    else if( sgnCart < 0 ){ sgnCart = -1; }
    else                  { sgnCart =  0; }
    //
    //                             vel away from ceneter
    if( abs(posSled-.38) > .08 && sgnCart*velSled > 1 )
    {
      Serial.println("Secure Edge Active");
      
      setOutSled = -sgnCart*5; //<--breaking if cart runs away
    }
    /////////////////////////////////////////////////////////
    ///////CATCH - SLIDING MODE//////////////////////////////
    /////////////////////////////////////////////////////////
    //
    else if( abs(x1Wrap) < catchAngle )
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
//    printToTerminal( collectData, deci,   tSec,
//                     x1,          x2,       x3,     x4,
//                     x1_FIR,      x2_FIR,   x3_FIR, x4_FIR,
//                     x1Wrap,      setOutSledNoComp, setOutSled, B_c_c,
//                     posPend1,    posPend2,         velPend1,   velPend2,
//                     posSled,     velSled                                );
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
    float railOffset = 0.05;  //<--position on rail under test [m]
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
        logStop   = 1;
        setOut    = 0; //no longer moving, return to complete stop,
      }                //where logging is also stoped
    }
    
    //choose to print readable data (0) to print for data collection (1)
    int collectData = 1;
    
    //choose nr of decimals printed after decimal point
    int deci = 5;

    Serial.print( tSec,             deci );
    Serial.print( ", "                   );
    Serial.print( setOutSled,       deci );
    Serial.print( ", "                   );
    Serial.print( posSled,          deci );
    Serial.print( ", "                   );
    Serial.print( velSled,          deci );
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
  /////////////////////////////////////////////////////////
  ///////KALMAN FILTER TEST////////////////////////////////
  /////////////////////////////////////////////////////////
  else if( setOut == 8 )
  {
    digitalWrite(ENABLESLED, HIGH);
    setOutPend1 = 0;
    setOutSled  = 0;
    
    float tSec   = float(float(time_stamp-time_now)/1000000);  // [s]
    
    int testTime = 20;    // [s]
    
    //-------initialize Kalman variables-------------------
    
    float xEstK[6][1] = { 0 };
    float    Pk[6][6] = { 0 };
    bool  firstRunK   = true;  //!!!!!!!!!!!!!
 
    //>>
    //>>>>---initialization--------------------------------
    //>>

    float Q[6][6]  = {0};

    //setting diagonal elements of Q
    Q[0][0] = 1; Q[1][1] = 1; Q[2][2] = 1;
    Q[3][3] = 1; Q[4][4] = 1; Q[5][5] = 1; 

    //introducing shorthand for scientific e-notation
    float eN5 = pow(10,-5); float eN6 = pow(10,-6); float eN7  = pow(10,- 7);
    float eN8 = pow(10,-8); float eN9 = pow(10,-9); float eN10 = pow(10,-10);

    float R[3][3] = { {  7.7114*eN7 ,  1.2205*eN8, -3.5968*eN10 },
                      {  1.2205*eN8 ,  9.2307*eN7, -3.1029*eN9  },
                      { -3.5968*eN10, -3.1029*eN9,  1.0616*eN9  } };
    
    if( firstRunK )
    {
      //initialize estimated states
      xEstK[5][0] = { posPend1, posPend2, posSled, 0, 0, 0 };
    
      firstRunK = false;
    }

    float y[3][1] = { posPend1, posPend2, posSled };
   
    //defining discrete state space linearized around x = [ 0 0 0 0 0 0 ]';
    float A[6][6] = 
      { { 1.0007    , 2.7512*eN5, 0  0.0066677 , -1.6497*eN7, 0       },
        { 3.8817*eN5, 1.0011    , 0 -1.2727*eN7,  0.0066632 , 0       },
        { 7.7711*eN6, 8.7217*eN6, 1 -2.5479*eN8, -5.2298*eN8, 0.00667 },
        { 0.21397   , 0.0082496 , 0  0.9993    , -4.9467*eN5, 0       },
        { 0.011639  , 0.34024   , 0 -3.8162*eN5,  0.99796   , 0       },
        { 0.0023302 , 0.0026152 , 0 -7.64  *eN6, -1.5681*eN5, 1       } };
    //
    float B[6][1] = { 1.1173*eN5, 1.7692*eN5, 3.5419*eN6, 
                      0.0033502,  0.005305,   0.001062   };
    //
    float C[3][6] =
      { {     1.0004, 1.3756*eN5, 0,  0.0033338 , -8.2487*eN8,        0 },
        { 1.9409*eN5,     1.0006, 0, -6.3636*eN8,  0.0033316 ,        0 },
        { 3.8855*eN6, 4.3608*eN6, 1, -1.274 *eN8, -2.6149*eN8, 0.003335 } };
    
    //>>
    //>>>>---prediction------------------------------------
    //>>

    /////////////////////////////////////
    //>> xPred = A*xEstK + B*u_last  <<//
    /////////////////////////////////////

    //>> A*xEstK
    float A_X_xEstK[6][1] = { 0 };
    //
    matrixMult( A_X_xEstK, A, xEstK,  6, 6, 6, 1 );
    
    //>> B*u_last
    float B_X_u[6][1] = { 0 };
    //
    matrixMult( B_X_uLast, B, u_last, 6, 1, 1, 1 );
    
    //>> xPred = A*xEstK + B*u_last
    float xPred[6][1] = { 0 };
    //
    matrixAdd( xPred, false, A_X_xEstK, B_X_uLast, 6, 1, 6, 1 );

    ///////////////////////////
    //>> Pk = A*Pk*A' + Q  <<//
    ///////////////////////////
    
    //>> A*Pk
    float A_X_Pk[6][6] = { 0 };
    //
    matrixMult( A_X_Pk, A, Pk, 6, 6, 6, 6 );
    
    //>> A'
    float AT[6][6] = { 0 };
    //
    matrixTranspose( AT, A, 6, 6 );
    
    //>> A*Pk*A'
    float A_X_Pk_X_AT[6][6] = { 0 };
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
    float CT[6][3] = { 0 };
    //
    matrixTranspose( CT, C, 3, 6 );
    
    //>> Pk*C'
    float Pk_X_CT[6][3] = { 0 };
    //
    matrixMult( Pk_X_CT, Pk, CT, 6, 6, 6, 3 );
    
    //>> C*Pk
    float C_X_Pk[3][6] = { 0 };
    //
    matrixMult( C_X_Pk, C, Pk, 3, 6, 6, 6 );
    
    //>> C*Pk*C'
    float C_X_Pk_X_CT[3][3] = { 0 };
    //
    matrixMult( C_X_Pk_X_CT, C_X_Pk, CT, 3, 6, 6, 3 );
    
    //>> C*Pk*C' + R
    float C_X_Pk_X_CT_ADD_R[3][3] = { 0 }
    //
    matrixAdd( C_X_Pk_X_CT_ADD_R, false, C_X_Pk_X_CT, R, 3, 3, 3, 3 );
    
    //>> inv( C*Pk*C' + R )
    float inv_C_X_Pk_X_CT_ADD_R[3][3] = { 0 }
    //
    inv3x3Matrix( inv_C_X_Pk_X_CT_ADD_R, C_X_Pk_X_CT_ADD_R )
    
    //>> K = Pk*C'*inv( C*Pk*C' + R )
    float K[6][3]  = { 0 };
    //
    matrixMult( K, Pk_X_CT, inv_C_X_Pk_X_CT_ADD_R, 6, 3, 3, 3 );
    
    ///////////////////////////////////////
    //>> xEstK = xPred + K*(y - C*xPred) <<//
    ///////////////////////////////////////
    
    //>> C*xPred
    float C_X_xPred[3][1] = { 0 };
    //
    matrixMult( C_X_xPred, C, xPred, 3, 6, 6, 1 );
    
    //>> (y - C*xPred)
    float y_SUB_C_X_xPred[3][1] = { 0 };
    //
    matrixAdd( y_SUB_C_X_xPred, true, y, C_X_xPred, 3, 1, 3, 1 );
    
    //>> K*(y - C*xPred)
    float K_X_y_SUB_C_X_xPred[6][1] = { 0 };
    //
    matrixMult( K_X_y_SUB_C_X_xPred, K, y_SUB_C_X_xPred, 6, 3, 3, 1 );
    
    //>> xEstK = xPred + K*(y - C*xPred)
    matrixAdd( xEstK, false, xPred, K_X_y_SUB_C_X_xPred, 6, 6, 6, 6 );
    //xEstK was updated
    
    ////////////////////////
    //>> Pk = I - K*C*Pk <<//
    ////////////////////////
    
    //>> I
    float I[6][6]  = {0};
    I[0][0] = I[1][1] = I[2][2] = I[3][3] = I[4][4] = I[5][5] = 1; 
    
    //>> K*C
    float K_X_C[6][6] = { 0 };
    //
    matrixMult( K_X_C, K, C, 6, 3, 3, 6 );
    
    //>> K*C*Pk
    float K_X_C_X_Pk[6][6] = { 0 };
    //
    matrixMult( K_X_C_X_Pk, K_X_C, Pk, 6, 6, 6, 6 );
    
    //>> Pk = I - K*C*Pk
    float P[6][6] = { 0 };
    //
    matrixAdd( Pk, true, I, K_X_C_X_Pk, 6, 6, 6, 6 );
    //Pk was updated
    
    //>>
    //>>>>---Kalman filter end-----------------------------
    //>>
    
    //store Kalman variables using reduced notation
    float x1_K = xEstK[0];
    float x2_K = xEstK[1];
    float x3_K = xEstK[2];
    float x4_K = xEstK[3];
    float x5_K = xEstK[4];
    float x6_K = xEstK[5];
    
    if( tSec > testTime )
    {
      digitalWrite(ENABLESLED, LOW);
      if( velSled == 0 )
      {
        logStop   = 1;
        setOut    = 0; //stop logging
      }
    }
    
    //choose nr of decimals printed after decimal point
    int deci = 5;

    //output for logging
    Serial.print( tSec,     deci );
    Serial.print( ", "           );
    Serial.print( posPend1, deci );
    Serial.print( ", "           );
    Serial.print( posPend2, deci );
    Serial.print( ", "           );
    Serial.print( posSled,  deci );
    Serial.print( ", "           );
    Serial.print( velPend1, deci );
    Serial.print( ", "           );
    Serial.print( velPend2, deci );
    Serial.print( ", "           );
    Serial.print( velSled,  deci );
    Serial.print( ", "           );
    Serial.print( x1_K,     deci );
    Serial.print( ", "           );
    Serial.print( x2_K,     deci );
    Serial.print( ", "           );
    Serial.print( x3_K,     deci );
    Serial.print( ", "           );
    Serial.print( x4_K,     deci );
    Serial.print( ", "           );
    Serial.print( x5_K,     deci );
    Serial.print( ", "           );
    Serial.print( x6_K,     deci );
    //<<
  } //<<<<KALMAN FILTER TEST <END<
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


void inv3x3Matrix( float * invM[][], float m[][] )
{
  float det  = m[0][0] * (m[1][1]*m[2][2] - m[1][2] * m[2][1]);
        det -= m[0][1] * (m[1][0]*m[2][2] - m[1][2] * m[2][0]);
        det += m[0][2] * (m[1][0]*m[2][1] - m[1][1] * m[2][0]);

  float invDet = 1.0 / (det);

  invM[0][0] = (invDet) * (m[1][1] * m[2][2] - m[1][2] * m[2][1]);
  invM[1][0] = (invDet) * (m[1][2] * m[2][0] - m[1][0] * m[2][2]);
  invM[2][0] = (invDet) * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);

  invM[0][1] = (invDet) * (m[0][2] * m[2][1] - m[0][1] * m[2][2]);
  invM[1][1] = (invDet) * (m[0][0] * m[2][2] - m[0][2] * m[2][0]);
  invM[2][1] = (invDet) * (m[0][1] * m[2][0] - m[0][0] * m[2][1]);

  invM[0][2] = (invDet) * (m[0][1] * m[1][2] - m[0][2] * m[1][1]);
  invM[1][2] = (invDet) * (m[0][2] * m[1][0] - m[0][0] * m[1][2]);
  invM[2][2] = (invDet) * (m[0][0] * m[1][1] - m[0][1] * m[1][0]);
}

void matrixTranspose( float * matrixT[][], float matrix[][],
                                           int rows, int cols ) 
{
  int i, j;
  for (   i = 0; i < rows; i++ )
  { 
    for ( j = 0; j < cols; j++ ) 
      matrixT[i][j] = matrix[j][i]; 
  }
}

void matrixAdd( float * add[][], bool substract,
                float   matrix1[][],      float matrix2[][],
                int     rows1, int cols1, int rows2, int cols2 )
{
  int i, j;

  //check if matrix dimentions match, if not: exit subroutine
  if( cols1 ~= cols2 || rows1 ~= rows2 )
  {
    Serial.print( "matrixAdd: Matrix Dimention Mismatch!" );
    logStop = 1;
    setOut  = 0; //stop logging
  }

  for(   i = 0; i < rows1; ++i )
  {
    for( j = 0; j < cols1; ++j )
    {
      if substract
      {
        add[i][j] = matrix1[i][j] - matrix2[i][j];
      }
      else
      {
        add[i][j] = matrix1[i][j] + matrix2[i][j];
      }
    }
  }
}

void matrixMult( float * mult[][],
                 float   matrix1[][],      float matrix2[][],
                 int     rows1, int cols1, int rows2, int cols2 )
{
  int i, j, k;

  //check if matrix dimentions match, if not: exit subroutine
  if( cols1 ~= rows2 )
  {
    Serial.print( "matrixMult: Matrix Dimention Mismatch!" );
    logStop = 1;
    setOut  = 0; //stop logging
  }

  for(     i = 0; i < rows1; ++i )
  {
    for(   j = 0; j < cols2; ++j )
    {
      mult[i][j] = 0;
      
      for( k = 0; k < cols1; ++k )
      {
        mult[i][j] += matrix1[i][k]*matrix2[k][j];
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
  
  float frictionComp = r / k_tau *( (sat_x4*(B_c_c)) + x4_0*B_v_c*0 );
  
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
    Serial.print( setOutSled,       deci );
    Serial.print( ", "                   );
    //Serial.print( posPend1,         deci );
    //Serial.print( ", "                   );
    //Serial.print( posPend2,         deci );
    //Serial.print( ", "                   );
    Serial.print( posSled,          deci );
    Serial.print( ", "                   );
    //Serial.print( velPend1,         deci );
    //Serial.print( ", "                   );
    //Serial.print( velPend2,         deci );
    //Serial.print( ", "                   );
    Serial.print( velSled,          deci );
//    Serial.print( x1,               deci );
//    Serial.print( ", "                   );
//    Serial.print( x1Wrap,           deci );
//    Serial.print( ", "                   );
//    Serial.print( x1_FIR,           deci );
//    Serial.print( ", "                   );
//    Serial.print( x2,               deci );
//    Serial.print( ", "                   );
//    Serial.print( x2_FIR,           deci );
//    Serial.print( ", "                   );
//    Serial.print( x3,               deci );
//    Serial.print( ", "                   );
//    Serial.print( x3_FIR,           deci );
//    Serial.print( ", "                   );
//    Serial.print( x4,               deci );
//    Serial.print( ", "                   );
//    Serial.print( x4_FIR,           deci );
//    Serial.print( ", "                   );
//    Serial.print( setOutSledNoComp, deci );
//    Serial.print( ", "                   );
//    Serial.print( setOutSled,       deci );
//    Serial.print( ", "                   );
//    Serial.print( B_c_c,            deci );
  }
  else
  {
    Serial.print( tSec,     deci );
    Serial.print( ", "           );
    Serial.print( posPend1, deci );
    Serial.print( ", "           );
    Serial.print( posPend2, deci );
    Serial.print( ", "           );
    Serial.print( posSled,  deci );
    Serial.print( ", "           );
    Serial.print( velPend1, deci );
    Serial.print( ", "           );
    Serial.print( velPend2, deci );
    Serial.print( ", "           );
    Serial.print( velSled,  deci );
  }
}
