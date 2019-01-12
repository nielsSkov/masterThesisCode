#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <math.h>

#define PI 3.1415926535897932384626433832795

//introducing shorthand for scientific e-notation
#define eN3  pow( 10,  -3 )
#define eN6  pow( 10,  -6 )
#define eN9  pow( 10,  -9 )
#define eN12 pow( 10, -12 )
#define e3   pow( 10,   3 )
#define e6   pow( 10,   6 )
#define e9   pow( 10,   9 )
#define e12  pow( 10,  12 )

//for reading csv-file
#define BUFFER_SIZE 10000

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

void matrixAdd( float add[], _Bool substract,
                float matrix1[],  float matrix2[],
                int   r1, int c1, int r2, int c2  )
{
  int i, j;

  //check if matrix dimentions match, if not: exit subroutine
  if( c1 != c2 || r1 != r2 )
  {
    printf( "matrixAdd: Matrix Dimention Mismatch!" );
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
    printf( "matrixMult: Matrix Dimention Mismatch!" );
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



//////////////////////////////////////////////////////////////
///////GLOBAL VARIABLES///////////////////////////////////////
//////////////////////////////////////////////////////////////

float Pk[6*6] =
{  .16325    ,   -.18187*eN3, -2.93254*eN6, -48.98472    ,  54.03861*eN3,  .69910*eN3,
  -.18187*eN3,    .15900    , -4.41683*eN6,  54.03963*eN3, -47.75004    , 1.09122*eN3,
 -2.93254*eN6,  -4.41683*eN6,  1.66750    ,   1.03221*eN3,   1.59675*eN3, -.49999*e3 ,
-48.98472    ,  54.03963*eN3,  1.03221*eN3,  14.69847*e3 , -15.65069    , -.19456    ,
 54.03861*eN3, -47.75004    ,  1.59675*eN3, -15.65069    ,  14.34062*e3 , -.30070    ,
   .69910*eN3,   1.09122*eN3,  -.49999*e3 ,   -.19456    ,   -.30070    ,  .14992*e6  };

_Bool  firstRunK   = 1;

float tSec     = 0;
float posPend1 = 0;
float posPend2 = 0;
float posSled  = 0;
float velPend1 = 0;
float velPend2 = 0;
float velSled  = 0;
float u_last   = 0; 

float xEstK[6*1] = { 0 };

void main()
{
  ///////CODE FOR DEBUGGING MATRIX MANIPULATION FUNCTIONS//////////////////////////
  //  
  //  float testM1[3*3] = { 1.7212,  8.4942,  -4.9037,
  //                       -9.1068,  1.7791,   7.8909,
  //                       10.6538,  3.7150,  -2.3342 };
  //   
  //  float testM2[3*3] = { 7.8594, -8.1829, -10.0287,
  //                        2.8055, -6.2399,   7.4899,
  //                        1.5767,  5.8865,   7.1679 };
  //  
  //  float testM3[3*4] = { 1.7212, 8.4942, -4.9037,  1.4168,
  //                       -9.1068, 1.7791,  7.8909, -9.5269,
  //                       10.6538, 3.7150, -2.3342, -5.0605 };
  //  
  //  float testM4[4*3] = { 7.8594, -8.1829, -10.0287,
  //                        2.8055, -6.2399,   7.4899,
  //                        1.5767,  5.8865,   7.1679,
  //                      -10.0527,  2.7379,  -4.4228 };
  //  
  //  float  Mres[3*3] = { 0 };
  //  float Mres2[4*3] = { 0 };
  //  
  //  matrixTranspose( Mres,  testM1, 3, 3 );                   //tested and works
  //  inv3x3Matrix(    Mres,  testM1 );                         //tested and works
  //  matrixAdd(       Mres,  1, testM1, testM2, 3, 3, 3, 3 );  //tested and works
  //  matrixAdd(       Mres,  0, testM1, testM2, 3, 3, 3, 3 );  //tested and works
  //  matrixMult(      Mres,  testM1, testM2, 3, 3, 3, 3 );     //tested and works
  //  matrixMult(      Mres,  testM3, testM4, 3, 4, 4, 3 );     //tested and works
  //  matrixTranspose( Mres2, testM3, 3, 4 );                   //tested and works
  //  
  //  printf( "%8.5f %8.5f %8.5f \n", Mres[0], Mres[1], Mres[2] );
  //  printf( "%8.5f %8.5f %8.5f \n", Mres[3], Mres[4], Mres[5] );
  //  printf( "%8.5f %8.5f %8.5f \n", Mres[6], Mres[7], Mres[8] );
  //  
  //  printf( "%8.5f %8.5f %8.5f \n", Mres2[0], Mres2[1],  Mres2[2] );
  //  printf( "%8.5f %8.5f %8.5f \n", Mres2[3], Mres2[4],  Mres2[5] );
  //  printf( "%8.5f %8.5f %8.5f \n", Mres2[6], Mres2[7],  Mres2[8] );
  //  printf( "%8.5f %8.5f %8.5f \n", Mres2[9], Mres2[10], Mres2[11] );
  //
  /////////////////////////////////////////////////////////////////////////////////
  
  //for csv-read
  char *homedir      = getenv("HOME");
  char *pathFromHome = "/thesis/matlab/twinKalman/data/";
  char *file         = "now4.csv";
  //
  char inputFile[100];
  strcpy( inputFile, homedir );
  strcat( inputFile, pathFromHome );
  strcat( inputFile, file );
  //
  char buffer[BUFFER_SIZE];
  FILE *f;
  char *field;
  //
  //opening csv file
  f = fopen( inputFile, "r" );
  //
  if( f == NULL)
  {
    printf( "unable to open csv-file '%s'\n", inputFile );
    exit(1);
  }
 
  while( (fgets( buffer, BUFFER_SIZE, f )) != NULL )
  {
               field = strtok( buffer, "," );
    tSec     = atof( field );
 
               field = strtok( NULL, "," );
    posPend1 = atof( field );

               field = strtok( NULL, "," );
    posPend2 = atof( field );

               field = strtok( NULL, "," );
    posSled  = atof( field );

               field = strtok( NULL, "," );
    velPend1 = atof( field );

               field = strtok( NULL, "," );
    velPend2 = atof( field );

               field = strtok( NULL, "," );
    velSled  = atof( field );

               field = strtok( NULL, "," );
    u_last   = atof( field );

    //creating wrapped vertion of measured theta1 for KF
    float pos1Wrap = (float)(fmod( (float)(posPend1 + PI), (float)(2*PI) ));
    if( pos1Wrap < 0 )
    {
      pos1Wrap = (float)(pos1Wrap + (float)(2*PI));
    }
    pos1Wrap = (float)(pos1Wrap - PI);

    //creating wrapped vertion of measured theta2 for KF
    float pos2Wrap = (float)(fmod( (float)(posPend2 + PI), (float)(2*PI) ));
    if( pos2Wrap < 0 )
    {
      pos2Wrap = (float)(pos2Wrap + (float)(2*PI));
    }
    pos2Wrap = (float)(pos2Wrap - PI);
    
    //>>
    //>>>>---initialization--------------------------------
    //>>

    //variables for matrix sizes
    int c1 = 0, r1 = 0, c2 = 0, r2 = 0;
    // 
    // c1: cols in matrix 1   r1: rows in matrix 1 
    // c2: cols in matrix 2   r2: rows in matrix 2
    // A[row][col] => A[ row *c1+ col ] = A[ vector element ]
    //
    // for matrix declarations as vectors
    // float A[r1][c1] => float A[r1*c1]

    c1 = 6, r1 = 6;
    float Q[6*6] = { 0 };
    //
    //setting diagonal elements of Q
    Q[0 *c1+ 0] = 10;   Q[1 *c1+ 1] = 10;   Q[2 *c1+ 2] = 10;
    Q[3 *c1+ 3] = 1000; Q[4 *c1+ 4] = 1000; Q[5 *c1+ 5] = 100000;

    float R[3*3] = { 3.00458*eN6, 4.4808*eN6,  .67671*eN6,
                     4.48088*eN6, 7.9224*eN6, 1.20058*eN6,
                      .67671*eN6, 1.2005*eN6,  .20079*eN6 };
    if( firstRunK )
    {
      //initialize estimated states
      r1 = 6; c1 = 1;
      xEstK[0 *c1+ 0] = pos1Wrap;
      xEstK[1 *c1+ 0] = pos2Wrap;
      xEstK[2 *c1+ 0] = posSled; //-.38
      xEstK[3 *c1+ 0] = velPend1;
      xEstK[4 *c1+ 0] = velPend2;
      xEstK[5 *c1+ 0] = velSled;

      firstRunK = 0;
    }
    
    float y[3*1] = { pos1Wrap, pos2Wrap, posSled };

    //defining discrete state space linearized around x = [ 0 0 0 0 0 0 ]';
    float A[6*6] =
      { 1.000713    , 27.51247*eN6, 0,   6.66766*eN3,   -.16497*eN6, 0          ,
       38.817329*eN6,  1.00113    , 0,   -.12727*eN6,   6.66319*eN3, 0          ,
        7.771088*eN6,  8.72165*eN6, 1, -25.47930*eN9, -52.29775*eN9, 6.67000*eN3,
        0.213968    ,  8.24961*eN3, 0,    .99929,     -49.46724*eN6, 0          ,
       11.639376*eN3,  0.34023    , 0, -38.16238*eN6,    .99795    , 0          ,
        2.330160*eN3,  2.61518*eN3, 0,  -7.63997*eN6, -15.68148*eN6, 1           };

    float B[6*1] = { 11.17302*eN6,
                     17.69227*eN6,
                      3.54193*eN6,
                      3.35023*eN3,
                      5.30502*eN3,
                      1.06204*eN3 };  
    
    float C[3*6] =
      { 1.00035    , 13.75623*eN6, 0,   3.33383*eN3, -82.486645*eN9, 0          ,
       19.40866*eN6,  1.00056    , 0, -63.63579*eN9,   3.331598*eN3, 0          ,
        3.88554*eN6,  4.36082*eN6, 1, -12.73965*eN9, -26.148875*eN9, 3.33500*eN3 };

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
    matrixAdd( xPred, 0, A_X_xEstK, B_X_uLast, 6, 1, 6, 1 );
    
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
    matrixAdd( Pk, 0, A_X_Pk_X_AT, Q, 6, 6, 6, 6 );
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
    matrixAdd( C_X_Pk_X_CT_ADD_R, 0, C_X_Pk_X_CT, R, 3, 3, 3, 3 );
    
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
    matrixAdd( y_SUB_C_X_xPred, 1, y, C_X_xPred, 3, 1, 3, 1 );

    //>> K*(y - C*xPred)
    float K_X_y_SUB_C_X_xPred[6*1] = { 0 };
    //
    matrixMult( K_X_y_SUB_C_X_xPred, K, y_SUB_C_X_xPred, 6, 3, 3, 1 );

    //>> xEstK = xPred + K*(y - C*xPred)
    matrixAdd( xEstK, 0, xPred, K_X_y_SUB_C_X_xPred, 6, 1, 6, 1 );
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
    //>>>>---Kalman filter end-----------------------------
    //>>
    
    //store Kalman states using reduced notation
    r1 = 6, c1 = 1;
    float x1 = xEstK[0 *c1+ 0];
    float x2 = xEstK[1 *c1+ 0];
    float x3 = xEstK[2 *c1+ 0];
    float x4 = xEstK[3 *c1+ 0];
    float x5 = xEstK[4 *c1+ 0];
    float x6 = xEstK[5 *c1+ 0];
    
    //print original csv-data
    printf( "%.5f, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f, ",
             tSec, posPend1, posPend2, posSled,
                   velPend1, velPend2, velSled, u_last         );

    //print Kalman filter state estimates
    printf( "%.5f, %.5f, %.5f, %.5f, %.5f, %.5f \n",
             x1,   x2,   x3,   x4,   x5,   x6       );
  
  }

  //close file
  fclose(f);

    
  //<<
} //<<<<MAIN-LOOP <END<
  //<<



///////PRINT-OUT CODE USED FOR DEBUGGING/////////////////////////////////////////
//  
//  float mm[6*3] = { 0 };
//  for( int i = 0; i < 6*3; i++ ){ mm[i] = K[i]; }
//  //
//  printf( "\n%10.5f, %10.5f, %10.5f  \n",   mm[0],  mm[1],  mm[2]  );
//  printf(   "%10.5f, %10.5f, %10.5f  \n",   mm[3],  mm[4],  mm[5]  );
//  printf(   "%10.5f, %10.5f, %10.5f  \n",   mm[6],  mm[7],  mm[8]  );
//  printf(   "%10.5f, %10.5f, %10.5f  \n",   mm[9],  mm[10], mm[11] );
//  printf(   "%10.5f, %10.5f, %10.5f  \n",   mm[12], mm[13], mm[14] );
//  printf(   "%10.5f, %10.5f, %10.5f  \n\n", mm[15], mm[16], mm[17] );
//  
//  float mm[6*6] = { 0 };
//  for( int i = 0; i < 6*6; i++ ){ mm[i] = Pk[i]; }
//  //
//  printf( "\n%10.5f, %10.5f, %10.5f, %10.5f, %10.5f, %10.5f \n",   mm[0],  mm[1],  mm[2],  mm[3],  mm[4],  mm[5] );
//  printf(   "%10.5f, %10.5f, %10.5f, %10.5f, %10.5f, %10.5f \n",   mm[6],  mm[7],  mm[8],  mm[9],  mm[10], mm[11] );
//  printf(   "%10.5f, %10.5f, %10.5f, %10.5f, %10.5f, %10.5f \n",   mm[12], mm[13], mm[14], mm[15], mm[16], mm[17] );
//  printf(   "%10.5f, %10.5f, %10.5f, %10.5f, %10.5f, %10.5f \n",   mm[18], mm[19], mm[20], mm[21], mm[22], mm[23] );
//  printf(   "%10.5f, %10.5f, %10.5f, %10.5f, %10.5f, %10.5f \n",   mm[24], mm[25], mm[26], mm[27], mm[28], mm[29] );
//  printf(   "%10.5f, %10.5f, %10.5f, %10.5f, %10.5f, %10.5f \n\n", mm[30], mm[31], mm[32], mm[33], mm[34], mm[35] );
//
/////////////////////////////////////////////////////////////////////////////////
