#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

//for reading csv-file
#define BUFFER_SIZE 8000

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
      
      matrixT[i *r1+ j] = matrix[j *c1+ i];
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

float PI = 3.141592653589793;

float Pk[6*6] =
  { 0.163251449842,  -0.000181879547,   -0.000002932546,   -48.984720791602,     0.054038618604,      0.000699102084,
   -0.000181879547,   0.159001471489,   -0.000004416835,     0.054039639205,   -47.750046527928,      0.001091223281,
   -0.000002932546,  -0.000004416835,    1.667500534233,     0.001032217157,     0.001596758435,   -499.999990015207,
  -48.984720791602,   0.054039639205,    0.001032217157, 14698.472855758520,   -15.650695499102,     -0.194565356555,
    0.054038618604, -47.750046527928,    0.001596758435,   -15.650695499102, 14340.624691169911,     -0.300709145006,
    0.000699102084,   0.001091223281, -499.999990015207,    -0.194565356555,    -0.300709145006, 149925.001515453070 };

_Bool  firstRunK   = 1;

//for csv-read
char filename[] = "tuneKF4.csv";
char buffer[BUFFER_SIZE];
FILE *f;
char *field;

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
  //opening csv file
  f = fopen( filename, "r" );
  //
  if( f == NULL)
  {
    printf("unable to open csv-file '%s'\n",filename);
    exit(1);
  }
 
  while( fgets( buffer, BUFFER_SIZE, f ) )
  {
               field = strtok( buffer, "," );
    tSec     = atoi( field );
 
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
    //
    // matrix array-vector interpretation
    // A[row][col] => A[ row *c1+ col ] = A[ vector element ]
    //
    // for matrix declarations as vectors
    // float A[r1][c1] => float A[r1*c1]

    float Q[6*6] = { 0, 0, 0, 0, 0, 0,
                     0, 0, 0, 0, 0, 0,
                     0, 0, 0, 0, 0, 0,
                     0, 0, 0, 0, 0, 0,
                     0, 0, 0, 0, 0, 0,
                     0, 0, 0, 0, 0, 0 };
    //
    //setting diagonal elements of Q
    Q[0 *c1+ 0] = 10;   Q[1 *c1+ 1] = 10;   Q[2 *c1+ 2] = 10;
    Q[3 *c1+ 3] = 1000; Q[4 *c1+ 4] = 1000; Q[5 *c1+ 5] = 100000;

    //introducing shorthand for scientific e-notation
    float eN5 = pow(10,-5), eN6 = pow(10,-6), eN7  = pow(10,- 7);
    float eN8 = pow(10,-8), eN9 = pow(10,-9), eN10 = pow(10,-10);

    float R[3*3] =
      { 0.000003004588387, 0.000004480884893, 0.000000676711753,
        0.000004480884893, 0.000007922412675, 0.000001200581026,
        0.000000676711753, 0.000001200581026, 0.000000200792964 };

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
    
    float A[6*6] =
          { 1.000713584960613,   0.000027512471459,                   0,   0.006667662014417,  -0.000000164973290,                   0,
            0.000038817329540,   1.001134697178922,                   0,  -0.000000127271579,   0.006663197672182,                   0,
            0.000007771088754,   0.000008721656425,   1.000000000000000,  -0.000000025479309,  -0.000000052297750,   0.006670001667500,
            0.213968450439665,   0.008249614567048,                   0,   0.999298455022885,  -0.000049467241072,                   0,
            0.011639376262599,   0.340238949099787,                   0,  -0.000038162382991,   0.997959822003739,                   0,
            0.002330160962757,   0.002615188679131,                   0,  -0.000007639970827,  -0.000015681480364,   1.000000000000000 };

    float B[6*1] = { 0.000011173020989,
                     0.000017692278595,
                     0.000003541930082,
                     0.003350230343635,
                     0.005305029736718,
                     0.001062047735194 };  
    
    float C[3*6] =
          { 1.000356792480306,   0.000013756235730,                   0,   0.003333831007208,  -0.000000082486645,                   0,
            0.000019408664770,   1.000567348589461,                   0,  -0.000000063635790,   0.003331598836091,                   0,
            0.000003885544377,   0.000004360828213,   1.000000000000000,  -0.000000012739655,  -0.000000026148875,   0.003335000833750 };
    
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
    matrixAdd( Pk, 1, A_X_Pk_X_AT, Q, 6, 6, 6, 6 );
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
      
    ////////////////////////
    //>> Pk = I - K*C*Pk <<//
    ////////////////////////
    
    //>> I
    float I[6*6]  = {0};
    r1 = 6, c1 = 6;
    I[0 *c1+ 0] = I[1 *c1+ 1] = I[2 *c1+ 2] = 1;
    I[3 *c1+ 3] = I[4 *c1+ 4] = I[5 *c1+ 5] = 1; 
    
    //>> K*C
    float K_X_C[6*6] = { 0 };
    //
    matrixMult( K_X_C, K, C, 6, 3, 3, 6 );
    
    //>> K*C*Pk
    float K_X_C_X_Pk[6*6] = { 0 };
    //
    matrixMult( K_X_C_X_Pk, K_X_C, Pk, 6, 6, 6, 6 );
    
    //>> Pk = I - K*C*Pk
    float P[6*6] = { 0 };
    //
    matrixAdd( Pk, 1, I, K_X_C_X_Pk, 6, 6, 6, 6 );
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








