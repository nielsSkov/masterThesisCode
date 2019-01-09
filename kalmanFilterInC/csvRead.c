#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define BUFFER_SIZE 8000

int main()
{
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

		//print csv
		printf( "%.5f, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f \n",
             tSec, posPend1, posPend2, posSled,
                   velPend1, velPend2, velSled, u_last           );
	}

  //close file
	fclose(f);
  
  return(0);
}

