/*=====[observador]========================================================
 * Copyright 2023 Santiago Esteva <sestevafi.uba.ar> * All rights reserved.
 * License: BSD-3-Clause <https://opensource.org/licenses/BSD-3-Clause>)
 *
 * Version: 1.3.0
 * Creation Date: 2023/04/15
 */

/*=====[Inclusions of private function dependencies]=========================*/

#include "../../observador_rtos/inc/observador.h"

/*=====[Definition macros of private constants]==============================*/

/*=====[Private function-like macros]========================================*/

/*=====[Definitions of external public global variables]=====================*/

/*=====[Definitions of public global variables]==============================*/

/*=====[Definitions of private global variables]=============================*/

#ifdef OBS_PRINT_RESULT
static float r_array[N_SAMPLES];
static float y_array[N_SAMPLES];
static float xh1_array[N_SAMPLES];
static float xh2_array[N_SAMPLES];
static float u_array[N_SAMPLES];
static float usat_array[N_SAMPLES];
static uint32_t i = 0;
#endif

/*=====[Prototypes (declarations) of private functions]======================*/

#ifdef OBS_PRINT_RESULT
static void obsGatherDebugSamples( Observer_t* obs, float y, float r );
#endif

/*=====[Implementations of public functions]=================================*/

void Observer_init(Observer_t *obs, float K_1, float K_2,float K0,float L,float A_1,float A_2,float A_3,float A_4,float B_1,float B_2,float C_1,float C_2,float uMin, float uMax)
{
   // Configuration
   obs->config.K[0] = K_1;
	obs->config.K[1] = K_2;
	obs->config.K0 = K0;
	obs->config.L = L;
	obs->config.A[0] = A_1;
	obs->config.A[1] = A_2;
	obs->config.A[2] = A_3;
	obs->config.A[3] = A_4;
	obs->config.B[0] = B_1;
	obs->config.B[1] = B_2;
	obs->config.C[0] = C_1;
	obs->config.C[1] = C_2;
   obs->config.uMin = uMin;
   obs->config.uMax = uMax;

   // State
   obs->state.u = 0.0f;
   obs->state.u_sat = 0.0f;
   obs->state.x_hat[0] = 0.0f;
   obs->state.x_hat[1] = 0.0f;

   obsPrintf( obs );
}

float Observer(Observer_t *obs, float y, float reference) {
   // u = K_o* r −K* x_hat ;
   // obs->state.u = (reference * obs->config.K0) - (obs->config.K[0] * obs->state.x_hat[0])- (obs->config.K[1] * obs->state.x_hat[1]);
   obs->state.u = (reference * obs->config.K0) - (obs->config.K[0] * obs->state.x_hat[0]);

   obs->state.u_sat = obs->state.u;

   // Apply saturation of actuator
   if( obs->state.u < obs->config.uMin ) {
      obs->state.u_sat = obs->config.uMin;
   }
   if( obs->state.u > obs->config.uMax ) {
      obs->state.u_sat = obs->config.uMax;
   }

   return obs->state.u_sat;
}

// Update PID controller for next iteration
void obsUpdate( Observer_t* obs, float y, float r )
{
   float Phi[2];

   // x_hat = Phi * x_hat + Gamma*u + L * ( y − C* x_hat ) ;
   Phi[0] = (obs->config.A[0]*obs->state.x_hat[0])+(obs->config.A[1]*obs->state.x_hat[1]);
   Phi[1] = (obs->config.A[2]*obs->state.x_hat[0])+(obs->config.A[3]*obs->state.x_hat[1]);

   obs->state.x_hat[0] = Phi[0] + obs->config.B[0]*obs->state.u + obs->config.L * ( y - obs->config.C[0]* obs->state.x_hat[0] );
   obs->state.x_hat[1] = Phi[1]  + obs->config.B[1]*obs->state.u + obs->config.L * ( y - obs->config.C[1]* obs->state.x_hat[1] );
#ifdef OBS_PRINT_RESULT
   /*----- Print PID results if is activated --------------------------------*/
   obsGatherDebugSamples (obs, y, r);
#endif
}

void console_print (float* buffer)
{
	int32_t integer, fraction;
   integer = (int)buffer[0];
	fraction = (int)(((buffer[0] - (float)integer)) * 1000);
   if (fraction<0)
   {
      fraction = (-1)*fraction;
      if (integer==0)
      {
            printf("-%d.%03d\t", integer, fraction);
      }
      else
      {
            printf("%d.%03d\t", integer, fraction);
      }
   }
   else
   {
      printf("%d.%03d\t", integer, fraction);
   }
}

// obs printf object
void obsPrintf( Observer_t* obs )
{   
   float32_t buffer[1];
   printf( "Observer structure (object)\r\n" );
   printf( "----------------------\r\n\r\n" );
   buffer[0] = obs->config.K0;
   printf( "\n\t K0 = " );
   console_print (buffer);
   buffer[0] = obs->config.K[0];
   printf( "\n\t  K_1 = ");
   console_print (buffer);
   buffer[0] = obs->config.K[1];
   printf( "\n\t  K_2 = ");
   console_print (buffer);
   buffer[0] = obs->config.L;
   printf( "\n\t  L = ");
   console_print (buffer);
   buffer[0] = obs->config.A[0];
   printf( "\n\t  A = [");
   console_print (buffer);
   printf( " , ");
   buffer[0] = obs->config.A[1];
   console_print (buffer);
   buffer[0] = obs->config.A[2];
   printf( " , ");
   console_print (buffer);
   buffer[0] = obs->config.A[3];
   printf( " , ");
   console_print (buffer);
   buffer[0] = obs->config.B[0];
   printf( "]\n\t  B = [");
   console_print (buffer);
   buffer[0] = obs->config.B[1];
   printf( " , ");
   console_print (buffer);
   buffer[0] = obs->config.C[0];
   printf( "]\n\t  C = [");
   console_print (buffer);
   buffer[0] = obs->config.C[1];
   printf( " , ");
   console_print (buffer);
   buffer[0] = obs->config.uMax;
   printf( "]\n\t  umax = ");
   console_print (buffer);
   buffer[0] = obs->config.uMin;
   printf( "\n\t  umin = ");
   console_print (buffer);
   printf( "\r\n" );
}


#ifdef OBS_PRINT_RESULT
static void obsGatherDebugSamples( Observer_t* obs, float y, float r )
{
	char ntos[64];

   // Save and show samples
   if( i< N_SAMPLES ) {
      if (i == 0) {
         uartWriteString( UART_USB, "\r\n\r\nGathering " );
         uint64ToString( N_SAMPLES, ntos, 10 );
         uartWriteString( UART_USB, ntos );
         uartWriteString( UART_USB, " obs samples...\r\n\r\n" );
      }

      // Save N_SAMPLES samples of R[k] and Y[k]
      r_array[i] = r;
      y_array[i] = y;
      xh1_array[i] = obs->state.x_hat[0];
      xh2_array[i] = obs->state.x_hat[1];
      u_array[i] = obs->state.u;
      usat_array[i] = obs->state.u_sat;
      i++;

   } else {
      // Indicate that finish take samples
      gpioWrite( LEDB, OFF );
      gpioWrite( LED1, ON );

      // Note thath printf() does not work properly

      // print r[k] samples
      uartWriteString( UART_USB, "r = [ " );
      for( i=INIT_SAMPLES; i<N_SAMPLES; i++) {
         uartWriteString( UART_USB, floatToString( r_array[i], ntos, 5 ) );
         uartWriteString( UART_USB, "," );
      }
      uartWriteString( UART_USB, "]\r\n\r\n" );

      // print y[k] samples
      uartWriteString( UART_USB, "y = [ " );
      for( i=INIT_SAMPLES; i<N_SAMPLES; i++) {
         uartWriteString( UART_USB, floatToString( y_array[i], ntos, 5 ) );
         uartWriteString( UART_USB, "," );
      }
      uartWriteString( UART_USB, "]\r\n\r\n" );

      #ifndef OPEN_LOOP
      // print x1_hat[k] samples
      uartWriteString( UART_USB, "x1_hat = [ " );
      for( i=INIT_SAMPLES; i<N_SAMPLES; i++) {
         uartWriteString( UART_USB, floatToString( xh1_array[i], ntos, 5 ) );
         uartWriteString( UART_USB, "," );
      }
      uartWriteString( UART_USB, "]\r\n\r\n" );

      // print x2_hat[k] samples
      uartWriteString( UART_USB, "x2_hat = [ " );
      for( i=INIT_SAMPLES; i<N_SAMPLES; i++) {
         uartWriteString( UART_USB, floatToString( xh2_array[i], ntos, 5 ) );
         uartWriteString( UART_USB, "," );
      }
      uartWriteString( UART_USB, "]\r\n\r\n" );

      // print u[k] samples
      uartWriteString( UART_USB, "u = [ " );
      for( i=INIT_SAMPLES; i<N_SAMPLES; i++) {
         uartWriteString( UART_USB, floatToString( u_array[i], ntos, 5 ) );
         uartWriteString( UART_USB, "," );
      }
      uartWriteString( UART_USB, "]\r\n\r\n" );

      // print uSat[k] samples
      uartWriteString( UART_USB, "uSat = [ " );
      for( i=INIT_SAMPLES; i<N_SAMPLES; i++) {
         uartWriteString( UART_USB, floatToString( usat_array[i], ntos, 5 ) );
         uartWriteString( UART_USB, "," );
      }
      uartWriteString( UART_USB, "]\r\n\r\n" );
      #endif

      uartWriteString( UART_USB, "All samples printed. PROGRAM HALTED.\r\n" );
      gpioWrite( LED2, ON );

      while(1);
   }
}

#endif

/*=====[Implementations of interrupt functions]==============================*/

/*=====[Implementations of private functions]================================*/
