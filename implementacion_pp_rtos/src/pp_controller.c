/*=====[pid_controller]========================================================
 * Copyright 2023 Santiago Esteva <sestevafi.uba.ar> * All rights reserved.
 * License: BSD-3-Clause <https://opensource.org/licenses/BSD-3-Clause>)
 *
 * Version: 1.3.0
 * Creation Date: 2023/04/15
 */

/*=====[Inclusions of private function dependencies]=========================*/

#include "../../implementacion_pp_rtos/inc/pp_controller.h"

/*=====[Definition macros of private constants]==============================*/

/*=====[Private function-like macros]========================================*/

/*=====[Definitions of external public global variables]=====================*/

/*=====[Definitions of public global variables]==============================*/

/*=====[Definitions of private global variables]=============================*/

#ifdef PP_PRINT_RESULT
static float r_array[N_SAMPLES];
static float y_array[N_SAMPLES];
static float y2_array[N_SAMPLES];
static float u_array[N_SAMPLES];
static float usat_array[N_SAMPLES];
static uint32_t i = 0;
#endif

/*=====[Prototypes (declarations) of private functions]======================*/

#ifdef PP_PRINT_RESULT
static void ppGatherDebugSamples( PPController_t* pp, float y2, float y, float r );
#endif

/*=====[Implementations of public functions]=================================*/

void pole_placement_init(PPController_t *pp, float K_1, float K_2,float K0,float uMin, float uMax)
{
   // Configuration
   pp->config.K[0] = K_1;
	pp->config.K[1] = K_2;
	pp->config.K0 = K0;
   pp->config.uMin = uMin;
   pp->config.uMax = uMax;

   // State
   pp->state.u = 0.0f;
   pp->state.u_sat = 0.0f;

   ppPrintf( pp );
}

float pole_placement_control(PPController_t *pp, float x_1, float x_2, float reference) {
   // u = r*k0-(k1*x1+k2*x2)
   pp->state.u = (reference * pp->config.K0) - (pp->config.K[0] * x_1 + pp->config.K[1] * x_2);

   // init u_sat with u
   pp->state.u_sat = pp->state.u;

   // Apply saturation of actuator
   if( pp->state.u < pp->config.uMin ) {
      pp->state.u_sat = pp->config.uMin;
   }
   if( pp->state.u > pp->config.uMax ) {
      pp->state.u_sat = pp->config.uMax;
   }

   return pp->state.u_sat;
}

// Update PID controller for next iteration
void ppUpdateController( PPController_t* pp, float y2, float y, float r )
{

#ifdef PP_PRINT_RESULT
   /*----- Print PID results if is activated --------------------------------*/
   ppGatherDebugSamples (pp, y2, y, r);
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

// PP printf object
void ppPrintf( PPController_t* pp )
{   
   float32_t buffer[1];
   printf( "Pole Placement structure (object)\r\n" );
   printf( "----------------------\r\n\r\n" );
   buffer[0] = pp->config.K0;
   printf( "\n\t K0 = " );
   console_print (buffer);
   buffer[0] = pp->config.K[0];
   printf( "\n\t  K_1 = ");
   console_print (buffer);
   buffer[0] = pp->config.K[1];
   printf( "\n\t  K_2 = ");
   console_print (buffer);
   buffer[0] = pp->config.uMax;
   printf( "\n\t  umax = ");
   console_print (buffer);
   buffer[0] = pp->config.uMin;
   printf( "\n\t  umin = ");
   console_print (buffer);
   printf( "\r\n" );
}


#ifdef PP_PRINT_RESULT
static void ppGatherDebugSamples( PPController_t* pp, float y2, float y, float r )
{
	char ntos[64];

   // Save and show samples
   if( i< N_SAMPLES ) {
      if (i == 0) {
         uartWriteString( UART_USB, "\r\n\r\nGathering " );
         uint64ToString( N_SAMPLES, ntos, 10 );
         uartWriteString( UART_USB, ntos );
         uartWriteString( UART_USB, " PP samples...\r\n\r\n" );
      }

      // Save N_SAMPLES samples of R[k] and Y[k]
      r_array[i] = r;
      y_array[i] = y;
      y2_array[i] = y;
      u_array[i] = pp->state.u;
      usat_array[i] = pp->state.u_sat;
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

      // print y2[k] samples
      uartWriteString( UART_USB, "y_2 = [ " );
      for( i=INIT_SAMPLES; i<N_SAMPLES; i++) {
         uartWriteString( UART_USB, floatToString( y2_array[i], ntos, 5 ) );
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

      uartWriteString( UART_USB, "All samples printed. PROGRAM HALTED.\r\n" );
      gpioWrite( LED2, ON );

      while(1);
   }
}

#endif

/*=====[Implementations of interrupt functions]==============================*/

/*=====[Implementations of private functions]================================*/
