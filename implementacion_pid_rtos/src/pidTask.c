/*=============================================================================
 * Copyright (c) 2019, Eric Pernia <ericpernia@gmail.com>
 * All rights reserved.
 * License: bsd-3-clause (see LICENSE.txt)
 * Date: 2019/10/8
 * Version: 1.1.0
 *
 * Changelog:
 * Version 1.1.0, 2019/10/8, Santiago Germino <sgermino@retro-ciaa.com>:
 *    1) Name change for SCALE_OUT(Y), SCALE_REF(R), SCALE_PID_OUT(U).
 *    2) Proper values for SCALE_Y, SCALE_R and SCALE_U.
 *    3) Proper scaling of Ki and Kd by h_s.
 *    4) Added calls to enable and initialize ADC & DAC.
 *===========================================================================*/

/*=====[Inclusion of own header]=============================================*/

#include "../../implementacion_pid_rtos/inc/pidTask.h"

#include "../../implementacion_pid_rtos/inc/pid_controller.h"
#include "sapi.h"

/*=====[Definition macros of private constants]==============================*/

#define SCALE_Y         0.003225806f    // 3.3 / 1023     10-bit ADC to Voltage
#define SCALE_R         0.003225806f    // 3.3 / 1023     10-bit ADC to Voltage
#define SCALE_U         310.0f          // 1023 / 3.3     Voltage to 10-bit DAC

extern float ref;
/*=====[Private function-like macros]========================================*/

/*=====[Definitions of private data types]===================================*/

/*=====[Definitions of external public global variables]=====================*/

/*=====[Definitions of public global variables]==============================*/

/*=====[Definitions of private global variables]=============================*/

/*=====[Prototypes (declarations) of private functions]======================*/

/*=====[Implementations of public functions]=================================*/

// Para calcular el tiempo que tarda el algoritmo y establecer un h minimo
// #define COUNT_CYCLES

// Task implementation
void pidControlTask( void* taskParmPtr )
{
   // Controller signals
   float r = 0.0f; // Measure of reference r[k]
   float y = 0.0f; // Measure of system output y[k]
   float u = 0.0f; // Calculated controller's output u[k]

   // h no puede ser menor ni al tiempo del algoritmo, y con va a ser un
   // multiplo del periodo de tick del RTOS
   uint32_t h_ms = 1; // Task periodicity (sample rate)
   float h_s = ((float)h_ms)/100.0f;

   // Enable ADC/DAC
   adcInit( ADC_ENABLE );
   dacInit( DAC_ENABLE );

   // PID Controller structure (like an object instance)
   PIDController_t PsPIDController;

   // PID Controller Initialization

   pidInit( &PsPIDController, 0.8f,0.02,0.005f,h_s,20.0f,1.0f,0.0f,3.3f);
   // Control mas "agresivo"
   // pidInit( &PsPIDController, 1.5f,0.02,0.005f,h_s,20.0f,1.0f,0.0f,3.3f);
   
   // Peridodic task each h_ms
   portTickType xPeriodicity =  h_ms / portTICK_RATE_MS;
   portTickType xLastWakeTime = xTaskGetTickCount();

   #ifdef COUNT_CYCLES
      // Configura el contador de ciclos con el clock de la EDU-CIAA NXP
      cyclesCounterConfig(EDU_CIAA_NXP_CLOCK_SPEED);
      volatile uint32_t cyclesElapsed = 0;
   #endif
   // gpioToggle( LED3 );

   while(true) {

      #ifdef COUNT_CYCLES
         // Resetea el contador de ciclos
         cyclesCounterReset();
      #endif
      // gpioToggle( LED3 );
      
      #ifdef OPEN_LOOP
         // Lazo abierto
         // r = adcRead( CH2 );
         r = ref / SCALE_R;
         dacWrite( DAC, r);
         r = r * SCALE_R;
         y = adcRead( CH1 ) * SCALE_Y;
      #else
         // Leer salida y[k] y refererencia r[k]
         y = adcRead( CH1 ) * SCALE_Y;
         // r = adcRead( CH2 ) * SCALE_R;
         r = ref;

         // Calculate PID controller output u[k]
         u = pidCalculateControllerOutput( &PsPIDController, y, r ) * SCALE_U;

         // Actualizar la salida u[k]
         dacWrite( DAC, u);
      #endif

      // Update PID controller for next iteration
      pidUpdateController( &PsPIDController, y, r );

      // printf("%f-%f,",r,u);
      #ifdef COUNT_CYCLES
         // Leer conteco actual de ciclos
         cyclesElapsed = DWT_CYCCNT;
         volatile float us = cyclesCounterToUs(cyclesElapsed);
      #endif

      // Send the task to the locked state during xPeriodicity
      // (periodical delay)
      vTaskDelayUntil( &xLastWakeTime, xPeriodicity );
   }
}

/*=====[Implementations of interrupt functions]==============================*/

/*=====[Implementations of private functions]================================*/

