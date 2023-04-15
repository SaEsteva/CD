/*=====[pid_controller]========================================================
 * Copyright 2023 Santiago Esteva <sestevafi.uba.ar> * All rights reserved.
 * License: BSD-3-Clause <https://opensource.org/licenses/BSD-3-Clause>)
 *
 * Version: 1.3.0
 * Creation Date: 2023/04/15
 *===========================================================================*/

/*=====[Inclusions of function dependencies]=================================*/

#include "../../implementacion_pp_rtos/inc/FreeRTOSConfig.h"
#include "../../implementacion_pp_rtos/inc/ppTask.h"
#include "FreeRTOS.h"
#include "task.h"

#include "sapi.h"

/*=====[Definition macros of private constants]==============================*/

/*=====[Definitions of extern global variables]==============================*/

/*=====[Definitions of public global variables]==============================*/

/*=====[Definitions of private global variables]=============================*/
#define GENERATOR	GPIO3
/*=====[Main function, program entry point after power on or reset]==========*/
StackType_t myTaskStack[configMINIMAL_STACK_SIZE];
StaticTask_t myTaskTCB;


float ref;

// static StackType_t taskControlStack[configMINIMAL_STACK_SIZE*15];
// static StaticTask_t taskControlTCB;

// // PID Controller structure (like an object instance)
// PIDController_t* PsPIDController;

void Generator( void* taskParmPtr )
{
   // printf("Inicio la tarea del Generador \n");
   gpioInit( GENERATOR, GPIO_OUTPUT );
   gpioWrite( GENERATOR, ON );
   // Envia la tarea al estado bloqueado durante 1 s (delay)
   vTaskDelay( 10 / portTICK_RATE_MS );
   gpioWrite( GENERATOR, OFF );

   ref = 1.0;
   // Tarea periodica cada 500 ms
   portTickType xPeriodicity =  50 / portTICK_RATE_MS;
   portTickType xLastWakeTime = xTaskGetTickCount();

   while(TRUE) {
      gpioToggle( GENERATOR );
      if (ref == 1.0){
         ref = 2.0;
      } else{
         ref = 1.0;
      }
      // printf("Cambio \n");
      // Envia la tarea al estado bloqueado durante xPeriodicity (delay periodico)
      vTaskDelayUntil( &xLastWakeTime, xPeriodicity );
   }
}

int main( void )
{
   boardInit();

   // Create a task in freeRTOS with dynamic memory
   xTaskCreate(
      ppControlTask,                 // Function that implements the task.
      (const char *)"ppControlTask", // Text name for the task.
      configMINIMAL_STACK_SIZE*2,     // Stack size in words, not bytes.
      0,                              // Parameter passed into the task.
      tskIDLE_PRIORITY+1,             // Priority at which the task is created.
      0                               // Pointer to the task created in the system
   );

   xTaskCreateStatic( Generator, "Generator", configMINIMAL_STACK_SIZE, NULL,tskIDLE_PRIORITY+1, myTaskStack, &myTaskTCB);

   vTaskStartScheduler(); // Initialize scheduler

   while( true ); // If reach heare it means that the scheduler could not start

   // YOU NEVER REACH HERE, because this program runs directly or on a
   // microcontroller and is not called by any Operating System, as in the 
   // case of a PC program.
   return 0;
}
