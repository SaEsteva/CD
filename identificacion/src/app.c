/*=====[app]===================================================================
 * Copyright 2019 Diego Fernández <dfernandez202@gmail.com>
 * All rights reserved.
 * License: BSD-3-Clause <https://opensource.org/licenses/BSD-3-Clause>)
 *
 * Version: 1.0.0
 * Creation Date: 2019/09/23
 */

/*=====[Inclusions of function dependencies]=================================*/

#include "FreeRTOS.h"
#include "task.h"

#include "sapi.h"

#include "arm_math.h"

#include "identification_ls.h"
// #include "identification_rls.h"
#include "identification_tasks.h"

/*=====[Definition macros of private constants]==============================*/

// ADC0 Channels for u and y
#define ADC0_CH_Y       CH1

// Noise signal limits
#define DAC_REFERENCE_VALUE_HIGH   666  // 1023 = 3.3V, 666 = 2.15V
#define DAC_REFERENCE_VALUE_LOW    356  // 1023 = 3.3V, 356 = 1.15V

// adcRead() returns 10 bits integer sample (uint16_t)
// sampleInVolts = (3.3 / 1023.0) * adcSample
#define getVoltsSampleFrom(adc0Channel) 3.3*(float)adcRead((adc0Channel))/1023.0

/*=====[Definitions of extern global variables]==============================*/

/*=====[Definitions of public global variables]==============================*/

/*=====[Definitions of private global variables]=============================*/

static StackType_t taskIdentificationStack[configMINIMAL_STACK_SIZE*15];
static StaticTask_t taskIdentificationTCB;

// t_IRLSdata* tIRLS1;
t_ILSdata* tILS1;

/*=====[Prototypes (declarations) of private functions]======================*/

void receiveData (float* buffer);

/*=====[Main function, program entry point after power on or reset]==========*/

int main (void)
{
    // ----- Configurations --------------------------
    boardConfig();

    // ----- Initialize ADC and DAC ------------------
    adcInit( ADC_ENABLE );
    dacInit( DAC_ENABLE );

    // tIRLS1 = (t_IRLSdata*) pvPortMalloc (sizeof(t_IRLSdata));
    tILS1 = (t_ILSdata*) pvPortMalloc (sizeof(t_ILSdata));

    // IRLS_Init(tIRLS1, 10, receiveData);
	ILS_Init(tILS1, 50, 10, receiveData);

	xTaskCreateStatic(
        ILS_Task,                   // task function
        "Identification Task",      // human-readable neame of task
        configMINIMAL_STACK_SIZE,   // task stack size
        (void*)tILS1,               // task parameter (cast to void*)
        tskIDLE_PRIORITY+1,         // task priority
        taskIdentificationStack,    // task stack (StackType_t)
        &taskIdentificationTCB      // pointer to Task TCB (StaticTask_t)
    );

    // xTaskCreateStatic(
    //     IRLS_Task,                  // task function
    //     "Identification Task",      // human-readable neame of task
    //     configMINIMAL_STACK_SIZE,   // task stack size
    //     (void*)tIRLS1,              // task parameter (cast to void*)
    //     tskIDLE_PRIORITY+1,         // task priority
    //     taskIdentificationStack,    // task stack (StackType_t)
    //     &taskIdentificationTCB      // pointer to Task TCB (StaticTask_t)
    // );

    vTaskStartScheduler();
    // ----- Repeat for ever -------------------------
    while(TRUE);

    // YOU NEVER REACH HERE, because this program runs directly or on a
    // microcontroller and is not called by any Operating System, as in the 
    // case of a PC program.
    return 0;
}

/*=====[Implementations of private functions]================================*/

// Generación del DAC y captura del ADC
void receiveData (float* buffer)
{
    float Y, U;
    
    uint16_t dacValue = 0;

    // random = limite_inferior + rand() % (limite_superior +1 - limite_inferior);
    dacValue = DAC_REFERENCE_VALUE_LOW + rand() % (DAC_REFERENCE_VALUE_HIGH+1 - DAC_REFERENCE_VALUE_LOW);

    dacWrite( DAC, dacValue );
    
    // Need at least 2.5 us to uptate DAC.
    delayInaccurateUs(5); 

    // dacSample = (1023.0 / 3.3) * sampleInVolts
    // 1023.0 / 3.3 = 310.0
    U = (float) dacValue * 3.3 / 1023.0;
	Y = (float) getVoltsSampleFrom( ADC0_CH_Y );

	buffer[0] = U;
	buffer[1] = Y;
}
