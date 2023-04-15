/*=====[identification_task]===================================================
 * Copyright 2019 Diego Fernández <dfernandez202@gmail.com>
 * All rights reserved.
 * License: BSD-3-Clause <https://opensource.org/licenses/BSD-3-Clause>)
 *
 * Version: 1.0.0
 * Creation Date: 2019/09/23
 */
 
/*=====[Inclusions of private function dependencies]=========================*/

#include "identification_tasks.h"

#include "sapi.h"

#include "arm_math.h"
#include <math.h>

#include "identification_ls.h"
// #include "identification_rls.h"

/*=====[Definition macros of private constants]==============================*/

/*=====[Private function-like macros]========================================*/

/*=====[Definitions of private data types]===================================*/

/*=====[Definitions of external public global variables]=====================*/

/*=====[Definitions of public global variables]==============================*/

/*=====[Definitions of private global variables]=============================*/

/*=====[Prototypes (declarations) of private functions]======================*/

void console_print (float* buffer);

/*=====[Implementations of public functions]=================================*/

void ILS_Task (void* taskParmPtr)
{
	t_ILSdata* tILS;
	portTickType xLastWakeTime;

	tILS = (t_ILSdata*) taskParmPtr;

	// Para implementar un delay relativo
	xLastWakeTime = xTaskGetTickCount();

	for(;;)
	{
        // Ejecuto el Identificador
        ILS_Run(tILS);

        if(tILS->i==2)
        {
            // Imprimo los parámetros calculados
            console_print(tILS->buffer_T);
        }

		vTaskDelayUntil( &xLastWakeTime, ( tILS->ts_Ms / portTICK_RATE_MS ) );
	}
}

// void IRLS_Task( void* taskParmPtr )
// {
//     t_IRLSdata* tIRLS;
// 	portTickType xLastWakeTime;

// 	tIRLS = (t_IRLSdata*) taskParmPtr;

// 	// Para implementar un delay relativo
// 	xLastWakeTime = xTaskGetTickCount();

// 	for(;;)
// 	{
//         // Ejecuto el Identificador
//         IRLS_Run(tIRLS);

//         // Imprimo los parámetros calculados
//         console_print(tIRLS->buffer_T);

// 		vTaskDelayUntil( &xLastWakeTime, ( tIRLS->ts_Ms / portTICK_RATE_MS ) );
// 	}
// }

/*=====[Implementations of interrupt functions]==============================*/

/*=====[Implementations of private functions]================================*/

// Función para imprimir los parámetros la planta (2do orden)
void console_print (float* buffer)
{
	uint32_t i;
	int32_t integer, fraction;

	// Imprimo los parámetros calculados
	printf ("Parametros = [\t");
	for (i = 0; i<5; i++)
	{
		// Casteo de float a int
		integer = (int)buffer[i];
		fraction = (int)(((buffer[i] - (float)integer)) * 1000);
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
        printf(",");
	}
	printf("]\n");
}