/*=====[pid_controller]========================================================
 * Copyright 2023 Santiago Esteva <sestevafi.uba.ar> * All rights reserved.
 * License: BSD-3-Clause <https://opensource.org/licenses/BSD-3-Clause>)
 *
 * Version: 1.3.0
 * Creation Date: 2023/04/15
 *===========================================================================*/

/*=====[Inclusion of own header]=============================================*/

#include "../../implementacion_pp_rtos/inc/ppTask.h"

#include "../../implementacion_pp_rtos/inc/pp_controller.h"
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
void ppControlTask( void* taskParmPtr )
{
	// Controller signals
	float r = 0.0f; // Measure of reference r[k]
	float y = 0.0f; // Measure of system output y[k]
	float y2 = 0.0f; // Measure of first state
	float u = 0.0f; // Calculated controller's output u[k]
	// h no puede ser menor ni al tiempo del algoritmo, y con va a ser un
	// multiplo del periodo de tick del RTOS
	uint32_t h_ms = 1; // Task periodicity (sample rate)
	float h_s = ((float)h_ms)/100.0f;
	
	// Enable ADC/DAC
	adcInit( ADC_ENABLE );
	dacInit( DAC_ENABLE );

	PPController_t PsPPController;

	// Control sobre compensado
	// pole_placement_init( &PsPPController, -0.38135351f, 0.13f,0.74387057f,0.0f,3.3f);
	// Control 2
	// pole_placement_init( &PsPPController, -0.22135351f, 0.1044f,0.88082428f,0.0f,3.3f);
	// COntrol 3
	// pole_placement_init( &PsPPController, -0.02135351f,0.0904f,1.07035843f,0.0f,3.3f);
	// Control 4
	// pole_placement_init( &PsPPController, -0.08135351,0.0925f,1.0113582f,0.0f,3.3f);
	
	pole_placement_init( &PsPPController, 0.01464649f,0.360004f,1.38176914f,0.0f,3.3f);

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
			// Leer salida y[k] y el estado y2[k]
			y = adcRead( CH1 ) * SCALE_Y;
			y2 = adcRead( CH2 ) * SCALE_Y;
			r = ref;

			// Calculate PP controller output u[k]
			u = pole_placement_control(&PsPPController,y2, y, r ) * SCALE_U;

			// Actualizar la salida u[k]
			dacWrite( DAC, u);
		#endif

		// Update Pole Placement controller for next iteration
		ppUpdateController( &PsPPController, y2, y, r );

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