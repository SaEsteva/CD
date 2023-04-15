/*=====[pid_controller]========================================================
 * Copyright 2023 Santiago Esteva <sestevafi.uba.ar> * All rights reserved.
 * License: BSD-3-Clause <https://opensource.org/licenses/BSD-3-Clause>)
 *
 * Version: 1.3.0
 * Creation Date: 2023/04/15
 */
#ifndef _PP_CONTROLLER_H_
#define _PP_CONTROLLER_H_

/*=====[Inclusions of public function dependencies]==========================*/

#include "sapi.h"

/*=====[C++ - begin]=========================================================*/

#ifdef __cplusplus
extern "C" {
#endif

/*=====[Definition macros of public constants]===============================*/

#define PP_PRINT_RESULT

// #define OPEN_LOOP

#ifdef PP_PRINT_RESULT
// Number of samples to save
#define N_SAMPLES       150
#define INIT_SAMPLES    28
#endif

/*=====[Public function-like macros]=========================================*/

#define getVoltsSampleFrom(adc0Channel) 3.3*(float)adcRead((adc0Channel))/1023.0

/*=====[Definitions of public data types]====================================*/

typedef struct {
	float K[2];
	float K0;
   float uMin;
   float uMax;
} pole_placement_config_t;

typedef struct {
   float u;
   float u_sat;
} pole_placement_State_t;

// PID strcuture (object)
typedef struct {
   pole_placement_config_t config;
   pole_placement_State_t state;
} PPController_t;

/*=====[Prototypes (declarations) of public functions]=======================*/

// PP Controller Initialization
void pole_placement_init(PPController_t *pp, float K_2,float K_1,float K0,float uMin, float uMax);

float pole_placement_control(PPController_t *pp, float x_1, float x_2, float reference);

void ppUpdateController(
   PPController_t* pp, // PID strcuture (object) by reference
   float y2,              // Measure of process output
   float y,              // Measure of process output
   float r               // Measure of process reference
);

// PID printf object
void ppPrintf( PPController_t* pp );

/*=====[Prototypes (declarations) of public interrupt functions]=============*/

/*=====[C++ - end]===========================================================*/

#ifdef __cplusplus
}
#endif

/*=====[Avoid multiple inclusion - end]======================================*/

#endif /* _PP_CONTROLLER_H_ */