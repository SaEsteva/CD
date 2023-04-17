/*=====[observador]========================================================
 * Copyright 2023 Santiago Esteva <sestevafi.uba.ar> * All rights reserved.
 * License: BSD-3-Clause <https://opensource.org/licenses/BSD-3-Clause>)
 *
 * Version: 1.3.0
 * Creation Date: 2023/04/15
 */
#ifndef _OBSERVER_H_
#define _OBSERVER_H_

/*=====[Inclusions of public function dependencies]==========================*/

#include "sapi.h"

/*=====[C++ - begin]=========================================================*/

#ifdef __cplusplus
extern "C" {
#endif

/*=====[Definition macros of public constants]===============================*/

#define OBS_PRINT_RESULT

// #define OPEN_LOOP

#ifdef OBS_PRINT_RESULT
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
	float L;
	float A[4];
	float B[2];
	float C[2];
   float uMin;
   float uMax;
} observer_config_t;

typedef struct {
   float u;
   float u_sat;
   float x_hat[2];
} observer_State_t;

typedef struct {
   observer_config_t config;
   observer_State_t state;
} Observer_t;

/*=====[Prototypes (declarations) of public functions]=======================*/

void Observer_init(Observer_t *obs, float K_1, float K_2,float K0,float L,float A_1,float A_2,float A_3,float A_4,float B_1,float B_2,float C_1,float C_2,float uMin, float uMax);
float Observer(Observer_t *obs, float y, float reference) ;
void obsUpdate( Observer_t* obs, float y, float r );
void obsPrintf( Observer_t* obs );

/*=====[Prototypes (declarations) of public interrupt functions]=============*/

/*=====[C++ - end]===========================================================*/

#ifdef __cplusplus
}
#endif

/*=====[Avoid multiple inclusion - end]======================================*/

#endif /* _PP_CONTROLLER_H_ */