/*
 * gneMain.h
 *
 *  Created on: 12 mar 2023
 *      Author: qubec
 */

#ifndef SRC_GEN_GENMAIN_H_
#define SRC_GEN_GENMAIN_H_

#include "stm32l4xx_hal.h"

#define _PI		3.14159265359
#define _2PI	6.28318530718

typedef enum _genWaveShape_t{
	genShapeSquare, genShapeTriangle, genShapeSine }genWaveShape_t;


uint16_t *genCreateWave( genWaveShape_t shape, float frequency, float dutyCycle, float peakPos,
		float amplitude, float bias, uint32_t sps, uint32_t *numPointsOut, float *realFrequencyOut );




#endif /* SRC_GEN_GENMAIN_H_ */
