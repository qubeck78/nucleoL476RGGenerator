/*
 * genMain.c
 *
 *  Created on: 12 mar 2023
 *      Author: qubec
 */

#include "genMain.h"
#include "math.h"


uint16_t wave[10000];

uint16_t *genCreateWave( genWaveShape_t shape, float frequency, float dutyCycle, float peakPos,
		float amplitude, float bias, uint32_t sps, uint32_t *numPointsOut, float *realFrequencyOut )
{
	uint32_t	i;
	uint32_t	j;
	uint32_t	numPoints;
	uint32_t	dutyCyclePoints;
	uint32_t	peakPosPoints;
	uint32_t	sineHalf1Pos;
	uint32_t	sineHalf2Pos;

	double		periodTime;
	double		baseTime;
	double		amplitudeDAC;

	uint16_t	biasDAC;

	periodTime			= 1.0 / frequency;
	baseTime	  		= 1.0 / sps;

	numPoints 			= periodTime / baseTime;

	*realFrequencyOut	= 1.0 / ( numPoints * baseTime );

	if( amplitude < 0 )
	{
		amplitude = 0;
	}
	if( amplitude > 3.3 )
	{
		amplitude = 3.3;
	}
	if( bias < 0 )
	{
		bias = 0;
	}
	if( bias > 3.3 )
	{
		bias = 3.3;
	}




	switch( shape )
	{

	case genShapeSquare:

		amplitudeDAC	= ( 4095 / 3.3 ) * amplitude;
		biasDAC			= ( 4095 / 3.3 ) * bias;

		dutyCyclePoints = numPoints * dutyCycle;

		for( i = 0; i < numPoints; i++ )
		{
			if( i > dutyCyclePoints )
			{
				wave[i] = biasDAC;

			}
			else
			{
				wave[i] = biasDAC + amplitudeDAC;

			}

			//banding
			if( wave[i] > 4095 )
			{
				wave[i] = 4095;
			}

		}

		break;

	case genShapeTriangle:

		amplitudeDAC	= ( 4095 / 3.3 ) * amplitude;
		biasDAC			= ( 4095 / 3.3 ) * bias;

		dutyCyclePoints = numPoints * dutyCycle;
		peakPosPoints	= dutyCyclePoints * peakPos;

		for( i = 0; i < dutyCyclePoints; i++ )
		{
			if( i > peakPosPoints )
			{
				//falling
				j = i - peakPosPoints;

				wave[i] = biasDAC + amplitudeDAC * ( 1 - ( (float)j / (float)( dutyCyclePoints - peakPosPoints ) ) );

			}
			else
			{
				//rising

				wave[i] = biasDAC + amplitudeDAC * ( (float)i / ( (float)peakPosPoints ) );

			}

			//banding
			if( wave[i] > 4095 )
			{
				wave[i] = 4095;
			}

		}
		for( i = dutyCyclePoints; i < numPoints; i++ )
		{
			wave[i] = biasDAC;
		}


		break;

	case genShapeSine:

		//sine wave needs amp peak to peak
		amplitude		/= 2;

		amplitudeDAC	= ( 4095 / 3.3 ) * amplitude;
		biasDAC			= ( 4095 / 3.3 ) * bias;

		/*
		 * standard sine
		 *
		if( dutyCycle >= 1.0 )
		{
			for( i = 0; i < numPoints; i++ )
			{
				wave[i] = biasDAC + sin( i * _2PI / numPoints ) * amplitudeDAC;

				//banding
				if( wave[i] > 4095 )
				{
					wave[i] = 4095;
				}
			}
		}
		*/

		//Parameterized sine, with duty cycle and peak position

		dutyCyclePoints = ( numPoints * dutyCycle ) / 2;	//calc for 1 sine half
		peakPosPoints	= dutyCyclePoints * peakPos;

		sineHalf1Pos	= ( numPoints / 4 ) - ( dutyCyclePoints / 2 );
		sineHalf2Pos	= ( numPoints / 2 ) + ( ( numPoints / 4 ) - ( dutyCyclePoints / 2 ) );

		for( i = 0; i < numPoints; i++ )
		{
			wave[i] = biasDAC;
		}

		j = dutyCyclePoints - peakPosPoints;

		for( i = 0; i < dutyCyclePoints; i++ )
		{

			if( i > peakPosPoints )
			{
				//falling

				//1st half
				wave[i + sineHalf1Pos] = biasDAC + cos( ( i - peakPosPoints ) * _PI / j / 2 ) * amplitudeDAC;

				//banding
				if( wave[i + sineHalf1Pos] > 4095 )
				{
					wave[i + sineHalf1Pos] = 4095;
				}


				//2nd half

				wave[i + sineHalf2Pos] = biasDAC - cos( ( i - peakPosPoints ) * _PI / j / 2 ) * amplitudeDAC;

				//banding
				if( wave[i + sineHalf2Pos] > 4095 )
				{
					wave[i + sineHalf2Pos] = 4095;
				}
			}
			else
			{

				//rising

				//1st half
				wave[i + sineHalf1Pos] = biasDAC + sin( i * _PI / peakPosPoints / 2 ) * amplitudeDAC;

				//banding
				if( wave[i + sineHalf1Pos] > 4095 )
				{
					wave[i + sineHalf1Pos] = 4095;
				}


				//2nd half

				wave[i + sineHalf2Pos] = biasDAC - sin( i * _PI / peakPosPoints / 2 ) * amplitudeDAC;

				//banding
				if( wave[i + sineHalf2Pos] > 4095 )
				{
					wave[i + sineHalf2Pos] = 4095;
				}

			}
		}

		break;

	default:

		break;
	}

	*numPointsOut = numPoints;

	return &wave;
}
