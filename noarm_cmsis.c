/*
 * noarm_cmsis.c
 *
 *  Created on: 23 mag 2017
 *      Author: Simone
 */


#include"noarm_cmsis.h"

void nonARM_product(  float32_t * pSrcA,float32_t * pSrcB, float32_t * pDst, int maxb)
{
	static int j;
	*(pDst) = 0;
	for(j = 0;j<maxb;j++)
	  {
		  *(pDst) += *(pSrcA + j) *  *(pSrcB+ j);

	  }

}

void noARM_FIR(float32_t * Input_f32_1kHz_15kHz, float32_t * FIRCoeffs32, int TEST_LENGTH, int N_TAPS,float32_t *Output, float32_t * x_i)
{
    int i,k;
    float yn = 0;
	for(i=0; i<TEST_LENGTH; i++)
    {
                                                 //  Alternative implementation
        for( k=0; k < N_TAPS-1; k++)               //  for(int k=N-1; k>0; k--)
        {                                        //  {
        	*(x_i + N_TAPS-k-1) = *(x_i +N_TAPS-k-2);//shift the data   //    x[k] = x[k-1];
        }                                        //  }


        *(x_i) = Input_f32_1kHz_15kHz[i]; // move input sample to buffer
        yn = 0; // clear output sample

        for(int k=0; k < N_TAPS; k++)
        {
            yn += FIRCoeffs32[k] * *(x_i + k); // multiply data on coefficients with accumulation
        }

        *(Output + i) = yn;
    }


}

