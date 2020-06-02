/*
  ******************************************************************************
Write a program that performs the following:
• Samples the accelerometer at 100Hz
• Implements a Low Pass FIR filter to filter each accelerometer axis at 5Hz Implements the computation of planar orientation from filtered accelerometer data
• Implements the computation of the mean on orientation data on 5 samples window
• Using the mean on orientation data, turns on only the LED oriented to the ground. Others must be off.
Implement the following user interface:
• At startup initialize everything, turn on green LED (for USB it is already turned on when a cable is connected) and wait for user input
• The following commands should be supported: ‘s’: start/stop streaming ‘d’: toggles data streaming or result streaming
• When not streaming, LEDs must be OFF.
  ******************************************************************************
*/

/*********** Includes  ****************/
#include "stm32f4xx.h"
#include "stm32f4_discovery.h"
#include "stm32f4_discovery_lis3dsh.h"

#include <stdio.h>

#include "arm_math.h"
#include "math_helper.h"
#include "noarm_cmsis.h"



/*********** Defines   ****************/
#define STREAM_PERIOD_MS 1000
#define LED_PERIOD_MS 500

/* ----------------------------------------------------------------------
* Defines for each of the tests performed
* ------------------------------------------------------------------- */
#define TEST_LENGTH_SAMPLES  320
#define SNR_THRESHOLD_F32    140.0f
#define BLOCK_SIZE            32
#define NUM_TAPS              29


/*********** Declarations *************/
/*------ Function prototypes ---------*/
void USART_Config(void);


/*------ Global variables  -----------*/
u8 streamActive = 0;
u8 dataReady = 0;

u8 dataReceived = 0;
u8 chRX = 0;

/* -------------------------------------------------------------------
 * The input signal and reference output (computed with MATLAB)
 * are defined externally in arm_fir_data.c.
 * ------------------------------------------------------------------- */

extern float32_t testInput_f32_1kHz_15kHz[TEST_LENGTH_SAMPLES];
extern float32_t refOutput[TEST_LENGTH_SAMPLES];

/* -------------------------------------------------------------------
 * Declare Test output buffer
 * ------------------------------------------------------------------- */

static float32_t testOutput[TEST_LENGTH_SAMPLES];

/* -------------------------------------------------------------------
 * Declare State buffer of size (numTaps + blockSize - 1)
 * ------------------------------------------------------------------- */

static float32_t firStateF32[BLOCK_SIZE + NUM_TAPS - 1];

/* ----------------------------------------------------------------------
 ** FIR Coefficients buffer generated using fir1() MATLAB function.
 ** fir1(28, 6/24)
 ** ------------------------------------------------------------------- */

const float32_t firCoeffs32[NUM_TAPS] = {
		-0.0018225230f, -0.0015879294f, +0.0000000000f, +0.0036977508f, +0.0080754303f, +0.0085302217f, -0.0000000000f, -0.0173976984f,
		-0.0341458607f, -0.0333591565f, +0.0000000000f, +0.0676308395f, +0.1522061835f, +0.2229246956f, +0.2504960933f, +0.2229246956f,
		+0.1522061835f, +0.0676308395f, +0.0000000000f, -0.0333591565f, -0.0341458607f, -0.0173976984f, -0.0000000000f, +0.0085302217f,
		+0.0080754303f, +0.0036977508f, +0.0000000000f, -0.0015879294f, -0.0018225230f
};


static float x[NUM_TAPS];



/* ----------------------------------------------------------------------
* Declare Global variables
* ------------------------------------------------------------------- */

uint32_t blockSize = BLOCK_SIZE;
uint32_t numBlocks = TEST_LENGTH_SAMPLES/BLOCK_SIZE;


/******** Cycle counter defines  **********/
volatile unsigned int cyc[2];
volatile unsigned int *DWT_CYCCNT = (volatile unsigned int *)0xE0001004; 	// Cycle counter
volatile unsigned int *DWT_CONTROL= (volatile unsigned int *)0xE0001000;	// counter control
volatile unsigned int *SCB_DEMCR  = (volatile unsigned int *)0xE000EDFC;
#define STOPWATCH_START {cyc[0]=*DWT_CYCCNT;} 								// start counting
#define STOPWATCH_STOP  {cyc[1]=*DWT_CYCCNT; cyc[1]=cyc[1]-cyc[0];}			// stop counting, result is in cyc[1]


/***********   Main  ******************/
int main(void)
{

	int i,k,j;
	arm_fir_instance_f32 S;
	arm_status status;
	float32_t  *inputF32, *outputF32;
	float32_t  snr;

	float yn = 0;
	int result[2] = {0,0};

	// init, reset and start the cycle counter
	*SCB_DEMCR = *SCB_DEMCR | 0x01000000;
	*DWT_CYCCNT = 0; 							// reset the counter
	*DWT_CONTROL = *DWT_CONTROL | 1 ; 			// enable the counter


	// LED initialization
    STM_EVAL_LEDInit(LED3);		// Orange
    STM_EVAL_LEDInit(LED4);		// Green
    STM_EVAL_LEDInit(LED5);		// Red
    STM_EVAL_LEDInit(LED6);		// Blue

    /* USART configuration */
	USART_Config();

	/* SysTick configuration */
	if (SysTick_Config(SystemCoreClock / 1000)) {
	/* Capture error */
		while(1);
	}

	STM_EVAL_LEDOn(LED4);

	while (1)
	{

		if (dataReceived == 1)
		{
			if (chRX == 's') {
				streamActive = 1 - streamActive;
	       		printf("Stream Toggle\r\n");
			} else {
				printf("Wrong command\r\n");
			}

			if (!streamActive)
			{
				STM_EVAL_LEDOff(LED6);
			}

			dataReceived = 0;
		}

		if (streamActive == 1)
		{
			if (dataReady == 1)
			{
				// NO CMSIS
			    yn = 0;
			    i = 0;

			    STOPWATCH_START	/* Start counting cycles */
				noARM_FIR(&testInput_f32_1kHz_15kHz,&firCoeffs32,TEST_LENGTH_SAMPLES,NUM_TAPS,&testOutput,&x);
			    STOPWATCH_STOP	/* Start counting cycles */
				result[0] = cyc[1];


				// CMSIS
				/* Initialize input and output buffer pointers */
				inputF32 = &testInput_f32_1kHz_15kHz[0];
				outputF32 = &testOutput[0];


				/* Call FIR init function to initialize the instance structure. */
				arm_fir_init_f32(&S, NUM_TAPS, (float32_t *)&firCoeffs32[0], &firStateF32[0], blockSize);

				/* ----------------------------------------------------------------------
				 ** Call the FIR process function for every blockSize samples
				 ** ------------------------------------------------------------------- */
				STOPWATCH_START	/* Start counting cycles */
				for(i=0; i < numBlocks; i++)
				{
					arm_fir_f32(&S, inputF32 + (i * blockSize), outputF32 + (i * blockSize), blockSize);
				}

				 STOPWATCH_STOP	/* Start counting cycles */
				 result[1] = cyc[1];
				/* ----------------------------------------------------------------------
				 ** Compare the generated output against the reference output computed
				 ** in MATLAB.
				 ** ------------------------------------------------------------------- */

				snr = arm_snr_f32(&refOutput[0], &testOutput[0], TEST_LENGTH_SAMPLES);

				printf("NO CMSIS: %d,\t CMSIS: %d\r\n", result[0], result[1]);

				dataReady = 0;

			}
		}


	}

}


/*********** Functions   ****************/

void USART_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Enable GPIO clock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	/* Enable UART clock */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

	/* Connect PXx to USARTx_Tx*/
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);

	/* Connect PXx to USARTx_Rx*/
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);

	/* Configure USART Tx as alternate function  */
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Configure USART Rx as alternate function  */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Configure the USART2 Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Enable the RX Interrupt */
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

	/* USARTx configured as follows:
		- BaudRate = 115200 baud
		- Word Length = 8 Bits
		- One Stop Bit
		- No parity
		- Hardware flow control disabled (RTS and CTS signals)
		- Receive and transmit enabled
	*/
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	/* USART initialization */
	USART_Init(USART2, &USART_InitStructure);

	/* Enable USART */
	USART_Cmd(USART2, ENABLE);

}



/*********** IRQ Handlers   ****************/

void SysTick_Handler(void)
{
	static int counterStream_ms = 0;
	static int counterLed_ms = 0;

	if (streamActive == 1)
	{
		counterStream_ms++;
		counterLed_ms++;

		if (counterStream_ms >= STREAM_PERIOD_MS)
		{
			dataReady = 1;
			counterStream_ms = 0;
		}

		if (counterLed_ms >= LED_PERIOD_MS)
		{
			STM_EVAL_LEDToggle(LED6);
			counterLed_ms = 0;
		}

	}

}




void USART2_IRQHandler(void)
{
	/* RX interrupt */
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
	{
		chRX = USART_ReceiveData(USART2);
		dataReceived = 1;


	}
}



/*********** printf define   ****************/

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
int __io_putchar(int ch)
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART */
  USART_SendData(USART2, (uint8_t)ch);

  /* Loop until the end of transmission */
  while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET)
  {}

  return ch;
}




