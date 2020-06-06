/*
 **********

 ****
 Write a program that performs the following:
 • Samples the accelerometer at 100Hz
 • Implements a Low Pass FIR filter to filter each accelerometer axis at 5Hz Implements the computation of planar orientation from filtered accelerometer data
 • Implements the computation of the mean on orientation data on 5 samples window
 • Using the mean on orientation data, turns on only the LED oriented to the ground. Others must be off.
 Implement the following user interface:
 • At startup initialize everything, turn on green LED (for USB it is already turned on when a cable is connected) and wait for user input
 • The following commands should be supported:
 ‘s’: start/stop streaming
 ‘d’: toggles data streaming or result streaming
 • When not streaming, LEDs must be OFF.

 Data streaming:
 • send accelerometer data (in g or mg) at 5Hz.
 • Sample format: X: 0000 Y: 0000 Z: 0000 (you can also stream only numbers in csv)
 Result streaming:
 • Send computed mean on orientation data (in degrees) at 20Hz.
 • Sample format: roll: 0000 pitch: 0000 (you can also stream only numbers in csv)
 ****

 */

/* Includes  **/


#include "stm32f4xx.h"
#include "stm32f4_discovery.h"
#include "stm32f4_discovery_lis3dsh.h"

#include <stdio.h>
#include <math.h>

#include "arm_math.h"
#include "math_helper.h"
#include "noarm_cmsis.h"

/* Defines   **/
#define STREAM_PERIOD_MS 1000
#define LED_PERIOD_MS 500

/* ----------------------------------------------------------------------
 * Defines for each of the tests performed
 * ------------------------------------------------------------------- */

#define SNR_THRESHOLD_F32    140.0f
#define BLOCK_SIZE           10 //dimensione finestra FIR
#define NUM_TAPS             10 //number of coefficients
#define MAX_LENGHT 			 10 //lunghezza buffer
#define WINDOW_MEAN 		 5 //finestra di elementi di cui fare la media

#define TIM3_CK_CNT   50000

/* Declarations */
/**------ Function prototypes --------*-*/
void USART_Config(void);
void Acc_Config(void);
void Ground_Orientation(float pitch, float roll);
void TimerConfiguration(void);

float mean(float array[]);
void shift(float buffer[], int dim);
/*------ Global variables  ----------*/
u8 streamActive = 0;
u8 streamData = 1; //all'inizio streammo dati grezzi
u8 streamResult = 0;
u8 streamDataFlag = 0;
u8 streamResultFlag = 0;

u8 dataReady = 0;

u8 dataReceived = 0;
u8 chRX = 0;

/* -------------------------------------------------------------------
 * Declare State buffer of size (numTaps + blockSize - 1)
 * ------------------------------------------------------------------- */
static float32_t firStateF32[BLOCK_SIZE + NUM_TAPS - 1];

/* ----------------------------------------------------------------------
 ** FIR Coefficients buffer generated using fir1() MATLAB function.
 ** fir1(28, 6/24)
 ** ------------------------------------------------------------------- */
const float32_t firCoeffs32[NUM_TAPS] = { 0.0120f, 0.0326f, 0.0888f, 0.1590f,
		0.2076f, 0.2076f, 0.1590f, 0.0888f, 0.0326f, 0.0120f };

volatile uint16_t CCR1_Val = TIM3_CK_CNT / 100;
volatile uint16_t CCR2_Val = TIM3_CK_CNT / 5; //data streaming
volatile uint16_t CCR3_Val = TIM3_CK_CNT / 20; //result streaming

/* ----------------------------------------------------------------------
 * Declare Global variables
 * ------------------------------------------------------------------- */

uint32_t blockSize = BLOCK_SIZE;

/* Cycle counter defines  */
volatile unsigned int cyc[2];
volatile unsigned int *DWT_CYCCNT = (volatile unsigned int *) 0xE0001004; // Cycle counter
volatile unsigned int *DWT_CONTROL = (volatile unsigned int *) 0xE0001000; // counter control
volatile unsigned int *SCB_DEMCR = (volatile unsigned int *) 0xE000EDFC;
#define STOPWATCH_START {cyc[0]=*DWT_CYCCNT;} 								// start counting
#define STOPWATCH_STOP  {cyc[1]=*DWT_CYCCNT; cyc[1]=cyc[1]-cyc[0];}			// stop counting, result is in cyc[1]

/*   Main  **/
int main(void) {

	arm_fir_instance_f32 S;
	float bufferX[MAX_LENGHT] = { 0 };
	float bufferY[MAX_LENGHT] = { 0 };
	float bufferZ[MAX_LENGHT] = { 0 };
	float bufferOutX[MAX_LENGHT] = { 0 }; //valori di x in uscita dal filtro FIR
	float bufferOutY[MAX_LENGHT] = { 0 };
	float bufferOutZ[MAX_LENGHT] = { 0 };
	float meanX[WINDOW_MEAN] = { 0 };
	float meanY[WINDOW_MEAN] = { 0 };
	float meanZ[WINDOW_MEAN] = { 0 };
	float resultMeanX = 0;
	float resultMeanY = 0;
	float resultMeanZ = 0;
	float rad = 0;
	float pitch = 0;
	float roll = 0;

	int16_t accData[3] = { 0, 0, 0 };

	//init, reset and start the cycle counter
	*SCB_DEMCR = *SCB_DEMCR | 0x01000000;
	*DWT_CYCCNT = 0; 							// reset the counter
	*DWT_CONTROL = *DWT_CONTROL | 1; 			// enable the counter

	// LED initialization
	STM_EVAL_LEDInit(LED3);		// Orange
	STM_EVAL_LEDInit(LED4);		// Green
	STM_EVAL_LEDInit(LED5);		// Red
	STM_EVAL_LEDInit(LED6);		// Blue

	USART_Config();
	TimerConfiguration();
	Acc_Config();

//	/* SysTick configuration */
//	if (SysTick_Config(SystemCoreClock / 1000)) {
//		while (1);
//	}

	printf(" Type s for toggle stream\r\n");
	printf(" During stream type d to stream data or result\r\n");
	arm_fir_init_f32(&S, NUM_TAPS, (float32_t *) &firCoeffs32[0],&firStateF32[0], blockSize);
	arm_fir_init_f32(&S, NUM_TAPS, (float32_t *) &firCoeffs32[0],&firStateF32[0], blockSize);
	arm_fir_init_f32(&S, NUM_TAPS, (float32_t *) &firCoeffs32[0],&firStateF32[0], blockSize);
	while (1) {

		if (dataReceived == 1) {

			if (chRX == 's') {
				streamActive = !streamActive;
				printf("Stream Toggle\r\n");
			} else if (streamActive == 1) {
				if (chRX == 'd') {
					if (streamData == 1) {
						streamData = 0;
						streamResult = 1;
					}
					else if (streamResult == 1) {
						streamData = 1;
						streamResult = 0;
					}
					printf("Type of stream toggler \r\n");
				}
			} else {
				printf("wrong command \r\n");
			}

			dataReceived = 0;
		}

		if (streamActive == 1) { // se pigiando s ho attivato lo streaming

			if (dataReady == 1) { // quando arriva un dato dall'Acc calcolo filtro

				/* Call FIR init function to initialize the instance structure. */

				LIS3DSH_ReadACC(accData); //leggo accelerazione sui 3 assi

				shift(bufferX,MAX_LENGHT); // shift verso sinistra buffer FIFO dei campioni che prendo dal Acc 100 volte al secondo
				shift(bufferY,MAX_LENGHT);
				shift(bufferZ,MAX_LENGHT);

				bufferX[MAX_LENGHT - 1] = accData[0]; // il dato lo inserisco nell'ultima posizione del buffer
				bufferY[MAX_LENGHT - 1] = accData[1];
				bufferZ[MAX_LENGHT - 1] = accData[2];
				//MAX_LENGHT = 10, mettiamo MAX_LENGHT - 1 perché l'array va da 0 a 9

				arm_fir_f32(&S, bufferX, bufferOutX, blockSize); // chiamo filtro su tutto il buffer con il nuovo valor campionato
				arm_fir_f32(&S, bufferY, bufferOutY, blockSize);
				arm_fir_f32(&S, bufferZ, bufferOutZ, blockSize);

				shift(meanX,WINDOW_MEAN); // shitf verso sinistra il vettore di risultati di cui faccio la media
				shift(meanY,WINDOW_MEAN);
				shift(meanZ,WINDOW_MEAN);

				//chiamo il filtro
				meanX[WINDOW_MEAN - 1] = bufferOutX[MAX_LENGHT - 1]; // a regime l'unico valore che mi interressa è l'ultimo perchè i precedenti sono somme parziali
				meanY[WINDOW_MEAN - 1] = bufferOutY[MAX_LENGHT - 1];
				meanZ[WINDOW_MEAN - 1] = bufferOutZ[MAX_LENGHT - 1];
				resultMeanX = mean(meanX);
				resultMeanY = mean(meanY);
				resultMeanZ = mean(meanZ);

				// calcolare od ogni gira la media sui nuovo 5 elementi
			}
			dataReady = 0;

			if (streamData == 1) {
				if (streamDataFlag) {
					printf("the data from the acceleration are: %d, %d, %d\r\n", accData[0], accData[1], accData[2]);
					streamDataFlag = 0;
				}

			}

			if (streamResult == 1) {
				if (streamResultFlag) {
					rad = atan2(resultMeanX, sqrt((pow(resultMeanY, 2)) + pow(resultMeanZ, 2)));
					pitch = rad * (180 / PI);

					rad = atan2(resultMeanY, resultMeanZ);
					roll = rad * (180 / PI);

					Ground_Orientation(pitch, roll);

					printf(" pitch: %f°\troll %f°\r\n", pitch, roll);

					streamResultFlag = 0;
				}

			}

		}
	}

}

/* IRQ Handlers   **/
//SETTA UN FLAG PER IL MAIN CHE DECIDE L'AZIONE A SECODA DELLA LETTERE IMMESSA
void USART2_IRQHandler(void) {
	/* RX interrupt */
	if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) {
		chRX = USART_ReceiveData(USART2);
		dataReceived = 1;
	}
}

//void SysTick_Handler(void) {
//	static int counterStream_ms = 0;
//	static int counterData_ms = 0;
//	static int counterResult_ms = 0;
//
//	if (streamActive == 1) {
//		counterStream_ms++;
//		counterData_ms++;
//		counterResult_ms++;
//
//		if (counterStream_ms >= CCR1_Val) {
//			dataReady = 1;
//			counterStream_ms = 0;
//		}
//
//		if (counterStream_ms >= CCR2_Val) {
//			streamDataFlag = 1;
//			counterData_ms = 0;
//		}
//		if (counterStream_ms >= CCR3_Val) {
//			streamResultFlag = 1;
//			counterResult_ms = 0;
//		}
//
//	}
//
//}

// questo è l'handler dell'interrupt del timer ovvero il codice da eseguire quando il timer su un certo canale manda un interrupt
// (va verificato qua su quale canale ho l'interrupt)
void TIM3_IRQHandler(void) {
	uint16_t capture1 = 0;
	uint16_t capture2 = 0;
	uint16_t capture3 = 0;
	// canale per prendere dati accellerometro
	if (TIM_GetITStatus(TIM3, TIM_IT_CC1) != RESET) {
		TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);

		//vanno salvati nel vettore sul quale applichiamo il FIR
		dataReady = 1;

		capture1 = TIM_GetCapture1(TIM3);
		TIM_SetCompare1(TIM3, capture1 + CCR1_Val);
	}
	//canali per printare dati sulla usart
	if (TIM_GetITStatus(TIM3, TIM_IT_CC2) != RESET) {
		TIM_ClearITPendingBit(TIM3, TIM_IT_CC2);

		streamDataFlag = 1;

		capture2 = TIM_GetCapture2(TIM3);
		TIM_SetCompare2(TIM3, capture2 + CCR2_Val);
	}

	if (TIM_GetITStatus(TIM3, TIM_IT_CC3) != RESET) {
		TIM_ClearITPendingBit(TIM3, TIM_IT_CC3);

		streamResultFlag = 1;
		capture3 = TIM_GetCapture3(TIM3);
		TIM_SetCompare3(TIM3, capture3 + CCR3_Val);
	}

}

/* -----------------------------------------------------------------------
 TIM3 Configuration: Output Compare Timing Mode:

 In this configuration TIM3 input clock (TIM3CLK) is set to 2 * APB1 clock (PCLK1),
 with PCLK1 = HCLK / 4.
 TIM3CLK = 2 * PCLK1

 => TIM3CLK = HCLK / 2 = SystemCoreClock/2

 To get TIM3 counter clock the prescaler is computed as follows:
 Prescaler = (TIM3CLK / TIM3 counter clock) - 1

 ----------------------------------------------------------------------- */
void TimerConfiguration(void) {
	uint16_t PrescalerValue = 0;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	/* TIM3 clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = 65535;
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	/* Prescaler configuration */
	// il clock con cui il nostro contatore avanza con questa equazione e TIME_CK_CNT Hz
	PrescalerValue = (uint16_t) ((SystemCoreClock / 2) / TIM3_CK_CNT) - 1;
	TIM_PrescalerConfig(TIM3, PrescalerValue, TIM_PSCReloadMode_Immediate);

	// Output Compare Mode per Channel1 e sotto per gli altri
	//(c'è una struttura IC per le modalità di compar dell'ingresso)
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = CCR1_Val;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Disable);

	// la struttura è sempre la stessa  quello che faccio è cambiare il channel e inizilizzarlo
	TIM_OCInitStructure.TIM_Pulse = CCR2_Val;
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Disable);

	TIM_OCInitStructure.TIM_Pulse = CCR3_Val;
	TIM_OC3Init(TIM3, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Disable);

	/* Configure the TIM3 gloabal Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* TIM Interrupts enable */
	TIM_ITConfig(TIM3, TIM_IT_CC1, ENABLE);
	TIM_ITConfig(TIM3, TIM_IT_CC2, ENABLE);
	TIM_ITConfig(TIM3, TIM_IT_CC3, ENABLE);

	/* TIM3 counter enable */
	// con questo comando parte effettivamente il conto
	TIM_Cmd(TIM3, ENABLE);

}
void USART_Config(void) {
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
	USART_InitStructure.USART_HardwareFlowControl =
	USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	/* USART initialization */
	USART_Init(USART2, &USART_InitStructure);

	/* Enable USART */
	USART_Cmd(USART2, ENABLE);

}

/* Functions   **/
void Acc_Config(void) {
	LIS3DSH_InitTypeDef AccInitStruct;

	AccInitStruct.Output_DataRate = LIS3DSH_DATARATE_400; //Data Output Rate possibilmnete pìù alta della frequenza di sampling desiderata = 400hz
	AccInitStruct.Axes_Enable = LIS3DSH_XYZ_ENABLE;
	AccInitStruct.SPI_Wire = LIS3DSH_SERIALINTERFACE_4WIRE; //sceglo la spi full duplez come interfaccia di comunicazione
	AccInitStruct.Self_Test = LIS3DSH_SELFTEST_NORMAL;
	AccInitStruct.Full_Scale = LIS3DSH_FULLSCALE_2; //imposto la scala massima e minima a +/- 2g
	AccInitStruct.Filter_BW = LIS3DSH_FILTER_BW_800; // filtro passa basso interno

	LIS3DSH_Init(&AccInitStruct);
	/*This function will call the SPI initialization function and then write the
	 selected configuration to the LIS3DSH registers.
	 ovvero LIS3DSH_LowLevel_Init(); contenuta nella libreria in dicovery_lis3dsh.c
	 questa ci permette di non dover configuarre tutti i registri di controllo e di dati a mano
	 CONFIGURA E ATTIVA L'SPI attivando dapprima i pin per mosi miso sck e poi inizializzando una struttura di cntrollo come ne abbiamo già viste
	 configura inoltre il CS usando un GPIO*/

}

//funzione che accende il led più vicino a terra
void Ground_Orientation(float pitch, float roll) {
	//spengo eventuali led accesi
	STM_EVAL_LEDOff(LED3);
	STM_EVAL_LEDOff(LED4);
	STM_EVAL_LEDOff(LED5);
	STM_EVAL_LEDOff(LED6);

	//in questo modo assicuro il corretto funzionamento anche con la scheda capovolta
	if (roll < -90) {
		roll = roll + 90;
	}
	if (roll > 90) {
		roll = roll - 90;
	}

	float diff = 0;
	//verifico se è maggiore il pitch o il roll
	diff = fabsf(pitch) - fabsf(roll);

	//nel caso in cui il pitch sia maggiore del roll, verifico il segno: se è positivo accendo il led corrispondente, altrimenti l'altro
	if (diff > 0) {
		if (pitch > 0)
			STM_EVAL_LEDOn(LED5);
		else
			STM_EVAL_LEDOn(LED4);
	} else {

		if (roll > 0)
			STM_EVAL_LEDOn(LED3);
		else
			STM_EVAL_LEDOn(LED6);
	}
}

//funzione che sposta verso sinistra tutti i valori dell'array
void shift(float buffer[], int dim) {
	int i = 0;
	for (i = 0; i < dim - 1; i++) {
		buffer[i] = buffer[i + 1];
	}
	buffer[dim-1] = 0;
}

float mean(float array[]) {
	float result = 0;
	for (int i = 0; i < WINDOW_MEAN; i++) {
		result += array[i];
	}
	return result / WINDOW_MEAN;
}
int __io_putchar(int ch) {
	/* Place your implementation of fputc here */
	/* e.g. write a character to the USART */
	USART_SendData(USART2, (uint8_t) ch);

	/* Loop until the end of transmission */
	while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET) {
	}

	return ch;
}




