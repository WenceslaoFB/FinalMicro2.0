/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdlib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
 typedef union{
        uint8_t u8[4];
        int8_t  i8[4];

        uint16_t u16[2];
        int16_t  i16[2];

        uint32_t u32;
        int32_t  i32;

        float    f;
}_sWork;

typedef union{
    struct{
        uint8_t b0:1;
        uint8_t b1:1;
        uint8_t b2:1;
        uint8_t b3:1;
        uint8_t b4:1;
        uint8_t b5:1;
        uint8_t b6:1;
        uint8_t b7:1;
    }bit;
    uint8_t byte;
}flag;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define race 				flag1.bit.b0
#define ON100MS				flag1.bit.b1
#define ON10MS				flag1.bit.b2
#define stop				flag1.bit.b3
#define readyToSend			flag1.bit.b4
#define espReadyToRecieve	flag1.bit.b5
#define killRace			flag1.bit.b6
#define espConnected		flag2.bit.b0
#define sendALIVE			flag2.bit.b1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
const char AT_com[]="AT\r\n";//4
const char AT_ans[]="AT\r\n\r\nOK\r\n";//10

const char CWQAP[]="AT+CWQAP\r\n";//10
const char ANS_CWQAP[]={"AT+CWQAP\r\n\r\nOK\r\n"};//16
const char ANS_CWQAP2[]={"AT+CWQAP\r\n\r\nOK\r\nWIFI DISCONNECT/r/n"};//35

const char CWMODE[]="AT+CWMODE=3\r\n";//13
const char ANS_CWMODE[]="AT+CWMODE=3\r\n\r\nOK\r\n";//19

const char CWJAP_casa[]="AT+CWJAP=\"FTTHBOUVET\",\"wenvla3112\"\r\n";//36
const char ANS_CWJAP_casa2[]="AT+CWJAP=\"FTTHBOUVET\",\"wenvla3112\"\r\n";//36
const char ANS_CWJAP_casa[]="AT+CWJAP=\"FTTHBOUVET\",\"wenvla3112\"\r\nWIFI CONNECTED\r\nWIFI GOT IP\r\n\r\nOK\r\n";//71

const char CIFSR[]="AT+CIFSR\r\n";//10
const char CIFSR_STAIP[]="+CIFSR:STAIP,";//13

const char CIPMUX[]="AT+CIPMUX=0\r\n";//13
const char ANS_CIPMUX[]="AT+CIPMUX=0\r\n\r\nOK\r\n";//19

const char CIPSTART[]="AT+CIPSTART=\"UDP\",\"192.168.1.195\",30017,3017\r\n";//46
const char ANS_CIPSTART2[]="AT+CIPSTART=\"UDP\",\"192.168.1.195\",30017,3017\r\nCONNECT\r\n\r\nOK\r\n";//61
const char ANS_CIPSTART[]="AT+CIPSTART=\"UDP\",\"192.168.1.195\",30017,3017\r\n";//46
const char ANS_CIPSTART_ERROR[]="AT+CIPSTART=\"UDP\",\"192.168.1.195\",30017,3017\r\nALREADY CONNECTED\r\n\r\nERROR\r\n";//74
const char AUTOMATIC_WIFI_CONNECTED[]={"WIFI CONNECTED\r\nWIFI GOT IP\r\n"};//29
const char WIFI_DISCONNECT[]="WIFI DISCONNECT\r\n";//17

const char CIPSEND[]="AT+CIPSEND=";//11
const char CIPSEND1[]={'A','T','+','C','I','P','S','E','N','D','='};
const char CIPSEND2[]={'\r','\n','\r','\n','O','K','\r','\n','>'};
const char CIPSEND3[]="Recv ";
const char CIPSEND4[]={" bytes\r\n\r\nSEND OK\r\n"};//25

const char OK[]="\r\nOK\r\n";
const char IPD[]="\r\n+IPD,";
const char UNER[]="UNER";
const char ALIVE[]={'U','N','E','R', 0x02 ,':',0xF0};
const char ACK_D0[]={'U','N','E','R',0x03,':',0xD0,0x0D,0xDC};
const int COORD_SENSORES[]={-5,-4,-3,-2,-1,1,2,3,4,5};

volatile uint8_t buf_rx[256];
volatile uint8_t buf_tx[256];

volatile uint16_t ADCData[32][8];

char espIP[15];

volatile uint8_t time100ms=0;
volatile uint8_t time10ms=0;

volatile uint8_t indRX_W=0;
volatile uint8_t indRX_R=0;
volatile uint8_t indTX_W=0;
volatile uint8_t indTX_R=0;
volatile uint8_t indADC=0;

uint8_t coincidencias = 0, coincidencias2 = 0, duty = 0, AT=0, decoCIPSEND=0, decodeCIF=0, decoIPD=0;
uint8_t timeout1=0, timeout2=0, timeoutADC=0, timeoutPID=0;
uint8_t largoIP=0, bytesToSend=0, bytesToSend_aux=0, timeToSendAlive=0;
uint8_t cks, bytesUNERprotocol, contByte=1, cmdPosInBuf;
uint8_t comandoActual=0, primLectADC=1;
uint8_t firstCalcu=1,timeOutArranque, calibADC=1,CALIBRADO=0;

_sWork PWM_motor1,PWM_motor2,jobTime,error;
_sWork valueADC[8], valueADCCAL[8];
_sWork Kp,Kd,Ki;
_sWork pwmBase;

volatile flag flag1,flag2;

float integral=0,derivativo=0,turn=0,Error=0,lastError=0;
float  Constante_Relacion[8],proporcional = 0, cuentapid=0, valorsensormax=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);
void initEsp();
void uart();
void recibirmensaje();
void udpCom(uint8_t cmd);
void DecodeComands(uint8_t *buffer,uint8_t indexCMD);
void leerADC();
void findLine(_sWork *BUFADC);
void calcPID(uint32_t pwmBase1,uint32_t pwmBase2);
void calibrarADC();
void calibrarVal(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){

	indRX_W++;
	HAL_UART_Receive_IT(&huart1, (uint8_t *) &buf_rx[indRX_W], 1);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){

	//indADC++;
	if(indADC==32){
		indADC=0;
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){

	if(htim->Instance == TIM4){//ENTRA CADA 10 MS
			indADC++;
			HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&ADCData,8*32);

			if(indADC==32){
					indADC=0;
				}

			time100ms--;
			if(!time100ms){
				time100ms=10;
				ON100MS = 1;
			}
			time10ms--;
			if(!time10ms){
				time10ms=1;
				ON10MS = 1;
			}
		}
}

void uart(){

	if((huart1.Instance->SR & UART_FLAG_TXE)==UART_FLAG_TXE){
		huart1.Instance->DR=buf_tx[indTX_R];
		indTX_R++;
	}

}

void initEsp(){

	if(readyToSend){
		switch(AT){
			case 0:
				memcpy((uint8_t*)&buf_tx[indTX_W],AT_com,4);
				indTX_W+=4;
				timeout2 = 10;
				readyToSend = 0;
			break;
			case 1:
				memcpy((uint8_t*)&buf_tx[indTX_W],CWQAP,10);
				indTX_W+=10;
				timeout2 = 10;
				readyToSend = 0;
			break;
			case 2:
				memcpy((uint8_t*)&buf_tx[indTX_W],CWMODE,13);
				indTX_W+=13;
				timeout2 = 10;
				readyToSend = 0;
			break;
			case 3:
				memcpy((uint8_t*)&buf_tx[indTX_W],CWJAP_casa,36);
				indTX_W+=36;
				timeout2 = 50;
				readyToSend = 0;
			break;
			case 4:
				memcpy((uint8_t*)&buf_tx[indTX_W],CIPMUX,13);
				indTX_W+=13;
				timeout2 = 10;
				readyToSend = 0;
			break;
			case 5:
				memcpy((uint8_t*)&buf_tx[indTX_W],CIFSR,10);
				indTX_W+=10;
				timeout2 = 10;
				readyToSend = 0;
			break;
			case 6:
				memcpy((uint8_t*)&buf_tx[indTX_W],CIPSTART,46);
				indTX_W+=46;
				timeout2 = 20;
				readyToSend = 0;
			break;
			case 7:
				duty++;
			break;
		}

	}

}

void recibirmensaje(){

	switch(AT){
		case 0:
			if(buf_rx[indRX_R]==AT_ans[coincidencias]){
				coincidencias++;

				if(coincidencias>6){
					//HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
					coincidencias = 0;
					AT++;
					readyToSend = 1;
				}
			}else{
				if(!timeout2){
					indRX_R=indRX_W;
					coincidencias = 0;
					readyToSend=1;
					break;
				}
			}
		break;
		case 1:
			if(buf_rx[indRX_R]==ANS_CWQAP[coincidencias]){
				coincidencias++;

				if(coincidencias>14){
					//HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
					coincidencias = 0;
					AT++;
					readyToSend = 1;
				}
			}else{
				if(!timeout2){
					indRX_R=indRX_W;
					coincidencias = 0;
					readyToSend=1;
					break;
				}
			}
			/*
			if(buf_rx[indRX_R]==AUTOMATIC_WIFI_CONNECTED[coincidencias]){
				coincidencias++;

				if(coincidencias>25){
					coincidencias = 0;
					AT++;
					readyToSend = 1;
				}
			}else{
				if(buf_rx[indRX_R]==WIFI_DISCONNECT[coincidencias]){
					coincidencias++;

					if(coincidencias>15){
						coincidencias = 0;
						AT=0;
						readyToSend = 1;
						//reiniciar pin enable y abortar comunicacion
						timeout1=10;
					}
				}else{
					if(!timeout2){
						indRX_R=indRX_W;
						coincidencias = 0;
						readyToSend=1;
						AT=2;
						break;
					}
				}
			}
		break;*/
		case 2:
			if(buf_rx[indRX_R]==ANS_CWMODE[coincidencias]){
				coincidencias++;

				if(coincidencias>17){
					coincidencias = 0;
					AT++;
					readyToSend = 1;
				}
			}else{
				if(!timeout2){
					indRX_R=indRX_W;
					coincidencias = 0;
					readyToSend=1;
					break;
				}
			}
		break;
		case 3:
			if(buf_rx[indRX_R]==ANS_CWJAP_casa[coincidencias]){
				coincidencias++;

				if(coincidencias>68){
					coincidencias = 0;
					AT++;
					readyToSend = 1;
				}
			}
		break;
		case 4:
			if(buf_rx[indRX_R]==ANS_CIPMUX[coincidencias]){
				coincidencias++;

				if(coincidencias>17){
					coincidencias = 0;
					AT+=2;
					readyToSend = 1;
				}
			}else{
				if(!timeout2){
					indRX_R=indRX_W;
					coincidencias = 0;
					readyToSend=1;
					break;
				}
			}
		break;
		case 5:
			switch(decodeCIF){
			case 0:
				if(buf_rx[indRX_R]==CIFSR[coincidencias]){
					coincidencias++;

					if(coincidencias>8){
						coincidencias = 0;
						decodeCIF++;
						//AT++;
						//readyToSend = 1;
					}
				}else{
					if(!timeout2){
						indRX_R=indRX_W;
						coincidencias = 0;
						readyToSend=1;
						break;
					}
				}
			break;
			case 1:
				if(buf_rx[indRX_R]==CIFSR_STAIP[coincidencias]){
					coincidencias++;

					if(coincidencias>10){
						coincidencias = 0;
						decodeCIF++;
						//AT++;
						//readyToSend = 1;
					}
				}
			break;
			case 2:
				espIP[coincidencias]=buf_rx[indRX_R];
				coincidencias++;
				if((buf_rx[indRX_R]=='"')&&(largoIP>1)){
					coincidencias=0;
					decodeCIF++;
				}
			break;
			case 3:
				if(buf_rx[indRX_R]==OK[coincidencias]){
					coincidencias++;

					if(coincidencias>4){
						coincidencias = 0;
						decodeCIF=0;
						AT++;
						readyToSend = 1;
					}
				}
			break;
			}
		break;
		case 6:
			if(buf_rx[indRX_R]==ANS_CIPSTART[coincidencias]){
				coincidencias++;

				if(coincidencias>44){
					coincidencias = 0;
					coincidencias2 = 0;
					AT=8;
					readyToSend = 1;
					espConnected=1;
					HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
					sendALIVE=1;
				}
			}/*
				if(buf_rx[indRX_R]==ANS_CIPSTART_ERROR[coincidencias2]){
					coincidencias2++;

					if(coincidencias>71){
						coincidencias = 0;
						coincidencias2 = 0;
						AT++;
						readyToSend = 1;
						espConnected=1;
					}
				}*/else{
					if(!timeout2){
						indRX_R=indRX_W;
						coincidencias = 0;
						readyToSend=1;
						break;
					}
				}

		break;
		case 7:
			switch(decoCIPSEND){
				case 0:
					if(buf_rx[indRX_R]==CIPSEND[coincidencias]){
						coincidencias++;

						if(coincidencias>9){
							coincidencias = 0;
							decoCIPSEND++;
							//AT++;
							//readyToSend = 1;
						}
					}else{
						if(!timeout2){
							indRX_R=indRX_W;
							coincidencias = 0;
							readyToSend=1;
							espReadyToRecieve=0;
							sendALIVE=0;
							timeToSendAlive=30;
							break;
						}
					}
				break;
				case 1:
					if((buf_rx[indRX_R]==bytesToSend+'0')&&((bytesToSend<10))){//reviso q sean menos de 10 bytes
						decoCIPSEND+=2;
					}else{

						if(buf_rx[indRX_R]==bytesToSend/10+'0'){
							decoCIPSEND++;
							bytesToSend_aux=bytesToSend/10;
							bytesToSend_aux*=10;
						}
					}
				break;
				case 2:
					if(buf_rx[indRX_R]==bytesToSend-bytesToSend_aux+'0'){
						decoCIPSEND++;
					}
				break;
				case 3:
					if(buf_rx[indRX_R]==CIPSEND2[coincidencias]){
						coincidencias++;

						if(coincidencias>7){
							coincidencias = 0;
							decoCIPSEND++;
							//AT++;
							readyToSend = 1;
							espReadyToRecieve=1;
						}
					}else{
						if(!timeout2){
							decoCIPSEND=0;
							indRX_R=indRX_W;
							coincidencias = 0;
							readyToSend=1;
							espReadyToRecieve=0;
							sendALIVE=0;
							timeToSendAlive=30;
							break;
						}
					}
				break;
				case 4:
					if(buf_rx[indRX_R]==CIPSEND3[coincidencias]){
						coincidencias++;

						if(coincidencias>3){
							coincidencias = 0;
							decoCIPSEND++;
						}
					}else{
						if(!timeout2){
							decoCIPSEND=0;
							indRX_R=indRX_W;
							coincidencias = 0;
							readyToSend=1;
							espReadyToRecieve=0;
							sendALIVE=0;
							timeToSendAlive=30;
							break;
						}
					}
				break;
				case 5:
					if((buf_rx[indRX_R]==bytesToSend+'0')&&((bytesToSend<10))){//reviso q sean menos de 10 bytes
						decoCIPSEND+=2;
					}else{

						if(buf_rx[indRX_R]==bytesToSend/10+'0'){
							decoCIPSEND++;
						}else{
							if(!timeout2){
								decoCIPSEND=0;
								indRX_R=indRX_W;
								coincidencias = 0;
								readyToSend=1;
								espReadyToRecieve=0;
								sendALIVE=0;
								timeToSendAlive=30;
								break;
							}
						}
					}
				break;
				case 6:
					if(buf_rx[indRX_R]==bytesToSend-bytesToSend_aux+'0'){
						decoCIPSEND++;
					}else{
						if(!timeout2){
							decoCIPSEND=0;
							indRX_R=indRX_W;
							coincidencias = 0;
							readyToSend=1;
							espReadyToRecieve=0;
							sendALIVE=0;
							timeToSendAlive=30;
							break;
						}
					}
				break;
				case 7:
					if(buf_rx[indRX_R]==CIPSEND4[coincidencias]){
						coincidencias++;

						if(coincidencias>17){
							coincidencias = 0;
							decoCIPSEND=0;
							AT=8;
							readyToSend=1;
							espReadyToRecieve=0;
						}
					}else{
						if(!timeout2){
							decoCIPSEND=0;
							indRX_R=indRX_W;
							coincidencias = 0;
							readyToSend=1;
							espReadyToRecieve=0;
							sendALIVE=0;
							timeToSendAlive=30;
							break;
						}
					}
				break;
			}
		break;
		case 8:
			switch(decoIPD){
				case 0:
					if(buf_rx[indRX_R]==IPD[coincidencias]){
						coincidencias++;

						if(coincidencias>4){
							coincidencias = 0;
							decoIPD++;
						}
					}else{
						if(coincidencias>0){
							indRX_R=indRX_W;
							coincidencias=0;
							break;
						}
					}
				break;
				case 1:
					if(buf_rx[indRX_R]==':'){
						decoIPD++;
					}
				break;
				case 2:
					if(buf_rx[indRX_R]==UNER[coincidencias]){
						coincidencias++;

						if(coincidencias>3){
							coincidencias = 0;
							decoIPD++;
							cks='U'^'N'^'E'^'R';
						}
					}else{
						if(coincidencias>0){
							indRX_R=indRX_W;
							coincidencias=0;
							decoIPD=0;
							break;
						}
					}
				break;
				case 3:
					bytesUNERprotocol=buf_rx[indRX_R];
					decoIPD++;
					cks^=buf_rx[indRX_R];
				break;
				case 4:
					if(buf_rx[indRX_R]==':'){
						decoIPD++;
						cks^=buf_rx[indRX_R];
					}else{
						indRX_R=indRX_W;
						coincidencias=0;
						decoIPD=0;
						break;
					}
				break;
				case 5:
					if(contByte==1){
						cmdPosInBuf=indRX_R;
					}
					if(contByte<bytesUNERprotocol){
						cks^=buf_rx[indRX_R];
						contByte++;
					}else{
						if(cks==buf_rx[indRX_R]){
							DecodeComands((uint8_t*)&buf_rx, cmdPosInBuf);
							//HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
							contByte=1;
							coincidencias=0;
							decoIPD=0;
						}else{
							contByte=1;
							coincidencias=0;
							decoIPD=0;
						}
					}
				break;
			}
		break;
		default:

		break;
	}
	indRX_R++;
}

void udpCom(uint8_t cmd){

	if(readyToSend){
		switch(cmd){
			case 0:
				if(!espReadyToRecieve){
					AT=7;
					memcpy((uint8_t*)&buf_tx[indTX_W],CIPSEND,11);
					indTX_W+=11;
					memcpy((uint8_t*)&buf_tx[indTX_W],"8\r\n",3);
					indTX_W+=3;
					bytesToSend=8;
					timeout2 = 8;
					readyToSend = 0;
				}else{
					memcpy((uint8_t*)&buf_tx[indTX_W],ALIVE,7);
					indTX_W+=7;
					buf_tx[indTX_W]='U'^'N'^'E'^'R'^0x02^':'^0xF0;
					indTX_W+=1;
					espReadyToRecieve=0;
					sendALIVE=0;
					readyToSend = 0;
				}
			break;
			case 1:
				if(!espReadyToRecieve){
					AT=7;
					memcpy((uint8_t*)&buf_tx[indTX_W],CIPSEND,11);
					indTX_W+=11;
					memcpy((uint8_t*)&buf_tx[indTX_W],"9\r\n",3);
					indTX_W+=3;
					bytesToSend=9;
					timeout2 = 20;
					readyToSend = 0;
				}else{
					memcpy((uint8_t*)&buf_tx[indTX_W],ACK_D0,9);
					indTX_W+=9;
					buf_tx[indTX_W]='U'^'N'^'E'^'R'^0x03^':'^0xD0^0x0D;
					indTX_W+=1;
					espReadyToRecieve=0;
					duty++;
				}
			break;
			/*case 2:
				if(!espReadyToRecieve){
					AT=7;
					memcpy((uint8_t*)&buf_tx[indTX_W],CIPSEND,11);
					indTX_W+=11;
					memcpy((uint8_t*)&buf_tx[indTX_W],"8\r\n",3);
					indTX_W+=3;
					bytesToSend=8;
					timeout2 = 8;
					readyToSend = 0;
				}else{
					memcpy((uint8_t*)&buf_tx[indTX_W],"'U','N','E','R','38',':',0XA1,7);
					indTX_W+=7;

					buf_tx[indTX_W]='U'^'N'^'E'^'R'^0x09^':'^valueADCCAL[0]^valueADCCAL[1]^valueADCCAL[2]^valueADCCAL[3]^valueADCCAL[4]^valueADCCAL[5]^valueADCCAL[6]^valueADCCAL[7];
					indTX_W+=1;
					espReadyToRecieve=0;
					sendALIVE=0;
					readyToSend = 0;
				}
			break;*/
		}
	}
}

void DecodeComands(uint8_t *buffer,uint8_t indexCMD){

	uint8_t i=1;

		switch(buffer[indexCMD]){
		case 0xA1://Send adc
				//udpCom(2);
		break;
			case 0xB3://STOP AUTITO
					comandoActual=0xB3;
					stop=1;
			break;
			case 0xD5: //START AUTITO(asi nomas)
					PWM_motor2.u32=200;
					PWM_motor1.u32=200;
					race=1;
					timeoutPID=2;
					calcPID(200,200);
			break;
			case 0xF1: //START AUTITO(mandar pwm)
					PWM_motor1.u8[0]=buffer[indexCMD+i];
					i++;
					PWM_motor1.u8[1]=buffer[indexCMD+i];
					i++;
					PWM_motor1.u8[2]=buffer[indexCMD+i];
					i++;
					PWM_motor1.u8[3]=buffer[indexCMD+i];
					PWM_motor2.u32=PWM_motor1.u32;
					comandoActual=0xF1;
					race=1;
					timeoutPID=2;
			break;
			case 0xC0:	//SETEAR PARAMETROS CONTROL PID
					Kp.u8[0]=buffer[indexCMD+i];
					i++;
					Kp.u8[1]=buffer[indexCMD+i];
					i++;
					Kp.u8[2]=buffer[indexCMD+i];
					i++;
					Kp.u8[3]=buffer[indexCMD+i];
					i++;
					Kp.u8[0]=buffer[indexCMD+i];
					i++;
					Kd.u8[1]=buffer[indexCMD+i];
					i++;
					Kd.u8[2]=buffer[indexCMD+i];
					i++;
					Kd.u8[3]=buffer[indexCMD+i];
					i++;
					Ki.u8[0]=buffer[indexCMD+i];
					i++;
					Ki.u8[1]=buffer[indexCMD+i];
					i++;
					Ki.u8[2]=buffer[indexCMD+i];
					i++;
					Ki.u8[3]=buffer[indexCMD+i];
					i++;
					comandoActual=0xC0;
			break;
			case 0xF0: //ALIVE
					sendALIVE=1;
					readyToSend=1;
			break;
			case 0xD0://JOB TIME
					PWM_motor1.u8[0]=buffer[indexCMD+i];
					i++;
					PWM_motor1.u8[1]=buffer[indexCMD+i];
					i++;
					PWM_motor1.u8[2]=buffer[indexCMD+i];
					i++;
					PWM_motor1.u8[3]=buffer[indexCMD+i];
					i++;

					PWM_motor2.u8[0]=buffer[indexCMD+i];
					i++;
					PWM_motor2.u8[1]=buffer[indexCMD+i];
					i++;
					PWM_motor2.u8[2]=buffer[indexCMD+i];
					i++;
					PWM_motor2.u8[3]=buffer[indexCMD+i];
					i++;

					jobTime.u8[0]=buffer[indexCMD+i];
					i++;
					jobTime.u8[1]=buffer[indexCMD+i];
					i++;
					jobTime.u8[2]=buffer[indexCMD+i];
					i++;
					jobTime.u8[3]=buffer[indexCMD+i];
					i++;
					jobTime.u32=jobTime.u32/100;
					__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,0);
					__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,PWM_motor1.u32);
					__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,PWM_motor2.u32);
					__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_4,0);
					race=1;
					killRace=1;
					duty=2;
					comandoActual=0xD0;
					//comando=0xD0;
					readyToSend=1;
			break;
			case 0xA5: //Calibrar ADC
				calibADC = 1;
				CALIBRADO =0;
			break;
		}
}

void leerADC(){

	if(indADC>2){
		primLectADC=0;
		for(uint8_t c=0;c<8;c++){
			valueADC[c].u32=( ADCData[indADC-1][c]+ADCData[indADC-2][c]+ADCData[indADC-3][c] ) / 3 ;

		}
	}else{
		if(!primLectADC){
			for(uint8_t c=0;c<8;c++){
				switch(indADC){
					case 2:
						valueADC[c].u32=(ADCData[1][c]+ADCData[0][c]+ADCData[31][c]) / 3;
						break;
					case 1:
						valueADC[c].u32=( ADCData[0][c]+ADCData[31][c]+ADCData[30][c] ) / 3 ;
						break;
					case 0:
						valueADC[c].u32=( ADCData[31][c]+ADCData[30][c]+ADCData[29][c] ) / 3;
						break;
				}
			}
		}
	}
}

uint8_t posMINCenter=0,posMINRight=0,posMINLeft=0;
uint16_t sensorValue=0;
float xMin=0,fx2_fx3,fx2_fx1,x2_x1,x2_x3,x2_x1cuad,x2_x3cuad,denominador, numerador, cuenta;
uint8_t f=0;

void findLine(_sWork *BUFADC){

	float aux[10];

		sensorValue = BUFADC[0].i32;


		posMINCenter=1;
		while(f<8){					//ENCUENTRO LA MENOR LECTURA
			if(sensorValue > BUFADC[f].i32){
				sensorValue=BUFADC[f].i32;
				posMINCenter=f+1;
			}
			aux[f+1]=BUFADC[f].i32;
			f++;
		}
		f=0;
		//posMINCenter+=1;
		aux[0]=aux[2];
		aux[9]=aux[7];

		posMINRight=posMINCenter+1;
		posMINLeft=posMINCenter-1;

		fx2_fx3=aux[posMINCenter]-aux[posMINRight];
		fx2_fx1=aux[posMINCenter]-aux[posMINLeft];

		x2_x1=COORD_SENSORES[posMINCenter]-COORD_SENSORES[posMINLeft];
		x2_x1cuad=(x2_x1*x2_x1);
		x2_x3=COORD_SENSORES[posMINCenter]-COORD_SENSORES[posMINRight];
		x2_x3cuad=(x2_x3*x2_x3);

		numerador=((x2_x1cuad*fx2_fx3) - (x2_x3cuad*fx2_fx1));
		denominador=(2*((x2_x1*fx2_fx3)-(x2_x3*fx2_fx1)));
		cuenta=numerador / denominador;
		if(denominador!=0){
			xMin=COORD_SENSORES[posMINCenter]-cuenta;
			error.f=xMin;
		}
		//return -xMin;

}

void calcPID(uint32_t pwmBase1,uint32_t pwmBase2){

	float pwm1,pwm2;

		integral+=error.f;

		if(integral > 1000){
				integral = 0;
		}

		derivativo=error.f-lastError;
		turn= (Kp.f*error.f) + (Kd.f*derivativo) + (Ki.f*integral);
		pwm1=pwmBase1-turn;
		pwm2=pwmBase2+turn;



		if(pwm1>400){
			pwm1=400;
		}
		if(pwm2>400){
			pwm2=400;
		}

		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,0);
		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,pwm1);
		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,pwm2);
		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_4,0);
		lastError=error.f;
}

void calibrarADC(){

	for (uint8_t i = 1; i < 8; i++) {
			Constante_Relacion[i] = (valueADC[0].f)/(valueADC[i].f);
			//bufADCCAL[i].f = (bufADC[i].f)*(Constante_Relacion[i]);
		}

		valorsensormax = valueADC[0].f;
		for (uint8_t i = 1; i < 8; i++) {
			if(valorsensormax < (valueADC[i].f * Constante_Relacion[i])){
				valorsensormax = (valueADC[i].f * Constante_Relacion[i]);
			}
		}
		valorsensormax = valorsensormax * 0.75;
}

void calibrarVal(void){
	valueADCCAL[0].f = valueADC[0].f;
		for(uint8_t i = 1; i<8; i++){
			valueADCCAL[i].f= (valueADC[i].f)*(Constante_Relacion[i]);
		}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim4);
  HAL_TIM_Base_Start(&htim3);
  //HAL_ADC_Start_IT(&hadc1);

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

  timeout2=30;
  //timeOut3=10;
  timeToSendAlive=30;
  timeout1=30;
  timeoutADC=0;
  readyToSend=1;
  race=0;
  killRace=0;
  stop=0;
  duty=0;
  espConnected=0;
  sendALIVE=0;

  PWM_motor1.u32=0;
  PWM_motor2.u32=0;
  time100ms=10;
  time10ms=1;

  //timeoutOK=200;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if(ON100MS){
	  	 		ON100MS=0;
	  	 	  	if(timeout1>0)
	  	 	  		timeout1--;
	  	 		if(timeout2>0)
	  	 			timeout2--;
	  	 		if(jobTime.u32>0)
	  	 			jobTime.u32--;
	  	 		if((timeToSendAlive>0)&&(espConnected))
	  	 			timeToSendAlive--;
	  	  }

	  if(ON10MS){
		  ON10MS=0;
		  if(timeoutADC>0){
			  timeoutADC--;
		  }
		  if(timeoutPID>0){
		  		timeoutPID--;
		  }
	  }

	  	  if( (!timeToSendAlive) && (espConnected) ){
	  		  sendALIVE=1;
	  		  espReadyToRecieve=0;
	  		  timeToSendAlive=50;
	  		  readyToSend=1;
	  	  }

	  	  if(sendALIVE){
	  	  	udpCom(0);
	  	  }

	  	  if(calibADC && !CALIBRADO){
	  		calibrarADC();
	  		CALIBRADO=1;
	  	  }

	  	switch(duty){
	  		case 0:
	  			if(!timeout1){
	  				initEsp();
	  				HAL_UART_Receive_IT(&huart1, (uint8_t *)&buf_rx[indRX_W], 1);
	  				duty++;
	  			}
	  		break;
	  		case 1:
	  			initEsp();
	  		break;
	  		case 2:
	  			//udpCom(0);
	  			//HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	  		break;
	  	}

	  	if(!timeoutADC){

	  		/*
	  		if(calibADC){
	  		  	leerADC();
	  		  	calibrarVal();
	  		  	findLine(valueADCCAL);
	  		}else{
	  			findLine(valueADC);
	  		}*/
	  		findLine(valueADC);
	  		timeoutADC=3;
	  	}

	  	if((race)&&(!timeoutPID)){
	  		  	calcPID(PWM_motor1.u32,PWM_motor2.u32);
	  		  	timeoutPID=2;
	  	}


	  		if( ( ( (!jobTime.u32) && (killRace) ) ) || (stop) ) {
	  				  stop=0;
	  				  race=0;
	  				  killRace=0;
	  				__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,0);
	  				__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,0);
	  				__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,0);
	  				__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_4,0);
	  		}

	  		if(indTX_R!=indTX_W){
	  			uart();
	  		}

	  		if(indRX_R!=indRX_W){
	  			recibirmensaje();
	  		}

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 8;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = 7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = 8;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 99;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 210;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 100;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 8400;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
