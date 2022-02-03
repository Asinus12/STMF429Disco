/* bLAZ bOGATAJ */
 // ko dodajas periferijo mors dodat source tudi v makefilu
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_adc.h"
#include <stdlib.h>
#include <stdio.h>
#include "mojInclude.h"
#include "stm32f4xx_spi.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_exti.h"
#include "misc.h"
#include "stm32f4xx_syscfg.h"
#include "stm32f4xx_it.h"
#include "stm32f429i_discovery_l3gd20.h"
#include "stdbool.h"



#define PAUSE_LONG  4000000L
#define PAUSE_SHORT 1000000L
#define GREEN  GPIO_Pin_13 
#define RED     GPIO_Pin_14

#define ABS(x)                    (x < 0) ? (-x) : x
#define L3G_Sensitivity_250dps    (float)114.285f        /*!< gyroscope sensitivity with 250 dps full scale [LSB/dps]  */
#define L3G_Sensitivity_500dps    (float)57.1429f        /*!< gyroscope sensitivity with 500 dps full scale [LSB/dps]  */
#define L3G_Sensitivity_2000dps   (float)14.285f         /*!< gyroscope sensitivity with 2000 dps full scale [LSB/dps] */




GPIO_InitTypeDef GPIO_InitStructure;
NVIC_InitTypeDef   NVIC_InitStructure;
EXTI_InitTypeDef EXTI_InitStructure;
USART_InitTypeDef USART_InitStructure;




void initGyro(){
    /* Initialize L3GD20 Structure */
  L3GD20_InitTypeDef L3GD20_InitStructure;
  L3GD20_FilterConfigTypeDef L3GD20_FilterStructure;
  
  L3GD20_InitStructure.Power_Mode = L3GD20_MODE_ACTIVE;
  L3GD20_InitStructure.Output_DataRate = L3GD20_OUTPUT_DATARATE_1;
  L3GD20_InitStructure.Axes_Enable = L3GD20_AXES_ENABLE;
  L3GD20_InitStructure.Band_Width = L3GD20_BANDWIDTH_4;
  L3GD20_InitStructure.BlockData_Update = L3GD20_BlockDataUpdate_Continous;
  L3GD20_InitStructure.Endianness = L3GD20_BLE_LSB;
  L3GD20_InitStructure.Full_Scale = L3GD20_FULLSCALE_500; 
  L3GD20_Init(&L3GD20_InitStructure);
  
  L3GD20_FilterStructure.HighPassFilter_Mode_Selection =L3GD20_HPM_NORMAL_MODE_RES;
  L3GD20_FilterStructure.HighPassFilter_CutOff_Frequency = L3GD20_HPFCF_0;
  L3GD20_FilterConfig(&L3GD20_FilterStructure) ;
  
  L3GD20_FilterCmd(L3GD20_HIGHPASSFILTER_ENABLE);
}
void GyroReadAngRate (float* pfData)
{
  uint8_t tmpbuffer[6] ={0};
  int16_t RawData[3] = {0};
  uint8_t tmpreg = 0;
  float sensitivity = 0;
  uint32_t wCounter =0;
  
  // Read CTRL4 reg for sensitivity and endiness value
  L3GD20_Read(&tmpreg,L3GD20_CTRL_REG4_ADDR,1);
  
  // Read  X, Y and Z
  L3GD20_Read(tmpbuffer,L3GD20_OUT_X_L_ADDR,6);
  
  /* Check in the control register for the data alignment (Big Endian or Little Endian)*/
  if(!(tmpreg & 0x40))
  {
    for(wCounter=0; wCounter<3; wCounter++)
    {
      RawData[wCounter] = (int16_t) (((uint16_t)tmpbuffer[2*wCounter+1] << 8) + tmpbuffer[2*wCounter]);
    }
  }
  else
  {
    for(wCounter=0; wCounter<3; wCounter++)
    {
      RawData[wCounter] = (int16_t) (((uint16_t)tmpbuffer[2*wCounter] << 8) + tmpbuffer[2*wCounter+1]);
    }
  }
  
  /* Set the sensitivity value set in the CRTL4 */
  switch(tmpreg & 0x30)
  {
  case 0x00:
    sensitivity=L3G_Sensitivity_250dps;
    break;
    
  case 0x10:
    sensitivity=L3G_Sensitivity_500dps;
    break;
    
  case 0x20:
    sensitivity=L3G_Sensitivity_2000dps;
    break;
  }

  /* Divide by sensitivity */
  for(wCounter=0; wCounter<3; wCounter++)
  {
    pfData[wCounter] = (float) RawData[wCounter]/sensitivity;
  }
}
static void GyroTest(void)
{
  float Buffer[6];
  uint8_t Xval, Yval, Zval = 0x00;
    
  /* Read Gyro Angular data */
  GyroReadAngRate(Buffer);
  
  /* Update autoreload and capture compare registers value*/
  Xval = ABS((int8_t)(Buffer[0]));
  Yval = ABS((int8_t)(Buffer[1])); 
  Zval = ABS((int8_t)(Buffer[2])); 

  
    // X AXIS 
    if ((int8_t)Buffer[0] > 15.0f)
    {       
        GPIO_ResetBits(GPIOG, GPIO_Pin_13); // zavrtimo nazaj 
        
    }
    if ((int8_t)Buffer[0] < -15.0f)
    {
        GPIO_SetBits(GPIOG, GPIO_Pin_13); // zavrtimo naprej
        
    }

    // Y AXIS
    if ((int8_t)Buffer[1] < -15.0f)
    {
        GPIO_ResetBits(GPIOG, GPIO_Pin_14); // zavrtimo desno
        
    }
    if ((int8_t)Buffer[1] > 15.0f)
    {
        GPIO_SetBits(GPIOG, GPIO_Pin_14); // zavrtimo levo
    } 

    // Z AXIS
     if ((int8_t)Buffer[2] < -15.0f)
    {
        GPIO_ResetBits(GPIOG, GPIO_Pin_13|GPIO_Pin_14); // zavrtimo desno
    }
    if ((int8_t)Buffer[2] > 15.0f)
    {
        GPIO_SetBits(GPIOG, GPIO_Pin_13|GPIO_Pin_14); // zavrtimo levo
    } 
   
}
uint32_t L3GD20_TIMEOUT_UserCallback(void)
{
  return 0;
}


static void delay( uint32_t nCount)
{
    while(nCount--)
        __asm("nop"); 
}
static void initGPIO(){
    
  /* GPIOD Periph clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);

  
  /* Configure PD12, PD13, PD14 and PD15 in output pushpull mode */
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_13| GPIO_Pin_14;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOG, &GPIO_InitStructure);
}
static void initPA0_EXTI(){

 
  /* Enable GPIOA clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  /* Enable SYSCFG clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
  
  /* Configure PA0 pin as input floating */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* Connect EXTI Line0 to PA0 pin */
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource0);

  /* Configure EXTI Line0 */
  EXTI_InitStructure.EXTI_Line = EXTI_Line0;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;  
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  /* Enable and set EXTI Line0 Interrupt to the lowest priority */
  NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);


}
static void Blink(){
    GPIO_ToggleBits(GPIOG, GPIO_Pin_13|GPIO_Pin_14);
    delay(PAUSE_LONG);
    delay(PAUSE_LONG);
    delay(PAUSE_LONG);
    GPIO_ToggleBits(GPIOG, GPIO_Pin_13|GPIO_Pin_14);
    delay(PAUSE_LONG);
    delay(PAUSE_LONG);
    delay(PAUSE_LONG);

}

void InitUart2(){
    	//USART2 is connected to APB1 Periph Clock Bus and we enabled it.
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	
	//GPIOA is connected to AHB1 Periph Clock Bus and we enabled it.
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

  //Configuration of the GPIO pin for communication
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF; //This is important. We will this pin except of INPUT, OUTPUT and ANALOG so we set as Alternate Function
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_5 | GPIO_Pin_6;   //Communicate on PA2 and PA3
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//We set the pin as Push Pull
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL; //And Pull Up. Keep it always on HIGH
	GPIO_InitStructure.GPIO_Speed	= GPIO_Speed_50MHz; //And pin frequency
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	//When we set a pin as Alternate Function, we need to specify what function do we use for, here we specified it as USART 2
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_USART2);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource6, GPIO_AF_USART2);
	
	
	USART_InitStructure.USART_BaudRate 	              = 9600; //Set the baudrate. Hw many datas will be sent in a second.
	USART_InitStructure.USART_HardwareFlowControl     = USART_HardwareFlowControl_None;  //Do not check the data transmitted 
	USART_InitStructure.USART_Mode                    = USART_Mode_Tx | USART_Mode_Rx;//Data tansmission modes
	USART_InitStructure.USART_Parity                  = USART_Parity_No; // USART parity settings
	USART_InitStructure.USART_StopBits                = USART_StopBits_1; //Set the stop bits
	USART_InitStructure.USART_WordLength              = USART_WordLength_8b;  //we will send the 8bit data
	USART_Init(USART2, &USART_InitStructure);   //and started the this configuration

	USART_Cmd(USART2, ENABLE); //USART is activated and ready to use

}


void printBuffer(uint8_t* string){
      uint8_t* p = string;

      do{
        USART_SendData(USART2, *p); // line feed
        delay(100000); 
        p++;
      }
      while(*p != '\0');

}

bool heartBeat = false;
char itoaArray[3];
int main(void)
{   


    // Initializes PG13 and PG14 as outputs and blink with PG13 and PG14
    initGPIO();
    Blink();
    
    // Configure ~5Hz heartbeat 
    (SysTick_Config(0xFFFFFFUL)); 

    // PA0 as EXTI  
    initPA0_EXTI();

    // Initializes serial debugging on UART2 (9600, 8, 1, N) on PD5 (tx) and PD6 (rx)
    InitUart2();
    
 
    // Initializes SPI, L3GD20, GPIO and other structs for gyroscope 
    initGyro();


    // gyro data 
    volatile float Buffer[6];
    volatile uint8_t Xval, Yval, Zval = 0x00;

    uint8_t buffer[] = {'X', ':', ' ', ' ', 'Y', ':', ' ', ' ', 'Z', ':', ' ', '\0'};

    while (1)
    {

      GyroReadAngRate(Buffer);

        Xval = ABS((int8_t)(Buffer[0]));
        Yval = ABS((int8_t)(Buffer[1])); 
        Zval = ABS((int8_t)(Buffer[2])); 

      if(heartBeat){
        // Immediatley reset
        heartBeat = false;

        // Toggle heartbeat led PG14 
        GPIO_ToggleBits(GPIOG, GPIO_Pin_14);

        USART_SendData(USART2, '\n'); // line feed
        delay(100000); 
        USART_SendData(USART2, '\r'); // Carriage return
        delay(100000); 

        __itoa(Xval, &itoaArray, 10);
        buffer[2] = itoaArray[0];
        __itoa(Yval, &itoaArray, 10);
        buffer[6] = itoaArray[0];
         __itoa(Zval, &itoaArray, 10);
        buffer[10] = itoaArray[0];       

        printBuffer(&buffer);
     
        // alert user 

        // check system 

      }
  
    } // end of while(1)

    return 0; // every int has a return 
}
