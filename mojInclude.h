
#include "stdint.h"




    //initPins();

    //      #define PERIPH_BASE               ((uint32_t)0x40000000) 
    //      #define AHB1PERIPH_BASE       (PERIPH_BASE + 0x00020000)
    //      #define GPIOG_BASE            (AHB1PERIPH_BASE + 0x1800)
    //      BSRR offset:                                       (0x18)
    
                                                         // set bit 13 
    #define GPIOG_BSRRLa                 0x40021818      // *((uint32_t*) GPIOG_BSRRLa) |= 1<<13;
	#define GPIOG_BSRRLp    ((uint32_t*) 0x40021818)     // *GPIOG_BSRRLp |= 1<<13
	#define GPIOG_BSRRLdp (*((uint32_t*) 0x40021818))    // GPIOG_BSRRLdp |= 1<<13;

    #define GPIOG_BSRRHdp (*((uint32_t*) 0x4002181A))


    /***********************************/
    /***** PG13 pin initialization *****/
    /***********************************/

    // // enable clock for GPIOG periphery
    // // RCC -> AHB1ENR = 0x40023800 -> 0x30
    // RCC->AHB1ENR |=  RCC_AHB1Periph_GPIOG;

    // // GPIOG = 0x40021800
	// // mode register MODER =0x00
    // GPIOG->MODER  &= ~(GPIO_MODER_MODER0 << (13 * 2));
	// GPIOG->MODER |= (((uint32_t) GPIO_Mode_OUT) << (13 * 2));
    
    // // speed register SPEEDR = 0x08, 
	// GPIOG->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR0 << (13 * 2));
	// GPIOG->OSPEEDR |= ((uint32_t)GPIO_Speed_25MHz << (13 * 2));
    
    // // output type register  OTYPER = 0x04
	// GPIOG->OTYPER  &= ~((GPIO_OTYPER_OT_0) << ((uint16_t)13)) ;
	// GPIOG->OTYPER |= (uint16_t)(((uint16_t)GPIO_OType_PP) << ((uint16_t)13));
    
    // // pull up-down register PUPDR = 0x0C
	// GPIOG->PUPDR &= ~(GPIO_PUPDR_PUPDR0 << ((uint16_t)13 * 2));
	// GPIOG->PUPDR |= (((uint32_t)GPIO_PuPd_NOPULL) << (13 * 2));

    // GPIOG_BSRRLdp |= 1<<13;
    // delay(PAUSE_LONG);
    // delay(PAUSE_LONG);
    // delay(PAUSE_LONG);

    // GPIOG_BSRRHdp |= 1<<13;
    // delay(PAUSE_LONG);
    // delay(PAUSE_LONG);
    // delay(PAUSE_LONG);
