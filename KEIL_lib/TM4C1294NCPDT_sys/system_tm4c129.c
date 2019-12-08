/**************************************************************************//**
 * @file     system_TM4C129.c
 * @brief    CMSIS Device System Source File for
 *           TI TIVA TM4C1294NCPDT Snowflake Device Series
 * @version  V1.00
 * @date     15. May 2013
 *
 * @note
 *
 *                                                             modified by Keil
 ******************************************************************************/

#include <stdint.h>
#include "TM4C1294NCPDT.h"

/*----------------------------------------------------------------------------
  DEFINES
 *----------------------------------------------------------------------------*/
//-------- <<< Use Configuration Wizard in Context Menu >>> ------------------
//
// This file can be used by the Keil uVision configuration wizard to set
// the following system clock configuration values.  Or the value of the
// macros can be directly edited below if not using the uVision configuration
// wizard.
//
//--------------------- Clock Configuration ----------------------------------
//
//  <e> Clock Configuration
//          <i> Uncheck this box to skip the clock configuration.
//
// The following controls whether the system clock is configured in the
// SystemInit() function.  If it is defined to be 1 then the system clock
// will be configured according to the macros in the rest of this file.
// If it is defined to be 0, then the system clock configuration is bypassed.
//
#define CLOCK_SETUP 1


//-----------------PLLREQ0-----------------//
//When controlling this register directly, software must change this value while the
//PLL is powered down. Writes to PLLFREQ0 are delayed from affecting the PLL until the RSCLKCFG
//register NEWFREQ bit is written with a 1.
//The PLL frequency can be calculated using the following equation:
//fVCO = (fIN * MDIV)
//where
//fIN = fXTAL/(Q+1)(N+1) or fPIOSC/(Q+1)(N+1)
//MDIV = MINT + (MFRAC / 1024)
//The Q and N values are programmed in the PLLFREQ1 register. Note that to reduce jitter, MFRAC
//should be programmed to 0x0.


//This bit controls power to the PLL. If set, the PLL power is applied and
//the PLL will oscillate based on the values in the PLLFREQ0 and
//PLLFREQ1 registers [23]
#define PLLPWR_ON  	1
#define PLLPWR_OFF 	0 //reset

//PLL M Fractional Value
//PLL M Integer Value
//This field contains the integer value of the PLL M value.
#define MFRAC				0 //reset [19:10]
#define MINT				0 //reset [9:0]


//-----------------PLLREQ1-----------------//
//PLL Q Value
//This field contains the PLL Q value
//PLL N Value
//This field contains the PLL N value.
#define Q						0 //reset [12:8]
#define	N						0 //reset [4:0]


//-----------------MEMTIM0-----------------//

//Specifies the length of the EEPROM bank clock high time
//Value Description
//0x0 1/2 system clock period
//0x1 1 system clock period
//0x2 1.5 system clock periods
//0x3 2 system clock periods
//0x4 2.5 system clock periods
//0x5 3 system clock periods
//0x6 3.5 system clock periods
//0x7 4 system clock periods
//0x8 4.5 system clock periods
#define EBCHT				0 //reset [25:22]

//Specifies the relationship of EEPROM clock to system clock
//Value Description
//0 EEPROM clock rising aligns with system clock rising
//1 EEPROM clock rising aligns with system clock falling
#define	EBCE				1 //reset [21]

//This field specifies the number of wait states inserted.
//Value Description
//0x0 0 wait states
//0x1 1 wait state
//0x2 2 wait states
//0x3 3 wait states
//0x4 4 wait states
//0x5 5 wait states
//0x6 6 wait states
//0x7 7 wait states
//0x8-0xF reserved
#define EWS					0 //reset [19:16]

//Specifies the length of the Flash bank clock high time
//Value Description
//0x0 1/2 system clock period
//0x1 1 system clock period
//0x2 1.5 system clock periods
//0x3 2 system clock periods
//0x4 2.5 system clock periods
//0x5 3 system clock periods
//0x6 3.5 system clock periods
//0x7 4 system clock periods
//0x8 4.5 system clock periods
#define FBCHT				0 //reset [9:6]

//Specifies the relationship of Flash clock to system clock
//Value Description
//0 Flash clock rising aligns with system clock rising
//1 Flash clock rising aligns with system clock falling
#define	FBCE				1 //reset [5]

//This field specifies the number of wait states inserted.
//Value Description
//0x0 0 wait states
//0x1 1 wait state
//0x2 2 wait states
//0x3 3 wait states
//0x4 4 wait states
//0x5 5 wait states
//0x6 6 wait states
//0x7 7 wait states
//0x8-0xF reserved
#define FWS					0 //reset [3:0]


//-----------------PLLSTAT-----------------//
//Read Only
//Value Description
//0 The PLL is unpowered or is not yet locked.
//1 The PLL powered and locked.
#define LOCK			0 //resert [0]

//POR State Power-On Reset State
//
//


#define NOXTAL 			(1U << 2)
#define PWRDN 			(1U << 3)
#define MOSCPUPRIS	(1U << 8)
#define MOSC				(0x3U << 20)
#define DSOSCSRC		(0x3U << 20)


/**
 * Initialize the system
 *
 * @param  none
 * @return none
 *
 * @brief  Setup the microcontroller system.
 *         Initialize the System.
 */
void SystemInit (void){
	#if(CLOCK_SETUP)
			volatile uint32_t i;
	#endif
		
	/* FPU settings ------------------------------------------------------------*/
	#if (__FPU_USED == 1)
		SCB->CPACR |= ((3UL << 10*2) |                 /* set CP10 Full Access */
									 (3UL << 11*2)  );               /* set CP11 Full Access */
	#endif
		
	//Step 1:
	//Once POR has completed, the PIOSC is acting as the system clock.

	//Is the system Clock PIOSC?
	#if(CLOCK_SETUP)
		uint32_t volatile RSCLK_CFG, MOSCPUP_RIS, LOCK_val ;
		
		do {RSCLK_CFG = SYSCTL->RSCLKCFG;}
		while((RSCLK_CFG & (1U<<20)) || (RSCLK_CFG & (1U<<21)) ||
					(RSCLK_CFG & (1U<<22)) || (RSCLK_CFG & (1U<<23))!= 0x00);
	//PIOSC is acting as the system clk
		
	 //Power up the MOSC by clearing the NOXTAL bit in the MOSCCTL register.
		
		SYSCTL->MOSCCTL &= ~NOXTAL;
		
	//If single-ended MOSC mode is required, the MOSC is ready to use. If crystal mode is required,
	//clear the PWRDN bit and wait for the MOSCPUPRIS bit to be set in the Raw Interrupt Status
	//(RIS), indicating MOSC crystal mode is read
		
		SYSCTL->MOSCCTL &= ~PWRDN;
		
		do {MOSCPUP_RIS = SYSCTL->RIS;}
		while((MOSCPUP_RIS & MOSCPUPRIS)!= MOSCPUPRIS);
		
	// Set the OSCSRC field to 0x3 in the RSCLKCFG register at offset 0x0B0
		SYSCTL->RSCLKCFG |= MOSC;
		
		
	//If the application also requires the MOSC to be the deep-sleep clock source, then program the
	//DSOSCSRC field in the DSCLKCFG register to 0x3
		SYSCTL->DSCLKCFG |= DSOSCSRC;
		
	//Write the PLLFREQ0 and PLLFREQ1 registers with the values of Q, N, MINT, and MFRAC to
	//the configure the desired VCO frequency setting.
	//Write the MEMTIM0 register to correspond to the new system clock setting.
		
		SYSCTL ->PLLFREQ0 = 0x00000000;
		SYSCTL ->PLLFREQ1 = 0x00000000;
		SYSCTL ->MEMTIM0	= 0x00300030;
		 
	//Wait for the PLLSTAT register to indicate the PLL has reached lock at the new operating point
		
	//	do{LOCK_val = SYSCTL ->PLLSTAT;}
	//	while(LOCK_val != 0x1U);
		
	#endif

}



void Q_onAssert(char const *module, int loc) {
    /* TBD: damage control */
    (void)module; /* avoid the "unused parameter" compiler warning */
    (void)loc;    /* avoid the "unused parameter" compiler warning */
    NVIC_SystemReset();
}


//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//




/*----------------------------------------------------------------------------
  Define clocks
 *----------------------------------------------------------------------------*/
#define __SYSTEM_CLOCK    (16000000ul)


/*----------------------------------------------------------------------------
  Clock Variable definitions
 *----------------------------------------------------------------------------*/
uint32_t SystemCoreClock = __SYSTEM_CLOCK; /*!< System Clock Frequency (Core Clock)*/


/*----------------------------------------------------------------------------
  Clock functions
 *----------------------------------------------------------------------------*/
void SystemCoreClockUpdate (void)            /* Get Core Clock Frequency      */
{
  /* TODO: Updates required to fully work with TM4C1294NCPDT series devices */
  SystemCoreClock = __SYSTEM_CLOCK;

}

