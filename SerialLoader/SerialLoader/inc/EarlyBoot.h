/*
 * EarlyBoot.h
 *
 *  Created on: Jan 8, 2019
 *      Author: fox
 */

#ifndef EARLYBOOT_H_
#define EARLYBOOT_H_

//#include "misc.h"     /* We don't use the peripheral library, but a few constants are useful */
#include "hardwareDefinitions.h"
#include "memory.h"
#include "ao_stm32l.h"

#define stm_eeprom ((uint8_t *) 0x08080000) /* This definition is not in ao_stm32l.h */

/*
 * Here are a bunch of macros to do the GPIO operations we need for antenna deployment
 */

#define ResetExternalWatchdog() \
		stm_gpio_set((struct stm_gpio *)GPIOWDResetPort,GPIOWDResetPinNum,1);\
		stm_gpio_set((struct stm_gpio *)GPIOWDResetPort,GPIOWDResetPinNum,0);

#define GPIORead_UmbilicalAttached() stm_gpio_get((struct stm_gpio *)GPIOAttachedPort,GPIOAttachedPinNum)
//#define GPIORead_DeploySenseRx() stm_gpio_get((struct stm_gpio *)GPIORxDeploySnsPort,GPIORxDeploySnsPinNum)
#define GPIORead_DeploySenseTx() stm_gpio_get((struct stm_gpio *)GPIOTxDeploySnsPort,GPIOTxDeploySnsPinNum)

//#define GPIOSetOn_AntDepTx() stm_gpio_set((struct stm_gpio *)GPIOTxAntDepPort,GPIOTxAntDepPinNum,1)
//#define GPIOSetOff_AntDepTx() stm_gpio_set((struct stm_gpio *)GPIOTxAntDepPort,GPIOTxAntDepPinNum,0)
//#define GPIOSetOn_AntDepRx() stm_gpio_set((struct stm_gpio *)GPIORxAntDepPort,GPIORxAntDepPinNum,1)
//#define GPIOSetOff_AntDepRx() stm_gpio_set((struct stm_gpio *)GPIORxAntDepPort,GPIORxAntDepPinNum,0)
#define GPIOSetOn_RFControl() stm_gpio_set((struct stm_gpio *)GPIOIhuRfPort,GPIOIhuRfPinNum,1)
#define GPIOSetOff_RFControl() stm_gpio_set((struct stm_gpio *)GPIOIhuRfPort,GPIOIhuRfPinNum,0)

#define GPIOSetOn_Led1() stm_gpio_set((struct stm_gpio *)GPIOLed1Port,GPIOLed1PinNum,1)
#define GPIOSetOff_Led1() stm_gpio_set((struct stm_gpio *)GPIOLed1Port,GPIOLed1PinNum,0)
#define GPIOToggle_Led1() stm_gpio_set((struct stm_gpio *)GPIOLed1Port,GPIOLed1PinNum,\
		1&(~stm_gpio_get((struct stm_gpio *)GPIOLed1Port,GPIOLed1PinNum)))

#define GPIOSetOn_Led2() stm_gpio_set((struct stm_gpio *)GPIOLed2Port,GPIOLed2PinNum,1)
#define GPIOSetOff_Led2() stm_gpio_set((struct stm_gpio *)GPIOLed2Port,GPIOLed2PinNum,0)
#define GPIOToggle_Led2() stm_gpio_set((struct stm_gpio *)GPIOLed1Port,GPIOLed1PinNum,\
		1&(~stm_gpio_get((struct stm_gpio *)GPIOLed2Port,GPIOLed2PinNum)))

#define GPIOSetOn_Led3() stm_gpio_set((struct stm_gpio *)GPIOLed3Port,GPIOLed3PinNum,1)
#define GPIOSetOff_Led3() stm_gpio_set((struct stm_gpio *)GPIOLed3Port,GPIOLed3PinNum,0)
#define GPIOToggle_Led3() stm_gpio_set((struct stm_gpio *)GPIOLed3Port,GPIOLed3PinNum,\
		1&(~stm_gpio_get((struct stm_gpio *)GPIOLed3Port,GPIOLed3PinNum)))

#define GPIOSetOn_Led4() stm_gpio_set((struct stm_gpio *)GPIOLed4Port,GPIOLed4PinNum,1)
#define GPIOSetOff_Led4() stm_gpio_set((struct stm_gpio *)GPIOLed4Port,GPIOLed4PinNum,0)
#define GPIOToggle_Led4() stm_gpio_set((struct stm_gpio *)GPIOLed4Port,GPIOLed4PinNum,\
		1&(~stm_gpio_get((struct stm_gpio *)GPIOLed4Port,GPIOLed4PinNum)))


#define GPIOToggle_Alert() stm_gpio_set((struct stm_gpio *)GPIOAlertPort,GPIOAlertPinNum,\
		1&(~stm_gpio_get((struct stm_gpio *)GPIOAlertPort,GPIOAlertPinNum)))
#define GPIOSetOff_Alert() stm_gpio_set((struct stm_gpio *)GPIOAlertPort,GPIOAlertPinNum,0);
#define GPIOSetOn_Alert() stm_gpio_set((struct stm_gpio *)GPIOAlertPort,GPIOAlertPinNum,1);

#define WaitWithNoOS(time) {\
		unsigned register int timeDecrement = time*PRE_OS_LOOP_FACTOR;\
		while ((timeDecrement--)>0){\
			ResetExternalWatchdog();\
			if((timeDecrement & ALERT_FLASH_PERIOD)!=0){ \
				GPIOSetOff_Alert();\
			}\
			else { \
				if((timeDecrement & ALERT_TONE_PERIOD)==0) \
				GPIOToggle_Alert();\
			}\
		};\
}


#endif /* EARLYBOOT_H_ */
