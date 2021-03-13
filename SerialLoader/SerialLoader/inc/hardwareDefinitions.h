/*
 * Hardware Specific Definitions
 *
 * This header file contains the definitions of what GPIOs and what
 * peripherals are used by the software for different hardware configurations.
 * The current possible hardware configurations are the Discovery Card and
 * the Fox IHU card.
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HARDWARE_DEFINITIONS_H
#define __HARDWARE_DEFINITIONS_H

/* Includes ------------------------------------------------------------------*/
//#include "FoxConfig.h"
#include "stm32l1xx.h"

/*------------------------------------------------------------------------------------------
 * This section is for the SPI driver
 */

/* Assume that all the SPI2 pins are on the same GPIO.  Otherwise must change this */

#define SPI2_CLK                         RCC_APB1Periph_SPI2
#define SPI2_APB						 1
#define SPI2_GPIO_PORT					 GPIOB
#define SPI2_GPIO_CLK					 RCC_AHBPeriph_GPIOB
#define SPI2_AF					 		 GPIO_AF_SPI2

#define SPI2_SCK_PIN                     GPIO_Pin_13
#define SPI2_SCK_SOURCE                  GPIO_PinSource13

#define SPI2_MISO_PIN                    GPIO_Pin_14
#define SPI2_MISO_SOURCE                 GPIO_PinSource14

#define SPI2_MOSI_PIN                    GPIO_Pin_15
#define SPI2_MOSI_SOURCE                 GPIO_PinSource15

#define SPI2_RX_DMA_NUMBER				 4
#define SPI2_TX_DMA_NUMBER				 5

#define SPI2_IRQ_HANDLER                 SPI2_IRQHandler
//////////////////////////////////////////////////////////

#define SPI1_CLK                         RCC_APB2Periph_SPI1
#define SPI1_APB						 2
#define SPI1_GPIO_PORT					 GPIOE
#define SPI1_GPIO_CLK					 RCC_AHBPeriph_GPIOE
#define SPI1_AF					 		 GPIO_AF_SPI1

#define SPI1_SCK_PIN                     GPIO_Pin_13
#define SPI1_SCK_SOURCE                  GPIO_PinSource13

#define SPI1_MISO_PIN                    GPIO_Pin_14
#define SPI1_MISO_SOURCE                 GPIO_PinSource14

#define SPI1_MOSI_PIN                    GPIO_Pin_15
#define SPI1_MOSI_SOURCE                 GPIO_PinSource15

#define SPI1_RX_DMA_NUMBER				 2
#define SPI1_TX_DMA_NUMBER				 3
#define SPI1_IRQ_HANDLER                 SPI1_IRQHandler


/*This is for the MRAM chip select */
#define SPI_MRAM_NSS_PIN                     GPIO_Pin_0
#define SPI_MRAM_NSS_GPIO_PORT               GPIOD
#define SPI_MRAM_NSS_GPIO_CLK                RCC_AHBPeriph_GPIOD
#define SPI_MRAM_NSS_SOURCE                  GPIO_PinSource0

/* This is for the CAN tranceiver chip select */
#define SPI_CAN_NSS_PIN					GPIO_Pin_1
#define SPI_CAN_NSS_GPIO_PORT			GPIOD
#define SPI_CAN_NSS_GPIO_CLK			RCC_AHBPeriph_GPIOD
#define SPI_CAN_NSS_SOURCE				GPIO_PinSource0

#define SPI_GYRO_NSS_PIN				GPIO_Pin_7
#define SPI_GYRO_NSS_GPIO_PORT			GPIOA
#define SPI_GYRO_NSS_GPIO_CLK			RCC_AHBPeriph_GPIOA
#define SPI_GYRO_NSS_SOURCE				GPIO_PinSource7

#define SPI_MOD_NSS_PIN					GPIO_Pin_5 /* Changed from Fox-1E to LTI */
#define SPI_MOD_NSS_GPIO_PORT			GPIOE
#define SPI_MOD_NSS_GPIO_CLK		    RCC_AHBPeriph_GPIOE
#define SPI_MOD_NSS_SOURCE				GPIO_PinSource6


#define DMA_ALLOC_TIMEOUT					(5000/portTICK_RATE_MS)  /* Wait 5 seconds for a DMA channel */
#define SPI_USER_TIMEOUT                    (5000/portTICK_RATE_MS) /* Waiting 20s--watchdog will trigger */

/*
 * End of SPI definitions
 *---------------------------------------------------------------------------------
 * Start of USART Definitions
 * COM1 is the DEBUG port; COM2 is the experiment port
 */
#define IHU_BOARD

#ifdef IHU_BOARD
#define COM1_USART 			USART1
#define COM1_TX_PORT 		GPIOA
#define COM1_RX_PORT 		GPIOA
#define COM1_USART_CLK 		RCC_APB2Periph_USART1
#define COM1_USART_APB		2
#define COM1_TX_PORT_CLK 	RCC_AHBPeriph_GPIOA
#define COM1_RX_PORT_CLK 	RCC_AHBPeriph_GPIOA
#define COM1_TX_PIN 		GPIO_Pin_9
#define COM1_RX_PIN 		GPIO_Pin_10
#define COM1_TX_PIN_SOURCE 	GPIO_PinSource9
#define COM1_RX_PIN_SOURCE 	GPIO_PinSource10
#define COM1_TX_AF 			GPIO_AF_USART1
#define COM1_RX_AF 			GPIO_AF_USART1
#define COM1_TX_DMA_CHAN	4
#define COM1_RX_DMA_CHAN	5
#define COM1_IRQ 			USART1_IRQn
#define COM1_IRQHandler 	USART1_IRQHandler

#define COM2_USART 			USART2
#define COM2_TX_PORT 		GPIOD
#define COM2_RX_PORT 		GPIOD
#define COM2_USART_CLK 		RCC_APB1Periph_USART2
#define COM2_USART_APB		1
#define COM2_TX_PORT_CLK 	RCC_AHBPeriph_GPIOD
#define COM2_RX_PORT_CLK 	RCC_AHBPeriph_GPIOD
#define COM2_TX_PIN 		GPIO_Pin_5
#define COM2_RX_PIN 		GPIO_Pin_6
#define COM2_TX_PIN_SOURCE 	GPIO_PinSource5
#define COM2_RX_PIN_SOURCE 	GPIO_PinSource6
#define COM2_TX_AF 			GPIO_AF_USART2
#define COM2_RX_AF 			GPIO_AF_USART2
#define COM2_TX_DMA_CHAN	7
#define COM2_RX_DMA_CHAN	6
#define COM2_IRQ 			USART2_IRQn
#define COM2_IRQHandler 	USART2_IRQHandler

/*
 * We can't use USART3 on the IHU board as long as we require DMA.  The DMA channels it uses are
 * in continuous use by the ADC/DAC
 */

#define COM3_USART 			USART3
#define COM3_TX_PORT 		GPIOC
#define COM3_RX_PORT 		GPIOC
#define COM3_USART_CLK 		RCC_APB1Periph_USART3
#define COM3_USART_APB		1
#define COM3_TX_PORT_CLK 	RCC_AHBPeriph_GPIOC
#define COM3_RX_PORT_CLK 	RCC_AHBPeriph_GPIOC
#define COM3_TX_PIN 		GPIO_Pin_10
#define COM3_RX_PIN 		GPIO_Pin_11
#define COM3_TX_PIN_SOURCE 	GPIO_PinSource10
#define COM3_RX_PIN_SOURCE 	GPIO_PinSource11
#define COM3_TX_AF 			GPIO_AF_USART3
#define COM3_RX_AF 			GPIO_AF_USART3
#define COM3_TX_DMA_CHAN	0
#define COM3_RX_DMA_CHAN	0
#define COM3_IRQ 			USART3_IRQn
#define COM3_IRQHandler 	USART3_IRQHandler


#endif  /* End of USART IHU Board definitions */

/*------------------------------------------------------------------
 *  ADC definitions
 */
#if defined(IHU_BOARD) || defined(DISCOVERY_BOARD)
/* The same for both boards */
#define ADCPort1 			ADC1
#define ADCPort1Clk			RCC_APB2Periph_ADC1
#define ADCPort1DMA			DMA1_Channel1
#define ADCPort1DMAClk		RCC_AHBPeriph_DMA1
#define ADCPort1DMAIntChan	DMA1_Channel1_IRQn
#define ADCTempChannel		ADC_Channel_16
#define ADCVRefChannel		ADC_Channel_17
#define ADCPort1DMAHTInt	DMA1_IT_HT1
#define ADCPort1DMATCInt	DMA1_IT_TC1
#define ADCPort1DMATEInt	DMA1_IT_TE1
#endif /* End of common things between boards */


#ifdef IHU_BOARD
#define ADCPort1AudioInGPIO	GPIOA
#define ADCPort1AudioInPin	GPIO_Pin_6
#define ADCAudioInChannel	ADC_Channel_6

#define ADCPACurrentGPIO	GPIOA
#define ADCPACurrentPin		GPIO_Pin_0
#define ADCPACurrentChannel	ADC_Channel_0

#define ADCTxTempGPIO		GPIOA
#define ADCTxTempPin		GPIO_Pin_1
#define ADCTxTempChannel	ADC_Channel_1

#define ADCVGACntlGPIO		GPIOA
#define ADCVGACntlPin		GPIO_Pin_3
#define ADCVGACntlChannel	ADC_Channel_3

/***********************The gyros will go away, but no hard for now. */

#define ADCSpare1GPIO		GPIOC
#define ADCSpare1Pin		GPIO_Pin_2
#define ADCSpare1Channel	ADC_Channel_12

#define ADCSpare2GPIO		GPIOC
#define ADCSpare2Pin		GPIO_Pin_3
#define ADCSpare2Channel	ADC_Channel_13

#define ADCSpare3GPIO		GPIOC
#define ADCSpare3Pin		GPIO_Pin_4
#define ADCSpare3Channel	ADC_Channel_14

#define ADCSpare4GPIO		GPIOC
#define ADCSpare4Pin		GPIO_Pin_5
#define ADCSpare4Channel	ADC_Channel_15

#define ADCSpare5GPIO		GPIOE
#define ADCSpare5Pin		GPIO_Pin_9
#define ADCSpare5Channel	ADC_Channel_24

#define ADCSpare6GPIO		GPIOE
#define ADCSpare6Pin		GPIO_Pin_10
#define ADCSpare6Channel	ADC_Channel_25
/****************************************End of Gyro */

#define ADCPlus3VChannel	ADC_Channel_18
#define ADCPort1Plus3VPin	GPIO_Pin_12
#define ADCPort1Plus3VGPIO 	GPIOB

#define ADCRssiGPIO			GPIOB  /* This is the ICR RSSI */
#define ADCRssiPin			GPIO_Pin_0
#define ADCRssiChannel		ADC_Channel_8

#define ADCVBattGPIO		GPIOA
#define ADCVBattPin			GPIO_Pin_5
#define ADCVBattChannel		ADC_Channel_5

#define ADCFwdPowerGPIO		GPIOB
#define ADCFwdPowerPin		GPIO_Pin_1
#define ADCFwdPowerChannel	ADC_Channel_9

#endif
/*---------------------------------------------------------------------
 * DAC Definitions
 */


#if defined(IHU_BOARD)
#define DACPort1AudioOutPin	GPIO_Pin_4
#define DACPort1AudioOutGPIO	GPIOA
/* If it is pin A4, it has to be DAC Channel 1 */
#define DACPort1 			DAC
#define DACPort1Channel		DAC_Channel_1
/* If it is DAC Channel 1, it has to be DMA Channel 2 */
#define DACPort1DMAIntChan	DMA1_Channel2_IRQn
#define DACPort1DMAChan		DMA1_Channel2
#define DACPort1DMANum		2

#if 0
/*
 * The ADC and DAC are set up to run synchronously and so we only
 * take interrupts on the ADC.  For the DAC we just double check the
 * flag on the ADC interrupt
 */

#define DACPort1DMAISRName	DMA1_Channel2_IRQHandler
#define DACPort1DMAHTInt	DMA1_IT_HT2
#define DACPort1DMATCInt	DMA1_IT_TC2
#define DACPort1DMATEInt	DMA1_IT_TE2
#endif

#define DACPort1DMAHTFlag	DMA1_FLAG_HT2
#define DACPort1DMATCFlag	DMA1_FLAG_TC2
#define DACPort1DMATEFlag	DMA1_FLAG_TE2
#define DACPort1BaseReg     DHR12R1

#define DACPort1Clk			RCC_APB1Periph_DAC
#define DACPort1GPIOClk		RCC_AHBPeriph_GPIOA
#define DACPort1DMAClk		RCC_AHBPeriph_DMA1
#endif



/*---------------------------------------------------------------------
 * GPIO Definitions
 * For Reference:
 * typedef enum gu {LED3, LED4,CommandStrobe,CommandData,Debug1,Debug2,MCOOut,BurnWire2,
 *				BurnWire1, DeploySense1, DeploySense2} Gpio_Use;
 *
 */



/*
 * Each GPIO input or output function has 3 or 4 definitions.  First is
 * GPIO <function>Port, which is the GPIO structure itself.  (GPIOA, GPIOB, etc)
 * Next, GPIO<function>Pin  is the pin bit as defined by the standard peripheral library.
 * If the function is a multi-bit input (mb output is not implemented) specify
 * only the least significant.  The gpio.c driver assumes that the bits are
 * adjacent.  The driver knows how many bits are used for a function.  Since that is
 * not board-specific, it is not here.  And finally specify the pin as an integer
 * for GPIO<function>PinNum.
 *
 * In the case of an input that will drive an interrupt, GPIO<function>Int
 * is the EXTI_PortSource for the specified GPIO port.
 */


#ifdef IHU_BOARD
/* These are LEDs on the IHU board */
#define GPIOLed1Port				GPIOC
#define GPIOLed1Pin					GPIO_Pin_6
#define GPIOLed1PinNum					6

#define GPIOLed2Port				GPIOC
#define GPIOLed2Pin					GPIO_Pin_7
#define GPIOLed2PinNum					7

/* These are LEDs on both boards */
#define GPIOLed3Port				GPIOC
#define GPIOLed3Pin					GPIO_Pin_8
#define GPIOLed3PinNum					8

#define GPIOLed4Port				GPIOC
#define GPIOLed4Pin					GPIO_Pin_9
#define GPIOLed4PinNum					9

#define GPIOAlertPort				GPIOC
#define GPIOAlertPin				GPIO_Pin_12
#define GPIOAlertPinNum				12

/* I2c Reset Pins */

#define GPIOI2c1ResetPort			GPIOE
#define GPIOI2c1ResetPin			GPIO_Pin_11
#define GPIOI2c1ResetPinNum			11

#define GPIOI2c2ResetPort			GPIOC
#define GPIOI2c2ResetPin			GPIO_Pin_3
#define GPIOI2c2ResetPinNum			3


/* Command data is actually 4 pins; the code assumes them to be contiguous */
#define GPIOCmdDataPort				GPIOE
#define GPIOCmdDataPin				GPIO_Pin_0
#define GPIOCmdDataPinNum			0
#define GPIOCmdDataNumBits			4
#define GPIOCmdDataPortEnable		STM_RCC_AHBENR_GPIOEEN

/* For linear, it is Pass Band Enable */
#define GPIOPBEnablePort			GPIOE
#define GPIOPBEnablePin				GPIO_Pin_6
#define GPIOPBEnablePinNum			0

/* For linear, it is Telemetry Enable */
#define GPIOTlmEnablePort			GPIOD
#define GPIOTlmEnablePin			GPIO_Pin_14
#define GPIOTlmEnablePinNum		    14

#define GPIOWDResetPort				GPIOD /* Watchdog Reset */
#define GPIOWDResetPin				GPIO_Pin_3
#define GPIOWDResetPinNum			3
#define GPIOWDResetPortEnable		STM_RCC_AHBENR_GPIODEN

#define GPIOAttachedPort			GPIOB
#define GPIOAttachedPin				GPIO_Pin_8
#define GPIOAttachedPinNum			8
#define GPIOAttachedPortEnable		STM_RCC_AHBENR_GPIOBEN

#define GPIOExp1EnbPort				GPIOD
#define GPIOExp1Pin					GPIO_Pin_4
#define GPIOExp1PinNum				4

/*
 * Line to enable Tx to transmit audio from the IHU.  AKA RxTx Control
 */
#define GPIOIhuRfPort				GPIOE
#define GPIOIhuRfPin				GPIO_Pin_12
#define GPIOIhuRfPinNum				12
/*
 * Rx ICR CD -- Command Receiver sees data signals
 */

#define GPIOCmdRxCDPort				GPIOD
#define GPIOCmdRxCDPin				GPIO_Pin_15
#define GPIOCmdRxCDPinNum			15

/*
 * Below here are GPIO ports that can generate interrupts (as well as
 * related ports
 */

/* For CAN controller/tranceiver*/

#define GPIOCanResetPort			GPIOC
#define GPIOCanResetPin				GPIO_Pin_14
#define GPIOCanResetPinNum				14

#define GPIOCanStbyPort				GPIOC
#define GPIOCanStbyPin				GPIO_Pin_15
#define GPIOCanStbyPinNum			15

#define GPIOCanRTSPort				GPIOD
#define GPIOCanRTSPin				GPIO_Pin_11
#define GPIOCanRTSPinNum			11
#define GPIOCanRTSNumBits			3

#define GPIOCanBFPort				GPIOD
#define GPIOCanBFPin				GPIO_Pin_7
#define GPIOCanBFPinNum				7
#define GPIOCanBFNumBits			2

#define GPIOCanIntPort				GPIOD
#define GPIOCanIntPin				GPIO_Pin_10
#define GPIOCanIntPinNum			10
/* All kinds of variants of the line and port number for EXTI */
#define GPIOCanIntRoutine     		EXTI15_10_IRQHandler
#define GPIOCanIntEXTIline			EXTI_Line10
#define GPIOCanIntEXTIport			EXTI_PortSourceGPIOD
#define GPIOCanIntEXTIirq        	EXTI15_10_IRQn
#define GPIOCanIntEXTITrigger		EXTI_Trigger_Falling

/*
 * Here are the four lines for coordinating the LIHU and the RT-IHU on Golf.  Only the incoming lines
 * from the RT-IHU generate interrupts, and they are both on the same interrupt line.  We almost don't
 * care which one changed.
 */

#define GPIOCoordL0Port				GPIOD
#define GPIOCoordL0Pin					GPIO_Pin_9
#define GPIOCoordL0PinNum				9

#define GPIOCoordL1Port				GPIOD
#define GPIOCoordL1Pin					GPIO_Pin_2
#define GPIOCoordL1PinNum				2

#define GPIOCoordR0Port				GPIOA
#define GPIOCoordR0Pin				GPIO_Pin_8
#define GPIOCoordR0PinNum			8
#define GPIOCoordR0IntEXTIline		EXTI_Line8
#define GPIOCoordR0IntEXTIport		EXTI_PortSourceGPIOA
#define GPIOCoordR0IntEXTIirq       EXTI9_5_IRQn
#define GPIOCoordR0IntEXTITrigger	EXTI_Trigger_Rising_Falling

#define GPIOCoordR1Port				GPIOB
#define GPIOCoordR1Pin				GPIO_Pin_5
#define GPIOCoordR1PinNum			5
#define GPIOCoordR1IntEXTIline		EXTI_Line5
#define GPIOCoordR1IntEXTIport		EXTI_PortSourceGPIOB
#define GPIOCoordR1IntEXTIirq       EXTI9_5_IRQn
#define GPIOCoordR1IntEXTITrigger	EXTI_Trigger_Rising_Falling
#define GPIOCoordRxIntRoutine		EXTI9_5_IRQHandler

/*
 * Command strobe
 */
#define GPIOCmdStrobePort			GPIOE
#define GPIOCmdStrobePin			GPIO_Pin_4
#define GPIOCmdStrobePinNum			4
/* All kinds of variants of the line and port number for EXTI */
#define GPIOCmdStrobeIntRoutine     EXTI4_IRQHandler
#define GPIOCmdStrobeEXTIline		EXTI_Line4
#define GPIOCmdStrobeEXTIport		EXTI_PortSourceGPIOE
#define GPIOCmdStrobeEXTIirq        EXTI4_IRQn
#define GPIOCmdStrobeEXTITrigger    EXTI_Trigger_Rising


#endif

/*
 * I2Cx Communication boards Interface
 */



#ifdef IHU_BOARD
#define I2Cx                          I2C1
#define I2Cx_SDA_GPIO_CLK             RCC_AHB1Periph_GPIOB
#define I2Cx_SDA_GPIO_PORT            GPIOB
#define I2Cx_SCL_GPIO_CLK             RCC_AHB1Periph_GPIOB
#define I2Cx_SCL_GPIO_PORT            GPIOB
#define I2Cx_EV_IRQn                  I2C1_EV_IRQn
#define I2Cx_ER_IRQn                  I2C1_ER_IRQn
#define I2Cx_EV_IRQHANDLER            I2C1_EV_IRQHandler
#define I2Cx_ER_IRQHANDLER            I2C1_ER_IRQHandler

#define I2C1_CLK                      RCC_APB1Periph_I2C1
#define I2C1_SDA_PIN                  GPIO_Pin_7
#define I2C1_SDA_SOURCE               GPIO_PinSource7
#define I2C1_SDA_AF                   GPIO_AF_I2C1
#define I2C1_SCL_PIN                  GPIO_Pin_6
#define I2C1_SCL_SOURCE               GPIO_PinSource6
#define I2C1_SCL_AF                   GPIO_AF_I2C1

#define I2C2_CLK                      RCC_APB1Periph_I2C2
#define I2C2_SDA_PIN                  GPIO_Pin_11
#define I2C2_SDA_SOURCE               GPIO_PinSource11
#define I2C2_SDA_AF                   GPIO_AF_I2C2
#define I2C2_SCL_PIN                  GPIO_Pin_10
#define I2C2_SCL_SOURCE               GPIO_PinSource10
#define I2C2_SCL_AF                   GPIO_AF_I2C2

#endif

#endif /* __HARDWARE_DEFINITIONS_H */

