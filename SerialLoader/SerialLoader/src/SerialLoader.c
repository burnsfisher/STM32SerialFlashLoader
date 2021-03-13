/*

 * SerialLoader.c
 *
 *  Created on: Jan 7, 2019
 *      Author: WB1FJ
 *
 * This is a boot loader for a USART serial port designed to work similarly to the Altos Metrum USB loader
 * It does not echo.  The commands are as follows:
 *
 * v  - Print info about the loader, version, and min/max addresses available
 * a  - Jump to the start of the application code
 * W<hex address>\n Expects a page (0x100 bytes) of binary data to follow
 * R<hex address>\n Sends a page of binary data
 * X<hex address>\n Clear the specified page
 * Z force watchdog reset
 * s Status for debugging
 */

//#include "hardwareDefinitions.h"
#include "ao_timer.h"
#include "ao_stm32l.h"
#include "ao_flash.h"
#include "EarlyBoot.h"
#include "stdbool.h"
#include "SerialLoader.h"
#include "Addresses.h"
#define CLOCK_SPEED 32000000 // This is 32MHz
//#define DEBUG
#define TIMEOUT (0xff8000)
#define UART_NUMBER 3
#if UART_NUMBER == 1
#define stm_usart stm_usart1
#elif UART_NUMBER == 2
#define stm_usart stm_usart2
#define stm_uart_gpio stm_gpiod
#define UART_TX_PIN 5
#define UART_RX_PIN 6
#elif UART_NUMBER == 3
#define stm_usart stm_usart3
#define stm_uart_gpio stm_gpioc
#define UART_TX_PIN 10
#define UART_RX_PIN 11
#ifdef DEBUG
#define stm_dbgusart stm_usart2
#define stm_dbguart_gpio stm_gpiod
#define dbgUART_TX_PIN 5
#define dbgUART_RX_PIN 6
#endif

#endif

// Forward routines
void InitUART(uint32_t Baudrate);
void UART_Write(char myChar);
char UART_Read(char prompt);
uint32_t readHex32(void);
void printString(char *);
void printHex32(uint32_t);
#ifdef DEBUG
void InitDbgUART(uint32_t BaudRate);
void dbgUART_Write(char character);
void dbgprintString(char *);
void dbgprintHex32(uint32_t);
#endif
void CopyRamText(void);
void InitRCC(void);
void InitGPIO(void);
void ao_boot_chain(uint32_t *);

uint32_t kindaRandom=0;
static inline void
ao_arch_block_interrupts(void) {
#ifdef AO_NONMASK_INTERRUPTS
	asm("msr basepri,%0" : : "r" (AO_STM_NVIC_BASEPRI_MASK));
#else
	asm("cpsid i");
#endif
}

static inline void
ao_arch_release_interrupts(void) {
#ifdef AO_NONMASK_INTERRUPTS
	asm("msr basepri,%0" : : "r" (0x0));
#else
	asm("cpsie i");
#endif
}
uint32_t SaveAcrossReset[4] __attribute__ ((section(".ERROR_RAM")));
//__attribute__ ((section(".firsttext"),noinline))
int main(void)
{
	char thisChar=0,prompt='o';
	uint32_t minAddress =  APPLICATION_BASE; // See also AO_BOOT_APPLCIATION_BASE in ao_arch.h
	//uint16_t *flashSize = (uint16_t *)0x1ff800cc;
	//uint32_t maxAddress = minAddress + ((*flashSize)*1024)-0x1000;
	uint32_t maxAddress = (uint32_t)APPLICATION_BOUND;
	ResetExternalWatchdog();
	ao_arch_block_interrupts();
	ao_clock_init();
	InitRCC();
	InitGPIO();
	InitUART(57600);
	GPIOSetOn_Alert(); //This actually silences it
	GPIOSetOff_Led1();


	while (true){ //Should be true
		//dbgUART_Write(prompt);
		thisChar = UART_Read(prompt);
		prompt = 'o';
		switch(thisChar){
		case 'v':{
			printString("GOLF Serial Loader\r\n");
			printString("manufacturer  amsat.org\r\n");
			printString("flash-range   ");
			printHex32(minAddress); printString(" "); printHex32(maxAddress); printString("\r\n");
			printString("Product       GolfSerialLoader\r\n");
			printString("Version       1.0\r\n");

			break;
		}
		case 'a':{
			SaveAcrossReset[0]=0; //Make sure we don't come back here
			SaveAcrossReset[1]=0;
			SaveAcrossReset[2]=0;
			ao_boot_chain((uint32_t *)APPLICATION_BASE);
			break;
		}
		case 'R':{
			volatile uint32_t *addr;
			uint32_t value,checksum=0;
			int i;

			addr = (volatile uint32_t *)readHex32();
			//#define DEBUG
			if((uint32_t)addr>=minAddress && (uint32_t)addr<maxAddress){
				for(i=0;i<=STM_FLASH_PAGESIZE_WORDS;i++){
					if(i == STM_FLASH_PAGESIZE_WORDS){
						// If we have written out the last word of the page, get the checksum
						value = checksum;
					} else {
						//Otherwise, get the next word and add in the checksum
						value=addr[i];
						checksum += value;
					}
					{
						int j;
						uint8_t *bytes = (uint8_t *)&value;
						for(j=0;j<4;j++){
							UART_Write(bytes[j]);
						}
					}
				}
			}
			break;
		}
		case 'W':{
			volatile uint32_t *addr;
			uint32_t word=0;
			uint32_t data[64];
			uint32_t readChecksum=0,calcChecksum=0;
			int i,j;
			addr = (volatile uint32_t *)readHex32();
#ifdef DEBUG
			//dbgprintHex32((uint32_t)addr);
#endif
			if(((uint32_t)addr>=minAddress) && ((uint32_t)addr<maxAddress)){
				//dbgprintString("Addr Ok ");dbgprintHex32(minAddress);dbgprintHex32(maxAddress);
			} else {
				GPIOSetOn_Led1(); // Signal an error
#ifdef DEBUG
				dbgprintString("Addr bad ");dbgprintHex32(minAddress);dbgprintHex32(maxAddress);
#endif
				break;
			}
			for(i=0;i<=(STM_FLASH_PAGESIZE_WORDS);i++){
				word = 0;
				for(j=0;j<4;j++){ // Bytes in a word
					uint32_t byte;
					byte = (uint32_t)UART_Read('.');
#ifdef DEBUG
					//dbgUART_Write('.');
#endif
					byte = byte << (8*j);
					word |= byte;
				}
				if(i == STM_FLASH_PAGESIZE_WORDS){
					//This is the last word being read, which is the checksum
					readChecksum = word;
				} else {
					data[i]=word;
					calcChecksum+=word;
				}
#ifdef DEBUG
				if(i<STM_FLASH_PAGESIZE_WORDS){
					//dbgprintHex32((uint32_t)(&addr[i]));dbgprintString(":");dbgprintHex32(data[i]);dbgprintString("\n\r");
				}
#endif
			}
			if(calcChecksum == readChecksum){
				ao_flash_page(&(addr[0]),&data[0]);
			}else {
#ifdef DEBUG
				dbgprintString("Checksum fail--read:");dbgprintHex32((uint32_t)(readChecksum));dbgprintString("Calc:");
				dbgprintHex32(calcChecksum);dbgprintString("\n\r");
#endif
			}
			break;
		}
#ifdef DEBUG
		case 's':{
			dbgprintString("\n\r");
			break;
		}
#endif
		case 'X':{
			volatile uint32_t * addr;
			addr = (volatile uint32_t *)readHex32();
			if((((uint32_t)addr & 0xff)==0) && ((uint32_t)addr>=minAddress) && ((uint32_t)addr<maxAddress)){
#ifdef DEBUG
				dbgprintString("Ok\n\r");
#endif
				ao_flash_erase_page((uint32_t *)addr);
			}
			break;
		}
		}
	}

}


bool UmbilicalAttached(void){
	/*
	 * Enable the GPIO for attached and check it for being set
	 */
	bool notCode3;
	stm_rcc.ahbenr |= (1 << GPIOAttachedPortEnable);
	stm_moder_set((struct stm_gpio *)GPIOAttachedPort,GPIOAttachedPinNum,STM_MODER_INPUT);
	stm_pupdr_set((struct stm_gpio *)GPIOAttachedPort,GPIOAttachedPinNum,STM_PUPDR_PULL_DOWN);

	stm_rcc.ahbenr |= (1 << GPIOCmdDataPortEnable);
	stm_moder_set((struct stm_gpio *)GPIOCmdDataPort,GPIOCmdDataPinNum,STM_MODER_INPUT);
	stm_pupdr_set((struct stm_gpio *)GPIOCmdDataPort,GPIOCmdDataPinNum,STM_PUPDR_NONE);

	stm_moder_set((struct stm_gpio *)GPIOCmdDataPort,(GPIOCmdDataPinNum+1),STM_MODER_INPUT);
	stm_pupdr_set((struct stm_gpio *)GPIOCmdDataPort,(GPIOCmdDataPinNum+1),STM_PUPDR_NONE);
	notCode3 = (GPIORead_CmdBit0()==1) || (GPIORead_CmdBit1() == 0); // Not code 2 on the command lines
	return (GPIORead_UmbilicalAttached() != 0) && notCode3;
}

/*
 * You might call the following stuff the USART Driver.  It all runs on programmed I/O, no interrupts.
 * It assumes nothing is set up initially
 */
void InitRCC(void)
{

	/*
	 * This is to initialize all the clocks.
	 */
	stm_rcc.ahbenr |= (1 << STM_RCC_AHBENR_GPIOAEN);
	stm_rcc.ahbenr |= (1 << STM_RCC_AHBENR_GPIOBEN);
	stm_rcc.ahbenr |= (1 << STM_RCC_AHBENR_GPIOCEN);
	stm_rcc.ahbenr |= (1 << STM_RCC_AHBENR_GPIODEN);
	stm_rcc.ahbenr |= (1 << STM_RCC_AHBENR_GPIOEEN);
	stm_rcc.ahbenr |= (1 << STM_RCC_AHBENR_GPIOHEN);
#if UART_NUMBER == 2 || defined(DEBUG)
	stm_rcc.apb1enr |= (1 << STM_RCC_APB1ENR_USART2EN);
#endif
#if UART_NUMBER == 1
	stm_rcc.apb2enr |= (1 << STM_RCC_APB2ENR_USART1EN);
#elif UART_NUMBER == 3
	stm_rcc.apb1enr |= (1 << STM_RCC_APB1ENR_USART3EN);
#else
#error Need to set up for another UART
#endif
}


void InitGPIO(void){
	/*
	 * Here we are setting up the GPIOs for whatever we need
	 */

	stm_moder_set((struct stm_gpio *)GPIOWDResetPort,GPIOWDResetPinNum,STM_MODER_OUTPUT);
	stm_otyper_set((struct stm_gpio *)GPIOWDResetPort,GPIOWDResetPinNum,STM_OTYPER_PUSH_PULL);
	stm_ospeedr_set((struct stm_gpio *)GPIOWDResetPort,GPIOWDResetPinNum,STM_OSPEEDR_400kHz);
	stm_pupdr_set((struct stm_gpio *)GPIOWDResetPort,GPIOWDResetPinNum,STM_PUPDR_NONE);

// Enable LED 1 (red) so we can signal a checksum error
	stm_moder_set((struct stm_gpio *)GPIOLed1Port,GPIOLed1PinNum,STM_MODER_OUTPUT);
	stm_otyper_set((struct stm_gpio *)GPIOLed1Port,GPIOLed1PinNum,STM_OTYPER_PUSH_PULL);
	stm_ospeedr_set((struct stm_gpio *)GPIOLed1Port,GPIOLed1PinNum,STM_OSPEEDR_400kHz);
	stm_pupdr_set((struct stm_gpio *)GPIOLed1Port,GPIOLed1PinNum,STM_PUPDR_NONE);
#if 0
	stm_moder_set((struct stm_gpio *)GPIOLed2Port,GPIOLed2PinNum,STM_MODER_OUTPUT);
	stm_otyper_set((struct stm_gpio *)GPIOLed2Port,GPIOLed2PinNum,STM_OTYPER_PUSH_PULL);
	stm_ospeedr_set((struct stm_gpio *)GPIOLed2Port,GPIOLed2PinNum,STM_OSPEEDR_400kHz);
	stm_pupdr_set((struct stm_gpio *)GPIOLed2Port,GPIOLed2PinNum,STM_PUPDR_NONE);

	stm_moder_set((struct stm_gpio *)GPIOLed3Port,GPIOLed3PinNum,STM_MODER_OUTPUT);
	stm_otyper_set((struct stm_gpio *)GPIOLed3Port,GPIOLed3PinNum,STM_OTYPER_PUSH_PULL);
	stm_ospeedr_set((struct stm_gpio *)GPIOLed3Port,GPIOLed3PinNum,STM_OSPEEDR_400kHz);
	stm_pupdr_set((struct stm_gpio *)GPIOLed3Port,GPIOLed3PinNum,STM_PUPDR_NONE);

	stm_moder_set((struct stm_gpio *)GPIOLed4Port,GPIOLed4PinNum,STM_MODER_OUTPUT);
	stm_otyper_set((struct stm_gpio *)GPIOLed4Port,GPIOLed4PinNum,STM_OTYPER_PUSH_PULL);
	stm_ospeedr_set((struct stm_gpio *)GPIOLed4Port,GPIOLed4PinNum,STM_OSPEEDR_400kHz);
	stm_pupdr_set((struct stm_gpio *)GPIOLed4Port,GPIOLed4PinNum,STM_PUPDR_NONE);
#endif
	/*
	 * Setup External IHU RF Control GPIO
	 */
	stm_moder_set((struct stm_gpio *)GPIOIhuRfPort,GPIOIhuRfPinNum,STM_MODER_OUTPUT);
	stm_otyper_set((struct stm_gpio *)GPIOIhuRfPort,GPIOIhuRfPinNum,STM_OTYPER_PUSH_PULL);
	stm_ospeedr_set((struct stm_gpio *)GPIOIhuRfPort,GPIOIhuRfPinNum,STM_OSPEEDR_400kHz);
	stm_pupdr_set((struct stm_gpio *)GPIOIhuRfPort,GPIOIhuRfPinNum,STM_PUPDR_NONE);

	/*
	 * Setup Alert GPIO
	 */
	stm_moder_set((struct stm_gpio *)GPIOAlertPort,GPIOAlertPinNum,STM_MODER_OUTPUT);
	stm_otyper_set((struct stm_gpio *)GPIOAlertPort,GPIOAlertPinNum,STM_OTYPER_PUSH_PULL);
	stm_ospeedr_set((struct stm_gpio *)GPIOAlertPort,GPIOAlertPinNum,STM_OSPEEDR_400kHz);
	stm_pupdr_set((struct stm_gpio *)GPIOAlertPort,GPIOAlertPinNum,STM_PUPDR_NONE);

	/*
	 * This lets us turn off the contingency transmitter while we are here
	 */
	stm_moder_set((struct stm_gpio *)GPIOIhuRfPort,GPIOIhuRfPinNum,STM_MODER_OUTPUT);
	stm_otyper_set((struct stm_gpio *)GPIOIhuRfPort,GPIOIhuRfPinNum,STM_OTYPER_PUSH_PULL);
	stm_ospeedr_set((struct stm_gpio *)GPIOIhuRfPort,GPIOIhuRfPinNum,STM_OSPEEDR_400kHz);
	stm_pupdr_set((struct stm_gpio *)GPIOIhuRfPort,GPIOIhuRfPinNum,STM_PUPDR_NONE);



#define GPIOIhuRfPort				GPIOE
#define GPIOIhuRfPin				GPIO_Pin_12
#define GPIOIhuRfPinNum				12
#define GPIOCmdDataPort				GPIOE
#define GPIOCmdDataPin				GPIO_Pin_0
#define GPIOCmdDataPinNum			0
#define GPIOCmdDataNumBits			4

}

void InitUART(uint32_t BaudRate)
{

	//
	stm_moder_set(&stm_uart_gpio,UART_RX_PIN,STM_MODER_ALTERNATE);
	stm_ospeedr_set(&stm_uart_gpio, UART_RX_PIN, STM_OSPEEDR_40MHz);
	stm_otyper_set(&stm_uart_gpio,UART_RX_PIN, STM_OTYPER_PUSH_PULL);
	stm_pupdr_set( &stm_uart_gpio,UART_RX_PIN,STM_PUPDR_PULL_UP);
	stm_afr_set(&stm_uart_gpio,UART_RX_PIN,STM_AFR_AF7);

	stm_moder_set(&stm_uart_gpio,UART_TX_PIN,STM_MODER_ALTERNATE);
	stm_ospeedr_set(&stm_uart_gpio, UART_TX_PIN, STM_OSPEEDR_40MHz);
	stm_otyper_set(&stm_uart_gpio,UART_TX_PIN, STM_OTYPER_PUSH_PULL);
	stm_pupdr_set( &stm_uart_gpio,UART_TX_PIN,STM_PUPDR_PULL_UP);
	stm_afr_set(&stm_uart_gpio,UART_TX_PIN,STM_AFR_AF7);

	stm_usart.cr2 =
			STM_USART_CR2_STOP_1 << STM_USART_CR2_STOP;  // One stop bit
	stm_usart.cr1 =
			1 << STM_USART_CR1_TE |
			1 << STM_USART_CR1_RE |
			1 << STM_USART_CR1_UE;
	// Enable Rx and Tx.  NOTE: Wordlength=8, Parity = none are all 0;
	stm_usart.cr3 = 0; // No hardware flow control
	stm_usart.brr = CLOCK_SPEED/BaudRate;

}
#ifdef DEBUG
void InitDbgUART(uint32_t BaudRate)
{

	//
	stm_moder_set(&stm_dbguart_gpio,dbgUART_RX_PIN,STM_MODER_ALTERNATE);
	stm_ospeedr_set(&stm_dbguart_gpio, dbgUART_RX_PIN, STM_OSPEEDR_40MHz);
	stm_otyper_set(&stm_dbguart_gpio,dbgUART_RX_PIN, STM_OTYPER_PUSH_PULL);
	stm_pupdr_set( &stm_dbguart_gpio,dbgUART_RX_PIN,STM_PUPDR_PULL_UP);
	stm_afr_set(&stm_dbguart_gpio,dbgUART_RX_PIN,STM_AFR_AF7);

	stm_moder_set(&stm_dbguart_gpio,dbgUART_TX_PIN,STM_MODER_ALTERNATE);
	stm_ospeedr_set(&stm_dbguart_gpio, dbgUART_TX_PIN, STM_OSPEEDR_40MHz);
	stm_otyper_set(&stm_dbguart_gpio,dbgUART_TX_PIN, STM_OTYPER_PUSH_PULL);
	stm_pupdr_set( &stm_dbguart_gpio,dbgUART_TX_PIN,STM_PUPDR_PULL_UP);
	stm_afr_set(&stm_dbguart_gpio,dbgUART_TX_PIN,STM_AFR_AF7);

	stm_dbgusart.cr2 =
			STM_USART_CR2_STOP_1 << STM_USART_CR2_STOP;  // One stop bit
	stm_dbgusart.cr1 =
			1 << STM_USART_CR1_TE |
			1 << STM_USART_CR1_RE |
			1 << STM_USART_CR1_UE;
	// Enable Rx and Tx.  NOTE: Wordlength=8, Parity = none are all 0;
	stm_dbgusart.cr3 = 0; // No hardware flow control
	stm_dbgusart.brr = CLOCK_SPEED/BaudRate;

}
void dbgUART_Write(char character){
	while((stm_dbgusart.sr & (1 << STM_USART_SR_TXE)) == 0){
		ResetExternalWatchdog();
	}
	stm_dbgusart.dr = character;
}
void dbgprintString(char *Buffer)
{
	char thisChar=*Buffer;
	int Len = 0xff;
	for (; (Len > 0) && (thisChar != 0); Len--) {
		while((stm_dbgusart.sr & (1 << STM_USART_SR_TXE)) == 0){
			ResetExternalWatchdog();
		}
		stm_dbgusart.dr = thisChar;
		Buffer++;
		thisChar = *Buffer;
	}
}
void dbgprintHex32(uint32_t number){
	uint8_t nybble;
	char character;
	int i;
	character=0;
	for(i=0;i<8;i++){
		nybble = (number>>28) & 0x0f;
		number = number << 4;
		if(nybble < 10)character = '0'+nybble;
		else character = 'A'+(nybble-10);
		//dbgUART_Write(character);
		while((stm_dbgusart.sr & (1 << STM_USART_SR_TXE)) == 0){
			ResetExternalWatchdog();
		}
		stm_dbgusart.dr = character;

	}
	dbgprintString(" ");
}


#endif
void UART_Write(char character){
	while((stm_usart.sr & (1 << STM_USART_SR_TXE)) == 0){
		ResetExternalWatchdog();
	}
	stm_usart.dr = character;
}

char UART_Read(char prompt) {
	int32_t promptit = 0;
	char x;
	while ((stm_usart.sr & (1 << STM_USART_SR_RXNE)) == 0){
		ResetExternalWatchdog();
		if((prompt != 0) && (--promptit<=0)){
			promptit = 0x7ff0000;
			UART_Write(prompt);
			//dbgUART_Write(prompt);
		}
	}
	x = stm_usart.dr;
	return (char)(x & 0xff);
}
uint32_t UART_ReadWord(char prompt){
	int32_t promptit = 0;
	uint8_t retValBytes[4];
	uint32_t *retVal = (uint32_t *)retValBytes;
	int i;
	UART_Write(prompt);
	for(i=0;i<4;i++){
		while ((stm_usart.sr & (1 << STM_USART_SR_RXNE)) == 0){
			ResetExternalWatchdog();
			if((prompt != 0) && (--promptit<=0)){
				promptit = 0x7ff0000;
				if(i==0)UART_Write(prompt);
				//dbgUART_Write(prompt);
			}
		}
		retValBytes[i] = stm_usart.dr;
	}
	return *retVal;

}
void printString(char *Buffer)
{
	char thisChar=*Buffer;
	int Len = 0xff;
	for (; (Len > 0) && (thisChar != 0); Len--) {
		UART_Write(thisChar);
		Buffer++;
		thisChar = *Buffer;
	}
}
uint32_t readHex32(void){
	uint32_t number = 0;
	char caseMask,num,ch;
	bool loop=true;
	caseMask = ~('a' ^ 'A'); // Clear the bit that is different between upper and lower case.
	while (loop){
		char letter;
		ch = num = UART_Read(0);
		letter = num & caseMask;
		if(num==' '){
			// Do nothing if it is a space
		} else if(num<='9' && num >= '0'){
			num -= '0'; //Get digit value for numerals
			number = (number << 4) | num;
		} else if ((letter<='F') && (letter>='A')) {
			letter = letter - 'A' +10;
			number = (number << 4) | letter;
		} else loop=false;
	}

	while((ch != '\n') && (ch != '\r')){
		ch = UART_Read(0);
	}
	return number;
}

void printHex32(uint32_t number){
	uint8_t nybble;
	char character;
	int i;
	character=0;
	for(i=0;i<8;i++){
		nybble = (number>>28) & 0x0f;
		number = number << 4;
		if(nybble < 10)character = '0'+nybble;
		else character = 'A'+(nybble-10);
		UART_Write(character);
	}
}
#if 0
//Already done in ao_boot_chain.c
void
inline ao_boot_chain(uint32_t *base)
{
	/* Thanks to Keith Packard, Altus Metrum for this routine. */
	uint32_t	sp;
	uint32_t	pc;

	sp = base[0];
	pc = base[1];
	if (0x08000100 <= pc && pc <= 0x08200000 && (pc & 1) == 1) {
		asm ("mov sp, %0" : : "r" (sp));
		asm ("mov lr, %0" : : "r" (pc));
		asm ("bx lr");
	}
}

//Already done in ao_interrupt
void inline CopyRamText(void){
	/* Here is what was done right at the start of the altos loader--
	 * 	Set interrupt vector table offset
	stm_nvic.vto = (uint32_t) &stm_interrupt_vector;
	memcpy(&_start__, &__text_end__, &_end__ - &_start__);
	memset(&__bss_start__, '\0', &__bss_end__ - &__bss_start__);
	main();
	 *
	 */

	/*
	 * We have some code in a segment called .ramtext which must be run out of RAM, not flash
	 * (that's what the STM32 manual says).  It is linked such that it SHOULD be in RAM, but
	 * in fact it is loaded into flash.  This code copies it from flash to RAM using variables
	 * that the linker script conveniently puts there for us.
	 */
	int i;
	extern uint32_t __ramTextStart;
	extern uint32_t __ramTextEnd;
	extern uint32_t __imageRamText;
	/*
	 * It's easier to make these look like an array, so we'll do that.
	 */
	uint8_t *dataSegment;		/* Start of RAM location of the text */
	uint8_t *dataSegEnd;		/* End RAM location of the text */
	uint8_t *dataImage;			/* ROM  location to copy init data from */
	dataSegment = (uint8_t*)(&__ramTextStart);
	dataSegEnd  = (uint8_t*)(&__ramTextEnd);
	dataImage   = (uint8_t*)(&__imageRamText);

	/*
	 * Now copy it.  We would use memcpy but we don't really want to have more code
	 * hanging around.
	 */
	for (i=0;i<(dataSegEnd-dataSegment);i++){
		dataSegment[i] = dataImage[i];
	}
}
#endif
