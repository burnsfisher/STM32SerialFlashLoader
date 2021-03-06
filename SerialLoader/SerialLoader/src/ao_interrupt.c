/*
 * Copyright © 2012 Keith Packard <keithp@keithp.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
 */

#include "stm32l.h"
#include <string.h>
#include <ao_boot.h>
#include "../inc/ao.h"
#include "SerialLoader.h"

extern char __stack__;
extern char __bss_start__, __bss_end__;
//extern char __ramTextDst,__ramTextSrc,__ramCopyEnd;

void CopyRamText(void){
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
	extern uint32_t __ramTextSrc;
	extern uint32_t __ramCopyEnd;
	extern uint32_t __ramTextDst;
	/*
	 * It's easier to make these look like an array, so we'll do that.
	 */
	uint8_t *dataSegment;		/* Start of RAM location of the text */
	uint8_t *dataSegEnd;		/* End RAM location of the text */
	uint8_t *dataImage;			/* ROM  location to copy init data from */
	dataSegment = (uint8_t*)(&__ramTextDst);
	dataSegEnd  = (uint8_t*)(&__ramCopyEnd);
	dataImage   = (uint8_t*)(&__ramTextSrc);

	/*
	 * Now copy it.  We would use memcpy but we don't really want to have more code
	 * hanging around.
	 */
	for (i=0;i<(dataSegEnd-dataSegment);i++){
		dataSegment[i] = dataImage[i];
	}
}


void ao_panic(uint8_t reason)
{
	(void) reason;
}


/* Interrupt functions */

void stm_halt_isr(void)
{
	ao_panic(AO_PANIC_CRASH);
}

void stm_ignore_isr(void)
{
}

const void *stm_interrupt_vector[];

void start(void)
{
	if(ao_boot_check_chain()){
		if(!UmbilicalAttached()){
			ao_boot_chain(AO_BOOT_APPLICATION_BASE);
		}
	}
	/* We get here if memory cells are set to one set of magic values (boot_check_chain)
	 * or if the attached line is high.  This is where we start the loader
	 */

	/* Set interrupt vector table offset */
	stm_nvic.vto = (uint32_t) &stm_interrupt_vector;
	//memcpy(&__ramTextDst, &__ramTextSrc, &__ramCopyEnd - &__ramTextSrc);
	CopyRamText();
	memset(&__bss_start__, '\0', &__bss_end__ - &__bss_start__);
	main();
}

#define STRINGIFY(x) #x

#define isr(name) \
	void __attribute__ ((weak)) stm_ ## name ## _isr(void); \
	_Pragma(STRINGIFY(weak stm_ ## name ## _isr = stm_ignore_isr))

#define isr_halt(name) \
	void __attribute__ ((weak)) stm_ ## name ## _isr(void); \
	_Pragma(STRINGIFY(weak stm_ ## name ## _isr = stm_halt_isr))

isr(nmi)
isr_halt(hardfault)
isr_halt(memmanage)
isr_halt(busfault)
isr_halt(usagefault)
isr(svc)
isr(debugmon)
isr(pendsv)
isr(systick)
isr(wwdg)
isr(pvd)
isr(tamper_stamp)
isr(rtc_wkup)
isr(flash)
isr(rcc)
isr(exti0)
isr(exti1)
isr(exti2)
isr(exti3)
isr(exti4)
isr(dma1_channel1)
isr(dma1_channel2)
isr(dma1_channel3)
isr(dma1_channel4)
isr(dma1_channel5)
isr(dma1_channel6)
isr(dma1_channel7)
isr(adc1)
isr(usb_hp)
isr(usb_lp)
isr(dac)
isr(comp)
isr(exti9_5)
isr(lcd)
isr(tim9)
isr(tim10)
isr(tim11)
isr(tim2)
isr(tim3)
isr(tim4)
isr(i2c1_ev)
isr(i2c1_er)
isr(i2c2_ev)
isr(i2c2_er)
isr(spi1)
isr(spi2)
isr(usart1)
isr(usart2)
isr(usart3)
isr(exti15_10)
isr(rtc_alarm)
isr(usb_fs_wkup)
isr(tim6)
isr(tim7)

#define i(addr,name)	[(addr)/4] = stm_ ## name ## _isr

__attribute__ ((section(".interrupt")))
const void *stm_interrupt_vector[] = {
	[0] = &__stack__,
	[1] = start,
	i(0x08, nmi),
	i(0x0c, hardfault),
	i(0x10, memmanage),
	i(0x14, busfault),
	i(0x18, usagefault),
	i(0x2c, svc),
	i(0x30, debugmon),
	i(0x38, pendsv),
	i(0x3c, systick),
	i(0x40, wwdg),
	i(0x44, pvd),
	i(0x48, tamper_stamp),
	i(0x4c, rtc_wkup),
	i(0x50, flash),
	i(0x54, rcc),
	i(0x58, exti0),
	i(0x5c, exti1),
	i(0x60, exti2),
	i(0x64, exti3),
	i(0x68, exti4),
	i(0x6c, dma1_channel1),
	i(0x70, dma1_channel2),
	i(0x74, dma1_channel3),
	i(0x78, dma1_channel4),
	i(0x7c, dma1_channel5),
	i(0x80, dma1_channel6),
	i(0x84, dma1_channel7),
	i(0x88, adc1),
	i(0x8c, usb_hp),
	i(0x90, usb_lp),
	i(0x94, dac),
	i(0x98, comp),
	i(0x9c, exti9_5),
	i(0xa0, lcd),
	i(0xa4, tim9),
	i(0xa8, tim10),
	i(0xac, tim11),
	i(0xb0, tim2),
	i(0xb4, tim3),
	i(0xb8, tim4),
	i(0xbc, i2c1_ev),
	i(0xc0, i2c1_er),
	i(0xc4, i2c2_ev),
	i(0xc8, i2c2_er),
	i(0xcc, spi1),
	i(0xd0, spi2),
	i(0xd4, usart1),
	i(0xd8, usart2),
	i(0xdc, usart3),
	i(0xe0, exti15_10),
	i(0xe4, rtc_alarm),
	i(0xe8, usb_fs_wkup),
	i(0xec, tim6),
	i(0xf0, tim7),
};
