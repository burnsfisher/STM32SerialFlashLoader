# STM32 Flash Loader for Serial Line
A Flash Loader for STM32L that matches pyMicroloader and uses a serial line 

V1.1, March 13, 2021


The intent of this loader it for use with a cross-development
environment for embedded microprocessors.  This loader is
intended to run on the embedded microprocessor, specifically on an
STM32L151.  It is patterned after a similar loader designed by Keith
Packard of Altus Metrum for use via a USB connection.  This one is
the equivalent for a serial connection.

This loader lives in the bottom 0x1000 (or 0x2000--specified by
APPLICATION_BASE) bytes of flash memory, so the program it is loading
must be built to load and start above it.

This has a lot of debug code in it (ifdef'ed out) and also has some
metadata so that it should be importable as a project in Atollic TrueStudio.
