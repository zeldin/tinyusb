/*
 * printf_retarget.c
 *
 *  Created on: Jan 11, 2013
 *      Author: hathach
 */

/*
 * Software License Agreement (BSD License)
 * Copyright (c) 2012, hathach (tinyusb.net)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 *
 * This file is part of the tiny usb stack.
 */

#include "board.h"

//-------------------------------------------------------------------- +
//                    LPCXpresso printf redirection                    +
//-------------------------------------------------------------------- +
#if BSP_UART_ENABLE
// Called by bottom level of printf routine within RedLib C library to write
// a character. With the default semihosting stub, this would write the character
// to the debugger console window . But this version writes
// the character to the UART.
int __sys_write (int iFileHandle, char *pcBuffer, int iLength)
{
  (void) iFileHandle;
	return board_uart_send(pcBuffer, iLength);
}

// Called by bottom level of scanf routine within RedLib C library to read
// a character. With the default semihosting stub, this would read the character
// from the debugger console window (which acts as stdin). But this version reads
// the character from the UART.
int __sys_readc (void)
{
	uint8_t c;
	board_uart_recv(&c, 1);
	return (int)c;
}

#endif