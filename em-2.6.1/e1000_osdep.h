/*
 * %W% %E%
 * Copyright (c) 2006-2007 Masayuki Murayama.  All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer. 
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution. 
 * 
 * 3. Neither the name of the author nor the names of its contributors may be
 *    used to endorse or promote products derived from this software without
 *    specific prior written permission. 
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 */

/**************************************************************************

Copyright (c) 2001-2006, Intel Corporation
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 1. Redistributions of source code must retain the above copyright notice,
    this list of conditions and the following disclaimer.

 2. Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.

 3. Neither the name of the Intel Corporation nor the names of its
    contributors may be used to endorse or promote products derived from
    this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.

***************************************************************************/

#ifndef _SOLARIS_OS_H_
#define _SOLARIS_OS_H_

#include <sys/types.h>
#include <sys/debug.h>
#include <sys/errno.h>
#include <sys/dditypes.h>
#include <sys/ddi.h>
#include <sys/sunddi.h>
#include <sys/ethernet.h>

#include <sys/pci.h>

#define usec_delay(x)		drv_usecwait(x)
#define msec_delay(x)		drv_usecwait(1000*(x))
#define msec_delay_irq(x)	drv_usecwait(1000*(x))

#define MSGOUT(S, A, B)     printf(S "\n", A, B)
#define DEBUGFUNC(F)        DEBUGOUT(F);

#ifdef EM_TRACE_FW
#define DEBUGOUT(S)	cmn_err(CE_CONT, S)
#define DEBUGOUT1(S,A)	cmn_err(CE_CONT, S, A)
#define DEBUGOUT2(S,A,B)	cmn_err(CE_CONT, S, A, B)
#define DEBUGOUT3(S,A,B,C)	cmn_err(CE_CONT, S, A, B, C)
#define DEBUGOUT7(S,A,B,C,D,E,F,G)	cmn_err(CE_CONT, S, A, B, C, D, E, F, G)
#else
#define DEBUGOUT(S)
#define DEBUGOUT1(S,A)
#define DEBUGOUT2(S,A,B)
#define DEBUGOUT3(S,A,B,C)
#define DEBUGOUT7(S,A,B,C,D,E,F,G)
#endif

#define STATIC			static
#define FALSE			B_FALSE
#define false			FALSE
#define TRUE			B_TRUE
#define true			TRUE
#define CMD_MEM_WRT_INVALIDATE	0x0010  /* BIT_4 */
#define PCI_COMMAND_REGISTER	PCIR_COMMAND

/* Mutex used in the shared code */
#define E1000_MUTEX			kmutex_t
#define E1000_MUTEX_INIT(mutex)		mutex_init((mutex), #mutex, \
					MUTEX_DRIVER, \
					NULL)
#define E1000_MUTEX_DESTROY(mutex)	mutex_destroy(mutex)
#define E1000_MUTEX_LOCK(mutex)		mutex_enter(mutex)
#define E1000_MUTEX_TRYLOCK(mutex)	mutex_tryenter(mutex)
#define E1000_MUTEX_UNLOCK(mutex)	mutex_exit(mutex)

typedef uint64_t	u64;
typedef uint32_t	u32;
typedef uint16_t	u16;
typedef uint8_t		u8;
typedef int64_t		s64;
typedef int32_t		s32;
typedef int16_t		s16;
typedef int8_t		s8;
typedef boolean_t	bool;

#define __le16		u16
#define __le32		u32
#define __le64		u64

struct e1000_hw;

#ifdef NO_82542_SUPPORT
#define E1000_REGISTER(hw, reg) reg
#else
#define E1000_REGISTER(hw, reg) (((hw)->mac.type >= e1000_82543) \
    ? reg : e1000_translate_register_82542(reg))
#endif

#define E1000_WRITE_FLUSH(a) E1000_READ_REG(a, E1000_STATUS)

/* Read from an absolute offset in the adapter's memory space */
#define E1000_READ_OFFSET(hw, offset) \
    e1000_reg_read32(hw, offset)

/* Write to an absolute offset in the adapter's memory space */
#define E1000_WRITE_OFFSET(hw, offset, value) \
    e1000_reg_write32(hw, offset, value)

/* Register READ/WRITE macros */

#define E1000_READ_REG(hw, reg) \
    e1000_reg_read32(hw, E1000_REGISTER(hw, reg))

#define E1000_WRITE_REG(hw, reg, value) \
    e1000_reg_write32(hw, E1000_REGISTER(hw, reg), value)

#define E1000_READ_REG_ARRAY(hw, reg, index) \
    e1000_reg_read32(hw, E1000_REGISTER(hw, reg) + ((index)<< 2))

#define E1000_WRITE_REG_ARRAY(hw, reg, index, value) \
    e1000_reg_write32(hw, \
        E1000_REGISTER(hw, reg) + ((index)<< 2), value)

#define E1000_READ_REG_ARRAY_DWORD E1000_READ_REG_ARRAY
#define E1000_WRITE_REG_ARRAY_DWORD E1000_WRITE_REG_ARRAY

#define E1000_READ_REG_ARRAY_BYTE(hw, reg, index) \
    e1000_reg_read8(hw, E1000_REGISTER(hw, reg) + index)

#define E1000_WRITE_REG_ARRAY_BYTE(hw, reg, index, value) \
    e1000_reg_write8(hw, E1000_REGISTER(hw, reg) + index, value)

#define E1000_WRITE_REG_ARRAY_WORD(hw, reg, index, value) \
    e1000_reg_write16(hw, E1000_REGISTER(hw, reg) + (index << 1), value)

#define E1000_WRITE_REG_IO(hw, reg, value) \
    e1000_io_write32(hw, reg, value)

#define E1000_READ_FLASH_REG(hw, reg) \
    e1000_flash_read32(hw, reg)

#define E1000_READ_FLASH_REG16(hw, reg) \
    e1000_flash_read16(hw, reg)

#define E1000_WRITE_FLASH_REG(hw, reg, value) \
    e1000_flash_write32(hw, reg, value)

#define E1000_WRITE_FLASH_REG16(hw, reg, value) \
    e1000_flash_write16(hw, reg, value)

uint8_t e1000_reg_read8(struct e1000_hw *, unsigned long);
uint16_t e1000_reg_read16(struct e1000_hw *, unsigned long);
uint32_t e1000_reg_read32(struct e1000_hw *, unsigned long);
void e1000_reg_write8(struct e1000_hw *, unsigned long, uint8_t);
void e1000_reg_write16(struct e1000_hw *, unsigned long, uint16_t);
void e1000_reg_write32(struct e1000_hw *, unsigned long, uint32_t);
uint32_t e1000_io_read32(struct e1000_hw *, unsigned long);
void e1000_io_write32(struct e1000_hw *, unsigned long, uint32_t);
uint16_t e1000_flash_read16(struct e1000_hw *, unsigned long);
uint32_t e1000_flash_read32(struct e1000_hw *, unsigned long);
void e1000_flash_write16(struct e1000_hw *, unsigned long, uint16_t);
void e1000_flash_write32(struct e1000_hw *, unsigned long, uint32_t);

#endif  /* _SOLARIS_OS_H_ */

