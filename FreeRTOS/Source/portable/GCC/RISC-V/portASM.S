/*
 * FreeRTOS Kernel V10.2.1
 * Copyright (C) 2019 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
 */

/*
 * The FreeRTOS kernel's RISC-V port is split between the the code that is
 * common across all currently supported RISC-V chips (implementations of the
 * RISC-V ISA), and code which tailors the port to a specific RISC-V chip:
 *
 * + The code that is common to all RISC-V chips is implemented in
 *   FreeRTOS\Source\portable\GCC\RISC-V-RV32\portASM.S.  There is only one
 *   portASM.S file because the same file is used no matter which RISC-V chip is
 *   in use.
 *
 * + The code that tailors the kernel's RISC-V port to a specific RISC-V
 *   chip is implemented in freertos_risc_v_chip_specific_extensions.h.  There
 *   is one freertos_risc_v_chip_specific_extensions.h that can be used with any
 *   RISC-V chip that both includes a standard CLINT and does not add to the
 *   base set of RISC-V registers.  There are additional
 *   freertos_risc_v_chip_specific_extensions.h files for RISC-V implementations
 *   that do not include a standard CLINT or do add to the base set of RISC-V
 *   registers.
 *
 * CARE MUST BE TAKEN TO INCLDUE THE CORRECT
 * freertos_risc_v_chip_specific_extensions.h HEADER FILE FOR THE CHIP
 * IN USE.  To include the correct freertos_risc_v_chip_specific_extensions.h
 * header file ensure the path to the correct header file is in the assembler's
 * include path.
 *
 * This freertos_risc_v_chip_specific_extensions.h is for use on RISC-V chips
 * that include a standard CLINT and do not add to the base set of RISC-V
 * registers.
 *
 */
#if __riscv_xlen == 64
#if __has_feature(capabilities)
	#define portWORD_SIZE 16
	#define store_x csc
	#define load_x clc
#else
	#define portWORD_SIZE 8
	#define store_x sd
	#define load_x ld
#endif
#elif __riscv_xlen == 32
#if __has_feature(capabilities)
	#define portWORD_SIZE 8
	#define store_x csc
	#define load_x clc
#else
	#define store_x sw
	#define load_x lw
	#define portWORD_SIZE 4
#endif
#else
	#error Assembler did not define __riscv_xlen
#endif

#include "freertos_risc_v_chip_specific_extensions.h"

/* Check the freertos_risc_v_chip_specific_extensions.h and/or command line
definitions. */
#ifndef portasmHAS_CLINT
	#error freertos_risc_v_chip_specific_extensions.h must define portasmHAS_CLINT to either 1 (CLINT present) or 0 (clint not present).
#endif

#ifndef portasmHANDLE_INTERRUPT
	#error portasmHANDLE_INTERRUPT must be defined to the function to be called to handle external/peripheral interrupts.  portasmHANDLE_INTERRUPT can be defined on the assmbler command line or in the appropriate freertos_risc_v_chip_specific_extensions.h header file.
#endif

/* Only the standard core registers are stored by default.  Any additional
registers must be saved by the portasmSAVE_ADDITIONAL_REGISTERS and
portasmRESTORE_ADDITIONAL_REGISTERS macros - which can be defined in a chip
specific version of freertos_risc_v_chip_specific_extensions.h.  See the notes
at the top of this file. */
#define portCONTEXT_SIZE (36 * portWORD_SIZE)

.global xPortStartFirstTask
.global freertos_risc_v_trap_handler
.global pxPortInitialiseStack
.extern pxCurrentTCB
.extern ulPortTrapHandler
.extern vTaskSwitchContext
.extern Timer_IRQHandler
.extern pullMachineTimerCompareRegister
.extern pullNextTime
.extern uxTimerIncrementsForOneTick /* size_t type so 32-bit on 32-bit core and 64-bits on 64-bit core. */
.extern xISRStackTop

/*-----------------------------------------------------------*/

.align 8
freertos_risc_v_trap_handler:
  cincoffset        csp, csp, -portCONTEXT_SIZE
  store_x           c1, 1 * portWORD_SIZE(csp)
  store_x           c3, 3 * portWORD_SIZE(csp)
  store_x           c4, 4 * portWORD_SIZE(csp)
  store_x           c5, 5 * portWORD_SIZE(csp)
  store_x           c6, 6 * portWORD_SIZE(csp)
  store_x           c7, 7 * portWORD_SIZE(csp)
  store_x           c8, 8 * portWORD_SIZE(csp)
  store_x           c9, 9 * portWORD_SIZE(csp)
  store_x           c10, 10 * portWORD_SIZE(csp)
  store_x           c11, 11 * portWORD_SIZE(csp)
  store_x           c12, 12 * portWORD_SIZE(csp)
  store_x           c13, 13 * portWORD_SIZE(csp)
  store_x           c14, 14 * portWORD_SIZE(csp)
  store_x           c15, 15 * portWORD_SIZE(csp)
  store_x           c16, 16 * portWORD_SIZE(csp)
  store_x           c17, 17 * portWORD_SIZE(csp)
  store_x           c18, 18 * portWORD_SIZE(csp)
  store_x           c19, 19 * portWORD_SIZE(csp)
  store_x           c20, 20 * portWORD_SIZE(csp)
  store_x           c21, 21 * portWORD_SIZE(csp)
  store_x           c22, 22 * portWORD_SIZE(csp)
  store_x           c23, 23 * portWORD_SIZE(csp)
  store_x           c24, 24 * portWORD_SIZE(csp)
  store_x           c25, 25 * portWORD_SIZE(csp)
  store_x           c26, 26 * portWORD_SIZE(csp)
  store_x           c27, 27 * portWORD_SIZE(csp)
  store_x           c28, 28 * portWORD_SIZE(csp)
  store_x           c29, 29 * portWORD_SIZE(csp)
  store_x           c30, 30 * portWORD_SIZE(csp)
  store_x           c31, 31 * portWORD_SIZE(csp)

  csrr              t0, mstatus # Required for MPIE bit.
	store_x           ct0, 32 * portWORD_SIZE(csp)

	portasmSAVE_ADDITIONAL_REGISTERS	/* Defined in freertos_risc_v_chip_specific_extensions.h to save any registers unique to the RISC-V implementation. */

  clgc              ct0, pxCurrentTCB # Load pxCurrentTCB.
  clc               ct0, 0(ct0) # Load pxCurrentTCB.
  store_x           csp, 0(ct0)				# Write sp to first TCB member.

  csrr              a0, mcause
  cspecialrw        ca1, cnull, mepcc # Read mepcc.

test_if_asynchronous:
	srli a2, a0, __riscv_xlen - 1		/* MSB of mcause is 1 if handing an asynchronous interrupt - shift to LSB to clear other bits. */
	beq a2, x0, handle_synchronous		/* Branch past interrupt handing if not asynchronous. */
  store_x           ca1, 0(csp)					/* Asynch so save unmodified exception return address. */

handle_asynchronous:

#if( portasmHAS_CLINT != 0 )

	test_if_mtimer:						/* If there is a CLINT then the mtimer is used to generate the tick interrupt. */

		addi t0, x0, 1

		slli t0, t0, __riscv_xlen - 1   /* LSB is already set, shift into MSB.  Shift 31 on 32-bit or 63 on 64-bit cores. */
		addi t1, t0, 7					/* 0x8000[]0007 == machine timer interrupt. */
		bne a0, t1, test_if_external_interrupt

    clgc            ct0, pullMachineTimerCompareRegister # Load cap to cap to compare register into t0.
    clc             ct0, 0(ct0) # Load the actual cap to compare register.
    clgc            ct1, pullNextTime # Same as above.
    clc             ct1, 0(ct1)

		#if( __riscv_xlen == 32 )

			/* Update the 64-bit mtimer compare match value in two 32-bit writes. */
      clw           t2, 0(ct1) # Load the low word of ullNextTime into t2.
      clw           t3, 4(ct1) # Load the high word of ullNextTime into t3.
      addi          t4, zero, -1
      csw           t4, 4(ct0) # Prevent spurious interrupts.
      csw           t2, 0(ct0) # Store low word of ullNextTime into compare register.
      csw           t3, 4(ct0) # Store high word of ullNextTime into compare register.
      clgc          ct0, uxTimerIncrementsForOneTick # Load the cap to ullTimerIncrementForOneTick into t0 (could this be optimized by storing in an array next to pullNextTime?).
      clw           t0, 0(ct0) # Load the actual value.
			add t4, t0, t2				/* Add the low word of ullNextTime to the timer increments for one tick (assumes timer increment for one tick fits in 32-bits). */
			sltu t5, t4, t2				/* See if the sum of low words overflowed (what about the zero case?). */
			add t6, t3, t5				/* Add overflow to high word of ullNextTime. */
      csw           t4, 0(ct1) # Store new low word of ullNextTime.
      csw           t6, 4(ct1) # Store new high word of ullNextTime.

		#endif /* __riscv_xlen == 32 */

		#if( __riscv_xlen == 64 )

			/* Update the 64-bit mtimer compare match value. */
			ld t2, 0(t1)			 	/* Load ullNextTime into t2. */
			sd t2, 0(t0)				/* Store ullNextTime into compare register. */
			ld t0, uxTimerIncrementsForOneTick  /* Load the value of ullTimerIncrementForOneTick into t0 (could this be optimized by storing in an array next to pullNextTime?). */
			add t4, t0, t2				/* Add ullNextTime to the timer increments for one tick. */
			sd t4, 0(t1)				/* Store ullNextTime. */

		#endif /* __riscv_xlen == 64 */

    clgc            ct0, xISRStackTop # Switch to ISR stack before function call.
    clc             csp, 0(ct0)

    clgc            ct0, xTaskIncrementTick
    cjalr           cra, ct0
		beqz a0, processed_source		/* Don't switch context if incrementing tick didn't unblock a task. */
    clgc            ct0, vTaskSwitchContext
    cjalr           cra, ct0
    beq             zero, zero, processed_source

	test_if_external_interrupt:			/* If there is a CLINT and the mtimer interrupt is not pending then check to see if an external interrupt is pending. */
		addi t1, t1, 4					/* 0x80000007 + 4 = 0x8000000b == Machine external interrupt. */
		bne a0, t1, as_yet_unhandled	/* Something as yet unhandled. */

#endif /* portasmHAS_CLINT */

  clgc              ct0, xISRStackTop # Switch to ISR stack before function call.
  clc               csp, 0(ct0)
  clgc              ct0, portasmHANDLE_INTERRUPT
  cjalr             cra, ct0 # Jump to the interrupt handler if there is no CLINT or if there is a CLINT and it has been determined that an external interrupt is pending.
  beq               zero, zero, processed_source

handle_synchronous:
  cincoffset        ca1, ca1, 4 # Synchronous so updated exception return address to the instruction after the instruction that generated the exeption.
  store_x           ca1, 0(csp) # Save updated exception return address.

test_if_environment_call:
	li t0, 11 							/* 11 == environment call. */
	bne a0, t0, is_exception			/* Not an M environment call, so some other exception. */
  clgc            ct0, xISRStackTop # Switch to ISR stack before function call.
  clc             csp, 0(ct0)

  clgc            ct0, vTaskSwitchContext
  cjalr           cra, ct0
  beq             zero, zero, processed_source

is_exception:
	ebreak
  beq             zero, zero, is_exception

as_yet_unhandled:
	ebreak
  beq             zero, zero, as_yet_unhandled

processed_source:
  clgc              ct1, pxCurrentTCB # Load cap to pxCurrentTCB.
  clc               ct1, 0(ct1) # Load pxCurrentTCB.
  load_x            csp, 0(ct1) # Read sp from first TCB member.

	/* Load mret with the address of the next instruction in the task to run next. */
  load_x            ct0, 0(csp)
  cspecialrw        cnull, ct0, mepcc # Write mepcc.

	portasmRESTORE_ADDITIONAL_REGISTERS	/* Defined in freertos_risc_v_chip_specific_extensions.h to restore any registers unique to the RISC-V implementation. */

	/* Load mstatus with the interrupt enable bits used by the task. */
  load_x            ct0, 32 * portWORD_SIZE(csp)
	csrw mstatus, t0						/* Required for MPIE bit. */

  load_x            c1, 1 * portWORD_SIZE(csp)
  load_x            c3, 3 * portWORD_SIZE(csp)
  load_x            c4, 4 * portWORD_SIZE(csp)
  load_x            c5, 5 * portWORD_SIZE(csp)
  load_x            c6, 6 * portWORD_SIZE(csp)
  load_x            c7, 7 * portWORD_SIZE(csp)
  load_x            c8, 8 * portWORD_SIZE(csp)
  load_x            c9, 9 * portWORD_SIZE(csp)
  load_x            c10, 10 * portWORD_SIZE(csp)
  load_x            c11, 11 * portWORD_SIZE(csp)
  load_x            c12, 12 * portWORD_SIZE(csp)
  load_x            c13, 13 * portWORD_SIZE(csp)
  load_x            c14, 14 * portWORD_SIZE(csp)
  load_x            c15, 15 * portWORD_SIZE(csp)
  load_x            c16, 16 * portWORD_SIZE(csp)
  load_x            c17, 17 * portWORD_SIZE(csp)
  load_x            c18, 18 * portWORD_SIZE(csp)
  load_x            c19, 19 * portWORD_SIZE(csp)
  load_x            c20, 20 * portWORD_SIZE(csp)
  load_x            c21, 21 * portWORD_SIZE(csp)
  load_x            c22, 22 * portWORD_SIZE(csp)
  load_x            c23, 23 * portWORD_SIZE(csp)
  load_x            c24, 24 * portWORD_SIZE(csp)
  load_x            c25, 25 * portWORD_SIZE(csp)
  load_x            c26, 26 * portWORD_SIZE(csp)
  load_x            c27, 27 * portWORD_SIZE(csp)
  load_x            c28, 28 * portWORD_SIZE(csp)
  load_x            c29, 29 * portWORD_SIZE(csp)
  load_x            c30, 30 * portWORD_SIZE(csp)
  load_x            c31, 31 * portWORD_SIZE(csp)
	cincoffset        csp, csp, portCONTEXT_SIZE

	mret
/*-----------------------------------------------------------*/

.align 8
.type xPortStartFirstTask, %function
xPortStartFirstTask:

#if( portasmHAS_CLINT != 0 )
	/* If there is a clint then interrupts can branch directly to the FreeRTOS
	trap handler.  Otherwise the interrupt controller will need to be configured
	outside of this file. */
.option nocapmode
  cspecialrw        ct0, cnull, pcc # Get PCC.
  la                t1, freertos_risc_v_trap_handler
  csetoffset        ct0, ct0, t1
  cspecialrw        cnull, ct0, mtcc # Write MTCC.
.option capmode
#endif /* portasmHAS_CLILNT */

  clgc              ct0, pxCurrentTCB # Load the cap to pxCurrentTCB.
  clc               ct0, 0(ct0) # Load pxCurrentTCB.
  load_x            csp, 0(ct0) # Read sp from first TCB member.

  load_x            c1, 0(csp) # Note for starting the scheduler the exception return address is used as the function return address.

	portasmRESTORE_ADDITIONAL_REGISTERS	/* Defined in freertos_risc_v_chip_specific_extensions.h to restore any registers unique to the RISC-V implementation. */

  load_x            ct0, 32 * portWORD_SIZE(csp) # mstatus
	csrrw  x0, mstatus, t0					/* Interrupts enabled from here! */

  load_x            c3, 3 * portWORD_SIZE(csp)
  load_x            c4, 4 * portWORD_SIZE(csp)
  load_x            c5, 5 * portWORD_SIZE(csp)
  load_x            c6, 6 * portWORD_SIZE(csp)
  load_x            c7, 7 * portWORD_SIZE(csp)
  load_x            c8, 8 * portWORD_SIZE(csp)
  load_x            c9, 9 * portWORD_SIZE(csp)
  load_x            c10, 10 * portWORD_SIZE(csp)
  load_x            c11, 11 * portWORD_SIZE(csp)
  load_x            c12, 12 * portWORD_SIZE(csp)
  load_x            c13, 13 * portWORD_SIZE(csp)
  load_x            c14, 14 * portWORD_SIZE(csp)
  load_x            c15, 15 * portWORD_SIZE(csp)
  load_x            c16, 16 * portWORD_SIZE(csp)
  load_x            c17, 17 * portWORD_SIZE(csp)
  load_x            c18, 18 * portWORD_SIZE(csp)
  load_x            c19, 19 * portWORD_SIZE(csp)
  load_x            c20, 20 * portWORD_SIZE(csp)
  load_x            c21, 21 * portWORD_SIZE(csp)
  load_x            c22, 22 * portWORD_SIZE(csp)
  load_x            c23, 23 * portWORD_SIZE(csp)
  load_x            c24, 24 * portWORD_SIZE(csp)
  load_x            c25, 25 * portWORD_SIZE(csp)
  load_x            c26, 26 * portWORD_SIZE(csp)
  load_x            c27, 27 * portWORD_SIZE(csp)
  load_x            c28, 28 * portWORD_SIZE(csp)
  load_x            c29, 29 * portWORD_SIZE(csp)
  load_x            c30, 30 * portWORD_SIZE(csp)
  load_x            c31, 31 * portWORD_SIZE(csp)
	cincoffset        csp, csp, portCONTEXT_SIZE
	cret
xPortStartFirstTaskEnd:
.size xPortStartFirstTask, xPortStartFirstTaskEnd - xPortStartFirstTask
/*-----------------------------------------------------------*/

/*
 * Unlike other ports pxPortInitialiseStack() is written in assembly code as it
 * needs access to the portasmADDITIONAL_CONTEXT_SIZE constant.  The prototype
 * for the function is as per the other ports:
 * StackType_t *pxPortInitialiseStack( StackType_t *pxTopOfStack, TaskFunction_t pxCode, void *pvParameters );
 *
 * As per the standard RISC-V ABI pxTopcOfStack is passed in in a0, pxCode in
 * a1, and pvParameters in a2.  The new top of stack is passed out in a0.
 *
 * RISC-V maps registers to ABI names as follows (X1 to X31 integer registers
 * for the 'I' profile, X1 to X15 for the 'E' profile, currently I assumed).
 *
 * Register		ABI Name	Description						Saver
 * x0			zero		Hard-wired zero					-
 * x1			ra			Return address					Caller
 * x2			sp			Stack pointer					Callee
 * x3			gp			Global pointer					-
 * x4			tp			Thread pointer					-
 * x5-7			t0-2		Temporaries						Caller
 * x8			s0/fp		Saved register/Frame pointer	Callee
 * x9			s1			Saved register					Callee
 * x10-11		a0-1		Function Arguments/return values Caller
 * x12-17		a2-7		Function arguments				Caller
 * x18-27		s2-11		Saved registers					Callee
 * x28-31		t3-6		Temporaries						Caller
 *
 * The RISC-V context is saved t FreeRTOS tasks in the following stack frame,
 * where the global and thread pointers are currently assumed to be constant so
 * are not saved:
 *
 * mstatus
 * x31
 * x30
 * x29
 * x28
 * x27
 * x26
 * x25
 * x24
 * x23
 * x22
 * x21
 * x20
 * x19
 * x18
 * x17
 * x16
 * x15
 * x14
 * x13
 * x12
 * x11
 * pvParameters
 * x9
 * x8
 * x7
 * x6
 * x5
 * portTASK_RETURN_ADDRESS
 * [chip specific registers go here]
 * pxCode
 */
.align 8
.type pxPortInitialiseStack, %function
pxPortInitialiseStack:

	csrr t0, mstatus					/* Obtain current mstatus value. */
	addi t1, x0, 0x188					/* Generate the value 0x1880, which are the MPIE and MPP bits to set in mstatus. */
	slli t1, t1, 4
	or t0, t0, t1						/* Set MPIE and MPP bits in mstatus value. */

  cincoffset        ca0, ca0, -portWORD_SIZE
  store_x           ct0, 0(ca0) # mstatus onto the stack.
	cincoffset        ca0, ca0, -(22 * portWORD_SIZE) # space for registers x11-x31
  store_x           ca2, 0(ca0) # Task parameters (pvParameters parameter) go into register X10/a0 on the stack.
  cincoffset        ca0, ca0, -(9 * portWORD_SIZE) # space for registers x2-x9.
  store_x           cnull, 0(ca0) # return address onto the stack, could be portTASK_RETURN_ADDRESS
	addi t0, x0, portasmADDITIONAL_CONTEXT_SIZE /* The number of chip specific additional registers. */
chip_specific_stack_frame:				/* First add any chip specific registers to the stack frame being created. */
	beq t0, x0, 1f						/* No more chip specific registers to save. */
	addi a0, a0, -portWORD_SIZE			/* Make space for chip specific register. */
	store_x cnull, 0(ca0)					/* Give the chip specific register an initial value of zero. */
	addi t0, t0, -1						/* Decrement the count of chip specific registers remaining. */
	j chip_specific_stack_frame			/* Until no more chip specific registers. */
1:
  cincoffset        ca0, ca0, -portWORD_SIZE
  store_x           ca1, 0(ca0) # mret value (pxCode parameter) onto the stack
	cret
pxPortInitialiseStackEnd:
.size pxPortInitialiseStack, pxPortInitialiseStackEnd - pxPortInitialiseStack
/*-----------------------------------------------------------*/
