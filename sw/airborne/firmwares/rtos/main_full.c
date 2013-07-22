/*
    FreeRTOS V7.4.0 - Copyright (C) 2013 Real Time Engineers Ltd.

    FEATURES AND PORTS ARE ADDED TO FREERTOS ALL THE TIME.  PLEASE VISIT
    http://www.FreeRTOS.org TO ENSURE YOU ARE USING THE LATEST VERSION.

    ***************************************************************************
     *                                                                       *
     *    FreeRTOS tutorial books are available in pdf and paperback.        *
     *    Complete, revised, and edited pdf reference manuals are also       *
     *    available.                                                         *
     *                                                                       *
     *    Purchasing FreeRTOS documentation will not only help you, by       *
     *    ensuring you get running as quickly as possible and with an        *
     *    in-depth knowledge of how to use FreeRTOS, it will also help       *
     *    the FreeRTOS project to continue with its mission of providing     *
     *    professional grade, cross platform, de facto standard solutions    *
     *    for microcontrollers - completely free of charge!                  *
     *                                                                       *
     *    >>> See http://www.FreeRTOS.org/Documentation for details. <<<     *
     *                                                                       *
     *    Thank you for using FreeRTOS, and thank you for your support!      *
     *                                                                       *
    ***************************************************************************


    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation AND MODIFIED BY the FreeRTOS exception.

    >>>>>>NOTE<<<<<< The modification to the GPL is included to allow you to
    distribute a combined work that includes FreeRTOS without being obliged to
    provide the source code for proprietary components outside of the FreeRTOS
    kernel.

    FreeRTOS is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
    FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
    details. You should have received a copy of the GNU General Public License
    and the FreeRTOS license exception along with FreeRTOS; if not itcan be
    viewed here: http://www.freertos.org/a00114.html and also obtained by
    writing to Real Time Engineers Ltd., contact details for whom are available
    on the FreeRTOS WEB site.

    1 tab == 4 spaces!

    ***************************************************************************
     *                                                                       *
     *    Having a problem?  Start by reading the FAQ "My application does   *
     *    not run, what could be wrong?"                                     *
     *                                                                       *
     *    http://www.FreeRTOS.org/FAQHelp.html                               *
     *                                                                       *
    ***************************************************************************


    http://www.FreeRTOS.org - Documentation, books, training, latest versions, 
    license and Real Time Engineers Ltd. contact details.

    http://www.FreeRTOS.org/plus - A selection of FreeRTOS ecosystem products,
    including FreeRTOS+Trace - an indispensable productivity tool, and our new
    fully thread aware and reentrant UDP/IP stack.

    http://www.OpenRTOS.com - Real Time Engineers ltd license FreeRTOS to High 
    Integrity Systems, who sell the code with commercial support, 
    indemnification and middleware, under the OpenRTOS brand.
    
    http://www.SafeRTOS.com - High Integrity Systems also provide a safety 
    engineered and independently SIL3 certified version for use in safety and 
    mission critical applications that require provable dependability.
*/

/******************************************************************************
 * NOTE 1:  This project provides two demo applications.  A simple blinky style
 * project, and a more comprehensive test and demo application.  The
 * mainCREATE_SIMPLE_BLINKY_DEMO_ONLY setting in main.c is used to select
 * between the two.  See the notes on using mainCREATE_SIMPLE_BLINKY_DEMO_ONLY
 * in main.c.  This file implements the comprehensive test and demo version.
 *
 * NOTE 2:  This file only contains the source code that is specific to the
 * full demo.  Generic functions, such FreeRTOS hook functions, and functions
 * required to configure the hardware, are defined in main.c.
 ******************************************************************************
 *
 * main_full() creates all the demo application tasks and a software timer, then
 * starts the scheduler.  The web documentation provides more details of the 
 * standard demo application tasks, which provide no particular functionality, 
 * but do provide a good example of how to use the FreeRTOS API.
 *
 * In addition to the standard demo tasks, the following tasks and tests are
 * defined and/or created within this file:
 *
 * "Reg test" tasks - These fill both the core and floating point registers with
 * known values, then check that each register maintains its expected value for
 * the lifetime of the task.  Each task uses a different set of values.  The reg
 * test tasks execute with a very low priority, so get preempted very
 * frequently.  A register containing an unexpected value is indicative of an
 * error in the context switching mechanism.
 *
 * "Check" timer - The check software timer period is initially set to three
 * seconds.  The callback function associated with the check software timer
 * checks that all the standard demo tasks, and the register check tasks, are
 * not only still executing, but are executing without reporting any errors.  If
 * the check software timer discovers that a task has either stalled, or
 * reported an error, then it changes its own execution period from the initial
 * three seconds, to just 200ms.  The check software timer callback function
 * also toggles the single LED each time it is called.  This provides a visual
 * indication of the system status:  If the LED toggles every three seconds,
 * then no issues have been discovered.  If the LED toggles every 200ms, then
 * an issue has been discovered with at least one task.
 */

/* Standard includes. */
#include <stdio.h>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"

/*-----------------------------------------------------------*/

/*
 * The check timer callback function, as described at the top of this file.
 */
static void prvCheckTimerCallback( xTimerHandle xTimer );

/*
 * Register check tasks, and the tasks used to write over and check the contents
 * of the FPU registers, as described at the top of this file.  The nature of
 * these files necessitates that they are written in an assembly file.
 */
extern void vRegTest1Task( void *pvParameters ) __attribute__((naked));
extern void vRegTest2Task( void *pvParameters ) __attribute__((naked));
extern void vRegTestClearFlopRegistersToParameterValue(  unsigned long ulValue ) __attribute__((naked));
extern void ulRegTestCheckFlopRegistersContainParameterValue(  unsigned long ulValue ) __attribute__((naked));

/* This is a naked function. */
void vRegTest1Task( void *pvParameters )
{
	__asm volatile
	(
		"	/* Fill the core registers with known values. */		\n"
		"	mov r0, #100											\n"
		"	mov r1, #101											\n"
		"	mov r2, #102											\n"
		"	mov r3, #103											\n"
		"	mov	r4, #104											\n"
		"	mov	r5, #105											\n"
		"	mov	r6, #106											\n"
		"	mov r7, #107											\n"
		"	mov	r8, #108											\n"
		"	mov	r9, #109											\n"
		"	mov	r10, #110											\n"
		"	mov	r11, #111											\n"
		"	mov r12, #112											\n"
		"															\n"
		"	/* Fill the VFP registers with known values. */			\n"
		"	vmov d0, r0, r1											\n"
		"	vmov d1, r2, r3											\n"
		"	vmov d2, r4, r5											\n"
		"	vmov d3, r6, r7											\n"
		"	vmov d4, r8, r9											\n"
		"	vmov d5, r10, r11										\n"
		"	vmov d6, r0, r1											\n"
		"	vmov d7, r2, r3											\n"
		"	vmov d8, r4, r5											\n"
		"	vmov d9, r6, r7											\n"
		"	vmov d10, r8, r9										\n"
		"	vmov d11, r10, r11										\n"
		"	vmov d12, r0, r1										\n"
		"	vmov d13, r2, r3										\n"
		"	vmov d14, r4, r5										\n"
		"	vmov d15, r6, r7										\n"
		"															\n"
		"reg1_loop:													\n"
		"	/* Check all the VFP registers still contain the values set above.\n"
		"	First save registers that are clobbered by the test. */	\n"
		"	push { r0-r1 }											\n"
		"															\n"
		"	vmov r0, r1, d0											\n"
		"	cmp r0, #100											\n"
		"	bne reg1_error_loopf									\n"
		"	cmp r1, #101											\n"
		"	bne reg1_error_loopf									\n"
		"	vmov r0, r1, d1											\n"
		"	cmp r0, #102											\n"
		"	bne reg1_error_loopf									\n"
		"	cmp r1, #103											\n"
		"	bne reg1_error_loopf									\n"
		"	vmov r0, r1, d2											\n"
		"	cmp r0, #104											\n"
		"	bne reg1_error_loopf									\n"
		"	cmp r1, #105											\n"
		"	bne reg1_error_loopf									\n"
		"	vmov r0, r1, d3											\n"
		"	cmp r0, #106											\n"
		"	bne reg1_error_loopf									\n"
		"	cmp r1, #107											\n"
		"	bne reg1_error_loopf									\n"
		"	vmov r0, r1, d4											\n"
		"	cmp r0, #108											\n"
		"	bne reg1_error_loopf									\n"
		"	cmp r1, #109											\n"
		"	bne reg1_error_loopf									\n"
		"	vmov r0, r1, d5											\n"
		"	cmp r0, #110											\n"
		"	bne reg1_error_loopf									\n"
		"	cmp r1, #111											\n"
		"	bne reg1_error_loopf									\n"
		"	vmov r0, r1, d6											\n"
		"	cmp r0, #100											\n"
		"	bne reg1_error_loopf									\n"
		"	cmp r1, #101											\n"
		"	bne reg1_error_loopf									\n"
		"	vmov r0, r1, d7											\n"
		"	cmp r0, #102											\n"
		"	bne reg1_error_loopf									\n"
		"	cmp r1, #103											\n"
		"	bne reg1_error_loopf									\n"
		"	vmov r0, r1, d8											\n"
		"	cmp r0, #104											\n"
		"	bne reg1_error_loopf									\n"
		"	cmp r1, #105											\n"
		"	bne reg1_error_loopf									\n"
		"	vmov r0, r1, d9											\n"
		"	cmp r0, #106											\n"
		"	bne reg1_error_loopf									\n"
		"	cmp r1, #107											\n"
		"	bne reg1_error_loopf									\n"
		"	vmov r0, r1, d10										\n"
		"	cmp r0, #108											\n"
		"	bne reg1_error_loopf									\n"
		"	cmp r1, #109											\n"
		"	bne reg1_error_loopf									\n"
		"	vmov r0, r1, d11										\n"
		"	cmp r0, #110											\n"
		"	bne reg1_error_loopf									\n"
		"	cmp r1, #111											\n"
		"	bne reg1_error_loopf									\n"
		"	vmov r0, r1, d12										\n"
		"	cmp r0, #100											\n"
		"	bne reg1_error_loopf									\n"
		"	cmp r1, #101											\n"
		"	bne reg1_error_loopf									\n"
		"	vmov r0, r1, d13										\n"
		"	cmp r0, #102											\n"
		"	bne reg1_error_loopf									\n"
		"	cmp r1, #103											\n"
		"	bne reg1_error_loopf									\n"
		"	vmov r0, r1, d14										\n"
		"	cmp r0, #104											\n"
		"	bne reg1_error_loopf									\n"
		"	cmp r1, #105											\n"
		"	bne reg1_error_loopf									\n"
		"	vmov r0, r1, d15										\n"
		"	cmp r0, #106											\n"
		"	bne reg1_error_loopf									\n"
		"	cmp r1, #107											\n"
		"	bne reg1_error_loopf									\n"
		"															\n"
		"	/* Restore the registers that were clobbered by the test. */\n"
		"	pop {r0-r1}												\n"
		"															\n"
		"	/* VFP register test passed.  Jump to the core register test. */\n"
		"	b reg1_loopf_pass										\n"
		"															\n"
		"reg1_error_loopf:											\n"
		"	/* If this line is hit then a VFP register value was found to be\n"
		"	incorrect. */											\n"
		"	b reg1_error_loopf										\n"
		"															\n"
		"reg1_loopf_pass:											\n"
		"															\n"
		"	cmp	r0, #100											\n"
		"	bne	reg1_error_loop										\n"
		"	cmp	r1, #101											\n"
		"	bne	reg1_error_loop										\n"
		"	cmp	r2, #102											\n"
		"	bne	reg1_error_loop										\n"
		"	cmp r3, #103											\n"
		"	bne	reg1_error_loop										\n"
		"	cmp	r4, #104											\n"
		"	bne	reg1_error_loop										\n"
		"	cmp	r5, #105											\n"
		"	bne	reg1_error_loop										\n"
		"	cmp	r6, #106											\n"
		"	bne	reg1_error_loop										\n"
		"	cmp	r7, #107											\n"
		"	bne	reg1_error_loop										\n"
		"	cmp	r8, #108											\n"
		"	bne	reg1_error_loop										\n"
		"	cmp	r9, #109											\n"
		"	bne	reg1_error_loop										\n"
		"	cmp	r10, #110											\n"
		"	bne	reg1_error_loop										\n"
		"	cmp	r11, #111											\n"
		"	bne	reg1_error_loop										\n"
		"	cmp	r12, #112											\n"
		"	bne	reg1_error_loop										\n"
		"															\n"
		"	/* Everything passed, increment the loop counter. */	\n"
		"	push { r0-r1 }											\n"
		"	ldr	r0, =ulRegTest1LoopCounter							\n"
		"	ldr r1, [r0]											\n"
		"	adds r1, r1, #1											\n"
		"	str r1, [r0]											\n"
		"	pop { r0-r1 }											\n"
		"															\n"
		"	/* Start again. */										\n"
		"	b reg1_loop												\n"
		"															\n"
		"reg1_error_loop:											\n"
		"	/* If this line is hit then there was an error in a core register value.\n"
		"	The loop ensures the loop counter stops incrementing. */\n"
		"	b reg1_error_loop										\n"
		"	nop														"
	);
}
/*-----------------------------------------------------------*/

/* This is a naked function. */
void vRegTest2Task( void *pvParameters )
{
	__asm volatile
	(
		"	/* Set all the core registers to known values. */		\n"
		"	mov r0, #-1												\n"
		"	mov r1, #1												\n"
		"	mov r2, #2												\n"
		"	mov r3, #3												\n"
		"	mov	r4, #4												\n"
		"	mov	r5, #5												\n"
		"	mov	r6, #6												\n"
		"	mov r7, #7												\n"
		"	mov	r8, #8												\n"
		"	mov	r9, #9												\n"
		"	mov	r10, #10											\n"
		"	mov	r11, #11											\n"
		"	mov r12, #12											\n"
		"															\n"
		"	/* Set all the VFP to known values. */					\n"
		"	vmov d0, r0, r1											\n"
		"	vmov d1, r2, r3											\n"
		"	vmov d2, r4, r5											\n"
		"	vmov d3, r6, r7											\n"
		"	vmov d4, r8, r9											\n"
		"	vmov d5, r10, r11										\n"
		"	vmov d6, r0, r1											\n"
		"	vmov d7, r2, r3											\n"
		"	vmov d8, r4, r5											\n"
		"	vmov d9, r6, r7											\n"
		"	vmov d10, r8, r9										\n"
		"	vmov d11, r10, r11										\n"
		"	vmov d12, r0, r1										\n"
		"	vmov d13, r2, r3										\n"
		"	vmov d14, r4, r5										\n"
		"	vmov d15, r6, r7										\n"
		"															\n"
		"reg2_loop:													\n"
		"															\n"
		"	/* Check all the VFP registers still contain the values set above.\n"
		"	First save registers that are clobbered by the test. */	\n"
		"	push { r0-r1 }											\n"
		"															\n"
		"	vmov r0, r1, d0											\n"
		"	cmp r0, #-1												\n"
		"	bne reg2_error_loopf									\n"
		"	cmp r1, #1												\n"
		"	bne reg2_error_loopf									\n"
		"	vmov r0, r1, d1											\n"
		"	cmp r0, #2												\n"
		"	bne reg2_error_loopf									\n"
		"	cmp r1, #3												\n"
		"	bne reg2_error_loopf									\n"
		"	vmov r0, r1, d2											\n"
		"	cmp r0, #4												\n"
		"	bne reg2_error_loopf									\n"
		"	cmp r1, #5												\n"
		"	bne reg2_error_loopf									\n"
		"	vmov r0, r1, d3											\n"
		"	cmp r0, #6												\n"
		"	bne reg2_error_loopf									\n"
		"	cmp r1, #7												\n"
		"	bne reg2_error_loopf									\n"
		"	vmov r0, r1, d4											\n"
		"	cmp r0, #8												\n"
		"	bne reg2_error_loopf									\n"
		"	cmp r1, #9												\n"
		"	bne reg2_error_loopf									\n"
		"	vmov r0, r1, d5											\n"
		"	cmp r0, #10												\n"
		"	bne reg2_error_loopf									\n"
		"	cmp r1, #11												\n"
		"	bne reg2_error_loopf									\n"
		"	vmov r0, r1, d6											\n"
		"	cmp r0, #-1												\n"
		"	bne reg2_error_loopf									\n"
		"	cmp r1, #1												\n"
		"	bne reg2_error_loopf									\n"
		"	vmov r0, r1, d7											\n"
		"	cmp r0, #2												\n"
		"	bne reg2_error_loopf									\n"
		"	cmp r1, #3												\n"
		"	bne reg2_error_loopf									\n"
		"	vmov r0, r1, d8											\n"
		"	cmp r0, #4												\n"
		"	bne reg2_error_loopf									\n"
		"	cmp r1, #5												\n"
		"	bne reg2_error_loopf									\n"
		"	vmov r0, r1, d9											\n"
		"	cmp r0, #6												\n"
		"	bne reg2_error_loopf									\n"
		"	cmp r1, #7												\n"
		"	bne reg2_error_loopf									\n"
		"	vmov r0, r1, d10										\n"
		"	cmp r0, #8												\n"
		"	bne reg2_error_loopf									\n"
		"	cmp r1, #9												\n"
		"	bne reg2_error_loopf									\n"
		"	vmov r0, r1, d11										\n"
		"	cmp r0, #10												\n"
		"	bne reg2_error_loopf									\n"
		"	cmp r1, #11												\n"
		"	bne reg2_error_loopf									\n"
		"	vmov r0, r1, d12										\n"
		"	cmp r0, #-1												\n"
		"	bne reg2_error_loopf									\n"
		"	cmp r1, #1												\n"
		"	bne reg2_error_loopf									\n"
		"	vmov r0, r1, d13										\n"
		"	cmp r0, #2												\n"
		"	bne reg2_error_loopf									\n"
		"	cmp r1, #3												\n"
		"	bne reg2_error_loopf									\n"
		"	vmov r0, r1, d14										\n"
		"	cmp r0, #4												\n"
		"	bne reg2_error_loopf									\n"
		"	cmp r1, #5												\n"
		"	bne reg2_error_loopf									\n"
		"	vmov r0, r1, d15										\n"
		"	cmp r0, #6												\n"
		"	bne reg2_error_loopf									\n"
		"	cmp r1, #7												\n"
		"	bne reg2_error_loopf									\n"
		"															\n"
		"	/* Restore the registers that were clobbered by the test. */\n"
		"	pop {r0-r1}												\n"
		"															\n"
		"	/* VFP register test passed.  Jump to the core register test. */\n"
		"	b reg2_loopf_pass										\n"
		"															\n"
		"reg2_error_loopf:											\n"
		"	/* If this line is hit then a VFP register value was found to be\n"
		"	incorrect. */											\n"
		"	b reg2_error_loopf										\n"
		"															\n"
		"reg2_loopf_pass:											\n"
		"															\n"
		"	cmp	r0, #-1												\n"
		"	bne	reg2_error_loop										\n"
		"	cmp	r1, #1												\n"
		"	bne	reg2_error_loop										\n"
		"	cmp	r2, #2												\n"
		"	bne	reg2_error_loop										\n"
		"	cmp r3, #3												\n"
		"	bne	reg2_error_loop										\n"
		"	cmp	r4, #4												\n"
		"	bne	reg2_error_loop										\n"
		"	cmp	r5, #5												\n"
		"	bne	reg2_error_loop										\n"
		"	cmp	r6, #6												\n"
		"	bne	reg2_error_loop										\n"
		"	cmp	r7, #7												\n"
		"	bne	reg2_error_loop										\n"
		"	cmp	r8, #8												\n"
		"	bne	reg2_error_loop										\n"
		"	cmp	r9, #9												\n"
		"	bne	reg2_error_loop										\n"
		"	cmp	r10, #10											\n"
		"	bne	reg2_error_loop										\n"
		"	cmp	r11, #11											\n"
		"	bne	reg2_error_loop										\n"
		"	cmp	r12, #12											\n"
		"	bne	reg2_error_loop										\n"
		"															\n"
		"	/* Increment the loop counter to indicate this test is still functioning\n"
		"	correctly. */											\n"
		"	push { r0-r1 }											\n"
		"	ldr	r0, =ulRegTest2LoopCounter							\n"
		"	ldr r1, [r0]											\n"
		"	adds r1, r1, #1											\n"
		"	str r1, [r0]											\n"
		"	pop { r0-r1 }											\n"
		"															\n"
		"	/* Start again. */										\n"
		"	b reg2_loop												\n"
		"															\n"
		"reg2_error_loop:											\n"
		"	/* If this line is hit then there was an error in a core register value.\n"
		"	This loop ensures the loop counter variable stops incrementing. */\n"
		"	b reg2_error_loop										\n"
		"	nop														\n"
	);
}

/* This is a naked function. */
void vRegTestClearFlopRegistersToParameterValue(  unsigned long ulValue )
{
	__asm volatile
	(
    /* Clobber the auto saved registers. */
	" vmov d0, r0, r0 \n"
	" vmov d1, r0, r0 \n"
	" vmov d2, r0, r0 \n"
	" vmov d3, r0, r0 \n"
	" vmov d4, r0, r0 \n"
	" vmov d5, r0, r0 \n"
	" vmov d6, r0, r0 \n"
	" vmov d7, r0, r0 \n"
	" bx lr \n"
	);
}

/* This is a naked function. */
void ulRegTestCheckFlopRegistersContainParameterValue(  unsigned long ulValue )
{
	__asm volatile
	(
    "   vmov r1, s0 \n"
    "   cmp r0, r1 \n"
    "   bne return_error \n"
    "   vmov r1, s1 \n"
    "   cmp r0, r1 \n"
    "   bne return_error \n"
    "   vmov r1, s2 \n"
    "   cmp r0, r1 \n"
    "   bne return_error \n"
     "  vmov r1, s3 \n"
    "   cmp r0, r1 \n"
    "   bne return_error \n"
    "   vmov r1, s4 \n"
    "   cmp r0, r1 \n"
    "   bne return_error \n"
    "   vmov r1, s5 \n"
    "   cmp r0, r1 \n"
    "   bne return_error \n"
    "   vmov r1, s6 \n"
    "   cmp r0, r1 \n"
    "   bne return_error \n"
    "   vmov r1, s7 \n"
    "   cmp r0, r1 \n"
    "   bne return_error \n"
    "   vmov r1, s8 \n"
    "   cmp r0, r1 \n"
    "   bne return_error \n"
    "   vmov r1, s9 \n"
    "   cmp r0, r1 \n"
    "   bne return_error \n"
    "   vmov r1, s10 \n"
    "   cmp r0, r1 \n"
    "   bne return_error \n"
    "   vmov r1, s11 \n"
    "   cmp r0, r1 \n"
    "   bne return_error \n"
    "   vmov r1, s12 \n"
    "   cmp r0, r1 \n"
    "   bne return_error \n"
    "   vmov r1, s13 \n"
    "   cmp r0, r1 \n"
    "   bne return_error \n"
    "   vmov r1, s14 \n"
    "   cmp r0, r1 \n"
    "   bne return_error \n"
    "   vmov r1, s15 \n"
    "   cmp r0, r1 \n"
    "   bne return_error \n"
      
    " return_pass: \n"
    "   mov r0, #1 \n"
    "   bx lr \n"

    " return_error: \n"
    "   mov r0, #0 \n"
    "   bx lr \n"

	);
}
