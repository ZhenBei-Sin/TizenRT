/****************************************************************************
 *
 * Copyright 2016 Samsung Electronics All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
 * either express or implied. See the License for the specific
 * language governing permissions and limitations under the License.
 *
 ****************************************************************************/
/****************************************************************************
 * arch/arm/src/common/up_usestack.c
 *
 *   Copyright (C) 2007-2009, 2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <tinyara/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <sched.h>
#include <debug.h>

#include <tinyara/kmalloc.h>
#include <tinyara/arch.h>

#include "up_internal.h"

/****************************************************************************
 * Pre-processor Macros
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Global Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_use_stack
 *
 * Description:
 *   Setup up stack-related information in the TCB using pre-allocated stack
 *   memory.  This function is called only from task_init() when a task or
 *   kernel thread is started (never for pthreads).
 *
 *   The following TCB fields must be initialized:
 *
 *   - adj_stack_size: Stack size after adjustment for hardware,
 *     processor, etc.  This value is retained only for debug
 *     purposes.
 *   - stack_alloc_ptr: Pointer to allocated stack
 *   - adj_stack_ptr: Adjusted stack_alloc_ptr for HW.  The
 *     initial value of the stack pointer.
 *
 * Inputs:
 *   - tcb: The TCB of new task
 *   - stack_size:  The allocated stack size.
 *
 *   NOTE:  Unlike up_stack_create() and up_stack_release, this function
 *   does not require the task type (ttype) parameter.  The TCB flags will
 *   always be set to provide the task type to up_use_stack() if it needs
 *   that information.
 *
 ****************************************************************************/

int up_use_stack(struct tcb_s *tcb, void *stack, size_t stack_size)
{
	size_t top_of_stack;
	size_t size_of_stack;

	/* Is there already a stack allocated? */

	if (tcb->stack_alloc_ptr) {
		/* Yes... Release the old stack allocation */

		up_release_stack(tcb, tcb->flags & TCB_FLAG_TTYPE_MASK);
	}

	/* Save the new stack allocation */

	tcb->stack_alloc_ptr = stack;

	/* The ARM uses a push-down stack:  the stack grows toward lower addresses
	 * in memory.  The stack pointer register, points to the lowest, valid
	 * work address (the "top" of the stack).  Items on the stack are
	 * referenced as positive word offsets from sp.
	 */

#ifdef CONFIG_ARCH_ARMV7A_FAMILY
	top_of_stack = (uint32_t)tcb->stack_alloc_ptr + stack_size;
#else
	top_of_stack = (uint32_t)tcb->stack_alloc_ptr + stack_size - 4;
#endif

	/* The ARM stack must be aligned; 4 byte alignment for OABI and 8-byte
	 * alignment for EABI. If necessary top_of_stack must be rounded down
	 * to the next boundary
	 */

	top_of_stack = STACK_ALIGN_DOWN(top_of_stack);

	/* The size of the stack in bytes is then the difference between
	 * the top and the bottom of the stack (+4 because if the top
	 * is the same as the bottom, then the size is one 32-bit element).
	 * The size need not be aligned.
	 */

#ifdef CONFIG_ARCH_ARMV7A_FAMILY
	size_of_stack = top_of_stack - (uint32_t)tcb->stack_alloc_ptr;
#else
	size_of_stack = top_of_stack - (uint32_t)tcb->stack_alloc_ptr + 4;
#endif

	/* Save the adjusted stack values in the struct tcb_s */

	tcb->adj_stack_ptr = (uint32_t *)top_of_stack;
	tcb->stack_base_ptr = tcb->stack_alloc_ptr;
	tcb->adj_stack_size = size_of_stack;

#ifdef CONFIG_STACK_COLORATION
	up_stack_color(tcb->stack_alloc_ptr, tcb->adj_stack_ptr);
#endif
	return OK;
}
