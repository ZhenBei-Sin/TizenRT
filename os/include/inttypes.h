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
 * include/inttypes.h
 *
 *   Copyright (C) 2010 Gregory Nutt. All rights reserved.
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
/**
 * @defgroup INTTYPES_LIBC INTTYPES
 * @brief Provides APIs for Integer Types
 * @ingroup KERNEL
 * @{
 */

/// @file inttypes.h
/// @brief Inttypes APIs

#ifndef __INCLUDE_INTTYPES_H
#define __INCLUDE_INTTYPES_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stddef.h>				/* for wchar_t */

/* Notes from www.opengroup.org:
 *
 * "The <inttypes.h> header shall include the <stdint.h> header."
 */

#include <stdint.h>

#include <arch/inttypes.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* "The following macros shall be defined. Each expands to a character string
 *  literal containing a conversion specifier, possibly modified by a lengt
 *  modifier, suitable for use within the format argument of a formatted
 *  input/output function when converting the corresponding integer type.
 *  These macros have the general form of PRI (character string literals for
 *  the fprintf() and fwprintf() family of functions) or SCN (character string
 *  literals for the fscanf() and fwscanf() family of functions), followed by
 *  the conversion specifier, followed by a name corresponding to a similar
 *  type name in <stdint.h>. In these names, N represents the width of the
 *  type as described in <stdint.h>. For example, PRIdFAST32 can be used in a
 *  format string to print the value of an integer of type int_fast32_t.
 *
 * "The fprintf() macros for signed integers are:
 *
 * PRIdN
 *  PRIdLEASTN
 *  PRIdFASTN
 *  PRIdMAX
 *  PRIdPTR
 *
 * PRIiN
 *  PRIiLEASTN
 *  PRIiFASTN
 *  PRIiMAX
 *  PRIiPTR
 *
 * "The fprintf() macros for unsigned integers are:
 *
 * PRIoN
 *  PRIoLEASTN
 *  PRIoFASTN
 *  PRIoMAX
 *  PRIoPTR
 *
 * PRIuN
 *  PRIuLEASTN
 *  PRIuFASTN
 *  PRIuMAX
 *  PRIuPTR
 *
 * PRIxN
 *  PRIxLEASTN
 *  PRIxFASTN
 *  PRIxMAX
 *  PRIxPTR
 *
 * PRIXN
 *  PRIXLEASTN
 *  PRIXFASTN
 *  PRIXMAX
 *  PRIXPTR
 *
 * "The fscanf() macros for signed integers are:
 *
 * SCNdN
 *  SCNdLEASTN
 *  SCNdFASTN
 *  SCNdMAX
 *  SCNdPTR
 *
 * SCNiN
 *  SCNiLEASTN
 *  SCNiFASTN
 *  SCNiMAX
 *  SCNiPTR
 *
 * "The fscanf() macros for unsigned integers are:
 *
 * SCNoN
 *  SCNoLEASTN
 *  SCNoFASTN
 *  SCNoMAX
 *  SCNoPTR
 *
 * SCNuN
 *  SCNuLEASTN
 *  SCNuFASTN
 *  SCNuMAX
 *  SCNuPTR
 *
 * SCNxN
 *  SCNxLEASTN
 *  SCNxFASTN
 *  SCNxMAX
 *  SCNxPTR
 *
 * "For each type that the implementation provides in <stdint.h>, the
 * corresponding fprintf() and fwprintf() macros shall be defined and the
 * corresponding fscanf() and fwscanf() macros shall be defined unless the
 * implementation does not have a suitable modifier for the type.
 */

/****************************************************************************
 * Type Definitions
 ****************************************************************************/

/* "The <inttypes.h> header shall include a definition of at least the
 * following type:
 *
 * imaxdiv_t
 *   Structure type that is the type of the value returned by the imaxdiv()
 *   function.
 */

typedef struct {
	intmax_t quot;
	intmax_t rem;
} imaxdiv_t;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/* "The following shall be declared as functions and may also be defined as
 *  macros. Function prototypes shall be provided."
 */

/**
 * @brief return absolute value
 * @details @b #include <inttypes.h> \n
 * POSIX API (refer to : http://pubs.opengroup.org/onlinepubs/9699919799/)
 * @since TizenRT v1.0
 */
intmax_t imaxabs(intmax_t j);
/**
 * @brief return quotient and remainder of an integer division
 * @details @b #include <inttypes.h> \n
 * POSIX API (refer to : http://pubs.opengroup.org/onlinepubs/9699919799/)
 * @since TizenRT v2.0
 */
imaxdiv_t imaxdiv(intmax_t numer, intmax_t denom);
/**
 * @brief convert string to integer
 * @details @b #include <inttypes.h> \n
 * POSIX API (refer to : http://pubs.opengroup.org/onlinepubs/9699919799/)
 * @since TizenRT v2.0
 */
intmax_t strtoimax(FAR const char *nptr, FAR char **endptr, int base);
/**
 * @brief convert string to unsigned integer
 * @details @b #include <inttypes.h> \n
 * POSIX API (refer to : http://pubs.opengroup.org/onlinepubs/9699919799/)
 * @since TizenRT v2.0
 */
uintmax_t strtoumax(FAR const char *nptr, FAR char **endptr, int base);
/**
 * @cond
 * @internal
 */
intmax_t wcstoimax(FAR const wchar_t *nptr, FAR wchar_t **endptr, int base);
/**
 * @internal
 */
uintmax_t wcstoumax(FAR const wchar_t *nptr, FAR wchar_t **endptr, int base);


/**
 * @endcond
 */

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif							/* __INCLUDE_INTTYPES_H */

/**
 * @}
 */
