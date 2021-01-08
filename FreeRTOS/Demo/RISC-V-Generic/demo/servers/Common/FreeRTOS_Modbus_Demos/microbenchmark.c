/*
 * FreeRTOS Kernel V10.1.1
 * Copyright (C) 2018 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
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

/******************************************************************************
 * NOTE 1:  This project provides a demo of the Modbus industrial protocol.
 *
 * NOTE 2:  This file only contains the source code that is specific to the
 * basic demo.  Generic functions, such FreeRTOS hook functions, and functions
 * required to configure the hardware are defined in main.c.
 ******************************************************************************
 */

/* Standard includes. */
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* Modbus includes. */
#include <modbus/modbus.h>
#include <modbus/modbus-helpers.h>

/* Microbenchmark includes */
#include "microbenchmark.h"

/* type definitions */

typedef struct _BenchmarkSample_t {
    char pcFunctionName[MODBUS_MAX_FUNCTION_NAME_LEN];
    uint32_t ulTimeDiff;
} BenchmarkSample_t;

/* static variable declarations */

/* The buffer holding PrintTaskStatus_t structs */
static BenchmarkSample_t *pxPrintBuffer = NULL;
static size_t xPrintBufferSize = 0;
static size_t xPrintBufferCount = 0;

/*-----------------------------------------------------------*/

void xMicrobenchmarkSample( char *pcFunctionName, uint32_t ulTimeDiff, BaseType_t xToPrint )
{
    BaseType_t xReturned;
    size_t xFunctionNameLen = strnlen(pcFunctionName, MODBUS_MAX_FUNCTION_NAME_LEN);

    /* initialise the buffer, if necessary */
    if ( pxPrintBuffer == NULL )
    {
        xPrintBufferSize = MAX_FUNCTIONS * MICROBENCHMARK_ITERATIONS;
        pxPrintBuffer = (BenchmarkSample_t *)pvPortMalloc(
                xPrintBufferSize * sizeof(BenchmarkSample_t));
        configASSERT( pxPrintBuffer != NULL );
    }

    if ( xToPrint )
    {
        /* populate BenchmarkSample_t struct and add to the print buffer */
        pxPrintBuffer[xPrintBufferCount].ulTimeDiff = ulTimeDiff;
        strncpy ( pxPrintBuffer[xPrintBufferCount].pcFunctionName, pcFunctionName, xFunctionNameLen + 1 );
        xPrintBufferCount += 1;

        /* TODO: If xPrintBufferCount = xPrintBufferSize, then we need to resize. */
    }
}

/*-----------------------------------------------------------*/

void vPrintMicrobenchmarkSamples(void)
{
	/* Print out column headings for the run-time stats table. */
    printf("modbus_function_name, time_diff\n");
    for(int i = 0; i < xPrintBufferCount; ++i)
    {
        printf("%s, %u\n",
                pxPrintBuffer[i].pcFunctionName,
                pxPrintBuffer[i].ulTimeDiff);
    }
}

/*-----------------------------------------------------------*/

