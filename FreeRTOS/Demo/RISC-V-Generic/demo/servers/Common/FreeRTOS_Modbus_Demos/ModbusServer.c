/*
   FreeRTOS V9.0.0 - Copyright (C) 2016 Real Time Engineers Ltd.
   All rights reserved

   VISIT http://www.FreeRTOS.org TO ENSURE YOU ARE USING THE LATEST VERSION.

   This file is part of the FreeRTOS distribution.

   FreeRTOS is free software; you can redistribute it and/or modify it under
   the terms of the GNU General Public License (version 2) as published by the
   Free Software Foundation >>!AND MODIFIED BY!<< the FreeRTOS exception.

 ***************************************************************************
 >>!   NOTE: The modification to the GPL is included to allow you to     !<<
 >>!   distribute a combined work that includes FreeRTOS without being   !<<
 >>!   obliged to provide the source code for proprietary components     !<<
 >>!   outside of the FreeRTOS kernel.                                   !<<
 ***************************************************************************

 FreeRTOS is distributed in the hope that it will be useful, but WITHOUT ANY
 WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 FOR A PARTICULAR PURPOSE.  Full license text is available on the following
link: http://www.freertos.org/a00114.html

 ***************************************************************************
 *                                                                       *
 *    FreeRTOS provides completely free yet professionally developed,    *
 *    robust, strictly quality controlled, supported, and cross          *
 *    platform software that is more than just the market leader, it     *
 *    is the industry's de facto standard.                               *
 *                                                                       *
 *    Help yourself get started quickly while simultaneously helping     *
 *    to support the FreeRTOS project by purchasing a FreeRTOS           *
 *    tutorial book, reference manual, or both:                          *
 *    http://www.FreeRTOS.org/Documentation                              *
 *                                                                       *
 ***************************************************************************

http://www.FreeRTOS.org/FAQHelp.html - Having a problem?  Start by reading
the FAQ page "My application does not run, what could be wrong?".  Have you
defined configASSERT()?

http://www.FreeRTOS.org/support - In return for receiving this top quality
embedded software for free we request you assist our global community by
participating in the support forum.

http://www.FreeRTOS.org/training - Investing in training allows your team to
be as productive as possible as early as possible.  Now you can receive
FreeRTOS training directly from Richard Barry, CEO of Real Time Engineers
Ltd, and the world's leading authority on the world's leading RTOS.

http://www.FreeRTOS.org/plus - A selection of FreeRTOS ecosystem products,
including FreeRTOS+Trace - an indispensable productivity tool, a DOS
compatible FAT file system, and our tiny thread aware UDP/IP stack.

http://www.FreeRTOS.org/labs - Where new FreeRTOS products go to incubate.
Come and try FreeRTOS+TCP, our new open source TCP/IP stack for FreeRTOS.

http://www.OpenRTOS.com - Real Time Engineers ltd. license FreeRTOS to High
Integrity Systems ltd. to sell under the OpenRTOS brand.  Low cost OpenRTOS
licenses offer ticketed support, indemnification and commercial middleware.

http://www.SafeRTOS.com - High Integrity Systems also provide a safety
engineered and independently SIL3 certified version for use in safety and
mission critical applications that require provable dependability.

1 tab == 4 spaces!
*/

/* Standard includes. */
#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* FreeRTOS+TCP includes. */
#include "FreeRTOS_IP.h"
#include "FreeRTOS_Sockets.h"

/* FreeRTOS+CLI includes */
#include "FreeRTOS_CLI.h"

/* Demo app includes. */
#include "ModbusServer.h"
#include "ModbusDemoConstants.h"

/* Modbus includes. */
#include <modbus/modbus.h>
#include <modbus/modbus-helpers.h>

/* Microbenchmark includes */
#if defined( MICROBENCHMARK )
#include "microbenchmark.h"
#endif

/* Modbus object capability includes */
#if defined(MODBUS_OBJECT_CAPS) || defined(MODBUS_OBJECT_CAPS_STUBS)
#include "modbus_object_caps.h"
#endif

/* Modbus network capability includes */
#if defined(MODBUS_NETWORK_CAPS)
#include "modbus_network_caps.h"
#endif

/*-----------------------------------------------------------*/

/* The execution cycle time for the Modbus server task */
#define prvMODBUS_SERVER_PERIODICITY pdMS_TO_TICKS( 100 )

/*-----------------------------------------------------------*/

/*
 * The task that runs the Modbus server.
 */
static void prvModbusServerTask( void *pvParameters );

/*
 * Initialise the Modbus server (e.g., state and context).
 */
static void prvModbusServerInitialization( uint16_t usPort );

/*
 * Open and configure the TCP socket.
 */
static Socket_t prvOpenTCPServerSocket( uint16_t usPort );

/*
 * Processes a Modbus request.
 */
static uint64_t prvProcessModbusRequest(const uint8_t *req, const int req_length,
        uint8_t *rsp, int *rsp_length);

/*
 * A connected socket is being closed.  Ensure the socket is closed at both ends
 * properly.
 */
static void prvGracefulShutdown( void );

/*-----------------------------------------------------------*/

/* Structure to hold queue messages (requests and responses). */
typedef struct _queue_msg_t
{
    int msg_length;
    uint8_t *msg;
} queue_msg_t;

/* The structure holding Modbus state information. */
static modbus_mapping_t *mb_mapping = NULL;

/* The structure holding Modbus context. */
static modbus_t *ctx = NULL;

#if defined( MICROBENCHMARK )
/* The variable that will be incremented by the Idle hook function, and used as a comparison
 * of idle time between different configurations. */
extern uint32_t ulIdleCycleCount;
#endif

/*-----------------------------------------------------------*/

void vStartModbusServerTask( uint16_t usStackSize, uint32_t ulPort, UBaseType_t uxPriority )
{
    xTaskCreate( prvModbusServerTask, "ModbusServer", usStackSize, ( void * ) ulPort, uxPriority, NULL );
}
/*-----------------------------------------------------------*/

void prvModbusServerTask( void *pvParameters )
{
    BaseType_t xReturned;
    uint64_t ulTimeDiff;
    char *pcModbusFunctionName;
    Socket_t xListeningSocket, xConnectedSocket;

    /* The strange casting is to remove compiler warnings on 32-bit machines. */
    uint16_t usPort = ( uint16_t ) ( ( uint32_t ) pvParameters ) & 0xffffUL;

    /* buffers for comms with libmodbus */
    uint8_t *req = ( uint8_t * )pvPortMalloc( MODBUS_MAX_STRING_LENGTH * sizeof( uint8_t ) );
    int req_length = 0;
    uint8_t *rsp = ( uint8_t * )pvPortMalloc( MODBUS_MAX_STRING_LENGTH * sizeof( uint8_t ) );
    int rsp_length = 0;

#if defined( MICROBENCHMARK )
    TickType_t xTaskTickCountStart;
    TickType_t xTaskTickCountEnd;
    TickType_t xPreviousWakeTime = xTaskGetTickCount();
    TickType_t xTimeIncrement = prvMODBUS_SERVER_PERIODICITY;
#endif

    /* Initialise the Modbus server state and context */
    prvModbusServerInitialization( usPort );

    /* Attempt to open the socket.  The port number is passed in the task
       parameter. */
    xListeningSocket = prvOpenTCPServerSocket( usPort );

    for( ;; )
    {
        /* Nothing for this task to do if the socket cannot be created. */
        if( xListeningSocket == FREERTOS_INVALID_SOCKET )
        {
            vTaskDelete( NULL );
        }

        /* Wait for an incoming connection. */
        xConnectedSocket = modbus_tcp_accept( ctx, &xListeningSocket );
        configASSERT( xConnectedSocket != NULL &&
                xConnectedSocket != FREERTOS_INVALID_SOCKET );

        /* Receive a request from the Modbus client. */
        req_length = modbus_receive( ctx, req );

        /* Process the socket as long as it remains connected. */
        while( req_length >= 0 )
        {
            /* get the modbus function name from the request */
            pcModbusFunctionName = modbus_get_function_name( ctx, req );

            /* Process the request. */
            ulTimeDiff = prvProcessModbusRequest( req, req_length, rsp, &rsp_length );
            configASSERT( xReturned != -1 );

#if defined( MICROBENCHMARK )
            /* Take a microbenchmark sample on the time required to process the request */
            xMicrobenchmarkSample( REQUEST_PROCESSING, pcModbusFunctionName, ulTimeDiff, pdTRUE );
#endif

            /* Reply to the Modbus client. */
            xReturned = modbus_reply( ctx, rsp, rsp_length );
            configASSERT( xReturned != -1 );

#if defined( MICROBENCHMARK )
            /* Save the current cycle count, and then save the count after
             * calling vTaskDelayUntil().  The difference is a proxy for the
             * spare processing time available in this task. */
            xTaskTickCountStart = xTaskGetTickCount();

            /* Block until the next, fixed execution period */
            vTaskDelayUntil( &xPreviousWakeTime, xTimeIncrement );

            /* Save the current cycle coutn again.
             * Subtract the initial count to support comparison against runs. */
            xTaskTickCountEnd = xTaskGetTickCount();

            /* Save the difference in cycle count as a benchmarking sample. */
            xMicrobenchmarkSample( SPARE_PROCESSING, pcModbusFunctionName,
                    xTaskTickCountEnd - xTaskTickCountStart, pdTRUE );
#endif

            /* Receive the next request from the Modbus client. */
            req_length = modbus_receive( ctx, req );
        }

        /* Close the socket correctly. */
        prvGracefulShutdown();

#if defined( MICROBENCHMARK )
        /* Print microbenchmark samples to stdout and do not reopen the port */
        vPrintMicrobenchmarkSamples();
#endif
    }
}

/*-----------------------------------------------------------*/

static void prvGracefulShutdown( void )
{
    modbus_close(ctx);
}
/*-----------------------------------------------------------*/

static Socket_t prvOpenTCPServerSocket( uint16_t usPort )
{
    struct freertos_sockaddr xBindAddress;
    Socket_t xSocket;
    static const TickType_t xReceiveTimeOut = portMAX_DELAY;
    const BaseType_t xBacklog = 1;

    /* Attempt to open the socket. */
    xSocket = modbus_tcp_listen( ctx, xBacklog);
    configASSERT( xSocket != FREERTOS_INVALID_SOCKET );

    return xSocket;
}

/*-----------------------------------------------------------*/

static void prvModbusServerInitialization( uint16_t port )
{
    /* Allocate and populate the ctx structure.  Pass NULL for the ip,
     * since it isn't necessary for the server to know its own ip address. */
    ctx = modbus_new_tcp( NULL, port );
    if ( ctx == NULL )
    {
        fprintf( stderr, "Failed to allocate ctx: %s\r\n",
                modbus_strerror( errno ) );
        modbus_free( ctx );
        _exit( 0 );
    }

#ifdef NDEBUG
    modbus_set_debug( ctx, pdFALSE );
#else
    modbus_set_debug( ctx, pdTRUE );
#endif

    /* initialise state (mb_mapping) */
#if defined(MODBUS_OBJECT_CAPS) && !defined(MODBUS_OBJECT_CAPS_STUB)
    mb_mapping = modbus_mapping_new_start_address_object_caps(
            ctx,
            UT_BITS_ADDRESS, UT_BITS_NB,
            UT_INPUT_BITS_ADDRESS, UT_INPUT_BITS_NB,
            UT_REGISTERS_ADDRESS, UT_REGISTERS_NB_MAX,
            UT_INPUT_REGISTERS_ADDRESS, UT_INPUT_REGISTERS_NB );
#else
    mb_mapping = modbus_mapping_new_start_address(
            UT_BITS_ADDRESS, UT_BITS_NB,
            UT_INPUT_BITS_ADDRESS, UT_INPUT_BITS_NB,
            UT_REGISTERS_ADDRESS, UT_REGISTERS_NB_MAX,
            UT_INPUT_REGISTERS_ADDRESS, UT_INPUT_REGISTERS_NB );
#endif

    /* check for successful initialisation of state */
    if ( mb_mapping == NULL )
    {
        fprintf( stderr, "Failed to allocate the mapping: %s\r\n",
                modbus_strerror( errno ) );
        modbus_free( ctx );
        _exit( 0 );
    }

    /* display the state if DEBUG */
    if ( modbus_get_debug( ctx ) )
    {
        print_mb_mapping( mb_mapping );
    }

    /* Initialize coils */
    modbus_set_bits_from_bytes( mb_mapping->tab_input_bits, 0, UT_INPUT_BITS_NB,
            UT_INPUT_BITS_TAB );

    /* Initialize discrete inputs */
    for ( int i = 0; i < UT_INPUT_REGISTERS_NB; i++ )
    {
        mb_mapping->tab_input_registers[i] = UT_INPUT_REGISTERS_TAB[i];
    }

#if defined(MODBUS_NETWORK_CAPS)
    /* Initialise Macaroon */
    BaseType_t xReturned = 0;
    char *key = "a bad secret";
    char *id = "id for a bad secret";
    char *location = "https://www.modbus.com/macaroons/";
    xReturned = initialise_server_network_caps( ctx, location, key, id );
    if (xReturned == -1) {
        fprintf( stderr, "Failed to initialise server macaroon\r\n" );
        modbus_free( ctx );
        _exit( 0 );
    }
#endif
}

/*-----------------------------------------------------------*/

/**
 * Process a Modbus request from a client to a server.
 *
 * Returns the cycle count to process the request.
 */
static uint64_t prvProcessModbusRequest(const uint8_t *req, const int req_length,
        uint8_t *rsp, int *rsp_length)
{
    BaseType_t xReturned;
    uint64_t ulRuntimeCounterStart;
    uint64_t ulRuntimeCounterEnd;

    /* Ensure access to mb_mapping and ctx cannot be interrupted while processing
     * a request from the client */
    taskENTER_CRITICAL();

    ulRuntimeCounterStart = get_cycle_count();

    /**
     * Perform preprocessing for object or network capabilities
     * then perform the normal processing
     * NB order matters here:
     * - First reduce permissions on state
     * - Then verify the network capability, if appropriate
     * - Then perform the normal processing
     * NB A configuration without object or network capabilities can still
     * be compiled for a CHERI system, it just wont restrict the state before
     * processing the request.
     * */
#if defined(MODBUS_OBJECT_CAPS_STUB)
    /* this is only used to evaluate the overhead of calling a function */
    xReturned = modbus_preprocess_request_object_caps_stub(ctx, req, mb_mapping);
    configASSERT(xReturned != -1);
#elif defined(MODBUS_OBJECT_CAPS)
    xReturned = modbus_preprocess_request_object_caps(ctx, req, mb_mapping);
    configASSERT(xReturned != -1);
#endif

#if defined(MODBUS_NETWORK_CAPS)
    xReturned = modbus_preprocess_request_network_caps(ctx, req, mb_mapping);
    configASSERT(xReturned != -1);
#endif

    xReturned = modbus_process_request(ctx, req, req_length,
            rsp, rsp_length, mb_mapping);

    ulRuntimeCounterEnd = get_cycle_count();

    /* critical access to mb_mapping and ctx is finished, so it's safe to exit */
    taskEXIT_CRITICAL();

    return ulRuntimeCounterEnd - ulRuntimeCounterStart;
}

/*-----------------------------------------------------------*/
