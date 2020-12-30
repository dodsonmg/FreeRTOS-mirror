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

/* Dimensions the buffer into which input characters are placed. */
#define cmdMAX_INPUT_SIZE	1024

/* Dimensions the buffer into which string outputs can be placed. */
#define cmdMAX_OUTPUT_SIZE	1024

/* Dimensions the buffer passed to the recv() call. */
#define cmdSOCKET_INPUT_BUFFER_SIZE 60

/* DEL acts as a backspace. */
#define cmdASCII_DEL		( 0x7F )

/* The maximum time to wait for a closing socket to close. */
#define cmdSHUTDOWN_DELAY	( pdMS_TO_TICKS( 5000 ) )

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

/*
 * Various buffers used by the command lin interpreter.
 */
static char cOutputString[ cmdMAX_OUTPUT_SIZE ], cLocalBuffer[ cmdSOCKET_INPUT_BUFFER_SIZE ];
static char cInputString[ cmdMAX_INPUT_SIZE ], cLastInputString[ cmdMAX_INPUT_SIZE ];

/* Const messages output by the command console. */
static const char * const pcWelcomeMessage = "FreeRTOS command server.\r\nType help to view a list of registered commands.\r\nType quit to end a session.\r\n\r\n>";
static const char * const pcEndOfOutputMessage = "\r\n[Press ENTER to execute the previous command again]\r\n>";
static const char * const pcNewLine = "\r\n";

/* The structure holding Modbus state information. */
static modbus_mapping_t *mb_mapping = NULL;

/* The structure holding Modbus context. */
static modbus_t *ctx = NULL;

/*-----------------------------------------------------------*/

void vStartModbusServerTask( uint16_t usStackSize, uint32_t ulPort, UBaseType_t uxPriority )
{
	xTaskCreate( prvModbusServerTask, "MODBUS_SERVER", usStackSize, ( void * ) ulPort, uxPriority, NULL );
}
/*-----------------------------------------------------------*/

void prvModbusServerTask( void *pvParameters )
{
int32_t lBytes, lByte, lSent;
char cRxedChar, cInputIndex = 0;
BaseType_t xMoreDataToFollow;
struct freertos_sockaddr xClient;
Socket_t xListeningSocket, xConnectedSocket;
socklen_t xSize = sizeof( xClient );
/* The strange casting is to remove compiler warnings on 32-bit machines. */
uint16_t usPort = ( uint16_t ) ( ( uint32_t ) pvParameters ) & 0xffffUL;
BaseType_t xReturned;

/* buffers for comms with libmodbus */
uint8_t *req = ( uint8_t * )pvPortMalloc( MODBUS_MAX_STRING_LENGTH * sizeof( uint8_t ) );
int req_length = 0;
uint8_t *rsp = ( uint8_t * )pvPortMalloc( MODBUS_MAX_STRING_LENGTH * sizeof( uint8_t ) );
int rsp_length = 0;

    /* Initialise the Modbus server state and context */
    prvModbusServerInitialization( usPort );

	memset( cInputString, 0x00, cmdMAX_INPUT_SIZE );

	for( ;; )
	{
		/* Attempt to open the socket.  The port number is passed in the task
		parameter. */
		xListeningSocket = prvOpenTCPServerSocket( usPort );

		/* Nothing for this task to do if the socket cannot be created. */
		if( xListeningSocket == FREERTOS_INVALID_SOCKET )
		{
			vTaskDelete( NULL );
		}

		/* Wait for an incoming connection. */
		xConnectedSocket = modbus_tcp_accept( ctx, &xListeningSocket );
        configASSERT( xConnectedSocket != NULL && xConnectedSocket != FREERTOS_INVALID_SOCKET );

        /* Receive a request from the Modbus client. */
        req_length = modbus_receive( ctx, req );

		/* Process the socket as long as it remains connected. */
		while( req_length >= 0 )
		{
            xReturned = modbus_process_request( ctx, req, req_length, rsp, &rsp_length,
                    mb_mapping );
            configASSERT( xReturned != -1 );

            xReturned = modbus_reply( ctx, rsp, rsp_length );
            configASSERT( xReturned != -1 );

            /* Receive a request from the Modbus client. */
            req_length = modbus_receive( ctx, req );
		}

		/* Close the socket correctly. */
		prvGracefulShutdown();
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

    /* Initialise queues for comms with prvCriticalSectionTask */
    /* xQueueRequest = xQueueCreate(modbusQUEUE_LENGTH, sizeof(queue_msg_t *)); */
    /* configASSERT(xQueueRequest != NULL); */

    /* xQueueResponse = xQueueCreate(modbusQUEUE_LENGTH, sizeof(queue_msg_t *)); */
    /* configASSERT(xQueueResponse != NULL); */
}

/*-----------------------------------------------------------*/
