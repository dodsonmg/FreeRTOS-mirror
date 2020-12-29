/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2020 Hesham Almatary
 *
 * This software was developed by SRI International and the University of
 * Cambridge Computer Laboratory (Department of Computer Science and
 * Technology) under DARPA contract HR0011-18-C-0016 ("ECATS"), as part of the
 * DARPA SSITH research programme.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

/*
FreeRTOS+TCP V2.0.11
Copyright (C) 2017 Amazon.com, Inc. or its affiliates.  All Rights Reserved.

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
the Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

 http://aws.amazon.com/freertos
 http://www.FreeRTOS.org
*/

/* Standard includes. */
#include <stdint.h>
#include <stdbool.h>

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* FreeRTOS+TCP includes. */
#include "FreeRTOS_IP.h"
#include "FreeRTOS_IP_Private.h"
#include "FreeRTOS_UDP_IP.h"
#include "FreeRTOS_Sockets.h"
#include "NetworkBufferManagement.h"
#include "NetworkInterface.h"

/* Board specific includes */
#include <virtio-net.h>
#include <plic_driver.h>
#include <bsp.h>

#ifdef __CHERI_PURE_CAPABILITY__
#include <cheri/cheri-utility.h>
#endif

static struct virtio_device *global_dev = NULL;
static struct virtio_net *global_vnet = NULL;

static TaskHandle_t EthTaskHandler = NULL;

static void prvTxRxHandlerTask( void *pvParameter )
{
NetworkBufferDescriptor_t *pxBufferDescriptor;
struct virtio_net *vnet = (struct virtio_net *) pvParameter;
size_t xBytesReceived;
/* Used to indicate that xSendEventStructToIPTask() is being called because
of an Ethernet receive event. */
IPStackEvent_t xRxEvent;

    for( ;; )
    {
        /* Wait for the Ethernet MAC interrupt to indicate that another packet
        has been received.  The task notification is used in a similar way to a
        counting semaphore to count Rx events, but is a lot more efficient than
        a semaphore. */

        ulTaskNotifyTake( pdFALSE, portMAX_DELAY );

        /* See how much data was received. */
        xBytesReceived = virtionet_receive_check(vnet);

        while (xBytesReceived > 0)
        {
            /* Allocate a network buffer descriptor that points to a buffer
            large enough to hold the received frame.  As this is the simple
            rather than efficient example the received data will just be copied
            into this buffer. */
            pxBufferDescriptor = pxGetNetworkBufferWithDescriptor( xBytesReceived, 0 );

            if( pxBufferDescriptor != NULL )
            {
                /* pxBufferDescriptor->pucEthernetBuffer now points to an Ethernet
                buffer large enough to hold the received data.  Copy the
                received data into pcNetworkBuffer->pucEthernetBuffer.
                Remember! While is is a simple robust technique –
                it is not efficient. */
                virtionet_read( vnet, (char *) pxBufferDescriptor->pucEthernetBuffer, xBytesReceived );
                pxBufferDescriptor->xDataLength = xBytesReceived;

                /* See if the data contained in the received Ethernet frame needs
                to be processed.  NOTE! It is preferable to do this in
                the interrupt service routine itself, which would remove the need
                to unblock this task for packets that don’t need processing. */
                if( eConsiderFrameForProcessing( pxBufferDescriptor->pucEthernetBuffer )
                                                                      == eProcessBuffer )
                {
                    /* The event about to be sent to the TCP/IP is an Rx event. */
                    xRxEvent.eEventType = eNetworkRxEvent;

                    /* pvData is used to point to the network buffer descriptor that
                    now references the received data. */
                    xRxEvent.pvData = ( void * ) pxBufferDescriptor;

                    /* Send the data to the TCP/IP stack. */
                    if( xSendEventStructToIPTask( &xRxEvent, 0 ) == pdFALSE )
                    {
                        /* The buffer could not be sent to the IP task so the buffer
                        must be released. */
                        vReleaseNetworkBufferAndDescriptor( pxBufferDescriptor );

                        /* Make a call to the standard trace macro to log the
                        occurrence. */
                        iptraceETHERNET_RX_EVENT_LOST();
                    }
                    else
                    {
                        /* The message was successfully sent to the TCP/IP stack.
                        Call the standard trace macro to log the occurrence. */
                        iptraceNETWORK_INTERFACE_RECEIVE();
                    }
                }
                else
                {
                    /* The Ethernet frame can be dropped, but the Ethernet buffer
                    must be released. */
                    vReleaseNetworkBufferAndDescriptor( pxBufferDescriptor );
                }
            }
            else
            {
                /* The event was lost because a network buffer was not available.
                Call the standard trace macro to log the occurrence. */
                iptraceETHERNET_RX_EVENT_LOST();
            }

            // Check if new data got received while we were processing the previous one
            xBytesReceived = virtionet_receive_check(vnet);
        }
    }
}
/*-----------------------------------------------------------*/

static int prvNetworkInterfaceInterruptHandler( void* CallBackRef ) {
BaseType_t  pxHigherPriorityTaskWoken = 0;
struct virtio_net *vnet = (struct virtio_net *) CallBackRef;

    if ( !vnet || !vnet->driver.running ) {
        FreeRTOS_debug_printf( ("Eth driver isn't functional yet\n") );
        return 0;
    }

    virtionet_handle_interrupt(vnet);

    if (EthTaskHandler == NULL) {
      return pxHigherPriorityTaskWoken;
    }

    // Uncomment if/when the driver is interrupt-based
    vTaskNotifyGiveFromISR(EthTaskHandler, &pxHigherPriorityTaskWoken);

    return pxHigherPriorityTaskWoken;
}
/*-----------------------------------------------------------*/

BaseType_t xNetworkInterfaceInitialise( void )
{
void *virtio_mmio_base = (void *) VIRTIO_NET_MMIO_ADDRESS;
struct virtio_net *vnet = NULL;
extern plic_instance_t Plic;

    #ifdef __CHERI_PURE_CAPABILITY__
        virtio_mmio_base = cheri_build_data_cap((ptraddr_t) virtio_mmio_base,
                                            VIRTIO_NET_MMIO_SIZE,
                                            __CHERI_CAP_PERMISSION_GLOBAL__ |
                                            __CHERI_CAP_PERMISSION_PERMIT_LOAD__ |
                                            __CHERI_CAP_PERMISSION_PERMIT_STORE__);
    #endif

    global_dev = virtio_setup_vd(virtio_mmio_base);

    if (!global_dev)
        return pdFAIL;

    vnet = virtionet_open(global_dev);
    if (!vnet)
        return pdFAIL;

    global_vnet = vnet;

  // Bring back when an interrupt-based driver is supported/used
#if 1
    /*
     * Initialize the interrupt controller and connect the ISR
     */
    PLIC_set_priority(&Plic, VIRTIO_NET_PLIC_INTERRUPT_ID, VIRTIO_NET_PLIC_INTERRUPT_PRIO);

    if(!PLIC_register_interrupt_handler(&Plic, VIRTIO_NET_PLIC_INTERRUPT_ID,
                                        &prvNetworkInterfaceInterruptHandler,
                                        vnet))
        return pdFAIL;
#endif

    /* Create a task to handle receive events */
    xTaskCreate(prvTxRxHandlerTask,
                "TxRxHandler",
                configMINIMAL_STACK_SIZE * 2U,
                vnet,
                configMAX_PRIORITIES - 1,
                &EthTaskHandler);

	return pdPASS;
}
/*-----------------------------------------------------------*/

static BaseType_t xNetworkInterfaceDestroy( void )
{
    FreeRTOS_debug_printf( ("xNetworkInterfaceDestroy\r\n") );

    virtionet_close(global_vnet);

    return pdPASS;
}
/*-----------------------------------------------------------*/

BaseType_t xNetworkInterfaceOutput( NetworkBufferDescriptor_t * const pxNetworkBuffer, BaseType_t xReleaseAfterSend )
{
    FreeRTOS_debug_printf( ("xNetworkInterfaceOutput\r\n") );

    if (!virtionet_write(global_vnet, (char *) pxNetworkBuffer->pucEthernetBuffer, pxNetworkBuffer->xDataLength))
    {
        return pdFAIL;
    }

    if ( xReleaseAfterSend != pdFALSE )
    {
        vReleaseNetworkBufferAndDescriptor( pxNetworkBuffer );
    }

    return pdPASS;
}
/*-----------------------------------------------------------*/

BaseType_t xGetPhyLinkStatus( void )
{
    return pdPASS;
}
/*-----------------------------------------------------------*/
