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

/* Demo includes */
#include "modbus_demo.h"
#include "modbus_client.h"

/* Modbus Macaroons includes */
#if defined(MACAROONS_LAYER)
#include "modbus_macaroons.h"
#endif

/*-----------------------------------------------------------*/

/* Helper function for converting to libmodbus formats */
static int convert_string_req(const char *req_string, uint8_t *req);

/*-----------------------------------------------------------*/

/* The structure holding connection information. */
static modbus_t *ctx = NULL;

#if defined(MACAROONS_LAYER)
/* The queue used to communicate Macaroons from client to server. */
extern QueueHandle_t xQueueClientServerMacaroons;

/* The queue used to communicate Macaroons from server to client. */
extern QueueHandle_t xQueueServerClientMacaroons;
#endif

/*-----------------------------------------------------------*/

void prvClientInitialization(char *ip, int port,
                             QueueHandle_t xQueueClientServer,
                             QueueHandle_t xQueueServerClient)
{
  /* initialise the connection */
  ctx = modbus_new_tcp(ip, port);
  if (ctx == NULL)
  {
    fprintf(stderr, "Failed to allocate ctx: %s\n",
            modbus_strerror(errno));
    modbus_free(ctx);
    _exit(0);
  }

#ifdef NDEBUG
  modbus_set_debug(ctx, pdFALSE);
#else
  modbus_set_debug(ctx, pdTRUE);
#endif

  modbus_set_request_queue(ctx, xQueueClientServer);
  modbus_set_response_queue(ctx, xQueueServerClient);
  modbus_set_server(ctx, pdFALSE);

#if defined(MACAROONS_LAYER)
  int rc;
  /**
   * serialise the macaroon and queue it for the client
   *
   * this is a TOFU operation. the first client to dequeue it gets it...
   * */
  rc = initialise_client_macaroon(ctx, xQueueServerClientMacaroons);
  configASSERT(rc != -1);

  printf("client macaroon initialised...\n");
#endif
}

/*-----------------------------------------------------------*/

void vClientTask(void *pvParameters)
{
  /* structure to hold the request and request length */
  modbus_queue_msg_t *pxRequest = (modbus_queue_msg_t *)pvPortMalloc(sizeof(modbus_queue_msg_t));
  pxRequest->msg = (uint8_t *)pvPortMalloc(MODBUS_MAX_STRING_LENGTH * sizeof(uint8_t));
  memset(pxRequest->msg, 0, MODBUS_MAX_STRING_LENGTH * sizeof(uint8_t));
  pxRequest->msg_length = 0;

  /* structure to hold the response and the response length */
  modbus_queue_msg_t *pxResponse = (modbus_queue_msg_t *)pvPortMalloc(sizeof(modbus_queue_msg_t));
  pxResponse->msg = (uint8_t *)pvPortMalloc(MODBUS_MAX_STRING_LENGTH * sizeof(uint8_t));
  memset(pxResponse->msg, 0, MODBUS_MAX_STRING_LENGTH * sizeof(uint8_t));
  pxResponse->msg_length = 0;

  TickType_t xNextWakeTime;

  /* libmodbus variables */
  int rc;
  uint8_t *tab_rp_bits = NULL;
  uint16_t *tab_rp_registers = NULL;
  uint8_t *tab_rp_string = NULL;
  int nb_points;

  /* Remove compiler warning about unused parameter. */
  (void)pvParameters;

  /* Initialise xNextWakeTime - this only needs to be done once. */
  xNextWakeTime = xTaskGetTickCount();

  /* Allocate and initialize the memory to store the bits */
  nb_points = (UT_BITS_NB > UT_INPUT_BITS_NB) ? UT_BITS_NB : UT_INPUT_BITS_NB;
  tab_rp_bits = (uint8_t *)pvPortMalloc(nb_points * sizeof(uint8_t));
  memset(tab_rp_bits, 0, nb_points * sizeof(uint8_t));

  /* Allocate and initialize the memory to store the registers */
  nb_points = (UT_REGISTERS_NB > UT_INPUT_REGISTERS_NB) ?
      UT_REGISTERS_NB : UT_INPUT_REGISTERS_NB;
  tab_rp_registers = (uint16_t *) pvPortMalloc(nb_points * sizeof(uint16_t));
  memset(tab_rp_registers, 0, nb_points * sizeof(uint16_t));

  /* Allocate and initialize the memory to store the string */
  tab_rp_string = (uint8_t *)pvPortMalloc(MODBUS_MAX_STRING_LENGTH * sizeof(uint8_t));
  int nb_chars = convert_string_req((const char *)TEST_STRING, tab_rp_string);

  /**************
   * STRING TESTS
   *************/

  /* WRITE_STRING */
#ifndef NDEBUG
  print_shim_info("modbus_client", __FUNCTION__);
  printf("\nWRITE_STRING\n");
#endif

  rc = modbus_write_string(ctx, tab_rp_string, nb_chars);
  configASSERT(rc != -1);
  vTaskDelayUntil(&xNextWakeTime, mainQUEUE_SEND_FREQUENCY_MS);

  /* READ_STRING */
#ifndef NDEBUG
  print_shim_info("modbus_client", __FUNCTION__);
  printf("\nREAD_STRING\n");
#endif

  memset(tab_rp_string, 0, MODBUS_MAX_STRING_LENGTH * sizeof(uint8_t));
  rc = modbus_read_string(ctx, tab_rp_string);
  configASSERT(rc != -1);
  vTaskDelayUntil(&xNextWakeTime, mainQUEUE_SEND_FREQUENCY_MS);

  /************
   * COIL TESTS
   ***********/

  /* WRITE_SINGLE_COIL */
#ifndef NDEBUG
  print_shim_info("modbus_client", __FUNCTION__);
  printf("\nWRITE_SINGLE_COIL\n");
#endif

#if defined(MACAROONS_LAYER)
  rc = modbus_write_bit_macaroons(ctx, UT_BITS_ADDRESS, ON);
#else
  rc = modbus_write_bit(ctx, UT_BITS_ADDRESS, ON);
#endif
  configASSERT(rc == 1);
  vTaskDelayUntil(&xNextWakeTime, mainQUEUE_SEND_FREQUENCY_MS);

  /* READ_SINGLE_COIL */
#ifndef NDEBUG
  print_shim_info("modbus_client", __FUNCTION__);
  printf("\nREAD_SINGLE_COIL\n");
#endif

#if defined(MACAROONS_LAYER)
  rc = modbus_read_bits_macaroons(ctx, UT_BITS_ADDRESS, 1, tab_rp_bits);
#else
  rc = modbus_read_bits(ctx, UT_BITS_ADDRESS, 1, tab_rp_bits);
#endif
  configASSERT(rc == 1);
  vTaskDelayUntil(&xNextWakeTime, mainQUEUE_SEND_FREQUENCY_MS);

  /* WRITE_MULTIPLE_COILS */
#ifndef NDEBUG
  print_shim_info("modbus_client", __FUNCTION__);
  printf("\nWRITE_MULTIPLE_COILS\n");
#endif
  {
    uint8_t tab_value[UT_BITS_NB];
    modbus_set_bits_from_bytes(tab_value, 0, UT_BITS_NB, UT_BITS_TAB);

#if defined(MACAROONS_LAYER)
    rc = modbus_write_bits_macaroons(ctx, UT_BITS_ADDRESS, UT_BITS_NB, tab_value);
#else
    rc = modbus_write_bits(ctx, UT_BITS_ADDRESS, UT_BITS_NB, tab_value);
#endif
  }
  configASSERT(rc == UT_BITS_NB);
  vTaskDelayUntil(&xNextWakeTime, mainQUEUE_SEND_FREQUENCY_MS);

  /* READ_MULTIPLE_COILS */
#ifndef NDEBUG
  print_shim_info("modbus_client", __FUNCTION__);
  printf("\nREAD_MULTIPLE_COILS\n");
#endif

#if defined(MACAROONS_LAYER)
  rc = modbus_read_bits_macaroons(ctx, UT_BITS_ADDRESS, UT_BITS_NB, tab_rp_bits);
#else
  rc = modbus_read_bits(ctx, UT_BITS_ADDRESS, UT_BITS_NB, tab_rp_bits);
#endif
  configASSERT(rc == UT_BITS_NB);
  vTaskDelayUntil(&xNextWakeTime, mainQUEUE_SEND_FREQUENCY_MS);

  /**********************
   * DISCRETE INPUT TESTS
   *********************/

  /* READ_INPUT_BITS */
#ifndef NDEBUG
  print_shim_info("modbus_client", __FUNCTION__);
  printf("\nREAD_INPUT_BITS\n");
#endif

#if defined(MACAROONS_LAYER)
  rc = modbus_read_input_bits_macaroons(ctx, UT_INPUT_BITS_ADDRESS, UT_INPUT_BITS_NB, tab_rp_bits);
#else
  rc = modbus_read_input_bits(ctx, UT_INPUT_BITS_ADDRESS, UT_INPUT_BITS_NB, tab_rp_bits);
#endif
  configASSERT(rc == UT_INPUT_BITS_NB);

  {
    /* further checks on the returned discrete inputs */
    int idx = 0;
    uint8_t value;
    nb_points = UT_INPUT_BITS_NB;
    while (nb_points > 0) {
        int nb_bits = (nb_points > 8) ? 8 : nb_points;
        value = modbus_get_byte_from_bits(tab_rp_bits, idx*8, nb_bits);
        configASSERT(value == UT_INPUT_BITS_TAB[idx]);

        nb_points -= nb_bits;
        idx++;
    }
  }
  vTaskDelayUntil(&xNextWakeTime, mainQUEUE_SEND_FREQUENCY_MS);

  /************************
   * HOLDING REGISTER TESTS
   ***********************/

  /* WRITE_SINGLE_REGISTER */
#ifndef NDEBUG
  print_shim_info("modbus_client", __FUNCTION__);
  printf("\nWRITE_SINGLE_REGISTER\n");
#endif

#if defined(MACAROONS_LAYER)
  rc = modbus_write_register_macaroons(ctx, UT_REGISTERS_ADDRESS, 0x1234);
#else
  rc = modbus_write_register(ctx, UT_REGISTERS_ADDRESS, 0x1234);
#endif
  configASSERT(rc == 1);
  vTaskDelayUntil(&xNextWakeTime, mainQUEUE_SEND_FREQUENCY_MS);

  /* READ_SINGLE_REGISTER */
#ifndef NDEBUG
  print_shim_info("modbus_client", __FUNCTION__);
  printf("\nREAD_SINGLE_REGISTER\n");
#endif

#if defined(MACAROONS_LAYER)
  rc = modbus_read_registers_macaroons(ctx, UT_REGISTERS_ADDRESS, 1, tab_rp_registers);
#else
  rc = modbus_read_registers(ctx, UT_REGISTERS_ADDRESS, 1, tab_rp_registers);
#endif
  configASSERT(rc == 1);
  configASSERT(tab_rp_registers[0] == 0x1234);
  vTaskDelayUntil(&xNextWakeTime, mainQUEUE_SEND_FREQUENCY_MS);

  /* WRITE_MULTIPLE_REGISTERS */
#ifndef NDEBUG
  print_shim_info("modbus_client", __FUNCTION__);
  printf("\nWRITE_MULTIPLE_REGISTERS\n");
#endif

#if defined(MACAROONS_LAYER)
  rc = modbus_write_registers_macaroons(ctx, UT_REGISTERS_ADDRESS, UT_REGISTERS_NB, UT_REGISTERS_TAB);
#else
  rc = modbus_write_registers(ctx, UT_REGISTERS_ADDRESS, UT_REGISTERS_NB, UT_REGISTERS_TAB);
#endif
  configASSERT(rc == UT_REGISTERS_NB);
  vTaskDelayUntil(&xNextWakeTime, mainQUEUE_SEND_FREQUENCY_MS);

  /* READ_MULTIPLE_REGISTERS */
#ifndef NDEBUG
  print_shim_info("modbus_client", __FUNCTION__);
  printf("\nREAD_MULTIPLE_REGISTERS\n");
#endif

#if defined(MACAROONS_LAYER)
  rc = modbus_read_registers_macaroons(ctx, UT_REGISTERS_ADDRESS, UT_REGISTERS_NB, tab_rp_registers);
#else
  rc = modbus_read_registers(ctx, UT_REGISTERS_ADDRESS, UT_REGISTERS_NB, tab_rp_registers);
#endif
  configASSERT(rc == UT_REGISTERS_NB);
  for (int i=0; i < UT_REGISTERS_NB; i++) {
      configASSERT(tab_rp_registers[i] == UT_REGISTERS_TAB[i]);
  }
  vTaskDelayUntil(&xNextWakeTime, mainQUEUE_SEND_FREQUENCY_MS);

  /* WRITE_AND_READ_REGISTERS */
#ifndef NDEBUG
  print_shim_info("modbus_client", __FUNCTION__);
  printf("\nWRITE_AND_READ_REGISTERS\n");
#endif

  nb_points = (UT_REGISTERS_NB > UT_INPUT_REGISTERS_NB) ? UT_REGISTERS_NB : UT_INPUT_REGISTERS_NB;
  memset(tab_rp_registers, 0, nb_points * sizeof(uint16_t));

#if defined(MACAROONS_LAYER)
  rc = modbus_write_and_read_registers_macaroons(ctx,
                                                 UT_REGISTERS_ADDRESS + 1,
                                                 UT_REGISTERS_NB - 1,
                                                 tab_rp_registers,
                                                 UT_REGISTERS_ADDRESS,
                                                 UT_REGISTERS_NB,
                                                 tab_rp_registers);
#else
  rc = modbus_write_and_read_registers(ctx,
                                       UT_REGISTERS_ADDRESS + 1,
                                       UT_REGISTERS_NB - 1,
                                       tab_rp_registers,
                                       UT_REGISTERS_ADDRESS,
                                       UT_REGISTERS_NB,
                                       tab_rp_registers);
#endif
  configASSERT(rc == UT_REGISTERS_NB);
  configASSERT(tab_rp_registers[0] == UT_REGISTERS_TAB[0]);
  for (int i = 1; i < UT_REGISTERS_NB; i++)
  {
        configASSERT(tab_rp_registers[i] == 0);
  }
  vTaskDelayUntil(&xNextWakeTime, mainQUEUE_SEND_FREQUENCY_MS);

  /**********************
   * INPUT REGISTER TESTS
   *********************/

  /* READ_INPUT_REGISTERS */
#ifndef NDEBUG
  print_shim_info("modbus_client", __FUNCTION__);
  printf("\nREAD_INPUT_REGISTERS\n");
#endif

#if defined(MACAROONS_LAYER)
  rc = modbus_read_input_registers_macaroons(ctx, UT_INPUT_REGISTERS_ADDRESS,
                                             UT_INPUT_REGISTERS_NB, tab_rp_registers);
#else
  rc = modbus_read_input_registers(ctx, UT_INPUT_REGISTERS_ADDRESS,
                                   UT_INPUT_REGISTERS_NB, tab_rp_registers);
#endif
  configASSERT(rc == UT_INPUT_REGISTERS_NB);
  for (int i = 0; i < UT_INPUT_REGISTERS_NB; i++)
  {
    configASSERT(tab_rp_registers[i] == UT_INPUT_REGISTERS_TAB[i]);
  }
  vTaskDelayUntil(&xNextWakeTime, mainQUEUE_SEND_FREQUENCY_MS);

  /*****************************
   * HOLDING REGISTER MASK TESTS
   ****************************/

  /* WRITE_SINGLE_REGISTER */
#ifndef NDEBUG
  print_shim_info("modbus_client", __FUNCTION__);
  printf("\nWRITE_SINGLE_REGISTER\n");
#endif

#if defined(MACAROONS_LAYER)
  rc = modbus_write_register_macaroons(ctx, UT_REGISTERS_ADDRESS, 0x12);
#else
  rc = modbus_write_register(ctx, UT_REGISTERS_ADDRESS, 0x12);
#endif
  configASSERT(rc == 1);
  vTaskDelayUntil(&xNextWakeTime, mainQUEUE_SEND_FREQUENCY_MS);

  /* MASK_WRITE_SINGLE_REGISTER */
#ifndef NDEBUG
  print_shim_info("modbus_client", __FUNCTION__);
  printf("\nMASK_WRITE_SINGLE_REGISTER\n");
#endif

#if defined(MACAROONS_LAYER)
  rc = modbus_mask_write_register_macaroons(ctx, UT_REGISTERS_ADDRESS, 0xF2, 0x25);
#else
  rc = modbus_mask_write_register(ctx, UT_REGISTERS_ADDRESS, 0xF2, 0x25);
#endif
  configASSERT(rc != -1);
  vTaskDelayUntil(&xNextWakeTime, mainQUEUE_SEND_FREQUENCY_MS);

  /* READ_SINGLE_REGISTER */
#ifndef NDEBUG
  print_shim_info("modbus_client", __FUNCTION__);
  printf("\nREAD_SINGLE_REGISTER\n");
#endif

#if defined(MACAROONS_LAYER)
  rc = modbus_read_registers_macaroons(ctx, UT_REGISTERS_ADDRESS, 1, tab_rp_registers);
#else
  rc = modbus_read_registers(ctx, UT_REGISTERS_ADDRESS, 1, tab_rp_registers);
#endif
  configASSERT(rc == 1);
  configASSERT(tab_rp_registers[0] == 0x17);
  vTaskDelayUntil(&xNextWakeTime, mainQUEUE_SEND_FREQUENCY_MS);

  /* Kill the app after the last request. */
  _exit(0);
}

/*-----------------------------------------------------------*/

/**
 * converts a string request to a uint8_t *
 *
 * input format "[FF][FF][FF]..."
 *
 * returns request length
 * */
static int convert_string_req(const char *req_string, uint8_t *req)
{
  int rc = -1;
  char buf[3];

  if (strlen(req_string) % 4 != 0 || strlen(req_string) == 0)
  {
    return rc;
  }

  /**
     * every 4 characters in the string is one hex integer
     * i.e., "[FF]" -> 0xFF
     * */
  for (size_t i = 0; i < (strlen(req_string)) / 4; ++i)
  {
    memcpy(buf, &req_string[i * 4 + 1], 2);
    buf[2] = '\0';
    sscanf(buf, "%hhx", &req[i]);
  }

  req[(strlen(req_string)) / 4] = '\0';

  return (strlen(req_string)) / 4 + 1;
}
