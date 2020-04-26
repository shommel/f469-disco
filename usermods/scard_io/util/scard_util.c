/**
 * @file       scard_util.c
 * @brief      Utility smart card functions
 * @author     Mike Tolkachev <contact@miketolkachev.dev>
 * @copyright  Copyright 2020 Crypto Advance GmbH. All rights reserved.
 */

#include "py/obj.h"
#include "py/runtime.h"
#include "py/builtin.h"
#include "py/mperrno.h"
#include "py/mphal.h"
#include "uart.h"
#include "t1_protocol.h"

USART_TypeDef* scard_get_usart_handle(mp_int_t usart_id) {
  switch(usart_id) {
  #ifdef USART0
    case 0:
      return USART0;
  #endif
  #ifdef USART1
    case 1:
      return USART1;
  #endif
  #ifdef USART2
    case 2:
      return USART2;
  #endif
  #ifdef USART3
    case 3:
      return USART3;
  #endif
  #ifdef USART4
    case 4:
      return USART4;
  #endif
  #ifdef USART5
    case 5:
      return USART5;
  #endif
  #ifdef USART6
    case 6:
      return USART6;
  #endif
  #ifdef USART7
    case 7:
      return USART7;
  #endif
  #ifdef USART8
    case 8:
      return USART8;
  #endif
  #ifdef USART9
    case 9:
      return USART9;
  #endif
  #ifdef USART10
    case 10:
      return USART10;
  #endif
    default:
      return NULL;
  }
  return NULL;
}

bool scard_init_usart(SMARTCARD_HandleTypeDef* sc_handle,
                      USART_TypeDef* usart_handle) {
  sc_handle->Instance = usart_handle;
  sc_handle->Init.WordLength = SMARTCARD_WORDLENGTH_9B;
  sc_handle->Init.StopBits = SMARTCARD_STOPBITS_1_5;
  sc_handle->Init.Parity = SMARTCARD_PARITY_EVEN;
  sc_handle->Init.Mode = SMARTCARD_MODE_TX_RX;
  sc_handle->Init.BaudRate = 10081; // Starting baudrate = 3,5MHz / 372etu
  sc_handle->Init.CLKPolarity = SMARTCARD_POLARITY_LOW;
  sc_handle->Init.CLKPhase = SMARTCARD_PHASE_1EDGE;
  sc_handle->Init.CLKLastBit = SMARTCARD_LASTBIT_ENABLE;
  sc_handle->Init.Prescaler = SMARTCARD_PRESCALER_SYSCLK_DIV12;
  sc_handle->Init.GuardTime = 16;
  sc_handle->Init.NACKState = SMARTCARD_NACK_DISABLE;

  return HAL_OK == HAL_SMARTCARD_Init(sc_handle);
}
