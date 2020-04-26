/**
 * @file       scard_util.h
 * @brief      Utility smart card functions
 * @author     Mike Tolkachev <contact@miketolkachev.dev>
 * @copyright  Copyright 2020 Crypto Advance GmbH. All rights reserved.
 */

#ifndef SCARD_UTIL_H
/// Avoids multiple inclusion of this file
#define SCARD_UTIL_H

/**
 * Returns USART handle for a given USART ID
 * @param usart_id  numerical USART ID, like 3 for USART3
 * @return          USART handle or NULL if USART does not exist
 */
extern USART_TypeDef* scard_get_usart_handle(mp_int_t usart_id);

/**
 * Initializes USART in the Smartcard mode
 * @param sc_handle     handle to smart card interface
 * @param usart_handle  handle to USART peripheral
 * @return              true if successful
 */
extern bool scard_init_usart(SMARTCARD_HandleTypeDef* sc_handle,
                             USART_TypeDef* usart_handle);

#endif // SCARD_UTIL_H