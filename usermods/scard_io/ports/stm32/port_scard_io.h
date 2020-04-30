/**
 * @file       scard_io_port.h
 * @brief      MicroPython _scard_io module: STM32 port definitions
 * @author     Mike Tolkachev <contact@miketolkachev.dev>
 * @copyright  Copyright 2020 Crypto Advance GmbH. All rights reserved.
 */

#ifndef SCARD_IO_PORT_H
/// Avoids multiple inclusion of this file
#define SCARD_IO_PORT_H

/// USART IO control commands used by smart card driver
typedef enum scard_usart_ioctl_cmd_ {
  scusart_init = 0,  ///< Initialize USART
  scusart_deinit     ///< Deinitialize USART
} scard_usart_ioctl_cmd_t;

/// Parameters of IO control function
typedef union scard_usart_ioctl_prm_ {
  void* na;  ///< Not available (default)
} scard_usart_ioctl_prm_t;

/**
 * USART IO control function
 * @param cmd  command
 * @param prm  parameter
 * @return     returned value depends on command
 */
typedef int32_t (*scard_usart_ioctl_t)(scard_usart_ioctl_cmd_t cmd,
                                       scard_usart_ioctl_prm_t prm);

/// USART descriptor
typedef struct scard_usart_dsc_ {
  uint8_t id;                ///< USART identifier, e.g. 3 for USART3
  USART_TypeDef* handle;     ///< USART handle
  IRQn_Type irqn;            ///< USART IRQ number
  scard_usart_ioctl_t ioctl; ///< USART IO control function
} scard_usart_dsc_t;

/// Smart card interface instance and handle
typedef struct scard_inst_ {
  mp_obj_base_t base;                    ///< Pointer to type of base class
  SMARTCARD_HandleTypeDef sc_handle;     ///< HAL Smartcard handle structure
  const scard_usart_dsc_t* p_usard_dsc;  ///< Pointer to USART descriptor
  mp_obj_t machine_uart_obj;             ///< machine.UART object used for IO
  pyb_uart_obj_t* uart_obj;              ///< Underlying UART object
  scard_cb_data_rx_t cb_data_rx;         ///< Callback for received data
  mp_obj_t cb_self;                      ///< Self parameter for callback(s)
} scard_inst_t, *scard_handle_t;

/// Pin descriptor
typedef struct scard_pin_dsc_ {
  mp_hal_pin_obj_t pin;     ///< Pin object
  unsigned int invert : 1;  ///< Logic: 0 - normal, 1 - inverted
} scard_pin_dsc_t;

/**
 * Writes state to output register of a pin
 * @param p_pin  pointer to pin descriptor
 * @param state  pin state
 */
static inline void scard_pin_write(scard_pin_dsc_t* p_pin, scard_pin_state_t state) {
  mp_hal_pin_write(p_pin->pin, (unsigned int)state ^ p_pin->invert);
}

/**
 * Returns state of a pin
 * @param p_pin  pointer to pin descriptor
 */
static inline scard_pin_state_t scard_pin_read(scard_pin_dsc_t* p_pin) {
  uint32_t state = mp_hal_pin_read(p_pin->pin) ? 1U : 0U;
  return (scard_pin_state_t)(state ^ p_pin->invert);
}

#endif // SCARD_IO_PORT_H
