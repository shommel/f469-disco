/**
 * @file       scard_io.h
 * @brief      MicroPython _scard_io module: common definitions
 * @author     Mike Tolkachev <contact@miketolkachev.dev>
 * @copyright  Copyright 2020 Crypto Advance GmbH. All rights reserved.
 */

#ifndef SCARD_IO_H
/// Avoids multiple inclusion of this file
#define SCARD_IO_H

/// State of a pin
typedef enum scard_pin_state_ {
  INACT = 0, ///< Active state
  ACT   = 1, ///< Inactive state
} scard_pin_state_t;

/**
 * Callback function handling received data
 *
 * This function is called by smart card interface when any data is received
 * from the card. It is guaranteed that this callback function is called from
 * a normal context, not from an interrupt service routine.
 * @param self  instance of a class which method is called
 * @param buf   buffer containing received data
 * @param len   length of data block in bytes
 */
typedef void (*scard_cb_data_rx_t)(mp_obj_t self, const uint8_t* buf,
                                   size_t len);

#include "port_scard_io.h"

#if defined(DEBUG) && defined(SCARD_DEBUG)
  /// Flag variable enabling debug output
  extern bool scard_class_debug;
#else
  /// Disables debug output forever
  #define scard_class_debug (0)
#endif

/**
 * Checks if smart card interface exists with given ID
 *
 * @param iface_id  interface ID (platform-dependant)
 * @return          true if interface exists
 */
extern bool scard_interface_exists(mp_const_obj_t iface_id);

/**
 * Initializes smart card interface
 *
 * @param iface_id    interface ID (platform-dependant)
 * @param io_pin      IO pin
 * @param clk_pin     CLK pin
 * @param cb_data_rx  callback function handling received data
 * @param self        self parameter for callback function
 * @return            handle to smart card interface or NULL if failed
 */
extern scard_handle_t scard_interface_init(mp_const_obj_t iface_id,
                                           mp_obj_t io_pin,
                                           mp_obj_t clk_pin,
                                           scard_cb_data_rx_t cb_data_rx,
                                           mp_obj_t cb_self);

/**
 * Deinitializes smart card interface
 *
 * @param handle  handle to smart card interface
 */
extern void scard_interface_deinit(scard_handle_t handle);

/**
 * Creates a new pin
 * @param user_obj   user-provided pin object
 * @param polarity   polarity: 0 - active low, 1 - active high
 * @param output     if true pin is configured as output
 * @param def_state  default state for an output pin
 * @return           pin descriptor
 */
extern scard_pin_dsc_t scard_pin(mp_obj_t user_obj, mp_int_t polarity,
                                 bool output, scard_pin_state_t def_state);

/**
 * Creates a new input pin
 * @param user_obj  user-provided pin object
 * @param polarity  polarity: 0 - active low, 1 - active high
 * @return          pin descriptor
 */
static inline scard_pin_dsc_t scard_pin_in(mp_obj_t user_obj,
                                           mp_int_t polarity) {
  return scard_pin(user_obj, polarity, false, 0U);
}

/**
 * Creates a new output pin
 * @param user_obj   user-provided pin object
 * @param polarity   polarity: 0 - active low, 1 - active high
 * @param def_state  default state
 * @return           pin descriptor
 */
static inline scard_pin_dsc_t scard_pin_out(mp_obj_t user_obj, mp_int_t polarity,
                                      scard_pin_state_t def_state) {
  return scard_pin(user_obj, polarity, true, def_state);
}

/**
 * Raises an exception for hardware error
 * @param message  message associated with exception
 */
static inline void scard_raise_hw_error(const char* message) {
  mp_raise_msg(&mp_type_OSError, message);
}

/**
 * Raises an exception for internal error
 * @param message  message associated with exception
 */
static inline void scard_raise_internal_error(const char* message) {
  mp_raise_msg(&mp_type_OSError, message);
}

#endif // SCARD_IO_H
