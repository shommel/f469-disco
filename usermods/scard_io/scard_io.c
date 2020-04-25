/**
 * @file       scard_io.c
 * @brief      MicroPython user module for low-level smart card communications
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
#include "util/scard_util.h"

/// T=0 protocol identifier
#define T0_PROTOCOL                     1U
/// T=1 protocol identifier
#define T1_PROTOCOL                     2U

// Pin class variables
#if defined(DEBUG) && defined(SCARD_DEBUG)
  STATIC bool scard_class_debug = SCARD_DEBUG ? true : false;
#else
  #define scard_class_debug (0)
#endif

typedef struct connection_obj_ {
    const pin_obj_t* io_pin;
    const pin_obj_t* clk_pin;
    const pin_obj_t* rst_pin;
    const pin_obj_t* pres_pin;
    uint8_t rst_pol;
    uint8_t pwr_pol;
    mp_obj_t handler;
    uint8_t protocol;
    SMARTCARD_HandleTypeDef sc_obj;
} connection_obj_t;

// Constructor
// .. class:: connection( usart_id, io_pin, clk_pin, rst_pin, pres_pin,
//                        handler, protocol=_scard_io.T1_protocol, rst_pol=0,
//                        pwr_pol=0 )
//
//  Constructs a connection object with given parameters:
//
//    - *usart_id* is a numeric USART identifier, like 3 for USART3
//    - *io_pin* specifies the bidirectional IO pin to use
//    - *clk_pin* specifies the CLK output pin to use
//    - *rst_pin* specifies the RST output pin to use
//    - *pres_pin* specifies the card presence input pin to use
//    - *pwr_pin* specifies the pin controling power to the card
//    - *handler* a function to be called on every event
//
//  Optional keyword parameters:
//
//    - *protocol* protocol identifier, by default T1_protocol
//    - *rst_pol* RST polarity, active low by default
//    - *pwr_pol* polarity of power control pin, active low by default
//
STATIC mp_obj_t connection_make_new(const mp_obj_type_t *type, size_t n_args,
                                    size_t n_kw, const mp_obj_t *args) {
  enum {
    ARG_usart_id = 0,
    ARG_io_pin,
    ARG_clk_pin,
    ARG_rst_pin,
    ARG_pres_pin,
    ARG_handler,
    ARG_n_args_min
  };
  // Check arguments
  mp_arg_check_num(n_args, n_kw, ARG_n_args_min, MP_OBJ_FUN_ARGS_MAX, true);

  // Create a new object
  connection_obj_t* self = m_new0(connection_obj_t, 1);

  // Initialize USART periferal in Smartcard mode
  mp_int_t usart_id = -1;
  if(mp_obj_is_int(args[ARG_usart_id])) {
    usart_id = mp_obj_get_int(args[ARG_usart_id]);
    USART_TypeDef* usart_handle = scard_get_usart_handle(usart_id);
    if(usart_handle) {
      if(!scard_init_usart(&self->sc_obj, usart_handle)) {
        mp_raise_msg(&mp_type_OSError, "failed to initialize USART");
      }
    } else {
      nlr_raise(mp_obj_new_exception_msg_varg(
        &mp_type_ValueError, "USART(%d) doesn't exist", usart_id));
    }
  } else {
    mp_raise_msg(&mp_type_TypeError, "usart_id is not an integer");
  }

  // Find specified pins
  self->io_pin = pin_find(args[ARG_io_pin]);
  self->clk_pin = pin_find(args[ARG_clk_pin]);
  self->rst_pin = pin_find(args[ARG_rst_pin]);
  self->pres_pin = pin_find(args[ARG_pres_pin]);

  // Configure pins
  bool ok = mp_hal_pin_config_alt(self->io_pin, MP_HAL_PIN_MODE_ALT,
                                  MP_HAL_PIN_PULL_UP, AF_FN_USART, usart_id);
  ok = ok && mp_hal_pin_config_alt(self->clk_pin, MP_HAL_PIN_MODE_ALT,
                                    MP_HAL_PIN_PULL_UP, AF_FN_USART, usart_id);


  // TODO: check that pin supports alternative function
  // TODO: Initialize UART pins
  // TODO: Initialize GPIO pins: detect, reset
  // TODO: Save event handler
  // TODO: Initialize timer
  // TODO: Initialize T=1 protocol
  // TODO: Make connection active
  return mp_const_notimplemented;
}

// .. method:: connection.reset()
//
//  Resets the card and internal buffers and the state of protocol's FSM
//
STATIC mp_obj_t connection_reset(mp_obj_t self) {
    //mp_hal_pin_low(self->pin_rst);

  // TODO: Reset card
  // TODO: Reset T=1 protocol
  return mp_const_notimplemented;
}

// .. method:: connection.transmit()
//
//  Transmits APDU asynchronously
//
//    - *bytes* bytes of APDU to transmit
//
STATIC mp_obj_t connection_transmit(mp_obj_t self, mp_obj_t bytes) {
  // TODO: Transmit APDU
  return mp_const_notimplemented;
}

// .. method:: connection.is_active()
//
//  Checks connection state, returning True if connection is active
//
STATIC mp_obj_t connection_is_active(mp_obj_t self) {
  // TODO: Return connection status
  return mp_const_true;
}

// .. method:: connection.close()
//
//  Closes connection releasing hardware resources
//
STATIC mp_obj_t connection_close(mp_obj_t self) {
  // TODO: Release timer
  // TODO: Release UART
  // TODO: Release UART pins
  // TODO: Release GPIO pins: detect, reset
  // TODO: Make connection inactive
  return mp_const_notimplemented;
}

/****************************** MODULE ******************************/

STATIC MP_DEFINE_CONST_FUN_OBJ_1(connection_reset_obj, connection_reset);
STATIC MP_DEFINE_CONST_FUN_OBJ_2(connection_transmit_obj, connection_transmit);
STATIC MP_DEFINE_CONST_FUN_OBJ_1(connection_is_active_obj, connection_is_active);
STATIC MP_DEFINE_CONST_FUN_OBJ_1(connection_close_obj, connection_close);

STATIC const mp_rom_map_elem_t connection_locals_dict_table[] = {
  // Instance methods
  { MP_ROM_QSTR(MP_QSTR_reset),        MP_ROM_PTR(&connection_reset_obj) },
  { MP_ROM_QSTR(MP_QSTR_transmit),     MP_ROM_PTR(&connection_transmit_obj) },
  { MP_ROM_QSTR(MP_QSTR_is_active),    MP_ROM_PTR(&connection_is_active_obj) },
  { MP_ROM_QSTR(MP_QSTR_close),        MP_ROM_PTR(&connection_close_obj) },
  { MP_ROM_QSTR(MP_QSTR___del__),      MP_ROM_PTR(&connection_close_obj) },
  { MP_ROM_QSTR(MP_QSTR___exit__),     MP_ROM_PTR(&connection_close_obj) },

  // Class constants
  { MP_ROM_QSTR(MP_QSTR_T0_protocol),  MP_ROM_INT(T0_PROTOCOL) },
  { MP_ROM_QSTR(MP_QSTR_T1_protocol),  MP_ROM_INT(T1_PROTOCOL) },
};
STATIC MP_DEFINE_CONST_DICT(connection_locals_dict, connection_locals_dict_table);

STATIC const mp_obj_type_t connection_type = {
  { &mp_type_type },
  .name = MP_QSTR_connection,
  .make_new = connection_make_new,
  .locals_dict = (void*)&connection_locals_dict,
};

STATIC const mp_rom_map_elem_t scard_io_module_globals_table[] = {
  { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR__scard_io) },
  { MP_ROM_QSTR(MP_QSTR_connection), MP_ROM_PTR(&connection_type) },
};
STATIC MP_DEFINE_CONST_DICT(scard_io_module_globals, scard_io_module_globals_table);

// Define module object
const mp_obj_module_t scard_io_user_cmodule = {
  .base = { &mp_type_module },
  .globals = (mp_obj_dict_t*)&scard_io_module_globals,
};

// Register the module to make it available in Python
MP_REGISTER_MODULE(MP_QSTR__scard_io, scard_io_user_cmodule, MODULE_SCARD_ENABLED);
