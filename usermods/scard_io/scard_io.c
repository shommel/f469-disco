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
#include "timer.h"
#include "uart.h"
#include "t1_protocol.h"
#include "util/scard_util.h"

/// T=0 protocol identifier
#define T0_PROTOCOL                     1U
/// T=1 protocol identifier
#define T1_PROTOCOL                     2U

/// Low state of pin
#define LOW                             (0U)
/// High state of pin
#define HIGH                            (1U)

/// Pin descriptor
typedef struct pin_dsc_ {
  mp_hal_pin_obj_t pin;
  unsigned int invert : 1;
} pin_dsc_t;

/// Object of _scard.connection class
typedef struct connection_obj_ {
  mp_obj_base_t base;
  mp_int_t usart_id;
  bool is_active;
  USART_TypeDef* usart_handle;
  SMARTCARD_HandleTypeDef sc_obj;
  pin_dsc_t rst_pin;
  pin_dsc_t pres_pin;
  pin_dsc_t pwr_pin;
  mp_obj_t handler;
  uint8_t protocol;
} connection_obj_t;

// Pin class variables
#if defined(DEBUG) && defined(SCARD_DEBUG)
  STATIC bool scard_class_debug = SCARD_DEBUG ? true : false;
#else
  #define scard_class_debug (0)
#endif

extern const mp_obj_type_t scard_connection_type;

STATIC inline void scard_pin_write(pin_dsc_t* p_pin, uint8_t state) {
  mp_hal_pin_write(p_pin->pin, state ^ p_pin->invert);
}

STATIC inline uint8_t scard_pin_read(pin_dsc_t* p_pin) {
  uint32_t state = mp_hal_pin_read(p_pin->pin) ? 1U : 0U;
  return state ^ p_pin->invert;
}

STATIC pin_dsc_t scard_pin(mp_obj_t user_obj, mp_int_t polarity, bool output,
                           uint8_t def_state) {
  pin_dsc_t pin = {
    .pin = pin_find(user_obj),
    .invert = polarity ? 0U : 1U
  };
  uint32_t mode = output ? MP_HAL_PIN_MODE_OUTPUT : MP_HAL_PIN_MODE_INPUT;
  uint32_t input_pull = polarity ? MP_HAL_PIN_PULL_DOWN : MP_HAL_PIN_PULL_UP;
  uint32_t pull = output ? MP_HAL_PIN_PULL_NONE : input_pull;
  if(output) {
    scard_pin_write(&pin, def_state);
  }
  mp_hal_pin_config(pin.pin, mode, pull, 0U);
}

STATIC inline pin_dsc_t scard_pin_in(mp_obj_t user_obj, mp_int_t polarity) {
  return scard_pin(user_obj, polarity, false, 0U);
}

STATIC inline pin_dsc_t scard_pin_out(mp_obj_t user_obj, mp_int_t polarity,
                                      uint8_t def_state) {
  return scard_pin(user_obj, polarity, true, def_state);
}

STATIC inline void scard_raise_hw_error(const char* message) {
  mp_raise_msg(&mp_type_OSError, message);
}


STATIC void scard_create_timer(connection_obj_t* self) {
  // TODO: create timer here

  // Use pyb_timer_type.make_new()
}

STATIC void scard_connection_init(connection_obj_t* self, size_t n_args,
                                  const mp_obj_t *pos_args, mp_map_t *kw_args) {
  // Parse arguments
  static const mp_arg_t allowed_args[] = {
    { MP_QSTR_io_pin,   MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
    { MP_QSTR_clk_pin,  MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
    { MP_QSTR_rst_pin,  MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
    { MP_QSTR_pres_pin, MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
    { MP_QSTR_pwr_pin,  MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
    { MP_QSTR_handler,  MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
    { MP_QSTR_protocol, MP_ARG_KW_ONLY  | MP_ARG_INT, {.u_int = 0}           },
    { MP_QSTR_rst_pol,  MP_ARG_KW_ONLY  | MP_ARG_INT, {.u_int = 0}           },
    { MP_QSTR_pres_pol, MP_ARG_KW_ONLY  | MP_ARG_INT, {.u_int = 0}           },
    { MP_QSTR_pwr_pol,  MP_ARG_KW_ONLY  | MP_ARG_INT, {.u_int = 0}           }
  };
  struct {
    mp_arg_val_t io_pin, clk_pin, rst_pin, pres_pin, pwr_pin, handler, protocol,
                 rst_pol, pres_pol, pwr_pol;
  } args;
  mp_arg_parse_all( n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args),
                    allowed_args, (mp_arg_val_t*)&args );

  // Initialize USART periferal in Smartcard mode
  if(!scard_init_usart(&self->sc_obj, self->usart_handle)) {
    scard_raise_hw_error("failed to initialize USART");
  }

  // Configure USART pins
  bool ok = mp_hal_pin_config_alt(pin_find(args.io_pin.u_obj),
                                  MP_HAL_PIN_MODE_ALT, MP_HAL_PIN_PULL_UP,
                                  AF_FN_USART, self->usart_id);
  ok = ok && mp_hal_pin_config_alt(pin_find(args.clk_pin.u_obj),
                                   MP_HAL_PIN_MODE_ALT, MP_HAL_PIN_PULL_UP,
                                   AF_FN_USART, self->usart_id);
  if(!ok) {
    scard_raise_hw_error("failed to configure USART pins");
  }

  // Configure GPIO pins
  self->rst_pin = scard_pin_out(args.rst_pin.u_obj, args.rst_pol.u_int, 1U);
  self->pres_pin = scard_pin_in(args.pres_pin.u_obj, args.pres_pol.u_int);
  self->pwr_pin = scard_pin_out(args.pwr_pin.u_obj, args.pwr_pol.u_int, 1U);

  // Save handler
  if(mp_obj_is_callable(args.handler.u_obj)) {
    self->handler = args.handler.u_obj;
  } else {
    mp_raise_ValueError("handler must be callable");
  }

  // Initialize timer
  scard_create_timer(self);


  // TODO: Initialize T=1 protocol
  // TODO: Make connection active
}

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
//    - *pres_pin* specifies the card presence detect pin to use
//    - *pwr_pin* specifies the pin controling power to the card
//    - *handler* a function to be called on every event
//
//  Optional keyword parameters:
//
//    - *protocol* protocol identifier, by default T1_protocol
//    - *rst_pol* RST polarity, active low by default
//    - *pres_pol* polarity of card presence detect pin, active low by default
//    - *pwr_pol* polarity of power control pin, active low by default
//
STATIC mp_obj_t connection_make_new(const mp_obj_type_t *type, size_t n_args,
                                    size_t n_kw, const mp_obj_t *args) {
  // Check arguments
  enum {
    ARG_usart_id = 0, ARG_io_pin, ARG_clk_pin, ARG_rst_pin, ARG_pres_pin,
    ARG_pwr_pin, ARG_handler,
    ARG_n_min // Minimal number of arguments
  };
  mp_arg_check_num(n_args, n_kw, ARG_n_min, MP_OBJ_FUN_ARGS_MAX, true);

  // Create a new object
  connection_obj_t* self = NULL;
  if(mp_obj_is_int(args[ARG_usart_id])) {
    mp_int_t usart_id = mp_obj_get_int(args[ARG_usart_id]);
    // Get USART handle for a given usart_id
    USART_TypeDef* usart_handle = scard_get_usart_handle(usart_id);
    if(usart_handle) {
      // Allocate a new object and fill basic parameters
      self = m_new_obj_with_finaliser(connection_obj_t);
      self->base.type = &scard_connection_type;
      self->usart_id = usart_id;
      self->is_active = false;
      self->usart_handle = usart_handle;
      // Initialize object
      mp_map_t kw_args;
      mp_map_init_fixed_table(&kw_args, n_kw, args + n_args);
      scard_connection_init(self, n_args - 1, args + 1, &kw_args);
    } else {
      nlr_raise(mp_obj_new_exception_msg_varg(
        &mp_type_ValueError, "USART%d doesn't exist", usart_id));
    }
  } else {
    mp_raise_TypeError("usart_id is not an integer");
  }

  return MP_OBJ_FROM_PTR(self);
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

const mp_obj_type_t scard_connection_type = {
  { &mp_type_type },
  .name = MP_QSTR_connection,
  .make_new = connection_make_new,
  .locals_dict = (void*)&connection_locals_dict,
};

STATIC const mp_rom_map_elem_t scard_io_module_globals_table[] = {
  { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR__scard_io) },
  { MP_ROM_QSTR(MP_QSTR_connection), MP_ROM_PTR(&scard_connection_type) },
};
STATIC MP_DEFINE_CONST_DICT(scard_io_module_globals, scard_io_module_globals_table);

// Define module object
const mp_obj_module_t scard_io_user_cmodule = {
  .base = { &mp_type_module },
  .globals = (mp_obj_dict_t*)&scard_io_module_globals,
};

// Register the module to make it available in Python
MP_REGISTER_MODULE(MP_QSTR__scard_io, scard_io_user_cmodule, MODULE_SCARD_ENABLED);
