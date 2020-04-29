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
#include "py/smallint.h"
#include "modmachine.h"
#include "uart.h"
#include "t1_protocol.h"
#include "scard_io.h"
#include "scard_util.h"

// Timer period in milliseconds
#define TIMER_PERIOD_MS                 (10U)

/// Protocol identifier
typedef enum protocol_ {
  T0_protocol = 1, ///< T=0 protocol identifier
  T1_protocol = 2  ///< T=1 protocol identifier
} protocol_t;

/// Handle of a protocol
typedef union proto_handle_ {
  t1_inst_t* t1; ///< Pointer to instance of T=1 protocol
  void* any;     ///< Abstract handle to any protocol
} proto_handle_t;

/// Object of _scard.connection class
typedef struct connection_obj_ {
  mp_obj_base_t base;             ///< Pointer to type of base class
  bool is_active;                 ///< State: true - alive, false - closed
  scard_handle_t sc_handle;    ///< Handle of smart card interface
  scard_pin_dsc_t rst_pin;        ///< RST pin descriptor
  scard_pin_dsc_t pres_pin;       ///< Card presence pin descriptor
  scard_pin_dsc_t pwr_pin;        ///< PWR pin descriptor
  mp_obj_t handler;               ///< Event handler, a callable object
  protocol_t protocol;            ///< Protocol
  mp_obj_t timer;                 ///< Timer object
  mp_uint_t prev_ticks_ms;        ///< Previous value of millisecond ticks
  proto_handle_t proto_handle;    ///< Handle to used protocol
} connection_obj_t;

#if defined(DEBUG) && defined(SCARD_DEBUG)
  /// Flag variable enabling debug output
  STATIC bool scard_class_debug = SCARD_DEBUG ? true : false;
#else
  /// Disables debug output forever
  #define scard_class_debug (0)
#endif

/// Type information for _scard.connection class
const mp_obj_type_t scard_connection_type;

/**
 * Timer task
 * @param self        instance of _scard.connection class
 * @param elapsed_ms  time in milliseconds passed since previous call
 */
static void connection_timer_task(connection_obj_t* self, uint32_t elapsed_ms) {
  // TODO: implement timer task
  __asm volatile(" nop");
  (void)self;
}

/**
 * Local implementation of utime.ticks_diff()
 *
 * Formula taken from micropython/extmod/utime_mphal.c
 * @param end    ending time
 * @param start  starting time
 * @return       ticks difference between end and start values
 */
static inline mp_int_t ticks_diff(mp_uint_t end, mp_uint_t start) {
  // Optimized formula avoiding if conditions. We adjust difference "forward",
  // wrap it around and adjust back.
  mp_int_t diff = ( (end - start + MICROPY_PY_UTIME_TICKS_PERIOD / 2) &
                    (MICROPY_PY_UTIME_TICKS_PERIOD - 1) ) -
                  MICROPY_PY_UTIME_TICKS_PERIOD / 2;
  return diff;
}

/**
 * __call__ special method running background tasks, usually by timer
 * @param self_in  instance of _scard.connection class casted to mp_obj_t
 * @param n_args   number of arguments
 * @param n_kw     number of keywords arguments
 * @param args     array containing arguments
 * @return         None
 */
STATIC mp_obj_t connection_call(mp_obj_t self_in, size_t n_args, size_t n_kw,
                                const mp_obj_t *args) {
  connection_obj_t* self = (connection_obj_t*)self_in;

  if(self->is_active) {
    mp_uint_t ticks_ms = mp_hal_ticks_ms();
    connection_timer_task(self, ticks_diff(ticks_ms, self->prev_ticks_ms));
    self->prev_ticks_ms = ticks_ms;
  }

  return mp_const_none;
}

/**
 * Creates a new machine.Timer object for background tasks
 * @param self      instance of _scard.connection class
 * @param timer_id  numerical timer identifier
 * @return          new instance of machine.Timer
 */
static mp_obj_t create_timer(connection_obj_t* self, mp_int_t timer_id) {
  mp_obj_t args[] =  {
    // Positional arguments
    MP_OBJ_NEW_SMALL_INT(timer_id), // id
    // Keyword arguments
    MP_OBJ_NEW_QSTR(MP_QSTR_callback), self,
    MP_OBJ_NEW_QSTR(MP_QSTR_period), MP_OBJ_NEW_SMALL_INT(TIMER_PERIOD_MS)
  };
  return machine_timer_type.make_new(&machine_timer_type, 1, 2, args);
}

/**
 * T=1 protocol: callback function that outputs bytes to serial port
 *
 * This function may be either blocking or non-blocking, but latter is
 * recommended for completely asynchronous operation.
 * IMPORTANT: Calling API functions of the protocol implementation from this
 * callback function may cause side effects and must be avoided!
 * @param buf         buffer containing data to transmit
 * @param len         length of data block in bytes
 * @param p_user_prm  user defined parameter
 */
static bool t1_cb_serial_out(const uint8_t* buf, size_t len, void* p_user_prm) {
  connection_obj_t* self = (connection_obj_t*)p_user_prm;
  (void)self;
  // TODO: implement

  return true;
}

/**
 * T=1 protocol: callback function that handles protocol events
 *
 * This protocol implementation guarantees that event handler is always called
 * just before termination of any external API function. It allows to call
 * safely any other API function(s) within user handler code.
 * Helper function t1_is_error_event() may be used to check if an occurred event
 * is an error.
 * @param ev_code     event code
 * @param ev_prm      event parameter depending on event code, typically NULL
 * @param p_user_prm  user defined parameter
 */
static void t1_cb_handle_event(t1_ev_code_t ev_code, const void* ev_prm,
                               void* p_user_prm) {
  connection_obj_t* self = (connection_obj_t*)p_user_prm;
  (void)self;
  // TODO: implement
}

/**
 * Initializes smart card protocol
 * @param self         instance of _scard.connection class
 * @param protocol_id  numerical protocol identifier
 */
static void protocol_init(connection_obj_t* self, mp_int_t protocol_id) {
  self->proto_handle.any = NULL;
  bool success = false;

  switch(protocol_id) {
    case T1_protocol:
      self->proto_handle.t1 = m_new0(t1_inst_t, 1);
      success = t1_init( self->proto_handle.t1, t1_cb_serial_out,
                         t1_cb_handle_event, self );
      if(!success) {
        scard_raise_internal_error("T=1 protocol initialization failed");
      }
      break;

    default:
      mp_raise_ValueError("unsupported protocol");
  }
  self->protocol = (protocol_t)protocol_id;
}

/**
 * Releases protocol context
 * @param self  instance of _scard.connection class
 */
static inline void protocol_deinit(connection_obj_t* self) {
  if(self->proto_handle.any != NULL) {
    if(self->protocol == T1_protocol) {
      m_del(t1_inst_t, self->proto_handle.t1, 1);
      self->proto_handle.t1 = NULL;
    }
  }
}

/**
 * Resets protocol
 * @param self  instance of _scard.connection class
 */
static inline void protocol_reset(connection_obj_t* self) {
  if(self->proto_handle.any != NULL) {
    if(self->protocol == T1_protocol) {
      t1_reset(self->proto_handle.t1, true);
    }
  }
}

/**
 * Initializes a newly created _scard.connection object
 * @param self      instance of _scard.connection class
 * @param n_args    number of arguments
 * @param pos_args  array of positional arguments
 * @param kw_args   map of keyword arguments
 */
static void scard_connection_init(connection_obj_t* self, size_t n_args,
                                  const mp_obj_t* pos_args, mp_map_t* kw_args) {
  // Parse arguments
  const mp_obj_t def_timer_id = MP_OBJ_NEW_SMALL_INT(-1);
  static const mp_arg_t allowed_args[] = {
    { MP_QSTR_usart_id,  MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
    { MP_QSTR_io_pin,    MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
    { MP_QSTR_clk_pin,   MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
    { MP_QSTR_rst_pin,   MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
    { MP_QSTR_pres_pin,  MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
    { MP_QSTR_pwr_pin,   MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
    { MP_QSTR_handler,   MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
    { MP_QSTR_protocol,  MP_ARG_KW_ONLY  | MP_ARG_INT, {.u_int = T1_protocol} },
    { MP_QSTR_rst_pol,   MP_ARG_KW_ONLY  | MP_ARG_INT, {.u_int = 0}           },
    { MP_QSTR_pres_pol,  MP_ARG_KW_ONLY  | MP_ARG_INT, {.u_int = 0}           },
    { MP_QSTR_pwr_pol,   MP_ARG_KW_ONLY  | MP_ARG_INT, {.u_int = 0}           },
    { MP_QSTR_timer_id,  MP_ARG_KW_ONLY  | MP_ARG_OBJ, {.u_obj = def_timer_id}}
  };
  struct {
    mp_arg_val_t usart_id, io_pin, clk_pin, rst_pin, pres_pin, pwr_pin, handler,
                 protocol, rst_pol, pres_pol, pwr_pol, timer_id;
  } args;
  mp_arg_parse_all( n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args),
                    allowed_args, (mp_arg_val_t*)&args );

  // Initialize smart card hardware interface
  mp_obj_t callback = mp_load_attr(self, MP_QSTR__io_callback);
  self->sc_handle = scard_interface_init(args.usart_id.u_obj, args.io_pin.u_obj,
                                         args.clk_pin.u_obj, callback);
  if(!self->sc_handle) {
    scard_raise_hw_error("failed to initialize USART in smart card mode");
  }

  // Configure GPIO pins
  self->rst_pin = scard_pin_out(args.rst_pin.u_obj, args.rst_pol.u_int, ACT);
  self->pres_pin = scard_pin_in(args.pres_pin.u_obj, args.pres_pol.u_int);
  self->pwr_pin = scard_pin_out(args.pwr_pin.u_obj, args.pwr_pol.u_int, INACT);

  // Save handler
  if(mp_obj_is_callable(args.handler.u_obj)) {
    self->handler = args.handler.u_obj;
  } else {
    mp_raise_TypeError("handler must be callable");
  }

  // Initialize requested smart card protocol
  protocol_init(self, args.protocol.u_int);

  // Save system ticks and initialize timer
  self->prev_ticks_ms = mp_hal_ticks_ms();
  if(mp_obj_is_type(args.timer_id.u_obj, &mp_type_NoneType)) {
    self->timer = MP_OBJ_NULL;
  } else if(mp_obj_is_int(args.timer_id.u_obj)) {
    self->timer = create_timer(self, mp_obj_get_int(args.timer_id.u_obj));
  } else {
    mp_raise_TypeError("timer_id must be integer or None");
  }

  // Make connection active
  self->is_active = true;
}

/**
 * @brief Constructor of _scard.connection
 *
 * Constructor
 * .. class:: connection( usart_id, io_pin, clk_pin, rst_pin, pres_pin,
 *                        pwr_pin, handler, protocol=_scard_io.T1_protocol,
 *                        rst_pol=0, pres_pol=0, pwr_pol=0, timer_id=-1 )
 *
 *  Constructs a connection object with given parameters:
 *
 *    - *usart_id* is a USART identifier, like 3 for USART3
 *    - *io_pin* specifies the bidirectional IO pin to use
 *    - *clk_pin* specifies the CLK output pin to use
 *    - *rst_pin* specifies the RST output pin to use
 *    - *pres_pin* specifies the card presence detect pin to use
 *    - *pwr_pin* specifies the pin controling power to the card
 *    - *handler* a function to be called on every event
 *
 *  Optional keyword parameters:
 *
 *    - *protocol* protocol identifier, by default T1_protocol
 *    - *rst_pol* RST polarity, active low by default
 *    - *pres_pol* polarity of card presence detect pin, active low by default
 *    - *pwr_pol* polarity of power control pin, active low by default
 *    - *timer_id* id of the timer to be used use for background tasks
 *
 * Argument *timer_id* can also be None, denying creation of a separate timer
 * for connection object. In this case, _scard.connection() must be called by
 * user code periodically at least once in 50ms to run background tasks.
 *
 * @param type    pointer to type structure
 * @param n_args  number of arguments
 * @param n_kw    number of keyword arguments
 * @param args    array containing arguments
 * @return        a new instance of _scard.connection class
 */
STATIC mp_obj_t connection_make_new(const mp_obj_type_t* type, size_t n_args,
                                    size_t n_kw, const mp_obj_t* args) {
  // Check arguments
  enum {
    ARG_usart_id = 0, ARG_io_pin, ARG_clk_pin, ARG_rst_pin, ARG_pres_pin,
    ARG_pwr_pin, ARG_handler,
    ARG_n_min // Minimal number of arguments
  };
  mp_arg_check_num(n_args, n_kw, ARG_n_min, MP_OBJ_FUN_ARGS_MAX, true);

  // Create a new object
  connection_obj_t* self = NULL;
  if(scard_interface_exists(args[ARG_usart_id])) {
    // Allocate a new object and fill basic parameters
    self = m_new_obj_with_finaliser(connection_obj_t);
    self->base.type = &scard_connection_type;
    self->is_active = false;
    // Initialize object
    mp_map_t kw_args;
    mp_map_init_fixed_table(&kw_args, n_kw, args + n_args);
    scard_connection_init(self, n_args, args, &kw_args);
  } else {
    mp_raise_ValueError("USART does not exists");
  }

  return MP_OBJ_FROM_PTR(self);
}

/**
 * @brief Resets connection
 *
 * .. method:: connection.reset()
 *
 *  Resets the card and internal buffers and the state of protocol's FSM
 *
 * @param self_in  instance of _scard.connection class
 * @return         None
 */
STATIC mp_obj_t connection_reset(mp_obj_t self_in) {
  connection_obj_t* self = (connection_obj_t*)self_in;
  protocol_reset(self);
  // TODO: Reset card
  return mp_const_none;
}

/**
 * @brief Transmits APDU asynchronously
 *
 * .. method:: connection.transmit()
 *
 *  Transmits APDU asynchronously
 *
 *    - *bytes* bytes of APDU to transmit
 *
 * @param self_in  instance of _scard.connection class
 * @param bytes    APDU to transmit as byte array
 * @return         None
 */
STATIC mp_obj_t connection_transmit(mp_obj_t self_in, mp_obj_t bytes) {
  connection_obj_t* self = (connection_obj_t*)self_in;
  (void)self;
  // TODO: Transmit APDU
  return mp_const_none;
}

/**
 * @brief Checks connection state
 *
 * .. method:: connection.is_active()
 *
 *  Checks connection state, returning True if connection is active
 *
 * @param self  instance of _scard.connection class
 * @return      connection state: True - alive, False - closed
 */
STATIC mp_obj_t connection_is_active(mp_obj_t self) {
  return ((connection_obj_t*)self)->is_active ? mp_const_true : mp_const_false;
}

/**
 * @brief Closes connection releasing hardware resources
 *
 * .. method:: connection.close()
 *
 *  Closes connection releasing hardware resources
 *
 * @param self_in  instance of _scard.connection class
 * @return         None
 */
STATIC mp_obj_t connection_close(mp_obj_t self_in) {
  connection_obj_t* self = (connection_obj_t*)self_in;
  if(self->is_active) {
    self->is_active = false;
    // Deinitialize timer
    if(self->timer) {
      mp_obj_t deinit_fn = mp_load_attr(self->timer, MP_QSTR_deinit);
      (void)mp_call_function_0(deinit_fn);
    }
    self->timer = NULL;
    // Apply reset and remove power
    scard_pin_write(&self->rst_pin, ACT);
    scard_pin_write(&self->pwr_pin, INACT);
    // Release hardware interface
    scard_interface_deinit(self->sc_handle);
    self->sc_handle = NULL;
    // Release protocol context
    protocol_deinit(self);
  }
  return mp_const_none;
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
  { MP_ROM_QSTR(MP_QSTR_T0_protocol),  MP_ROM_INT(T0_protocol) },
  { MP_ROM_QSTR(MP_QSTR_T1_protocol),  MP_ROM_INT(T1_protocol) },
};
STATIC MP_DEFINE_CONST_DICT(connection_locals_dict, connection_locals_dict_table);

const mp_obj_type_t scard_connection_type = {
  { &mp_type_type },
  .name = MP_QSTR_connection,
  .make_new = connection_make_new,
  .call = connection_call,
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
