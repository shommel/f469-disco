/**
 * @file       scard_io_stm32.c
 * @brief      MicroPython _scard_io module: STM32 port
 * @author     Mike Tolkachev <contact@miketolkachev.dev>
 * @copyright  Copyright 2020 Crypto Advance GmbH. All rights reserved.
 */

#include <stdio.h> // TODO: remove
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

/// Maximum number of interface instances = maximum number of USARTs
#define MAX_INSTANCES                   (11U)
/// Length of receive buffer
#define RX_BUF_LEN                      (270)

/// Type information for smart card interface instance
const mp_obj_type_t scard_inst_type;

/// Type of key used for callback table (handle of smart card driver)
typedef SMARTCARD_HandleTypeDef* callback_key_t;

/// One record in callback table
typedef struct callback_rec_ {
  callback_key_t key;  ///< Key associated with callback object
  mp_obj_t callback;   ///< Callback object
} callback_rec_t;

/// Callback table
typedef struct callback_table_ {
  callback_rec_t values[MAX_INSTANCES]; ///< Records
  int n_values;                         ///< Number of values
} callback_table_t;

/// Statically allocated table to map smart card handles to callback objects
static callback_table_t callback_table = { .n_values = 0 };

int32_t usart0_ioctl(scard_usart_ioctl_cmd_t cmd, scard_usart_ioctl_prm_t prm) {
  return 0;
}

int32_t usart1_ioctl(scard_usart_ioctl_cmd_t cmd, scard_usart_ioctl_prm_t prm) {
  return 0;
}

int32_t usart2_ioctl(scard_usart_ioctl_cmd_t cmd, scard_usart_ioctl_prm_t prm) {
  return 0;
}

int32_t usart3_ioctl(scard_usart_ioctl_cmd_t cmd, scard_usart_ioctl_prm_t prm) {
  return 0;
}

int32_t usart4_ioctl(scard_usart_ioctl_cmd_t cmd, scard_usart_ioctl_prm_t prm) {
  return 0;
}

int32_t usart5_ioctl(scard_usart_ioctl_cmd_t cmd, scard_usart_ioctl_prm_t prm) {
  return 0;
}

int32_t usart6_ioctl(scard_usart_ioctl_cmd_t cmd, scard_usart_ioctl_prm_t prm) {
  return 0;
}

int32_t usart7_ioctl(scard_usart_ioctl_cmd_t cmd, scard_usart_ioctl_prm_t prm) {
  return 0;
}

int32_t usart8_ioctl(scard_usart_ioctl_cmd_t cmd, scard_usart_ioctl_prm_t prm) {
  return 0;
}

int32_t usart9_ioctl(scard_usart_ioctl_cmd_t cmd, scard_usart_ioctl_prm_t prm) {
  return 0;
}

int32_t usart10_ioctl(scard_usart_ioctl_cmd_t cmd, scard_usart_ioctl_prm_t prm) {
  return 0;
}

/// Table of USART descriptors
static const scard_usart_dsc_t usart_dsc_table[] = {
#ifdef USART0
  { .id = 0U, .handle = USART0, .irqn = USART0_IRQn, .ioctl = &usart0_ioctl },
#endif
#ifdef USART1
  { .id = 1U, .handle = USART1, .irqn = USART1_IRQn, .ioctl = &usart1_ioctl },
#endif
#ifdef USART2
  { .id = 2U, .handle = USART2, .irqn = USART2_IRQn, .ioctl = &usart2_ioctl },
#endif
#ifdef USART3
  { .id = 3U, .handle = USART3, .irqn = USART3_IRQn, .ioctl = &usart3_ioctl },
#endif
#ifdef USART4
  { .id = 4U, .handle = USART4, .irqn = USART4_IRQn, .ioctl = &usart4_ioctl },
#endif
#ifdef USART5
  { .id = 5U, .handle = USART5, .irqn = USART5_IRQn, .ioctl = &usart5_ioctl },
#endif
#ifdef USART6
  { .id = 6U, .handle = USART6, .irqn = USART6_IRQn, .ioctl = &usart6_ioctl },
#endif
#ifdef USART7
  { .id = 7U, .handle = USART7, .irqn = USART7_IRQn, .ioctl = &usart7_ioctl },
#endif
#ifdef USART8
  { .id = 8U, .handle = USART8, .irqn = USART8_IRQn, .ioctl = &usart8_ioctl },
#endif
#ifdef USART9
  { .id = 9U, .handle = USART9, .irqn = USART9_IRQn, .ioctl = &usart9_ioctl },
#endif
#ifdef USART10
  { .id = 10U, .handle = USART10, .irqn = USART10_IRQn, .ioctl = &usart10_ioctl},
#endif
  { .handle = NULL } // Terminating record
};

/**
 * Finds USART descriptor in the table
 *
 * @param usart_id  USART identifier
 * @return          pointer to constant USART descriptor
 */
static const scard_usart_dsc_t* find_descriptor(mp_int_t usart_id) {
  const scard_usart_dsc_t* p_dsc = usart_dsc_table;
  while(p_dsc->handle) {
    if(p_dsc->id == usart_id) {
      return p_dsc;
    }
    ++p_dsc;
  }
  return NULL;
}

/**
 * Finds index of callback record in table
 *
 * WARNING: This function does not disable interrupts, so use carefully
 *
 * @param key  key used to access callback record
 * @return     index of callback record or -1 if not found
 */
static inline int find_callback_idx(callback_key_t key) {
  callback_rec_t* p_rec = callback_table.values;
  for(int i = 0U; i < callback_table.n_values; ++i) {
    if(p_rec->key == key) {
      return i;
    }
  }
  return -1;
}

/**
 * Adds callback object to global callback table
 *
 * @param key       key used to access callback record
 * @param callback  callback record
 * @return          true if successful
 */
static bool add_callback(callback_key_t key, mp_obj_t callback) {
  mp_uint_t irq_state = disable_irq();
  if(callback_table.n_values < MAX_INSTANCES) {
    if(find_callback_idx(key) < 0) {
      callback_table.values[callback_table.n_values].key = key;
      callback_table.values[callback_table.n_values++].callback = callback;
      enable_irq(irq_state);
      return true;
    }
  }
  enable_irq(irq_state);
  return false;
}

/**
 * Removes callback object from global callback table (if found)
 *
 * @param key  key used to access callback record
 */
static void remove_callback(callback_key_t key) {
  mp_uint_t irq_state = disable_irq();
  int idx = find_callback_idx(key);
  if(idx >= 0) {
    for(int i = idx; i < callback_table.n_values - 1; ++i) {
      callback_table.values[i] = callback_table.values[i + 1];
    }
    --callback_table.n_values;
  }
  enable_irq(irq_state);
}

bool scard_interface_exists(mp_const_obj_t iface_id) {
  if(mp_obj_is_int(iface_id)) {
    mp_int_t usart_id = mp_obj_get_int(iface_id);
    return find_descriptor(usart_id) != NULL;
  }
  return false;
}

/**
 * Initializes USART in smart card mode
 * @param sc_handle     handle of smart card low-level driver
 * @param usart_handle  USART handle
 * @return              true if successful
 */
static bool init_smartcard(SMARTCARD_HandleTypeDef* sc_handle,
                           USART_TypeDef* usart_handle) {
  // TODO: Enable the USARTx interface clock.
  // TODO: NVIC configuration
  // (+++) Configure the USARTx interrupt priority.
  // (+++) Enable the NVIC USART IRQ handle.
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

/**
 * Initializes pins of USART
 * @param io_pin    IO pin
 * @param clk_pin   CLK pin
 * @param usart_id  USART identifier
 * @return          true if successful
 */
static bool init_pins(mp_obj_t io_pin, mp_obj_t clk_pin, uint8_t usart_id) {
  // Configure USART pins
  bool ok = mp_hal_pin_config_alt(pin_find(io_pin),
                                  MP_HAL_PIN_MODE_ALT, MP_HAL_PIN_PULL_UP,
                                  AF_FN_USART, usart_id);
  ok = ok && mp_hal_pin_config_alt(pin_find(clk_pin),
                                   MP_HAL_PIN_MODE_ALT, MP_HAL_PIN_PULL_UP,
                                   AF_FN_USART, usart_id);
  return ok;
}

/**
 * Create a machine.UART object and sets callback
 *
 * @param handle     smart card interface handle
 * @param uart_id    identifier of UART
 * @param callback   callable object scheduled on receive event
 * @return           machine.UART object
 */
static mp_obj_t create_machine_uart(mp_int_t usart_id, mp_obj_t callback) {
  // Call constructor of machine.UART class
  mp_obj_t args_new[] = {
    // Positional arguments
    MP_OBJ_NEW_SMALL_INT(usart_id), // id
    MP_OBJ_NEW_SMALL_INT(9600),     // baudrate
    MP_OBJ_NEW_SMALL_INT(8),        // bits
    // Keyword arguments
    MP_OBJ_NEW_QSTR(MP_QSTR_timeout), MP_OBJ_NEW_SMALL_INT(0),
    MP_OBJ_NEW_QSTR(MP_QSTR_timeout_char), MP_OBJ_NEW_SMALL_INT(0),
    MP_OBJ_NEW_QSTR(MP_QSTR_rxbuf), MP_OBJ_NEW_SMALL_INT(RX_BUF_LEN)
  };
  mp_obj_t uart = pyb_uart_type.make_new(&pyb_uart_type, 3, 3, args_new);

  // Call machine.UART.irq()
  mp_obj_t args_irq[] = {
      callback,                             // handler
      MP_OBJ_NEW_SMALL_INT(UART_FLAG_IDLE), // trigger
      mp_const_false,                       // hard
  };
  (void)mp_call_function_n_kw(mp_load_attr(uart, MP_QSTR_irq), 3, 0, args_irq);

  return uart;
}

scard_handle_t scard_interface_init(mp_const_obj_t iface_id, mp_obj_t io_pin,
                                    mp_obj_t clk_pin,
                                    scard_cb_data_rx_t cb_data_rx,
                                    mp_obj_t cb_self) {
  scard_handle_t self = NULL;

  if(scard_class_debug) {
    mp_printf(&mp_plat_print, "\r\nInitializing scard_interface"); // TODO: remove
  }

  if(mp_obj_is_int(iface_id)) {
    mp_int_t usart_id = mp_obj_get_int(iface_id);
    const scard_usart_dsc_t* p_usard_dsc = find_descriptor(usart_id);
    if(p_usard_dsc) {
      // Create a new interface instance
      self = m_new0(scard_inst_t, 1);
      self->base.type = &scard_inst_type;
      self->p_usard_dsc = p_usard_dsc;

      // Create machine.UART and set callback
      mp_obj_t uart_cb = mp_load_attr(self, MP_QSTR_uart_callback);
      self->machine_uart_obj = create_machine_uart(usart_id, uart_cb);
      // Get pointer to underlying system object to use C API instead of Python.
      // Yes, this is a horrible abstraction leak, made intentionally for the
      // sake of performance.
      self->uart_obj = MP_STATE_PORT(pyb_uart_obj_all)[usart_id - 1];
      if(self->uart_obj == NULL) {
        scard_raise_internal_error("failed to obtain system UART object");
      }

      // Overwrite USART registers with settings for smart card
      if(!init_smartcard(&self->sc_handle, p_usard_dsc->handle)) {
        scard_raise_hw_error("failed to initialize USART in smart card mode");
      }

      // Initialize pins
      if(!init_pins(io_pin, clk_pin, p_usard_dsc->id)) {
        scard_raise_hw_error("failed to configure USART pins");
      }

      // Save callback and self parameter
      self->cb_data_rx = cb_data_rx;
      self->cb_self = cb_self;

    } else {
      mp_raise_ValueError("USART does not exists");
    }
  } else {
    mp_raise_TypeError("usart_id is not an integer");
  }

  return self;
}

void scard_interface_deinit(scard_handle_t handle) {
  // TODO: abort transfer
  // TODO: disable USART interrupt
  // TODO: deinit machine.UART
  HAL_SMARTCARD_DeInit(&handle->sc_handle);
  remove_callback(&handle->sc_handle);
  m_del(scard_inst_t, handle, 1);
}

scard_pin_dsc_t scard_pin(mp_obj_t user_obj, mp_int_t polarity, bool output,
                          scard_pin_state_t def_state) {
  scard_pin_dsc_t pin = {
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
  return pin;
}

/**
  * @brief Tx Transfer completed callbacks
  * @param  hsc: pointer to a SMARTCARD_HandleTypeDef structure that contains
  *                the configuration information for SMARTCARD module.
  * @retval None
  */
void HAL_SMARTCARD_TxCpltCallback(SMARTCARD_HandleTypeDef *hsc) {
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hsc);
  // TODO: implement
}

/**
  * @brief Rx Transfer completed callbacks
  * @param  hsc: pointer to a SMARTCARD_HandleTypeDef structure that contains
  *                the configuration information for SMARTCARD module.
  * @retval None
  */
void HAL_SMARTCARD_RxCpltCallback(SMARTCARD_HandleTypeDef *hsc) {
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hsc);
  // TODO: implement
}

/**
  * @brief SMARTCARD error callbacks
  * @param  hsc: pointer to a SMARTCARD_HandleTypeDef structure that contains
  *                the configuration information for SMARTCARD module.
  * @retval None
  */
 void HAL_SMARTCARD_ErrorCallback(SMARTCARD_HandleTypeDef *hsc) {
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hsc);
  // TODO: implement
}

STATIC mp_obj_t uart_callback(mp_obj_t self_in) {
  scard_inst_t* self = (scard_inst_t*)self_in;

  while(uart_rx_any(self->uart_obj)) {
    // TODO: remove
      mp_printf(&mp_plat_print, "%c", uart_rx_char(self->uart_obj));
  }
}

STATIC MP_DEFINE_CONST_FUN_OBJ_1(uart_callback_obj, uart_callback);

STATIC const mp_rom_map_elem_t scard_inst_locals_dict_table[] = {
  // Instance methods
  { MP_ROM_QSTR(MP_QSTR_uart_callback), MP_ROM_PTR(&uart_callback_obj) },
};
STATIC MP_DEFINE_CONST_DICT(scard_inst_locals_dict, scard_inst_locals_dict_table);

const mp_obj_type_t scard_inst_type = {
  { &mp_type_type },
  .locals_dict = (void*)&scard_inst_locals_dict,
};
