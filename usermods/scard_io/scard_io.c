// Include required definitions first.
#include "py/obj.h"
#include "py/runtime.h"
#include "py/builtin.h"
#include "py/mperrno.h"
#include "py/mphal.h"
#include "t1_protocol.h"

/// T=0 protocol identifier
#define T0_PROTOCOL                     1U
/// T=1 protocol identifier
#define T1_PROTOCOL                     2U

// TODO: Include necessary HAL header(s)
#if 0
STATIC bool sdcard_init(SMARTCARD_HandleTypeDef* p_sc, USART_TypeDef* usart_base,
                        uint32_t baud_rate) {
    p_sc->Instance = usart_base;
    p_sc->Init.WordLength = SMARTCARD_WORDLENGTH_9B;
    p_sc->Init.StopBits = SMARTCARD_STOPBITS_1_5;
    p_sc->Init.Parity = SMARTCARD_PARITY_EVEN;
    p_sc->Init.Mode = SMARTCARD_MODE_TX_RX;
    p_sc->Init.BaudRate = baud_rate;
    p_sc->Init.CLKPolarity = SMARTCARD_POLARITY_LOW;
    p_sc->Init.CLKPhase = SMARTCARD_PHASE_1EDGE;
    p_sc->Init.CLKLastBit = SMARTCARD_LASTBIT_ENABLE;
    p_sc->Init.Prescaler = SMARTCARD_PRESCALER_SYSCLK_DIV12;
    p_sc->Init.GuardTime = 16;
    p_sc->Init.NACKState = SMARTCARD_NACK_DISABLE;

    return HAL_OK == HAL_SMARTCARD_Init(p_sc);
}
#endif

STATIC mp_obj_t connection_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw,
                                    const mp_obj_t *args) {
    // TODO: Parse parameters
    // TODO: Check that UART is available
    // TODO: Create instance
    // TODO: Save event handler
    // TODO: Initialize timer
    // TODO: Initialize UART in smart card mode
    // TODO: Initialize UART pins
    // TODO: Initialize GPIO pins: detect, reset
    // TODO: Initialize T=1 protocol
    // TODO: Make connection active
    return mp_const_notimplemented;
}

STATIC mp_obj_t connection_reset(void) {
    // TODO: Reset card
    // TODO: Reset T=1 protocol
    return mp_const_notimplemented;
}

STATIC mp_obj_t connection_transmit(mp_obj_t bytes) {
    // TODO: Transmit APDU
    return mp_const_notimplemented;
}

STATIC mp_obj_t connection_is_active(void) {
    // TODO: Return connection status
    return mp_const_notimplemented; // mp_const_true
}

STATIC mp_obj_t connection_close(void) {
    // TODO: Release timer
    // TODO: Release UART
    // TODO: Release UART pins
    // TODO: Release GPIO pins: detect, reset
    // TODO: Make connection inactive
    return mp_const_notimplemented;
}

/****************************** MODULE ******************************/

STATIC MP_DEFINE_CONST_FUN_OBJ_0(connection_reset_obj, connection_reset);
STATIC MP_DEFINE_CONST_FUN_OBJ_1(connection_transmit_obj, connection_transmit);
STATIC MP_DEFINE_CONST_FUN_OBJ_0(connection_is_active_obj, connection_is_active);
STATIC MP_DEFINE_CONST_FUN_OBJ_0(connection_close_obj, connection_close);

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
