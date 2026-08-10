#ifndef VISIONARM_UART_C_TRANSACTION_CACHE_H
#define VISIONARM_UART_C_TRANSACTION_CACHE_H

#include "visionarm_uart_c/protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum va_uart_cache_decision {
    VA_UART_CACHE_NEW_TRANSACTION = 0,
    VA_UART_CACHE_DUPLICATE,
    VA_UART_CACHE_CONFLICTING_DUPLICATE
} va_uart_cache_decision_t;

typedef struct va_uart_cached_result {
    va_uart_cache_decision_t decision;
    bool acknowledged;
    uint16_t result_code;
    uint16_t detail_code;
} va_uart_cached_result_t;

typedef struct va_uart_transaction_cache {
    bool valid;
    uint8_t message_type;
    uint32_t transaction_id;
    uint16_t payload_crc;
    bool acknowledged;
    uint16_t result_code;
    uint16_t detail_code;
} va_uart_transaction_cache_t;

void va_uart_transaction_cache_reset(va_uart_transaction_cache_t* cache);
void va_uart_transaction_cache_record(va_uart_transaction_cache_t* cache,
                                      uint8_t message_type,
                                      uint32_t transaction_id,
                                      uint16_t payload_crc,
                                      bool acknowledged,
                                      uint16_t result_code,
                                      uint16_t detail_code);
va_uart_cached_result_t va_uart_transaction_cache_lookup(
    const va_uart_transaction_cache_t* cache,
    uint8_t message_type,
    uint32_t transaction_id,
    uint16_t payload_crc);

#ifdef __cplusplus
}
#endif

#endif
