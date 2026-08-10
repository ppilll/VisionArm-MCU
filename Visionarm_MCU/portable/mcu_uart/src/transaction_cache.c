#include "visionarm_uart_c/transaction_cache.h"

#include <string.h>

void va_uart_transaction_cache_reset(va_uart_transaction_cache_t* cache) {
    if (cache != NULL) {
        memset(cache, 0, sizeof(*cache));
    }
}

void va_uart_transaction_cache_record(va_uart_transaction_cache_t* cache,
                                      uint8_t message_type,
                                      uint32_t transaction_id,
                                      uint16_t payload_crc,
                                      bool acknowledged,
                                      uint16_t result_code,
                                      uint16_t detail_code) {
    if (cache == NULL) {
        return;
    }
    cache->valid = true;
    cache->message_type = message_type;
    cache->transaction_id = transaction_id;
    cache->payload_crc = payload_crc;
    cache->acknowledged = acknowledged;
    cache->result_code = result_code;
    cache->detail_code = detail_code;
}

va_uart_cached_result_t va_uart_transaction_cache_lookup(
    const va_uart_transaction_cache_t* cache,
    uint8_t message_type,
    uint32_t transaction_id,
    uint16_t payload_crc) {
    va_uart_cached_result_t result = {
        VA_UART_CACHE_NEW_TRANSACTION, false, 0U, 0U};
    if (cache == NULL || !cache->valid ||
        cache->message_type != message_type ||
        cache->transaction_id != transaction_id) {
        return result;
    }
    if (cache->payload_crc != payload_crc) {
        result.decision = VA_UART_CACHE_CONFLICTING_DUPLICATE;
        return result;
    }
    result.decision = VA_UART_CACHE_DUPLICATE;
    result.acknowledged = cache->acknowledged;
    result.result_code = cache->result_code;
    result.detail_code = cache->detail_code;
    return result;
}
