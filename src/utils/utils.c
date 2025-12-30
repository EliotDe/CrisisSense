#include "utils.h"
#include <stdint.h>
#include <stddef.h>

int8_t mymemcpy(void* out_buffer, const void* in_buffer, size_t len){
    if (!out_buffer || !in_buffer) return UTILS_ERR_INVALID_PARAM;

    uint8_t* cout = (uint8_t*)out_buffer;
    const uint8_t* cin = (const uint8_t*)in_buffer;
    
    for (size_t i = 0; i < len; i++){
        cout[i] = cin[i];
    }

    return UTILS_OK;
}

int8_t mymemmove(void* out_buffer, const void* in_buffer, size_t len){
    if (!out_buffer || !in_buffer) return UTILS_ERR_INVALID_PARAM;

    uint8_t* cout = (uint8_t*)out_buffer;
    const uint8_t* cin = (const uint8_t*)in_buffer;

    if (cout < cin) {
        for (size_t i = 0; i < len; i++){
            cout[i] = cin[i];
        }
    }
    else{
        for (size_t i = len; i > 0; i--){
            cout[i] = cin[i];
        }
    }

    return UTILS_OK;
}