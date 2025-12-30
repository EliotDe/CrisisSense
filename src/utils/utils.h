#ifndef UTILS_H
#define UTILS_H

#include <stdint.h>
#include <stddef.h>

#define UTILS_OK 0
#define UTILS_ERR_INVALID_PARAM -1

int8_t mymemcpy(void* out_buffer, const void* in_buffer, size_t len);
int8_t mymemmove(void* out_buffer, const void* in_buffer, size_t len);

#endif