/* Stub string.h for freestanding kernel builds.
   stb_truetype includes <string.h>; memset/memcpy are overridden via
   STBTT_memset / STBTT_memcpy macros in font.c. */
#pragma once

#include <stddef.h>

void *memset(void *s, int c, size_t n);
void *memcpy(void *dest, const void *src, size_t n);
void *memmove(void *dest, const void *src, size_t n);
int   memcmp(const void *s1, const void *s2, size_t n);
size_t strlen(const char *s);
