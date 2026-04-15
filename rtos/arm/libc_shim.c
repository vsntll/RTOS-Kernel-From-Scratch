/* The Phase 8 build deliberately avoids linking any libc (kernel_arm.c/
 * uart.c are hand-written without it, and the Makefile passes
 * -nostdlib) -- but xrce/'s portable sources assume the two most basic
 * <string.h> functions exist, same as they would on any real embedded
 * target's libc (newlib, picolibc, etc). Rather than pull in a full libc
 * for just this, or add a freestanding special-case to xrce/ itself
 * (which should stay written the ordinary way, since real targets do have
 * a libc), this is the minimal, ROS2-demo-only substitute for the two
 * symbols actually referenced: memcpy and strlen. */

#include <stddef.h>

void *memcpy(void *dst, const void *src, size_t n) {
    unsigned char *d = (unsigned char *)dst;
    const unsigned char *s = (const unsigned char *)src;
    for (size_t i = 0; i < n; i++) {
        d[i] = s[i];
    }
    return dst;
}

size_t strlen(const char *s) {
    size_t n = 0;
    while (s[n] != '\0') {
        n++;
    }
    return n;
}
