/*
 * spinlock.h - test-and-set spinlock using GCC built-ins
 *
 * Uses __sync_lock_test_and_set / __sync_lock_release which GCC lowers to
 * LOCK XCHG / MOV+MFENCE on x86-64.  No libatomic needed (always inline).
 */
#pragma once

typedef volatile int spinlock_t;

#define SPINLOCK_INIT  0

static inline void spin_lock(spinlock_t *l) {
    /* Test-and-test-and-set: spin on load (cache-friendly) until the CAS
     * might succeed, then try to acquire. */
    while (__sync_lock_test_and_set(l, 1)) {
        while (*l) __asm__ volatile ("pause");
    }
}

static inline void spin_unlock(spinlock_t *l) {
    __sync_lock_release(l);
}
