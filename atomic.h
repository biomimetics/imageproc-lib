#ifndef __ATOMIC_H
#define __ATOMIC_H

#include <stdint.h>

// The following macros create a spinlock with name <name>. A spinlock is a
// 16-bit integer. To lock the spinlock, call <name>_lock. To release the
// spinlock, call <name>_unlock. To try to lock the spinlock and return
// true if locking succeeded, call <name>_trylock. This will return true if the
// lock was not locked before (lock was acquired). To reset the spinlock, call
// <name>_reset. This forcefully sets the lock to 0. Spinlocks must be located
// in the near data space. The lock status is held in the uppermost bit (bit15)
// and the lower bits may be used to hold other information. This _H macro
// should be placed in header files. The following _C macro must be placed in
// ONLY ONE source file. If a spinlock is not public to other modules, use both
// the _C and _H macros in the source file.
#define DECLARE_SPINLOCK_H(name)                            \
extern uint16_t __attribute__((near)) _##name##_raw_lock;   \
static inline void name##_reset(void) {                     \
  _##name##_raw_lock = 0;                                   \
}                                                           \
static inline void name##_unlock(void) {                    \
  __asm__ volatile (                                        \
    "bclr %0, #15"                                          \
    :: "U" (_##name##_raw_lock)                             \
  );                                                        \
}                                                           \
static inline unsigned char name##_trylock(void) {          \
  unsigned char ret;                                        \
  __asm__ volatile (                                        \
    "btsts %1, #15\n"                                       \
    "bra z, 0f\n"                                           \
    "clr %0\n"                                              \
    "bra 1f\n"                                              \
    "0: setm %0\n"                                          \
    "1:"                                                    \
    : "=d" (ret)                                            \
    : "U" (_##name##_raw_lock)                              \
    : "cc"                                                  \
  );                                                        \
  return ret;                                               \
}                                                           \
static void name##_lock(void) {                             \
  while(!name##_trylock());                                 \
}

#define DECLARE_SPINLOCK_C(name)                        \
uint16_t __attribute__((near)) _##name##_raw_lock;

#endif
