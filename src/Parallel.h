#include"pbrt.h"

//"volatile" makes compiler to load the value from memory, not from the register.
// If you not do it,computer will load the value, and store it in register.It is wrong to operate the parallel computing.
typedef volatile int32_t AtomicInt32;
#ifdef PBRT_HAS_64_BIT_ATOMICS
typedef volatile int64_t AtomicInt64;
#endif

//  32-bit atomic add, address pointed to by v, parameter is delta.
inline int32_t AtomicAdd(AtomicInt32 *v, int32_t delta) {
#ifdef WIN32
	int32_t origValue;
	_asm__volatile_("lock\n"
		"xadd1 %0,%1"
		: "=r"(origValue), "=m"(*v) : "0"(delta)
		: "memory");
	return origValue + delta;
#elif defined(_APPLE_) && !(defined(_i386_) || defined(_amd64_))
	return OSAtomicAdd32Barrier(delta, v);
#else

#endif
}

inline int32_t AtomicCompareAndSwap(AtomicInt32 *v, int32_t newValue, int32_t oldValue);

inline float AtomicAdd(volatile float *val, float delta) {
	union bits { float f; int32_t i; };
	bits oldVal, newVal;
	do {
		oldVal.f = *val;
		newVal.f = oldVal.f + delta;
	} while (AtomicCompareAndSwap(((AtomicInt32 *)val), newVal.i, oldVal.i) != oldVal.i);
	return newVal.f;
}





