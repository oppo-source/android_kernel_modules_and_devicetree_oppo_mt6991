/*************************************************************************/ /*!
@File
@Title          Common header containing type definitions for portability
@Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
@Description    Contains variable and structure definitions. Any platform
                specific types should be defined in this file.
@License        Dual MIT/GPLv2

The contents of this file are subject to the MIT license as set out below.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

Alternatively, the contents of this file may be used under the terms of
the GNU General Public License Version 2 ("GPL") in which case the provisions
of GPL are applicable instead of those above.

If you wish to allow use of your version of this file only under the terms of
GPL, and not to allow others to use your version of this file under the terms
of the MIT license, indicate your decision by deleting the provisions above
and replace them with the notice and other provisions required by GPL as set
out in the file called "GPL-COPYING" included in this distribution. If you do
not delete the provisions above, a recipient may use your version of this file
under the terms of either the MIT license or GPL.

This License is also included in this distribution in the file called
"MIT-COPYING".

EXCEPT AS OTHERWISE STATED IN A NEGOTIATED AGREEMENT: (A) THE SOFTWARE IS
PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
PURPOSE AND NONINFRINGEMENT; AND (B) IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/ /**************************************************************************/

#ifndef IMG_DEFS_H
#define IMG_DEFS_H

#if defined(__linux__) && defined(__KERNEL__)
#include <linux/types.h>
#else
#include <stddef.h>
#endif
#if !(defined(__linux__) && defined(__KERNEL__))
#if defined(__riscv)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wundef"
#endif
#include <assert.h>
#if defined(__riscv)
#pragma GCC diagnostic pop
#endif
#endif

#include "img_types.h"

#if defined(NO_INLINE_FUNCS)
	#define	INLINE
	#define	FORCE_INLINE
#else
#if defined(__cplusplus) || defined(INTEGRITY_OS)
	#if	!defined(INLINE)
		#define INLINE				inline
	#endif
	#define	FORCE_INLINE			static inline
#else
#if	!defined(INLINE)
	#define	INLINE					__inline
#endif
#if (defined(UNDER_WDDM) || defined(WINDOWS_WDF)) && defined(_X86_)
	#define	FORCE_INLINE			__forceinline
#else
	#define	FORCE_INLINE			static __inline
#endif
#endif
#endif

/* True if the GCC version is at least the given version. False for older
 * versions of GCC, or other compilers.
 */
#if defined(__GNUC__)
#define GCC_VERSION_AT_LEAST(major, minor) \
	(__GNUC__ > (major) || \
	(__GNUC__ == (major) && __GNUC_MINOR__ >= (minor)))
#else
#define GCC_VERSION_AT_LEAST(major, minor) 0
#endif

#if defined(__clang__)
#define CLANG_VERSION_AT_LEAST(major) \
	(__clang_major__ >= (major))
#else
#define CLANG_VERSION_AT_LEAST(major) 0
#endif

/* Use Clang's __has_extension and __has_builtin macros if available. */
#if defined(__has_extension)
#define has_clang_extension(e) __has_extension(e)
#else
#define has_clang_extension(e) 0
#endif

#if defined(__has_builtin)
#define has_clang_builtin(e) __has_builtin(e)
#else
#define has_clang_builtin(e) 0
#endif

/* Use this in any file, or use attributes under GCC - see below */
#ifndef PVR_UNREFERENCED_PARAMETER
#define	PVR_UNREFERENCED_PARAMETER(param) ((void)(param))
#endif

/* static_assert(condition, "message to print if it fails");
 *
 * Assert something at compile time. If the assertion fails, try to print
 * the message, otherwise do nothing. static_assert is available if:
 *
 * - It's already defined as a macro (e.g. by <assert.h> in C11)
 * - We're using MSVC which exposes static_assert unconditionally
 * - We're using a C++ compiler that supports C++11
 * - We're using GCC 4.6 and up in C mode (in which case it's available as
 *   _Static_assert)
 *
 * In all other cases, fall back to an equivalent that makes an invalid
 * declaration.
 */
#if !defined(static_assert) && !defined(_MSC_VER) && \
		(!defined(__cplusplus) || __cplusplus < 201103L) || defined(__KLOCWORK__)
	/* static_assert isn't already available */
	#if !defined(__cplusplus) && (GCC_VERSION_AT_LEAST(4, 6) || \
								  (defined(__clang__) && has_clang_extension(c_static_assert)))
		#define static_assert _Static_assert
	#else
		#define static_assert(expr, message) \
			extern int static_assert_failed[(expr) ? 1 : -1] __attribute__((unused))
	#endif
#endif

/*
 * unreachable("explanation") can be used to indicate to the compiler that
 * some parts of the code can never be reached, like the default branch
 * of a switch that covers all real-world possibilities, even though there
 * are other ints that exist for instance.
 *
 * The message will be printed as an assert() when debugging.
 *
 * Note: there is no need to add a 'return' or any error handling after
 * calling unreachable(), as this call will never return.
 */
#if defined(__linux__) && defined(__KERNEL__)
/* Kernel has its own unreachable(), which is a simple infinite loop */
#elif GCC_VERSION_AT_LEAST(4, 5) || has_clang_builtin(__builtin_unreachable)
	#define unreachable(msg) \
		do { \
			assert(!(msg)); \
			__builtin_unreachable(); \
		} while (false)
#elif defined(_MSC_VER)
	#define unreachable(msg) \
		do { \
			assert(!(msg)); \
			__assume(0); \
		} while (false)
#else
	#define unreachable(msg) \
		do { \
			assert(!(msg)); \
			while (1); \
		} while (false)
#endif

/*
 * assume(x > 2 && x <= 7) works like an assert(), except it hints to the
 * compiler what it can assume to optimise the code, like a limited range
 * of parameter values.
 */
#if has_clang_builtin(__builtin_assume)
	#define assume(expr) \
		do { \
			assert(expr); \
			__builtin_assume(expr); \
		} while (false)
#elif defined(_MSC_VER)
	#define assume(expr) \
		do { \
			assert(expr); \
			__assume(expr); \
		} while (false)
#elif defined(__linux__) && defined(__KERNEL__)
	#define assume(expr) ((void)(expr))
#elif GCC_VERSION_AT_LEAST(4, 5) || has_clang_builtin(__builtin_unreachable)
	#define assume(expr) \
		do { \
			if (unlikely(!(expr))) \
				unreachable("Assumption isn't true: " # expr); \
		} while (false)
#else
	#define assume(expr) assert(expr)
#endif

/*! Macro to calculate the n-byte aligned value from that supplied rounding up.
 * n must be a power of two.
 *
 * Both arguments should be of a type with the same size otherwise the macro may
 * cut off digits, e.g. imagine a 64 bit address in _x and a 32 bit value in _n.
 */
#define PVR_ALIGN(_x, _n)	(((_x)+((_n)-1U)) & ~((_n)-1U))

#if defined(_WIN32)

#if defined(WINDOWS_WDF)

	/*
	 * For WINDOWS_WDF drivers we don't want these defines to overwrite calling conventions propagated through the build system.
	 * This 'empty' choice helps to resolve all the calling conv issues.
	 *
	 */
	#define IMG_CALLCONV
	#define C_CALLCONV

	#define IMG_INTERNAL
	#define IMG_RESTRICT __restrict

	/*
	 * The proper way of dll linking under MS compilers is made of two things:
	 * - decorate implementation with __declspec(dllexport)
	 *   this decoration helps compiler with making the so called
	 *   'export library'
	 * - decorate forward-declaration (in a source dependent on a dll) with
	 *   __declspec(dllimport), this decoration helps the compiler to make
	 *   faster and smaller code in terms of calling dll-imported functions
	 *
	 * Usually these decorations are performed by having a single macro define
	 * making that expands to a proper __declspec() depending on the
	 * translation unit, dllexport inside the dll source and dllimport outside
	 * the dll source. Having IMG_EXPORT and IMG_IMPORT resolving to the same
	 * __declspec() makes no sense, but at least works.
	 */
	#define IMG_IMPORT __declspec(dllexport)
	#define IMG_EXPORT __declspec(dllexport)

#else

	#define IMG_CALLCONV __stdcall
	#define IMG_INTERNAL
	#define	IMG_EXPORT	__declspec(dllexport)
	#define IMG_RESTRICT __restrict
	#define C_CALLCONV	__cdecl

	/*
	 * IMG_IMPORT is defined as IMG_EXPORT so that headers and implementations
	 * match. Some compilers require the header to be declared IMPORT, while
	 * the implementation is declared EXPORT.
	 */
	#define	IMG_IMPORT	IMG_EXPORT

#endif

#if defined(UNDER_WDDM)
	#ifndef	_INC_STDLIB
		#if defined(__mips)
			/* do nothing */
		#elif defined(UNDER_MSBUILD)
			/* do nothing */
		#else
			_CRTIMP void __cdecl abort(void);
		#endif
	#endif
#endif /* UNDER_WDDM */
#else
	#if (defined(__linux__) || defined(__QNXNTO__)) && defined(__KERNEL__)
		#define IMG_INTERNAL
		#define IMG_EXPORT
		#define IMG_CALLCONV
	#elif defined(__linux__) || defined(__METAG) || defined(__mips) || defined(__QNXNTO__) || defined(__riscv) || defined(__APPLE__)
		#define IMG_CALLCONV
		#define C_CALLCONV

		#if defined(__METAG)
			#define IMG_INTERNAL
		#else
			#define IMG_INTERNAL    __attribute__((visibility("hidden")))
		#endif

		#define IMG_EXPORT      __attribute__((visibility("default")))
		#define IMG_RESTRICT    __restrict__
	#elif defined(INTEGRITY_OS)
		#define IMG_CALLCONV
		#define IMG_INTERNAL
		#define IMG_EXPORT
		#define IMG_RESTRICT
		#define C_CALLCONV
		#define __cdecl

		#ifndef USE_CODE
			#define IMG_ABORT() printf("IMG_ABORT was called.\n")
		#endif
	#else
		#error("define an OS")
	#endif

#endif

/* Use default definition if not overridden */
#ifndef IMG_ABORT
	#if defined(EXIT_ON_ABORT)
		#define IMG_ABORT()	exit(1)
	#else
		#define IMG_ABORT()	abort()
	#endif
#endif

/* The best way to suppress unused parameter warnings using GCC is to use a
 * variable attribute. Place the __maybe_unused between the type and name of an
 * unused parameter in a function parameter list e.g. 'int __maybe_unused var'.
 * This should only be used in GCC build environments, for example, in files
 * that compile only on Linux.
 * Other files should use PVR_UNREFERENCED_PARAMETER
 */

/* Kernel macros for compiler attributes */
/* Note: param positions start at 1 */
#if defined(__linux__) && defined(__KERNEL__)
	#include <linux/compiler.h>

	#if !defined(__fallthrough)
		#if GCC_VERSION_AT_LEAST(7, 0) || CLANG_VERSION_AT_LEAST(10)
			#define __fallthrough __attribute__((__fallthrough__))
		#else
			#define __fallthrough
		#endif
	#endif
#elif defined(__GNUC__) || defined(HAS_GNUC_ATTRIBUTES)
	#define __must_check       __attribute__((warn_unused_result))
	#define __maybe_unused     __attribute__((unused))
	#define __malloc           __attribute__((malloc))

	/* Bionic's <sys/cdefs.h> might have defined these already */
	/* See https://android.googlesource.com/platform/bionic.git/+/master/libc/include/sys/cdefs.h */
	#if !defined(__packed)
		#define __packed           __attribute__((packed))
	#endif
	#if !defined(__aligned)
		#define __aligned(n)       __attribute__((aligned(n)))
	#endif
	#if !defined(__noreturn)
		#define __noreturn         __attribute__((noreturn))
	#endif

	/* That one compiler that supports attributes but doesn't support
	 * the printf attribute... */
	#if defined(__GNUC__)
		#if defined(__MINGW32__)
		    #define __printf(fmt, va)  __attribute__((format(gnu_printf, (fmt), (va))))
		#else
			#define __printf(fmt, va)  __attribute__((format(printf, (fmt), (va))))
		#endif
	#else
		#define __printf(fmt, va)
	#endif /* defined(__GNUC__) */

	#if defined(__cplusplus) && (__cplusplus >= 201703L)
		#define __fallthrough [[fallthrough]]
	#elif GCC_VERSION_AT_LEAST(7, 0) || CLANG_VERSION_AT_LEAST(10)
		#define __fallthrough __attribute__((__fallthrough__))
	#else
		#define __fallthrough
	#endif

	#define __user
	#define __force
	#define __iomem
#else
	/* Silently ignore those attributes */
	#define __printf(fmt, va)
	#define __packed
	#define __aligned(n)
	#define __must_check
	#define __maybe_unused
	#define __malloc

	#if defined(_MSC_VER) || defined(CC_ARM)
		#define __noreturn __declspec(noreturn)
	#else
		#define __noreturn
	#endif

	/* This may already been defined, e.g. by SAL (Source Annotation Language) */
	#if !defined(__fallthrough)
		#define __fallthrough
	#endif

	#define __user
	#define __force
	#define __iomem
#endif


/* Other attributes, following the same style */
#if defined(__GNUC__) || defined(HAS_GNUC_ATTRIBUTES)
	#define __const_function      __attribute__((const))
#else
	#define __const_function
#endif


/* GCC builtins */
#if defined(__linux__) && defined(__KERNEL__)
	#include <linux/compiler.h>
#elif defined(__GNUC__) || defined(INTEGRITY_OS)

/* Klocwork does not support __builtin_expect, which makes the actual condition
 * expressions hidden during analysis, affecting it negatively. */
#if !defined(__KLOCWORK__) && !defined(INTEGRITY_OS) && !defined(DEBUG)
	#define likely(x)   __builtin_expect(!!(x), 1)
	#define unlikely(x) __builtin_expect(!!(x), 0)
#endif

	/* Compiler memory barrier to prevent reordering */
	#define barrier() __asm__ __volatile__("": : :"memory")
#else
	#define barrier() static_assert(0, "barrier() isn't supported by your compiler");
#endif

/* That one OS that defines one but not the other... */
#ifndef likely
	#define likely(x)   (x)
#endif
#ifndef unlikely
	#define unlikely(x) (x)
#endif

#if !defined(BITS_PER_BYTE)
#define BITS_PER_BYTE (8)
#endif /* BITS_PER_BYTE */

/* These two macros are also provided by the kernel */
#ifndef BIT
#define BIT(b) (1UL << (b))
#endif

#ifndef BIT_ULL
#define BIT_ULL(b) (1ULL << (b))
#endif

#define BIT_SET(f, b)     BITMASK_SET((f),    BIT(b))
#define BIT_UNSET(f, b)   BITMASK_UNSET((f),  BIT(b))
#define BIT_TOGGLE(f, b)  BITMASK_TOGGLE((f), BIT(b))
#define BIT_ISSET(f, b)   BITMASK_HAS((f),    BIT(b))

#define BITMASK_SET(f, m)     do { ((f) |= (m)); } while (false)
#define BITMASK_UNSET(f, m)   do { ((f) &= ~(m)); } while (false)
#define BITMASK_TOGGLE(f, m)  do { ((f) ^= (m)); } while (false)
#define BITMASK_HAS(f, m)     (((f) & (m)) == (m)) /* the bits from the mask are all set */
#define BITMASK_ANY(f, m)     (((f) & (m)) != 0U)  /* any bit from the mask is set */

#ifndef MAX
#define MAX(a ,b)	(((a) > (b)) ? (a) : (b))
#endif

#ifndef MIN
#define MIN(a, b)	(((a) < (b)) ? (a) : (b))
#endif

#ifndef CLAMP
#define CLAMP(min, max, n)  ((n) < (min) ? (min) : ((n) > (max) ? (max) : (n)))
#endif

#define SWAP(X, Y) (X) ^= (Y); (Y) ^= (X); (X) ^= (Y);

#if defined(__linux__) && defined(__KERNEL__)
	#include <linux/kernel.h>
	#include <linux/bug.h>
#endif

/* Get a structure's address from the address of a member */
#define IMG_CONTAINER_OF(ptr, type, member) \
	(type *) ((uintptr_t) (ptr) - offsetof(type, member))

/* Get a new pointer with an offset (in bytes) from a base address, useful
 * when traversing byte buffers and accessing data in buffers through struct
 * pointers.
 * Note, this macro is not equivalent to or replacing offsetof() */
#define IMG_OFFSET_ADDR(addr, offset_in_bytes) \
	(void*)&(((IMG_UINT8*)(void*)(addr))[offset_in_bytes])

/* Get a new pointer with an offset (in bytes) from a base address, version
 * for volatile memory.
 */
#define IMG_OFFSET_ADDR_VOLATILE(addr, offset_in_bytes) \
	(volatile void*)&(((volatile IMG_UINT8*)(volatile void*)(addr))[offset_in_bytes])

/* Get a new pointer with an offset (in dwords) from a base address, useful
 * when traversing byte buffers and accessing data in buffers through struct
 * pointers.
 * Note, this macro is not equivalent to or replacing offsetof() */
#define IMG_OFFSET_ADDR_DW(addr, offset_in_dwords) \
	(void*)(((IMG_UINT32*)(void*)(addr)) + (offset_in_dwords))

/* The number of elements in a fixed-sized array */
#ifndef ARRAY_SIZE
#define ARRAY_SIZE(ARR) (sizeof(ARR) / sizeof((ARR)[0]))
#endif

/* To guarantee that __func__ can be used, define it as a macro here if it
   isn't already provided by the compiler. */
#if defined(_MSC_VER) || (defined(__cplusplus) && __cplusplus < 201103L)
#define __func__ __FUNCTION__
#endif

#if defined(__cplusplus)
/* C++ Specific:
 * Disallow use of copy and assignment operator within a class.
 * Should be placed under private. */
#define IMG_DISALLOW_COPY_AND_ASSIGN(C) \
	C(const C&); \
	void operator=(const C&)
#endif

#if defined(SUPPORT_PVR_VALGRIND) && !defined(__METAG) && !defined(__mips) && !defined(__riscv)
	#include "/usr/include/valgrind/memcheck.h"

	#define VG_MARK_INITIALIZED(pvData,ui32Size) VALGRIND_MAKE_MEM_DEFINED(pvData,ui32Size)
	#define VG_MARK_NOACCESS(pvData,ui32Size) VALGRIND_MAKE_MEM_NOACCESS(pvData,ui32Size)
	#define VG_MARK_ACCESS(pvData,ui32Size) VALGRIND_MAKE_MEM_UNDEFINED(pvData,ui32Size)
	#define VG_ASSERT_DEFINED(pvData,ui32Size) VALGRIND_CHECK_MEM_IS_DEFINED(pvData,ui32Size)
#else
	#if defined(_MSC_VER)
	#	define PVR_MSC_SUPPRESS_4127 __pragma(warning(suppress:4127))
	#else
	#	define PVR_MSC_SUPPRESS_4127
	#endif

	#define VG_MARK_INITIALIZED(pvData,ui32Size) PVR_MSC_SUPPRESS_4127 do { } while (false)
	#define VG_MARK_NOACCESS(pvData,ui32Size) PVR_MSC_SUPPRESS_4127 do { } while (false)
	#define VG_MARK_ACCESS(pvData,ui32Size) PVR_MSC_SUPPRESS_4127 do { } while (false)
	#define VG_ASSERT_DEFINED(pvData,ui32Size) PVR_MSC_SUPPRESS_4127 do { } while (false)
#endif

#define IMG_STRINGIFY_IMPL(x) # x
#define IMG_STRINGIFY(x) IMG_STRINGIFY_IMPL(x)

#define IMG_CONCATENATE_IMPL(x,y) x ## y
#define IMG_CONCATENATE(x,y) IMG_CONCATENATE_IMPL(x,y)

#if defined(DEBUG) && !defined(INTEGRITY_OS)
#define IMG_PAGESLOG2BYTES(_tcast, _npages, _log2) \
	({ \
		PVR_ASSERT( ((IMG_UINT64)(1ULL) << (sizeof(_tcast)*8UL)) >= ((IMG_UINT64)(_npages) << (_log2)) ); \
		(_tcast)(_npages) << (_log2); \
	})
#else
#define IMG_PAGESLOG2BYTES(_tcast, _npages, _log2) ((_npages) << (_log2))
#endif

#define IMG_PAGE2BYTES32(logsize) IMG_PAGESLOG2BYTES(IMG_UINT32,IMG_UINT32_C(1),logsize)
#define IMG_PAGE2BYTES64(logsize) ((IMG_UINT64)IMG_UINT64_C(1) << (logsize))

#define IMG_PAGES2BYTES32(pages,logsize) IMG_PAGESLOG2BYTES(IMG_UINT32,pages,logsize)
#define IMG_PAGES2BYTES64(pages,logsize) ((IMG_UINT64)(pages) << (logsize))

#define IMG_PAGE_SHIFT_4KB   12U
#define IMG_PAGE_SHIFT_16KB  14U
#define IMG_PAGE_SHIFT_64KB  16U
#define IMG_PAGE_SHIFT_256KB 18U
#define IMG_PAGE_SHIFT_1MB   20U
#define IMG_PAGE_SHIFT_2MB   21U

#if defined(INTEGRITY_OS)
	/* Definitions not present in INTEGRITY. */
	#define PATH_MAX	200
#endif

#if defined(__clang__) || defined(__GNUC__)
	/* __SIZEOF_POINTER__ is defined already by these compilers */
#elif defined(INTEGRITY_OS)
	#if defined(__Ptr_Is_64)
		#define __SIZEOF_POINTER__ 8
	#else
		#define __SIZEOF_POINTER__ 4
	#endif
#elif defined(_WIN32)
	#define __SIZEOF_POINTER__ sizeof(char *)
#else
	#warning Unknown OS - using default method to determine whether CPU arch is 64-bit.
	#define __SIZEOF_POINTER__ sizeof(char *)
#endif

/* RDI8567: gcc/clang/llvm load/store optimisations may cause issues with
 * uncached device memory allocations. Some pointers are made 'volatile'
 * to prevent those optimisations being applied to writes through those
 * pointers.
 */
#if (GCC_VERSION_AT_LEAST(7, 0) || defined(__clang__)) && (defined(__arm64__) || defined(__aarch64__))
#define NOLDSTOPT volatile
/* after applying 'volatile' to a pointer, we may need to cast it to 'void *'
 * to keep it compatible with its existing uses.
 */
#define NOLDSTOPT_VOID (void *)

#define NOLDSTOPT_REQUIRED 1
#else
#define NOLDSTOPT
#define NOLDSTOPT_VOID
#endif

#define PVR_PRE_DPF (void) printf

/* C STD >= C99 */
#if !defined(INTEGRITY_OS) && defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 199901L)
#define IMG_FLEX_ARRAY_MEMBER
#define IMG_FLEX_ARRAY_SIZE(size, count) ((size) * (count))
#else
/* In C STD prior to C99 flexible array members are an extension feature and syntax requires alternative approach */
#define IMG_FLEX_ARRAY_MEMBER (1)
#define IMG_FLEX_ARRAY_SIZE(size, count) ((size) * ((count) - 1))
#endif

#endif /* IMG_DEFS_H */
/*****************************************************************************
 End of file (img_defs.h)
*****************************************************************************/
