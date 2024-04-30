/*
* Copyright Novelda AS 2024.
*/
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#define X4_BIT
#define X4_CODE
#define X4_NOINIT
#define X4_XDATA
#define X4_INLINE

#define X4_STATIC_ASSERT(COND,MSG) typedef char static_assertion_##MSG[(COND)?1:-1]

#if defined(__AVR__) || defined(__ICC8051__)

    #define X4_PACK_START()
    #define X4_PACK_END()

#elif defined(__GNUC__) || defined(_MSC_VER)

    #define X4_PACK_START() _Pragma("pack(1)")
    #define X4_PACK_END()   _Pragma("pack()")

#endif

#ifdef __cplusplus
}
#endif

