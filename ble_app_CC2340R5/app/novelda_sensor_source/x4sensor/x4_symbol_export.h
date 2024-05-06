/*
* Copyright Novelda AS 2024.
*/
#pragma once

#ifdef BUILDING_NOVELDA_X4SENSOR
// When building a dynamic library
#ifdef _MSC_VER 
#define X4_SYMBOL_EXPORT __declspec(dllexport)
#else // _MSC_VER
#define X4_SYMBOL_EXPORT __attribute__ ((visibility ("default")))
#endif // _MSC_VER
#elif USING_NOVELDA_X4SENSOR
// When using a dynamic library
#ifdef _MSC_VER
#define X4_SYMBOL_EXPORT __declspec(dllimport)
#else // _MSC_VER
#define X4_SYMBOL_EXPORT
#endif // _MSC_VER
#endif

