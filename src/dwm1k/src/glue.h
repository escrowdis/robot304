#ifndef _GLUE_H_INCLUDED
#define _GLUE_H_INCLUDED

#include <stdlib.h>
#include <string>
#include <chrono>

#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue) (bitvalue ? bitSet(value, bit) : bitClear(value, bit))

typedef bool boolean;
typedef uint8_t byte;
typedef uint16_t word;
typedef std::string String;

using timepoint = std::chrono::time_point<std::chrono::system_clock>;

#endif