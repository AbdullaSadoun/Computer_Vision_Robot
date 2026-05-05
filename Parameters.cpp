/*
Parameters.cpp
- Defines the global params() accessor that returns the single shared Parameters instance
- Uses a local static so the struct is zero-initialized before first access
by: Abdulla Sadoun
Date: February 10, 2026
*/
// Parameters.cpp
#include "Parameters.h"

const Parameters& params() {
    /*
    Constructs a static Parameters on first call and returns a const reference to it.
    All fields use their in-class default values; no file I/O or environment lookup.
    */
    static Parameters p{};
    return p;
}
 
