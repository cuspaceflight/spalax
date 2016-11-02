#pragma once
#include <memory>
#include <printf.h>
#include <exception>

#define UtilAssert(value, msg) \
if (!(value)) { \
    printf("Assertion failed at %s:%i with error: %s\n",__FILE__, __LINE__, msg); \
    throw std::runtime_error("Failed UtilAssert"); \
}

// MSVC doesn't appear to set __cplusplus correctly
#ifndef _MSC_VER
#if __cplusplus < 201402L
// no make_unique support
namespace std {
    template<typename T, typename ...Args>
    std::unique_ptr<T> make_unique(Args&& ...args)
    {
        return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
    }
}
#endif
#endif




