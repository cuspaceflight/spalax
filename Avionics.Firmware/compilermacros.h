// compilermacros.h - convenience macros for the compiler
//
// Macros and definitions which allow use of non-standard features of the
// compiler or to annotate extra information.
#pragma once
#ifndef COMPILER_MACROS_H
#define COMPILER_MACROS_H

// COMPILER_UNUSED_ARG is a macro which wraps function parameters in function
// definitions which are unused by the body. It is a no-op in terms of
// functionality but will silence unused argument warnings. Using it is more
// explicit than added, e.g., "(void)argname" to the function body.
//
// Example:
//  int foo(COMPILER_UNUSED_ARG(int bar)) { return 1; }
//
#ifdef __GNUC__
#   define COMPILER_UNUSED_ARG(arg) arg __attribute__((unused))
#else
#   define COMPILER_UNUSED_ARG(arg) arg
#endif

#endif // COMPILER_MACROS_H
