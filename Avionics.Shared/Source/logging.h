#ifndef LOGGING_H
#define LOGGING_H

#ifdef WIN32
#define PRINT printToVSConsole
// Required to print to VS Console
void printToVSConsole(const char* format, ...);
#else
#include <stdio.h>
#define PRINT printf

#endif


#endif /* LOGGING_H */