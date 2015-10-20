#include <logging.h>


#ifdef WIN32
#include <Windows.h>
#include <stdio.h>

void printToVSConsole(const char* format, ...) {
	va_list args;
	va_start(args, format);
	char buff[1024];
	_vsnprintf_s(buff, sizeof(buff), 1024, format, args);
	OutputDebugStringA(buff);
	va_end(args);
}

#endif