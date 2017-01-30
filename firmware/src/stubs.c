// Stubs to allow compilation of C++ without a proper runtime library

#include <stdio.h>

#include "ch.h"
#include "hal.h"

void _exit(int status){
    (void) status;
    chSysHalt("_exit called");

    while(TRUE){}
}

pid_t _getpid(void){
    return 1;
}

void _kill(pid_t id){
    (void) id;
}

void __cxa_pure_virtual(void) {
    chSysHalt("Pure Virtual Called");
}

int _open_r(void *reent, const char *file, int flags, int mode) {
    (void)reent; (void)file; (void)flags; (void)mode;
    return -1;
}

int __register_exitproc(int type, void (*fn) (void), void *arg, void *d) {
    (void) type;
    (void) fn;
    (void) arg;
    (void) d;


    return 0;
}

