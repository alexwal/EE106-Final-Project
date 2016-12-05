#ifndef COMMS_H_
#define COMMS_H_

#include "mbed.h"
#include "MODSERIAL.h"


extern MODSERIAL pc; //the serial port that everyone uses to printf.


void init_comms(); //starts up comms, just as you'd expect.  Call once before any printfs.

int gets_cr(MODSERIAL &src, char *s, int max); //gets a string that is terminated by \r.  appends a \0 to it as well.
void handle_incoming_traffic(void const *n); //handle all incoming traffic on the serial port (and call RPC on it)


#endif
