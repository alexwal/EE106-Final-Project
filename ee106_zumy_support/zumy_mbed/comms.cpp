#include "mbed.h"
#include "rtos.h" //for some reason, this cannot be included in imu.h.  I don't know why, but i put it here it works so ok.
#include "mbed_rpc.h"
#include "comms.h"

char rpc_input_buf[256];
char rpc_output_buf[1024];
//The input and output buffers.


//SerialRPCInterface SerialRPC(USBTX, USBRX, 115200);
MODSERIAL pc(USBTX, USBRX); // tx, rx
//Serial pc(USBTX,USBRX);
Thread* commsThread; //a thread that handles incoming traffice at no more than 10hz, but realistically, less because of the implimention of gets_cr
//for some reason, it's a thread* instead of a thread.  * shrug  *

//copy_paste from 192.  gets when using carriange returns
int gets_cr(MODSERIAL &src, char *s, int max) {
    int counter = 0;
    char c = 0;
    while(c != '\r')//src.readable() && c != '\r') 
    {
        c = src.getc();
        //pc.printf("%c \n\r",c);
        *(s++) = c;
        counter++;
        if (counter == max-1) break;
    }
    //*(s+1) = ' ';
    *(s++) = '\0';

    return counter;
}

void handle_incoming_traffic(void const *n)
{
    while(1) //so this thread doesn't run out of code!
    {
        //pc.gets(rpc_input_buf, 256);
        gets_cr(pc,rpc_input_buf,256); //works around the ctrl-enter (in minicom at least) thing. 
        //However, everything needs to be appended with a space...

        RPC::call(rpc_input_buf, rpc_output_buf);

        pc.printf("%s \n\r", rpc_output_buf);
        Thread::wait(10);
    }
}

void init_comms()
{
    #if DEVICE_ANALOGIN
    //RPC::add_rpc_class<RpcAnalogIn>();
    #endif
    RPC::add_rpc_class<RpcAnalogIn>();
    RPC::add_rpc_class<RpcDigitalIn>();
    RPC::add_rpc_class<RpcDigitalOut>();
    RPC::add_rpc_class<RpcDigitalInOut>();
    RPC::add_rpc_class<RpcPwmOut>();
    RPC::add_rpc_class<RpcTimer>();
    //RPC::add_rpc_class<RpcBusOut>();
    //RPC::add_rpc_class<RpcBusIn>();
    //RPC::add_rpc_class<RpcBusInOut>();
    RPC::add_rpc_class<RpcSerial>();
    
    //AnalogOut not avaliable on mbed LPC11U24 so only compile for other devices
    #if DEVICE_ANALOGOUT
    //RPC::add_rpc_class<RpcAnalogOut>();
    #endif
 
    pc.baud(115200);
    //LPC1768 Apparently possible to go up to 921600, which is 8x this.  Odroid can't seem to go that fast.
    wait_ms(20); //pause for just a bit.
    pc.printf("Hello world! \n\r");

    commsThread = new Thread(handle_incoming_traffic);
}