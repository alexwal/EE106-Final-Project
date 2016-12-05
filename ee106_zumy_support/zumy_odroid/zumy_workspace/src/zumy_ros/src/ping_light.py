
#Script whose sole job is to turn on LED1, which is the LED that willbe allocated to "Can I be pinged"
#and then close as quick as possible
#meant to be run from the networking script.

from mbedrpc import *

if __name__ == '__main__':
    time.sleep(.1)
    dev='/dev/ttyACM0' #the com port... may not be this, at which point we'll have to change it.
    mbed=SerialRPC(dev, 115200)
    time.sleep(.1)
    led = DigitalOut(mbed,LED1)
    time.sleep(.1)
    led.write(1)