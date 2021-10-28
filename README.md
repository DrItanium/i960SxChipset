This sketch is meant to act as the chipset for an Intel i960Sx processor. 
The MCU used in this sketch is an ATMEGA1284p using the MightyCore board
package. 

I have modified the fuses of my ATMEGA1284p to make D1 (PB1) to act as CLKO
instead of an actual GPIO pin. I lose a pin but have a single clock source for
everything in the system. The i960 will use this as it's clock source as well. 

This chipset acts as interface between the serializers "surrounding" the i960
and the rest of the system. Those devices will talk to the chipset over SPI and
I2C. 

All of the documents around designing hardware for the i960Sx series of
processors comes from the early 1990s. So there is a lot of references to PALs,
PLDs, and FPGAs to act as chipset components. While I can get access to some of
the PAL code, I do not have access to the FPGA code used. The ATMEGA1284p is
the next best thing with all of its builtin functionality and ample resources
to act as the intermediary. 

The core of this sketch is the system bringup functionality and the bus state
machine diagram as shown in the i960SA/SB reference manual. 

The lib directory contains a copy of SdFat because I needed to improve the performance of enabling access to the SdCard
beyond what digitalWrite provides.
