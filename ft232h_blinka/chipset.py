import time

import board
import busio
import digitalio
from adafruit_bus_device.spi_device import SPIDevice

# make a storage block of 1 gigabyte
# zero out memory to make sure it is allocated
mcuReset = digitalio.DigitalInOut(board.D7)
mcuReset.direction = digitalio.Direction.OUTPUT
mcuReset.value = False
i960Memory = bytearray(1024 * 1024 * 1024)
with busio.SPI(board.SCK, board.MOSI, board.MISO) as spi:
    cs = digitalio.DigitalInOut(board.D4)
    ack = digitalio.DigitalInOut(board.D6)
    ack.direction = digitalio.Direction.OUTPUT
    ack.value = True
    atmega1284pSync = digitalio.DigitalInOut(board.D5)
    cs.direction = digitalio.Direction.OUTPUT
    cs.value = True
    atmega1284pSync.direction = digitalio.Direction.INPUT
    device = SPIDevice(spi, cs, baudrate=5000000)
    previousValue = True
    readCommand = bytearray(4)
    readCommand[0] = 0x3
    readCommand[1] = 0x00
    readCommand[2] = 0x00
    readCommand[3] = 0x00
    inputBuffer = bytearray(8)
    outputBuffer = bytearray(2)
    mcuReset.value = True
    while True:
        # keep waiting until sync pin goes low
        if (atmega1284pSync.value != previousValue):
            if previousValue:
                with device as spi:
                    spi.write(readCommand)
                    spi.readinto(inputBuffer)
                print("Bytes Got: ")
                for b in inputBuffer:
                    print(b)

            else:
                print("Sync disabled!")
            previousValue = not previousValue
