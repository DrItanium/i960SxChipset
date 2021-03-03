import time

import board
import busio
import digitalio
from adafruit_bus_device.spi_device import SPIDevice

# make a storage block of 1 gigabyte
i960Memory = bytearray(1024 * 1024 * 1024)
# zero out memory to make sure it is allocated
for i in i960Memory:
    i = 0
with busio.SPI(board.SCK, board.MOSI, board.MISO) as spi:
    cs = digitalio.DigitalInOut(board.C0)
    atmega1284pSync = digitalio.DigitalInOut(board.C1)
    cs.direction = digitalio.Direction.OUTPUT
    cs.value = True
    atmega1284pSync.direction = digitalio.Direction.INPUT
    device = SPIDevice(spi, cs, baudrate=5000000)
    while True:
        # keep waiting until sync pin goes low
        if atmega1284pSync.value == False:
            print("Sync enabled!");
