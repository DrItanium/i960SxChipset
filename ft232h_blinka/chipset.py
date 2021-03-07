import time

import board
import busio
import digitalio
from adafruit_bus_device.spi_device import SPIDevice
from statemachine import StateMachine, State

# make a storage block of 1 gigabyte
# zero out memory to make sure it is allocated
def pulseLow(pin):
    pin.value = False
    pin.value = True

#        getIODIRBAddress()   { return chooseAddress<0x01, 0x10>(); }
#        getIOPOLAAddress()   { return chooseAddress<0x02, 0x01>(); }
#        getIOPOLBAddress()   { return chooseAddress<0x03, 0x11>(); }
#        getGPINTENAAddress() { return chooseAddress<0x04, 0x02>(); }
#        getGPINTENBAddress() { return chooseAddress<0x05, 0x12>(); }
#        getDEFVALAAddress()  { return chooseAddress<0x06, 0x03>(); }
#        getDEFVALBAddress()  { return chooseAddress<0x07, 0x13>(); }
#        getIntConAAddress()  { return chooseAddress<0x08, 0x04>(); }
#        getIntConBAddress()  { return chooseAddress<0x09, 0x14>(); }
#        getIOConAddress()    { return chooseAddress<0x0A, 0x05>(); }
#        getGPPUAAddress()    { return chooseAddress<0x0C, 0x06>(); }
#        getGPPUBAddress()    { return chooseAddress<0x0D, 0x16>(); }
#        getINTFAAddress()    { return chooseAddress<0x0E, 0x07>(); }
#        getINTFBAddress()    { return chooseAddress<0x0F, 0x17>(); }
#        getINTCAPAAddress()  { return chooseAddress<0x10, 0x08>(); }
#        getINTCAPBAddress()  { return chooseAddress<0x11, 0x18>(); }
#        getGPIOAAddress()    { return chooseAddress<0x12, 0x09>(); }
#        getGPIOBAddress()    { return chooseAddress<0x13, 0x19>(); }
#        getOLATAAddress()    { return chooseAdd`ress<0x14, 0x0A>(); }
#        getOLATBAddress()    { return chooseAddress<0x15, 0x1A>(); }
class MCP23S17:
    addresses = {
                'iodira' : 0x00,
                'iodirb' : 0x01,
                'iopola' : 0x02,
                'iopolb' : 0x03,
                'gpintena' : 0x04,
                'gpintenb' : 0x05,
                'defvala' : 0x06,
                'defvalb' : 0x07,
                'intcona' : 0x08,
                'intconb' : 0x09,
                'iocon' : 0x0A,
                'gppua' : 0x0c,
                'gppub' : 0x0d,
                'intfa' : 0x0e,
                'intfb' : 0x0f,
                'intcapa' : 0x10,
                'intcapb' : 0x11,
                'gpioa' : 0x12,
                'gpiob' : 0x13,
                'olata' : 0x14,
                'olatb' : 0x15,
        }
    def __init__(self, index):
        self.idx = index & 0b111
        self.rdOp = 0b01000000 | (self.idx << 1) | 1
        self.wrOp = 0b01000000 | (self.idx << 1)
        self.hardwareAddressPinsEnabled = False
    
    def performRead(self, device, addr):
        output = bytearray(1)
        command = bytearray(2)
        command[0] = self.rdOp
        command[1] = addr
        with device as spi:
            spi.write(command)
            spi.readinto(output)
        return output[0]
    
    def performWrite(self, device, addr, value):
        command = bytearray(3)
        command[0] = self.wrOp
        command[1] = addr
        command[2] = value
        with device as spi:
            spi.write(command)
    
    def performRead16(self, device, addrA, addrB):
        first = self.performRead(device, addrA)
        second = self.performRead(device, addrB)
        return first | (second << 8)
    
    def performWrite16(self, device, addrA, addrB, value):
        self.performWrite(device, addrA, value & 0xFF)
        self.performWrite(device, addrB, ((value & 0xFF00) >> 8))
    def _refreshIOCon(self, device):
        result = self.getIOCon(device)
        self.hardwareAddressPinsEnabled = (result & 0b00001000) != 0

    def getIOCon(self, device):
        return self.performRead(device, addresses['iocon'])

    def setIOCon(self, device, value):
        self.performWrite(device, addresses['iocon'], value)
        self._refreshIOCon(device)

    def enableHardwareAddressPins(self, device):
        if not self.hardwareAddressPinsEnabled:
            self.setIOCon(device, self.getIOCon(device) | 0b00001000)

    def disableHardwareAddressPins(self, device):
        if self.hardwareAddressPinsEnabled:
            self.setIOCon(device, self.getIOCon(device) & 0b11110110)

    def writeGPIOsDirection(self, device, pattern):
        self.performWrite16(device, addresses['iodira'], addresses['iodirb'], pattern)

    def readGPIOsDirection(self, device):
        return self.performRead16(device, addresses['iodira'], addresses['iodirb'])

    def readGPIOs(self, device):
        return self.performRead16(device, addresses['gpioa'], addresses['gpiob'])

    def writeGPIOs(self, device, value):
        self.performWrite16(device, addresses['gpioa'], addresses['gpiob'], value)

    def writePortB(self, device, value):
        self.performWrite(device, addresses['gpiob'], value)

    def writePortA(self, device, value):
        self.performWrite(device, addresses['gpioa'], value)






print("Starting up!")
reset960 = digitalio.DigitalInOut(board.C0)
reset960.direction = digitalio.Direction.OUTPUT
reset960.value = False # hold in reset for the time being
int0 = digitalio.DigitalInOut(board.C1)
int0.direction = digitalio.Direction.OUTPUT
int0.value = True # default to high
wr = digitalio.DigitalInOut(board.C2)
wr.direction = digitalio.Direction.INPUT
den = digitalio.DigitalInOut(board.C3)
den.direction = digitalio.Direction.INPUT
addrState = digitalio.DigitalInOut(board.C4)
addrState.direction = digitalio.Direction.INPUT
ready = digitalio.DigitalInOut(board.C5)
ready.direction = digitalio.Direction.OUTPUT
ready.value = True # default to high
blast = digitalio.DigitalInOut(board.C6)
blast.direction = digitalio.Direction.INPUT
fail = digitalio.DigitalInOut(board.C7)
fail.direction = digitalio.Direction.INPUT
cs = digitalio.DigitalInOut(board.D4)
cs.direction = digitalio.Direction.OUTPUT
cs.value = True
i960Memory = bytearray(1024 * 1024 * 1024)
dataLines = MCP23S17(0b000)
addrLower16 = MCP23S17(0b001)
addrUpper16 = MCP23S17(0b010)
extraLines = MCP23S17(0b011)

def makeDataLinesOutput():
    dataLines.writeGPIOsDirection(0)

def makeDataLinesInput():
    dataLines.writeGPIOsDirection(0xFFFF)

with busio.SPI(board.SCK, board.MOSI, board.MISO) as spi:
    device = SPIDevice(spi, cs, baudrate=5000000)
    # enable the hardware address lines on dataLines
    dataLines.enableHardwareAddressPins(device)
    # now each of them are separate things
    dataLines._refreshIOCon(device)
    addrLower16._refreshIOCon(device)
    addrUpper16._refreshIOCon(device)
    extraLines._refreshIOCon(device)

    addrLower16.writeGPIOsDirection(device, 0xFFFF)
    addrUpper16.writeGPIOsDirection(device, 0xFFFF)
    dataLines.writeGPIOsDirection(device, 0xFFFF)
    extraLines.writeGPIOsDirection(device, 0b0000000001011111)
    # set the lock and HOLD pins correctly
    extraLines.writePortA(device, 0b10000000)
    previousValue = True
    print("Done Setting up!")
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
