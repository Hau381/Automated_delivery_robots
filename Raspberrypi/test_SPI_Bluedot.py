# sudo pip3 install bluedot
import spidev
from bluedot import BlueDot


def dpad(pos):
    if pos.top:
        print("Forward")
        x=spi.xfer([0x30])
    elif pos.bottom:
        print("Reverse")
        x=spi.xfer([0x31])
    elif pos.left:
        print("Left")
        x=spi.xfer([0x32])
    elif pos.right:
        print("Right")
        x=spi.xfer([0x33])
    elif pos.middle:
        print("Code Killed")
        x=spi.xfer([0x34])
        
spi_bus = 0

spi_device = 0



spi = spidev.SpiDev()

spi.open(spi_bus, spi_device)

spi.max_speed_hz = 1000000
# Send a null byte to check for value

bd = BlueDot()
bd.when_pressed = dpad
#spi.close()