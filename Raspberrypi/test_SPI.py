import spidev
import time


spi_bus = 0

spi_device = 0



spi = spidev.SpiDev()

spi.open(spi_bus, spi_device)

spi.max_speed_hz = 1000000
# Send a null byte to check for value
spi.xfer([0x31])
while True:
    c = spi.readbytes(7)
    #print(c)
    CRC = c[0]
    for i in range(1,6) :
        CRC ^= c[i]
    if CRC == c[5] and c[0] == 0xAA and c[1] == 0x55:
        rate1 = int.from_bytes([c[2],c[3]],"little",signed = "True")
        #print(rate1)
        print(c)
        time.sleep(1)