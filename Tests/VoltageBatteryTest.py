from smbus import SMBus
import time
bus = SMBus(1)
# smbus only support 8 bit register address, so write 2 byte 0 first
bus.write_word_data(0x14, 0x13, 0) ## ADC Channel 4 = 0x13
msb = bus.read_byte(0x14)
lsb = bus.read_byte(0x14)
value = (msb << 8) | lsb

A4_Voltage = value / (4095) * 3.3
Battery_Voltage = A4_Voltage *3

while True:
    print("Voltage: {:.3f} V".format(Battery_Voltage))
    time.sleep(1)
