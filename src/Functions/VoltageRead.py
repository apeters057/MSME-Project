import time
import board
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
from smbus import SMBus

############ ADS 1115 ############
i2c = busio.I2C(board.SCL, board.SDA)
ads = ADS.ADS1115(i2c,gain=8) #Gains {2 / 3: 6.144, 1: 4.096, 2: 2.048, 4: 1.024, 8: 0.512, 16: 0.256}
channel = ADS.P2 # Choose the ADC channel (0-3)
# Create an analog input channel on the specified channel
chan = AnalogIn(ads, channel)

############ Battery Volatage ############
bus = SMBus(1)
# smbus only support 8 bit register address, so write 2 byte 0 first
bus.write_word_data(0x14, 0x13, 0) ## ADC Channel 4 = 0x13
msb = bus.read_byte(0x14)
lsb = bus.read_byte(0x14)
value = (msb << 8) | lsb

A4_Voltage = value / (4095) * 3.3
Battery_Voltage = A4_Voltage *3

def VoltageRead_ADS1115():
    voltage = chan.voltage
    formated_voltage = "{:.4f}".format(voltage)
    return formated_voltage

def VoltageRead_Battery():
    
    formated_BatteryVoltage = ("{:.3f} V".format(Battery_Voltage))
    return Battery_Voltage


# while True:
#     # Read the voltage and print it
#     print(VoltageRead())
#     time.sleep(1)