import time
import board
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn

i2c = busio.I2C(board.SCL, board.SDA)
ads = ADS.ADS1115(i2c, gain=8)

# Choose the ADC channel (0-3)
channel = ADS.P2
# Create an analog input channel on the specified channel
chan = AnalogIn(ads, channel)

while True:
    # Read the voltage and print it
    voltage = chan.voltage
    print("Voltage: {:.4f} V".format(voltage))
    time.sleep(1)