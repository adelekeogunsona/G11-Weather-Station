from machine import Pin, I2C
import bme280_float as bme280
import time

i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=400000)

print("Initializing sensor at 0x77...")
try:
    sensor = bme280.BME280(i2c=i2c, address=0x77)
    print("SUCCESS at 400kHz!")

    for i in range(5):
        print(sensor.values)
        time.sleep(2)

except Exception as e:
    print(f"FAILED: {e}")
    import sys
    sys.print_exception(e)