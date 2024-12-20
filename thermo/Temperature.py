# type: ignore
from thermo.TempSensorADT7410 import TempSensorADT7410
import machine

# I2C configuration for TempSensorADT7410
i2c = machine.I2C(0, scl=machine.Pin(5), sda=machine.Pin(4), freq=100000)
temp_sensor = TempSensorADT7410(i2c)

class Temperature:
    def get_valid_temperature():
        """Get valid temperature reading with retries"""
        for attempt in range(5):
            temperature = temp_sensor.get_fahrenheit()
            if temperature is not None:
                return temperature

    raise RuntimeError("Failed to get a valid temperature reading.")