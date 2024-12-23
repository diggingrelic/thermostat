# type: ignore
from thermo.TempSensorADT7410 import TempSensorADT7410
import machine

# I2C configuration for TempSensorADT7410
i2c = machine.I2C(0, scl=machine.Pin(5), sda=machine.Pin(4), freq=100000)
temp_sensor = TempSensorADT7410(i2c)

class Temperature:
    def __init__(self):
        self.sensor = temp_sensor  # Use the ADT7410 sensor we initialized at module level
        self.last_valid_temp = None
        self.last_read_time = 0
        
    def get_valid_temperature(self):
        """Get valid temperature reading with retries"""
        for attempt in range(5):
            temperature = self.sensor.get_fahrenheit()
            if temperature is not None:
                return temperature

        raise RuntimeError("Failed to get a valid temperature reading.")