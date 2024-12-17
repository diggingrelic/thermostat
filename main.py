# type: ignore
import network
import time
import machine
from machine import mem32
import gc
from umqtt.simple import MQTTClient
from TempSensorADT7410 import TempSensorADT7410
from mysecrets import SSID, PASSWORD, AIO_USERNAME, AIO_KEY
from HeaterController import HeaterController
from ThermostatController import ThermostatController
from TimeManager import TimeManager

# Debug flag for verbose logging
debug = True

# MQTT details
AIO_SERVER = "io.adafruit.com"
AIO_PORT = 1883
AIO_FEED_RELAY = f"{AIO_USERNAME}/feeds/relay"
AIO_FEED_SETPOINT = f"{AIO_USERNAME}/feeds/setpoint"
AIO_FEED_TEMP = f"{AIO_USERNAME}/feeds/temp"
AIO_FEED_CYCLE_DELAY = f"{AIO_USERNAME}/feeds/cycle_delay"
AIO_FEED_TEMP_DIFF = f"{AIO_USERNAME}/feeds/temp_diff"

# Global instances
time_manager = None
thermostat = None

def log(message):
    if debug:
        print(message)

def mqtt_callback(topic, msg):
    global time_manager, thermostat
    topic = topic.decode()
    msg = msg.decode()
    log(f"MQTT Message received: {topic} - {msg}")
    
    if topic == "time/seconds":
        time_manager.handle_time_message(msg)
    elif topic == AIO_FEED_RELAY:
        thermostat.set_enabled(msg)
    elif topic == AIO_FEED_SETPOINT:
        if thermostat.set_setpoint(msg):
            log(f"Setpoint updated to: {msg}")
        else:
            log("Invalid setpoint value received")
    elif topic == AIO_FEED_CYCLE_DELAY:
        try:
            if thermostat.heater.update_cycle_time(msg):
                log(f"Cycle delay updated to: {msg} minutes")
            else:
                log(f"Invalid cycle delay value: {msg} (must be between 5-20 minutes)")
        except Exception as e:
            log(f"Error updating cycle delay: {e}")
    elif topic == AIO_FEED_TEMP_DIFF:
        try:
            if thermostat.heater.update_temp_differential(msg):
                log(f"Temperature differential updated to: {msg}°F")
            else:
                log(f"Invalid temperature differential value: {msg} (must be between 1-5°F)")
        except Exception as e:
            log(f"Error updating temperature differential: {e}")

def get_initial_state(client):
    log("Fetching initial state...")
    client.publish(AIO_FEED_SETPOINT + "/get", "")
    client.publish(AIO_FEED_RELAY + "/get", "")
    client.publish(AIO_FEED_CYCLE_DELAY + "/get", "")
    client.publish(AIO_FEED_TEMP_DIFF + "/get", "")
    time.sleep(2)

def main():
    global time_manager, thermostat

    if connect_to_wifi(SSID, PASSWORD):
        blink_led(5, 0.2)

        # Initialize MQTT client
        client = MQTTClient("pico", AIO_SERVER, AIO_PORT, AIO_USERNAME, AIO_KEY)
        
        # Initialize managers and controllers
        time_manager = TimeManager(client, log)
        led = machine.Pin("LED", machine.Pin.OUT)
        relay_pin = machine.Pin(14, machine.Pin.OUT)
        set_pin_drive_strength(14, 12)
        
        heater = HeaterController(relay_pin, time_manager)
        thermostat = ThermostatController(heater, led, time_manager)
        
        # Setup MQTT
        client.set_callback(mqtt_callback)
        gc.collect()

        try:
            client.connect()
            
            # Subscribe to feeds
            time_manager.subscribe()
            time_manager.sync_time()
            client.subscribe(AIO_FEED_RELAY)
            client.subscribe(AIO_FEED_SETPOINT)
            client.subscribe(AIO_FEED_CYCLE_DELAY)
            client.subscribe(AIO_FEED_TEMP_DIFF)
            
            get_initial_state(client)

            # Initialize temperature sensor
            i2c = machine.I2C(0, scl=machine.Pin(5), sda=machine.Pin(4), freq=100000)
            temp_sensor = TempSensorADT7410(i2c)

            while True:
                try:
                    client.check_msg()
                    time_manager.check_sync_needed()

                    # Update temperature and thermostat
                    temperature = temp_sensor.get_fahrenheit()
                    if temperature is not None:
                        status = thermostat.update(temperature)
                        if status:
                            log(status)
                        
                        # Publish temperature to Adafruit
                        client.publish(AIO_FEED_TEMP, str(temperature))

                    time.sleep(1)
                    gc.collect()

                except Exception as e:
                    log(f"Error in main loop: {e}")
                    time.sleep(5)
                    gc.collect()

        except Exception as e:
            log(f"Failed to connect to MQTT broker: {e}")
            return

if __name__ == "__main__":
    main()
