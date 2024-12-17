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

# Define GPIO pad registers
PADS_BANK0_BASE = 0x4001C000
PAD_GPIO = PADS_BANK0_BASE + 0x04
PAD_GPIO_MPY = 4
PAD_DRIVE_BITS = 4

# Global instances
time_manager = None
thermostat = None
temp_sensor = None

def set_pin_drive_strength(pin, mA):
    """Sets the drive strength of a GPIO pin."""
    addr = PAD_GPIO + PAD_GPIO_MPY * pin
    mem32[addr] &= 0xFFFFFFFF ^ (0b11 << PAD_DRIVE_BITS)

    if mA <= 2:
        mem32[addr] |= 0b00 << PAD_DRIVE_BITS
    elif mA <= 4:
        mem32[addr] |= 0b01 << PAD_DRIVE_BITS
    elif mA <= 8:
        mem32[addr] |= 0b10 << PAD_DRIVE_BITS
    else:
        mem32[addr] |= 0b11 << PAD_DRIVE_BITS

def log(message):
    if debug:
        print(message)

def connect_to_wifi(ssid, password):
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)

    if not wlan.isconnected():
        log("Connecting to Wi-Fi...")
        wlan.connect(ssid, password)
        time.sleep(10)

        # Wait for connection with retries
        for attempt in range(5):
            if wlan.isconnected():
                break
            wlan.active(False)
            time.sleep(1)
            wlan.active(True)
            wlan.connect(ssid, password)
            time.sleep(10)
            log(f"Attempt {attempt + 1}")

    if wlan.isconnected():
        log("Connected to Wi-Fi")
        log(f"Network Config: {wlan.ifconfig()}")
        return True
    else:
        log("Failed to connect to Wi-Fi")
        return False

def blink_led(times, delay):
    """Blink onboard LED"""
    led = machine.Pin("LED", machine.Pin.OUT)
    for _ in range(times):
        led.on()
        time.sleep(delay)
        led.off()
        time.sleep(delay)

def get_valid_temperature():
    """Get valid temperature reading with retries"""
    global temp_sensor
    for attempt in range(5):
        temperature = temp_sensor.get_fahrenheit()
        if temperature is not None:
            return temperature
        time.sleep(1)
    raise RuntimeError("Failed to get a valid temperature reading.")

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

def reconnect_with_backoff(client, attempts=5):
    delay = 1
    for attempt in range(attempts):
        try:
            log(f"Reconnecting to MQTT broker, attempt {attempt + 1}...")
            client.disconnect()
            time.sleep(delay)
            gc.collect()
            client.connect()
            client.subscribe(AIO_FEED_RELAY)
            client.subscribe(AIO_FEED_SETPOINT)
            client.subscribe(AIO_FEED_CYCLE_DELAY)
            client.subscribe(AIO_FEED_TEMP_DIFF)
            log("Reconnected and subscribed to feeds.")
            return
        except Exception as e:
            log(f"Reconnection attempt {attempt + 1} failed: {e}")
            delay *= 2
    log("Failed to reconnect after multiple attempts.")

last_wifi_status = None
last_wifi_check = 0
def log_wifi_status():
    global last_wifi_status, last_wifi_check
    current_time = time.time()
    if current_time - last_wifi_check >= 10:
        wlan = network.WLAN(network.STA_IF)
        current_status = wlan.isconnected()
        if current_status != last_wifi_status:
            if current_status:
                log(f"Wi-Fi connected, IP: {wlan.ifconfig()[0]}")
            else:
                log("Wi-Fi disconnected.")
            last_wifi_status = current_status
        last_wifi_check = current_time

def main():
    global time_manager, thermostat, temp_sensor

    if connect_to_wifi(SSID, PASSWORD):
        blink_led(5, 0.2)

        # Initialize MQTT client and sensors
        client = MQTTClient("pico", AIO_SERVER, AIO_PORT, AIO_USERNAME, AIO_KEY)
        i2c = machine.I2C(0, scl=machine.Pin(5), sda=machine.Pin(4), freq=100000)
        temp_sensor = TempSensorADT7410(i2c)
        
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

        last_temp_update = time.time()
        last_published_temp = None

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

            while True:
                try:
                    client.check_msg()
                    time_manager.check_sync_needed()
                except OSError as e:
                    log(f"Error during MQTT message check: {e}")
                    reconnect_with_backoff(client)

                try:
                    temperature = get_valid_temperature()
                    log(f"Temp Reading: {temperature:.2f} °F")
                except RuntimeError as e:
                    log(f"Error getting temperature: {e}")
                    continue

                current_time = time.time()
                if current_time - last_temp_update >= 60 or (
                    last_published_temp is None or abs(temperature - last_published_temp) > 0.5
                ):
                    try:
                        client.publish(AIO_FEED_TEMP, f"{temperature:.2f}")
                        log(f"Published Temp Reading: {temperature:.2f} °F - reading was +/- 0.5 deg")
                        last_temp_update = current_time
                        last_published_temp = temperature
                    except Exception as e:
                        log(f"Error publishing temperature: {e}")

                status = thermostat.update(temperature)
                if status:
                    log(status)

                log_wifi_status()
                time.sleep(5)
                gc.collect()

        except Exception as e:
            log(f"Failed to connect to MQTT broker: {e}")
            return

if __name__ == "__main__":
    main()
