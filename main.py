# type: ignore
import network
import time
import machine
from machine import mem32
import gc
from umqtt.simple import MQTTClient
from TempSensorADT7410 import TempSensorADT7410
from mysecrets import SSID, PASSWORD, AIO_USERNAME, AIO_KEY  # Import secrets

# Debug flag for verbose logging
debug = True

# MQTT details
AIO_SERVER = "io.adafruit.com"
AIO_PORT = 1883
AIO_FEED_RELAY = f"{AIO_USERNAME}/feeds/relay"
AIO_FEED_SETPOINT = f"{AIO_USERNAME}/feeds/setpoint"
AIO_FEED_TEMP = f"{AIO_USERNAME}/feeds/temp"

# Define the base address and offsets for the GPIO pad registers
PADS_BANK0_BASE = 0x4001C000
PAD_GPIO = PADS_BANK0_BASE + 0x04
PAD_GPIO_MPY = 4
PAD_DRIVE_BITS = 4

def set_pin_drive_strength(pin, mA):
    """Sets the drive strength of a GPIO pin.

    Args:
        pin (int): The pin number.
        mA (int): The desired drive strength in mA (2, 4, 8, or 12).
    """

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

# Pin configuration
led = machine.Pin("LED", machine.Pin.OUT)
relay = machine.Pin(14, machine.Pin.OUT)  # Replace 2 with the GPIO pin connected to your relay
set_pin_drive_strength(14, 12)  # Set drive strength to 12mA


setpoint = 40.0  # Default setpoint
relay_state = False  # Track desired relay state
current_relay_state = False  # Track actual relay state

# I2C configuration for TempSensorADT7410
i2c = machine.I2C(0, scl=machine.Pin(5), sda=machine.Pin(4), freq=100000)
temp_sensor = TempSensorADT7410(i2c)  # Initialize the temperature sensor

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

        # Wait for connection
        for attempt in range(5):
            if wlan.isconnected():
                break
            wlan.active(False)  # Reset WLAN to clear resources
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

def blink_led(times, duration):
    for _ in range(times):
        led.on()
        time.sleep(duration)
        led.off()
        time.sleep(duration)

def mqtt_callback(topic, msg):
    global setpoint, relay_state
    log(f"MQTT Message received: {topic.decode()} - {msg.decode()}")
    if topic.decode() == AIO_FEED_RELAY:
        relay_state = msg.decode().lower() == "on"
    elif topic.decode() == AIO_FEED_SETPOINT:
        try:
            setpoint = float(msg.decode())
            log(f"Setpoint updated to: {setpoint}")
        except ValueError:
            log("Invalid setpoint value received")

def get_valid_temperature():
    """
    Tries to get a valid temperature reading from the sensor.
    Retries up to 5 times with 1-second pauses.
    """
    for attempt in range(5):
        temperature = temp_sensor.get_fahrenheit()
        if temperature is not None:
            return temperature
        time.sleep(1)
    raise RuntimeError("Failed to get a valid temperature reading.")

def get_initial_state(client):
    log("Fetching initial setpoint and relay state...")
    client.publish(AIO_FEED_SETPOINT + "/get", "")
    client.publish(AIO_FEED_RELAY + "/get", "")
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
            log("Reconnected and subscribed to feeds.")
            return
        except Exception as e:
            log(f"Reconnection attempt {attempt + 1} failed: {e}")
            delay *= 2  # Exponential backoff
    log("Failed to reconnect after multiple attempts.")

def update_relay_state(temperature):
    global current_relay_state
    desired_relay_state = relay_state and temperature <= setpoint and setpoint >= 30

    if desired_relay_state != current_relay_state:
        current_relay_state = desired_relay_state
        if current_relay_state:
            relay.on()
            led.on()
            log("Relay turned ON (Temperature below setpoint)")
        else:
            relay.off()
            led.off()
            log("Relay turned OFF (Temperature above setpoint)")

last_wifi_status = None  # Track last Wi-Fi status
last_wifi_check = 0  # Track last Wi-Fi status check
def log_wifi_status():
    global last_wifi_status, last_wifi_check
    current_time = time.time()
    # Only check Wi-Fi status every 10 seconds
    if current_time - last_wifi_check >= 10:
        wlan = network.WLAN(network.STA_IF)
        current_status = wlan.isconnected()
        if current_status != last_wifi_status:  # Log only if the status changes
            if current_status:
                log(f"Wi-Fi connected, IP: {wlan.ifconfig()[0]}")
            else:
                log("Wi-Fi disconnected.")
            last_wifi_status = current_status
        last_wifi_check = current_time

def main():
    if connect_to_wifi(SSID, PASSWORD):
        blink_led(5, 0.2)

        client = MQTTClient("pico", AIO_SERVER, AIO_PORT, AIO_USERNAME, AIO_KEY)
        client.set_callback(mqtt_callback)
        gc.collect()
        try:
            client.connect()
            client.subscribe(AIO_FEED_RELAY)
            client.subscribe(AIO_FEED_SETPOINT)
            get_initial_state(client)
        except Exception as e:
            log(f"Failed to connect to MQTT broker: {e}")
            return

        last_temp_update = time.time()
        last_published_temp = None  # Track the last published temperature

        try:
            while True:
                try:
                    # Check for new MQTT messages
                    client.check_msg()
                except OSError as e:
                    log(f"Error during MQTT message check: {e}")
                    reconnect_with_backoff(client)  # Attempt to reconnect the MQTT client

                # Get temperature once per loop iteration
                try:
                    temperature = get_valid_temperature()
                    log(f"Temp Reading: {temperature:.2f} °F")
                except RuntimeError as e:
                    log(f"Error getting temperature: {e}")
                    continue

                # Publish temperature every 60 seconds or if it changes significantly
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

                # Update relay state
                update_relay_state(temperature)

                log_wifi_status()
                time.sleep(5)
                gc.collect()

        except KeyboardInterrupt:
            log("Exiting...")
            client.disconnect()

if __name__ == "__main__":
    main()
