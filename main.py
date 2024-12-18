# type: ignore
import network
import time
import machine
from machine import mem32
import gc
from umqtt.simple import MQTTClient
from TempSensorADT7410 import TempSensorADT7410
from mysecrets import SSID, PASSWORD, AIO_USERNAME, AIO_KEY
#from HeaterController import HeaterController
#from ThermostatController import ThermostatController
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
AIO_FEED_MIN_TIME_ON = f"{AIO_USERNAME}/feeds/min_time_on"
AIO_FEED_HEATER_STATUS = f"{AIO_USERNAME}/feeds/heater_status"
AIO_FEED_RUNTIME = f"{AIO_USERNAME}/feeds/runtime_mins"
AIO_FEED_TIMER = f"{AIO_USERNAME}/feeds/timer"  # NEW: Timer feed

# Define GPIO pad registers
PADS_BANK0_BASE = 0x4001C000
PAD_GPIO = PADS_BANK0_BASE + 0x04
PAD_GPIO_MPY = 4
PAD_DRIVE_BITS = 4

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

# Initialize controllers
led = machine.Pin("LED", machine.Pin.OUT)
relay_pin = machine.Pin(14, machine.Pin.OUT)
set_pin_drive_strength(14, 12)

setpoint = 40.0  # Default setpoint
relay_state = False  # Track desired relay state
current_relay_state = False  # Track actual relay state
cycle_delay = 10  # Default cycle delay in minutes
temp_diff = 2.0  # Default temperature differential in °F
last_relay_change = 0  # Track when relay last changed state
min_time_on = 15  # Default minimum on-time in minutes
heater_start_time = 0  # Track when heater turns on
total_runtime = 0  # Track accumulated runtime in minutes
heater_status = 0  # Track current heater status (0=off, 1=on)
time_manager = None  # Initialize time_manager as None

# I2C configuration for TempSensorADT7410
i2c = machine.I2C(0, scl=machine.Pin(5), sda=machine.Pin(4), freq=100000)
temp_sensor = TempSensorADT7410(i2c)

timer_end_time = 0  # When timer should finish (0 = no timer)
current_timer_hours = 0  # Current timer value for feed updates

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
    for _ in range(times):
        led.on()
        time.sleep(delay)
        led.off()
        time.sleep(delay)

def mqtt_callback(topic, msg, client):
    global setpoint, relay_state, temp_diff, cycle_delay, min_time_on, heater_status, total_runtime, time_manager, timer_end_time, current_timer_hours
    topic_str = topic.decode() if isinstance(topic, bytes) else topic
    msg_str = msg.decode()
    
    log(f"MQTT message received - Topic: {topic_str}, Message: {msg_str}")
    
    if topic_str == "time/seconds":
        if time_manager:
            time_manager.handle_time_message(msg)
    elif topic_str == AIO_FEED_RELAY:
        new_relay_state = msg_str.lower() == "on"
        if new_relay_state != relay_state:
            relay_state = new_relay_state
            log(f"Relay command received: {relay_state} (Actual state: {current_relay_state}, min time holding: {current_relay_state and not relay_state})")
        else:
            log(f"Relay command unchanged: command={relay_state}, actual={current_relay_state}")
    elif topic_str == AIO_FEED_SETPOINT:
        try:
            new_setpoint = float(msg_str)
            if new_setpoint != setpoint:
                current = setpoint  # Store current value
                setpoint = new_setpoint
                log(f"Setpoint: {current} -> {setpoint}")
            else:
                log(f"Setpoint unchanged: current={setpoint}, received={new_setpoint}")
        except ValueError:
            log("Invalid setpoint value received")
    elif topic_str == AIO_FEED_CYCLE_DELAY:
        try:
            new_cycle_delay = float(msg_str)
            if new_cycle_delay != cycle_delay:
                if 1 <= new_cycle_delay <= 20:
                    current = cycle_delay  # Store current value
                    cycle_delay = new_cycle_delay
                    log(f"Cycle delay: {current} -> {cycle_delay} minutes")
                else:
                    log(f"Invalid cycle delay value: {new_cycle_delay} (must be between 1-20 minutes)")
            else:
                log(f"Cycle delay unchanged: current={cycle_delay}, received={new_cycle_delay}")
        except ValueError:
            log("Invalid cycle delay value received")
    elif topic_str == AIO_FEED_TEMP_DIFF:
        try:
            new_temp_diff = float(msg_str)
            if new_temp_diff != temp_diff:
                if 1 <= new_temp_diff <= 5:
                    current = temp_diff  # Store current value
                    temp_diff = new_temp_diff
                    log(f"Temperature differential: {current} -> {temp_diff}°F")
                else:
                    log(f"Invalid temperature differential value: {new_temp_diff} (must be between 1-5°F)")
            else:
                log(f"Temperature differential unchanged: current={temp_diff}, received={new_temp_diff}")
        except ValueError:
            log("Invalid temperature differential value received")
    elif topic_str == AIO_FEED_MIN_TIME_ON:
        try:
            new_min_time = float(msg_str)
            if new_min_time != min_time_on:
                if 1 <= new_min_time <= 60:
                    current = min_time_on  # Store current value
                    min_time_on = new_min_time
                    log(f"Minimum on-time: {current} -> {min_time_on} minutes")
                else:
                    log(f"Invalid minimum on-time value: {new_min_time} (must be between 1-60 minutes)")
            else:
                log(f"Minimum on-time unchanged: current={min_time_on}, received={new_min_time}")
        except ValueError:
            log("Invalid minimum on-time value received")
    elif topic_str == AIO_FEED_HEATER_STATUS:
        try:
            new_status = int(msg_str)
            if new_status != heater_status:
                heater_status = new_status
                log(f"Heater status updated: {heater_status}")
        except ValueError:
            log("Invalid heater status value received")
    elif topic_str == AIO_FEED_RUNTIME:
        try:
            new_runtime = float(msg_str)
            if new_runtime != total_runtime:
                total_runtime = new_runtime
                log(f"Runtime updated: {total_runtime} minutes")
        except ValueError:
            log("Invalid runtime value received")
    elif topic_str == AIO_FEED_TIMER:
        try:
            if msg_str.startswith('R:'):  # Added: Skip our remaining time updates
                return
                
            if msg_str.lower() in ('0', 'off'):
                if timer_end_time > 0:
                    timer_end_time = 0
                    current_timer_hours = 0
                    relay_state = False
                    client.publish(AIO_FEED_RELAY, "OFF")
                    log("Timer cancelled, relay commanded OFF")
            else:
                hours = float(msg_str)
                if 1 <= hours <= 12:  # Between 1 and 12 hours
                    timer_end_time = time.time() + (hours * 3600)  # Convert to seconds
                    current_timer_hours = hours
                    relay_state = True
                    client.publish(AIO_FEED_RELAY, "ON")
                    log(f"Timer set for {hours:.1f} hours, relay commanded ON")
                else:
                    log(f"Invalid timer duration: {hours} (must be between 1-12 hours)")
        except ValueError:
            log(f"Invalid timer format: {msg_str} (use hours between 1-12)")

def get_valid_temperature():
    """Get valid temperature reading with retries"""
    for attempt in range(5):
        temperature = temp_sensor.get_fahrenheit()
        if temperature is not None:
            return temperature
        time.sleep(1)
    raise RuntimeError("Failed to get a valid temperature reading.")

def get_initial_state(client, retries=3):
    """Fetch initial state with retries"""
    global setpoint, temp_diff, cycle_delay, relay_state, min_time_on, total_runtime, timer_end_time, current_timer_hours
    
    for retry in range(retries):
        log("Fetching initial state...")
        received_values = set()
        
        # Request values one at a time with small delays
        client.publish(AIO_FEED_RELAY + "/get", "")
        time.sleep(0.5)
        client.publish(AIO_FEED_SETPOINT + "/get", "")
        time.sleep(0.1)
        client.publish(AIO_FEED_CYCLE_DELAY + "/get", "")
        time.sleep(0.1)
        client.publish(AIO_FEED_TEMP_DIFF + "/get", "")
        time.sleep(0.1)
        client.publish(AIO_FEED_MIN_TIME_ON + "/get", "")
        time.sleep(0.1)
        client.publish(AIO_FEED_RUNTIME + "/get", "")
        time.sleep(0.1)
        client.publish(AIO_FEED_TIMER + "/get", "")
        time.sleep(0.1)
        
        timeout = time.time() + 5
        while time.time() < timeout:
            try:
                client.check_msg()
                
                # Check which values have been received
                if setpoint != 40.0:
                    received_values.add('setpoint')
                if cycle_delay != 10:
                    received_values.add('cycle_delay')
                if temp_diff != 2.0:
                    received_values.add('temp_diff')
                if min_time_on != 15:
                    received_values.add('min_time_on')
                if isinstance(relay_state, bool):
                    received_values.add('relay')
                if isinstance(total_runtime, (int, float)):
                    received_values.add('runtime')
                if isinstance(current_timer_hours, (int, float)):
                    received_values.add('timer')
                
                # If we got all values, we're done
                if len(received_values) >= 7:  # Updated count to 7
                    log(f"Received all initial values after {retry + 1} attempts")
                    return True
                    
            except Exception as e:
                log(f"Error checking messages: {e}")
            
            time.sleep(0.1)
        
        # Log what we're still waiting for
        missing = {'setpoint', 'cycle_delay', 'temp_diff', 'min_time_on', 'relay', 'runtime', 'timer'} - received_values
        log(f"Attempt {retry + 1}: Still waiting for: {', '.join(missing)}")
        
        if retry < retries - 1:
            time.sleep(1)
    
    return False

# Add to global variables
MQTT_KEEPALIVE = 120  # Increase keepalive to 2 minutes
MQTT_RECONNECT_DELAY = 5  # 5 seconds between reconnection attempts

def create_mqtt_client():
    """Create and configure MQTT client with proper settings"""
    client = MQTTClient(
        client_id="pico",
        server=AIO_SERVER,
        port=AIO_PORT,
        user=AIO_USERNAME,
        password=AIO_KEY,
        keepalive=MQTT_KEEPALIVE
    )
    return client

def reconnect_with_backoff(client, attempts=5):
    """Reconnect to MQTT with exponential backoff"""
    delay = MQTT_RECONNECT_DELAY
    for attempt in range(attempts):
        try:
            log(f"Reconnecting to MQTT broker, attempt {attempt + 1}...")
            try:
                client.sock.close()  # Explicitly close the socket
                client.disconnect()
            except Exception as e:
                log(f"Disconnect error: {e}")
            
            time.sleep(delay)
            gc.collect()
            
            client = create_mqtt_client()  # Create fresh client
            client.set_callback(mqtt_callback)
            client.connect()
            client.subscribe(AIO_FEED_RELAY)
            client.subscribe(AIO_FEED_SETPOINT)
            client.subscribe(AIO_FEED_CYCLE_DELAY)
            client.subscribe(AIO_FEED_TEMP_DIFF)
            client.subscribe(AIO_FEED_MIN_TIME_ON)
            client.subscribe(AIO_FEED_TIMER)  # NEW: Subscribe to timer feed
            log("Reconnected and subscribed to feeds.")
            return client
            
        except Exception as e:
            log(f"Reconnection attempt {attempt + 1} failed: {e}")
            delay *= 2  # Exponential backoff
            if attempt < attempts - 1:
                time.sleep(delay)
    
    log("Failed to reconnect after multiple attempts.")
    return None

def update_relay_state(temperature, client):
    global current_relay_state, last_relay_change, last_logged_bounds
    global heater_start_time, total_runtime, heater_status
    current_time = time.time()
    
    # Calculate temperature bounds using differential
    temp_high = setpoint + temp_diff
    temp_low = setpoint - temp_diff
    
    # Only log bounds if they've changed
    current_bounds = (temp_low, temp_high, setpoint, temp_diff)
    if current_bounds != last_logged_bounds:
        log(f"Control bounds: {temp_low:.1f}°F to {temp_high:.1f}°F (Setpoint: {setpoint:.1f}°F ±{temp_diff:.1f}°F)")
        last_logged_bounds = current_bounds
    
    # Check if relay is currently on
    if current_relay_state:
        time_on = current_time - heater_start_time
        # If commanded OFF and minimum time met, turn off
        if not relay_state and time_on >= (min_time_on * 60):
            current_relay_state = False
            last_relay_change = current_time  # Start cycle delay timer
            relay_pin.off()
            led.off()
            heater_status = 0
            # Calculate runtime for this session
            if heater_start_time > 0:
                runtime = (current_time - heater_start_time) / 60
                total_runtime += runtime
                client.publish(AIO_FEED_RUNTIME, str(int(total_runtime)))
            client.publish(AIO_FEED_HEATER_STATUS, "0")
            log("Relay turned OFF (minimum time elapsed)")
            return
        # If minimum time not met, stay on
        elif not relay_state and time_on < (min_time_on * 60):
            remaining = (min_time_on * 60) - time_on
            log(f"Relay commanded OFF but held ON for minimum time: {remaining:.1f} seconds remaining")
            return
    
    # Only check cycle delay if we're trying to turn ON
    if not current_relay_state and relay_state:
        if current_time - last_relay_change < cycle_delay * 60:
            remaining = (cycle_delay * 60) - (current_time - last_relay_change)
            log(f"Cycle delay: {remaining:.1f} seconds remaining")
            return
    
    # Changed: Proper differential logic
    desired_relay_state = relay_state and setpoint >= 30 and (
        (not current_relay_state and temperature <= temp_low) or  # Turn ON below low limit
        (current_relay_state and temperature <= temp_high)        # Stay ON until high limit
    )

    if desired_relay_state != current_relay_state:
        current_relay_state = desired_relay_state
        last_relay_change = current_time
        
        if current_relay_state:
            relay_pin.on()
            led.on()
            heater_start_time = current_time  # Track when relay turns on
            heater_status = 1  # Update heater status
            client.publish(AIO_FEED_HEATER_STATUS, "1")  # Publish status change
            log(f"Relay turned ON (Temperature {temperature:.1f}°F below low limit {temp_low:.1f}°F)")
        else:
            relay_pin.off()
            led.off()
            heater_status = 0  # Update heater status
            # Calculate runtime for this session
            if heater_start_time > 0:
                runtime = (current_time - heater_start_time) / 60  # Convert to minutes
                total_runtime += runtime
                client.publish(AIO_FEED_RUNTIME, str(int(total_runtime)))
            client.publish(AIO_FEED_HEATER_STATUS, "0")  # Publish status change
            log(f"Relay turned OFF (Temperature {temperature:.1f}°F above high limit {temp_high:.1f}°F)")

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

# Add to global variables
last_ping = 0
ping_interval = 30  # seconds

def ping_mqtt(client):
    """Send a ping to keep connection alive"""
    try:
        client.ping()
        return True
    except Exception as e:
        log(f"Ping failed: {e}")
        return False

# Add to global variables
last_logged_temp = None
last_logged_bounds = None

def main():
    global time_manager, thermostat, temp_sensor, last_ping, timer_end_time, current_timer_hours, relay_state
    
    if connect_to_wifi(SSID, PASSWORD):
        blink_led(5, 0.2)

        client = create_mqtt_client()
        client.set_callback(lambda topic, msg: mqtt_callback(topic, msg, client))
        gc.collect()
        
        try:
            log("Attempting MQTT connection...")
            client.connect()
            log("MQTT connected successfully")
            # Subscribe to all control feeds
            client.subscribe(AIO_FEED_RELAY)
            client.subscribe(AIO_FEED_SETPOINT)
            client.subscribe(AIO_FEED_CYCLE_DELAY)
            client.subscribe(AIO_FEED_TEMP_DIFF)
            client.subscribe(AIO_FEED_MIN_TIME_ON)
            client.subscribe(AIO_FEED_HEATER_STATUS)
            client.subscribe(AIO_FEED_TIMER)  # Add this line
            
            time_manager = TimeManager(client, log)
            
            # Wait for initial state before proceeding
            if not get_initial_state(client):
                log("Failed to get initial state, restarting...")
                return
            
            log("Initial state received, starting temperature monitoring...")
            
        except Exception as e:
            log(f"Failed to connect to MQTT broker: {e}")
            return

        # Initialize all tracking variables
        last_temp_update = time.time()
        last_ping = time.time()  # Keep ping monitoring
        last_logged_temp = None
        last_timer_check = time.time()  # NEW: Initialize timer check

        try:
            while True:
                try:
                    client.check_msg()  # Check for incoming messages from subscriptions
                    
                    current_time = time.time()
                    
                    # Keep connection health monitoring
                    if current_time - last_ping >= ping_interval:
                        if not ping_mqtt(client):
                            raise OSError("Ping failed")
                        last_ping = current_time

                    # Time sync check (only when needed)
                    if time_manager:
                        time_manager.check_sync_needed()

                    try:
                        temperature = get_valid_temperature()
                        
                        # Only log if significant change
                        if last_logged_temp is None or abs(temperature - last_logged_temp) > 0.5:
                            log(f"Temp Reading: {temperature:.2f} °F")
                            last_logged_temp = temperature
                            
                        # Always publish every 60 seconds (no log)
                        if current_time - last_temp_update >= 60:
                            client.publish(AIO_FEED_TEMP, f"{temperature:.2f}")
                            last_temp_update = current_time
                            
                        update_relay_state(temperature, client)
                    except RuntimeError as e:
                        log(f"Error getting temperature: {e}")
                        continue

                    log_wifi_status()
                    time.sleep(5)
                    gc.collect()

                    # Timer check and update
                    if timer_end_time > 0:
                        if current_time >= timer_end_time:
                            timer_end_time = 0
                            current_timer_hours = 0
                            relay_state = False
                            client.publish(AIO_FEED_RELAY, "OFF")
                            client.publish(AIO_FEED_TIMER, "0")  # Changed: Remove "R:" prefix for final 0
                            log("Timer expired, relay commanded OFF")
                        elif current_time - last_timer_check >= 60:  # Update every minute
                            remaining_hours = (timer_end_time - current_time) / 3600
                            if remaining_hours != current_timer_hours:
                                current_timer_hours = remaining_hours
                                client.publish(AIO_FEED_TIMER, f"R:{remaining_hours:.2f}")
                            last_timer_check = current_time

                except OSError as e:
                    log(f"MQTT Error: {e}")
                    client = reconnect_with_backoff(client)
                    if not client:
                        return

        except KeyboardInterrupt:
            log("Exiting...")
            client.disconnect()

if __name__ == "__main__":
    main()

