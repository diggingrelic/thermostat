# type: ignore
import network
import time
import machine
import gc
from umqtt.simple2 import MQTTClient
from umqtt.simple2 import MQTTException
import config.py as config
from TempSensorADT7410 import TempSensorADT7410
from mysecrets import SSID, PASSWORD, AIO_USERNAME, AIO_KEY
from TimeManager import TimeManager
from thermo.PinDriveStrength import set_pin_drive_strength

# Initialize controllers
led = machine.Pin("LED", machine.Pin.OUT)
relay_pin = machine.Pin(14, machine.Pin.OUT)
set_pin_drive_strength(14, 12)

# Initialize state variables
setpoint = 40.0  # Default setpoint
relay_state = False  # Track desired relay state
current_relay_state = False  # Track actual relay state
cycle_delay = 10  # Default cycle delay in minutes
temp_diff = 2.0  # Default temperature differential in °F
last_relay_change = 0  # Track when relay last changed state
min_time_on = 15  # Default minimum on-time in minutes
heater_status = 0  # Track current heater status (0=off, 1=on)
timer_end_time = 0  # When timer should finish (0 = no timer)
current_timer_hours = 0  # Current timer value for feed updates

# Initialize runtime tracking
heater_start_time = 0  # Track when heater turns on
total_runtime = 0  # Track accumulated runtime in minutes
daily_runtime = 0  # Track runtime for current day

# Initialize status tracking
last_status_message = ""  # Track last message to prevent duplicates
last_logged_temp = None  # Track last logged temperature
last_logged_bounds = None  # Track last logged temperature bounds

# Initialize connection state
mqtt_state = config.MQTT_STATE_DISCONNECTED
wifi_state = config.WIFI_STATE_DISCONNECTED

# Initialize time management
time_manager = None  # Initialize time_manager as None

# I2C configuration for TempSensorADT7410
i2c = machine.I2C(0, scl=machine.Pin(5), sda=machine.Pin(4), freq=100000)
temp_sensor = TempSensorADT7410(i2c)

last_status_update = 0
status_update_interval = 300  # 5 minutes for OK updates
last_runtime_status = 0  # Track last runtime status message

last_daily_post = 0  # Track when we last posted daily cost

last_wifi_status = None
last_wifi_check = 0

last_ping = 0
ping_interval = 30  # seconds

# Initialize MQTT timing variables
last_mqtt_attempt = time.time()
timer_end_time = 0
current_timer_hours = 0

def log(message):
    if config.debug:
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

def mqtt_callback(topic, msg, retained=False, duplicate=False):
    """
    Callback for received subscription messages.
    
    Args:
        topic: The topic the message was received on
        msg: The message content
        retained: Boolean indicating if this was a retained message
        duplicate: Boolean indicating if this was a duplicate delivery
    """
    global client, setpoint, relay_state, temp_diff, cycle_delay, min_time_on, heater_status, current_timer_hours, timer_end_time
    topic_str = topic.decode() if isinstance(topic, bytes) else topic
    msg_str = msg.decode()
    
    log(f"MQTT message received - Topic: {topic_str}, Message: {msg_str}")
    
    if topic_str == "time/seconds":
        if time_manager:
            time_manager.handle_time_message(msg)
    elif topic_str == config.AIO_FEED_RELAY:
        new_relay_state = msg_str.lower() == "on"
        if new_relay_state != relay_state:
            relay_state = new_relay_state
            # Clear timer if relay commanded OFF
            if not relay_state and timer_end_time > 0:
                timer_end_time = 0
                current_timer_hours = 0
                client.publish(config.AIO_FEED_TIMER, "0")  # Update timer feed
                log("Timer cancelled due to relay OFF command")
            log(f"Relay command received: {relay_state} (Actual state: {current_relay_state}, min time holding: {current_relay_state and not relay_state})")
        else:
            log(f"Relay command unchanged: command={relay_state}, actual={current_relay_state}")
    elif topic_str == config.AIO_FEED_SETPOINT:
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
    elif topic_str == config.AIO_FEED_CYCLE_DELAY:
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
    elif topic_str == config.AIO_FEED_TEMP_DIFF:
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
    elif topic_str == config.AIO_FEED_MIN_TIME_ON:
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
    elif topic_str == config.AIO_FEED_HEATER_STATUS:
        try:
            new_status = int(msg_str)
            if new_status != heater_status:
                heater_status = new_status
                log(f"Heater status updated: {heater_status}")
        except ValueError:
            log("Invalid heater status value received")
    elif topic_str == config.AIO_FEED_TIMER:
        try:
            if msg_str.startswith('R:'):  # Handle remaining time updates
                if timer_end_time == 0:  # Only process if we're initializing
                    hours = float(msg_str[2:])  # Skip "R:" prefix
                    if hours > 0:  # If there's time remaining
                        timer_end_time = time.time() + (hours * 3600)  # Convert to seconds
                        current_timer_hours = hours
                        relay_state = True
                        log(f"Timer restored: {hours:.2f} hours remaining")
                return
                
            if msg_str.lower() in ('0', 'off'):
                if timer_end_time > 0:
                    timer_end_time = 0
                    current_timer_hours = 0
                    relay_state = False
                    client.publish(config.AIO_FEED_RELAY, "OFF")
                    log("Timer cancelled, relay commanded OFF")
            else:
                hours = float(msg_str)
                if 1 <= hours <= 12:  # Between 1 and 12 hours
                    timer_end_time = time.time() + (hours * 3600)  # Convert to seconds
                    current_timer_hours = hours
                    relay_state = True
                    client.publish(config.AIO_FEED_RELAY, "ON")
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
    global setpoint, temp_diff, cycle_delay, relay_state, min_time_on
    
    for retry in range(retries):
        log("Fetching initial state...")
        received_values = set()
        
        # Request values
        client.publish(config.AIO_FEED_RELAY + "/get", "")
        time.sleep(0.5)
        client.publish(config.AIO_FEED_SETPOINT + "/get", "")
        time.sleep(0.1)
        client.publish(config.AIO_FEED_CYCLE_DELAY + "/get", "")
        time.sleep(0.1)
        client.publish(config.AIO_FEED_TEMP_DIFF + "/get", "")
        time.sleep(0.1)
        client.publish(config.AIO_FEED_MIN_TIME_ON + "/get", "")
        time.sleep(0.1)
        client.publish(config.AIO_FEED_TIMER + "/get", "")
        time.sleep(0.1)
        
        timeout = time.time() + 5
        while time.time() < timeout:
            try:
                client.check_msg()
                
                # Mark values as received when we get any response
                if isinstance(setpoint, float):
                    received_values.add('setpoint')
                if isinstance(cycle_delay, float):
                    received_values.add('cycle_delay')
                if isinstance(temp_diff, float):
                    received_values.add('temp_diff')
                if isinstance(min_time_on, float):
                    received_values.add('min_time_on')
                if isinstance(relay_state, bool):
                    received_values.add('relay')
                if isinstance(timer_end_time, (int, float)):
                    received_values.add('timer')
                
                # If we got all values, we're done
                if len(received_values) >= 6:
                    log(f"Received all initial values after {retry + 1} attempts")
                    return True
                    
            except Exception as e:
                log(f"Error checking messages: {e}")
            
            time.sleep(0.1)
        
        # Log what we're still waiting for
        missing = {'setpoint', 'cycle_delay', 'temp_diff', 'min_time_on', 'relay', 'timer'} - received_values
        log(f"Attempt {retry + 1}: Still waiting for: {', '.join(missing)}")
        
        if retry < retries - 1:
            time.sleep(1)
    
    return False

# Add to global variables
MQTT_KEEPALIVE = 120  # Increase keepalive interval
MQTT_RECONNECT_DELAY = 5  # 5 seconds between reconnection attempts

def create_mqtt_client():
    """Create and configure MQTT client with proper settings"""
    global client
    client = MQTTClient(
        client_id="pico",
        server=config.AIO_SERVER,
        port=config.AIO_PORT,
        user=AIO_USERNAME,
        password=AIO_KEY,
        keepalive=MQTT_KEEPALIVE,
        socket_timeout=5,
        message_timeout=10
    )
    return client

def reconnect_with_backoff(client, attempts=5):
    """Reconnect to MQTT with exponential backoff"""
    delay = MQTT_RECONNECT_DELAY
    for attempt in range(attempts):
        try:
            log(f"Reconnecting to MQTT broker, attempt {attempt + 1}...")
            # Properly clean up the old client
            try:
                client.disconnect()
            except Exception as e:
                log(f"Error during disconnect: {e}")
            
            time.sleep(1)
            gc.collect()

            client = create_mqtt_client()
            client.set_callback(mqtt_callback)
            client.connect()
            time.sleep(1)

            # Subscribe to all feeds
            feeds = [
                config.AIO_FEED_RELAY,
                config.AIO_FEED_SETPOINT,
                config.AIO_FEED_CYCLE_DELAY,
                config.AIO_FEED_TEMP_DIFF,
                config.AIO_FEED_MIN_TIME_ON,
                config.AIO_FEED_TIMER
            ]
            
            for feed in feeds:
                client.subscribe(feed)
                
            log("Reconnected and subscribed to feeds.")
            send_status(client, "OK RECONNECT: MQTT restored")
            return client

        except Exception as e:
            log(f"Reconnection attempt {attempt + 1} failed: {e}")
            send_status(client, f"ERROR: MQTT reconnect failed - attempt {attempt + 1}")
            delay *= 2  # Exponential backoff
            if attempt < attempts - 1:
                time.sleep(delay)

    log("Failed to reconnect after multiple attempts.")
    return None

def format_runtime(minutes):
    """Convert total minutes to 'X hours & Y min' format"""
    hours = int(minutes // 60)
    mins = int(minutes % 60)
    if hours > 0:
        return f"{hours}h {mins}m"
    return f"{mins}m"

def update_runtime(client, force_update=False):
    """Update runtime. Call periodically and when heater turns off."""
    global daily_runtime, heater_start_time, last_runtime_update, last_runtime_status
    
    if current_relay_state and heater_start_time > 0:
        current_time = time.time()
        
        # If this is our first update for this session
        if last_runtime_update == 0:
            last_runtime_update = heater_start_time
            
        # Calculate minutes since last update
        minutes_since_update = (current_time - last_runtime_update) / 60
        
        # Only update if significant change (> 1 minute) or forced
        if force_update or minutes_since_update >= 1:
            daily_runtime += minutes_since_update
            last_runtime_update = current_time  # Remember when we did this update
            
            # Send runtime status every 15 minutes
            if current_time - last_runtime_status >= 900:  # 15 minutes = 900 seconds
                send_status(client, f"OK RUNTIME: {format_runtime(daily_runtime)}")
                last_runtime_status = current_time
            
            if config.debug:
                log(f"Runtime updated: {format_runtime(daily_runtime)}")

def update_relay_state(temperature, client):
    global current_relay_state, last_relay_change, last_logged_bounds
    global heater_start_time, heater_status, last_runtime_update, last_runtime_status
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
            update_runtime(client, force_update=True)  # Update before turning off
            current_relay_state = False
            last_relay_change = current_time
            relay_pin.off()
            led.off()
            heater_status = 0
            heater_start_time = 0  # Reset start time
            last_runtime_update = 0  # Reset runtime update tracker
            last_runtime_status = 0  # Reset runtime status tracker
            client.publish(config.AIO_FEED_HEATER_STATUS, "0")
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
        if current_relay_state:
            update_runtime(client, force_update=True)  # Update before state change
        
        current_relay_state = desired_relay_state
        last_relay_change = current_time
        
        if current_relay_state:
            relay_pin.on()
            led.on()
            heater_start_time = current_time
            heater_status = 1
            client.publish(config.AIO_FEED_HEATER_STATUS, "1")
            log(f"Relay turned ON (Temperature {temperature:.1f}°F below low limit {temp_low:.1f}°F)")
        else:
            relay_pin.off()
            led.off()
            heater_status = 0
            heater_start_time = 0  # Reset start time
            last_runtime_update = 0  # Reset runtime update tracker
            last_runtime_status = 0  # Reset runtime status tracker
            client.publish(config.AIO_FEED_HEATER_STATUS, "0")
            log(f"Relay turned OFF (Temperature {temperature:.1f}°F above high limit {temp_high:.1f}°F)")

def log_wifi_status(client):
    """Monitor WiFi status and send meaningful updates"""
    global last_wifi_status, last_wifi_check
    current_time = time.time()
    if current_time - last_wifi_check >= 10:
        wlan = network.WLAN(network.STA_IF)
        current_status = wlan.isconnected()
        if current_status != last_wifi_status:
            if current_status:
                log(f"Wi-Fi connected, IP: {wlan.ifconfig()[0]}")
                send_status(client, "OK RECONNECT: WiFi restored")
            else:
                log("Wi-Fi disconnected.")
                send_status(client, "ERROR: WiFi disconnected")
            last_wifi_status = current_status
        last_wifi_check = current_time

def ping_mqtt(client):
    """Send a ping to keep connection alive"""
    try:
        client.ping()
        return True
    except Exception as e:
        log(f"Ping failed: {e}")
        return False

def send_status(client, message, force=False):
    """
    Send status update to status feed.
    Args:
        client: MQTT client
        message: Status message without timestamp
        force: If True, send even if duplicate
    """
    global last_status_update, last_status_message
    current_time = time.time()
    
    # Don't send duplicate errors within 5 minutes unless forced
    if (message.startswith("ERROR") and 
        last_status_message.startswith("ERROR") and 
        current_time - last_status_update < 300 and 
        not force):
        return
        
    # For OK messages, add current state
    if message.startswith("OK") and not message.startswith("OK BOOT"):
        temp = get_valid_temperature()
        state_info = (f"T:{temp:.1f}F S:{setpoint:.1f}F "
                     f"H:{'ON' if current_relay_state else 'OFF'}")
        if timer_end_time > 0:
            hours_left = (timer_end_time - current_time) / 3600
            state_info += f" TMR:{hours_left:.1f}h"
        message = f"{message} - {state_info}"
    
    # Only send if forced, message changed, or interval elapsed for OK messages
    if (force or 
        message != last_status_message or 
        (message.startswith("OK") and current_time - last_status_update >= status_update_interval)):
        
        try:
            client.publish(config.AIO_FEED_STATUS, message[:128])  # Limit to 128 chars
            last_status_message = message
            last_status_update = current_time
            if config.debug:
                log(f"Status update: {message}")
        except Exception as e:
            log(f"Failed to send status: {e}")

def check_wifi_connection():
    """Monitor WiFi and attempt reconnection"""
    global wifi_state, last_wifi_attempt
    current_time = time.time()
    last_wifi_attempt = 0
    WIFI_RETRY_DELAY = 10  # seconds
    
    wlan = network.WLAN(network.STA_IF)
    if not wlan.isconnected():
        if wifi_state != config.WIFI_STATE_DISCONNECTED:
            wifi_state = config.WIFI_STATE_DISCONNECTED
            log("WiFi disconnected")
            
        # Only attempt reconnect after delay
        if current_time - last_wifi_attempt >= WIFI_RETRY_DELAY:
            last_wifi_attempt = current_time
            wifi_state = config.WIFI_STATE_CONNECTING
            log("Attempting WiFi connection...")
            
            wlan.active(False)
            time.sleep(1)
            wlan.active(True)
            wlan.connect(SSID, PASSWORD)
            
            # Wait briefly for connection
            for _ in range(10):
                if wlan.isconnected():
                    wifi_state = config.WIFI_STATE_CONNECTED
                    log(f"WiFi connected, IP: {wlan.ifconfig()[0]}")
                    return True
                time.sleep(1)
            
            log("WiFi connection attempt failed")
            return False
            
    else:
        if wifi_state != config.WIFI_STATE_CONNECTED:
            wifi_state = config.WIFI_STATE_CONNECTED
            log(f"WiFi connected, IP: {wlan.ifconfig()[0]}")
        return True

def handle_mqtt_connection(client):
    """State machine for MQTT connection handling"""
    global mqtt_state, last_mqtt_attempt
    current_time = time.time()
    MQTT_RETRY_DELAY = 30
    
    # Only attempt reconnection if WiFi is available
    if wifi_state != config.WIFI_STATE_CONNECTED:
        mqtt_state = config.MQTT_STATE_DISCONNECTED
        return client
        
    if mqtt_state == config.MQTT_STATE_DISCONNECTED:
        if current_time - last_mqtt_attempt >= MQTT_RETRY_DELAY:
            last_mqtt_attempt = current_time
            mqtt_state = config.MQTT_STATE_CONNECTING
            client = reconnect_with_backoff(client)
            if client:
                mqtt_state = config.MQTT_STATE_CONNECTED
                
    return client

def set_mqtt_connected(success=True):
    """Set MQTT state and log the change"""
    global mqtt_state
    mqtt_state = config.MQTT_STATE_CONNECTED if success else config.MQTT_STATE_DISCONNECTED
    if config.debug:
        log(f"MQTT state changed to: {'CONNECTED' if success else 'DISCONNECTED'}")

def update_daily_cost(client, force=False):
    """Update daily cost at 10:45 PM or when forced"""
    global daily_runtime, last_daily_post
    
    current_time = time.localtime()
    
    # Check if it's 10:45 PM (22:45) or if force update requested
    if force or (current_time[3] == 22 and current_time[4] == 45):
        # Only post once per day
        if last_daily_post != current_time[7]:  # current_time[7] is day of year
            # Convert runtime from minutes to hours and calculate cost
            daily_cost = (daily_runtime / 60) * config.HEATER_COST_PER_HOUR
            
            try:
                client.publish(config.AIO_FEED_DAILY_COST, f"{daily_cost:.2f}")
                log(f"Daily cost posted: ${daily_cost:.2f} ({daily_runtime} minutes)")
                send_status(client, f"OK DAILY: Cost=${daily_cost:.2f} Runtime={format_runtime(daily_runtime)}")
                
                # Add status message before reset
                send_status(client, f"OK RESET: Daily runtime reset from {format_runtime(daily_runtime)} to 0")
                
                # Reset for next day
                last_daily_post = current_time[7]
                daily_runtime = 0
                
            except Exception as e:
                log(f"Failed to post daily cost: {e}")
                send_status(client, f"ERROR: Failed to post daily cost - {str(e)}")

def main():
    global last_logged_temp, client
    global time_manager, thermostat, temp_sensor, last_ping, timer_end_time, current_timer_hours, relay_state, last_runtime_update
    
    if connect_to_wifi(SSID, PASSWORD):
        blink_led(5, 0.2)

        client = create_mqtt_client()
        client.set_callback(mqtt_callback)
        
        try:
            log("Attempting MQTT connection...")
            client.connect()
            set_mqtt_connected(True)
            log("MQTT connected successfully")
            
            # Subscribe to feeds
            feeds = [
                config.AIO_FEED_RELAY,
                config.AIO_FEED_SETPOINT,
                config.AIO_FEED_CYCLE_DELAY,
                config.AIO_FEED_TEMP_DIFF,
                config.AIO_FEED_MIN_TIME_ON,
                config.AIO_FEED_HEATER_STATUS,
                config.AIO_FEED_TIMER
            ]
            
            for feed in feeds:
                client.subscribe(feed)
                time.sleep(0.1)
            
            time_manager = TimeManager(client, log)
            
            if not get_initial_state(client):
                send_status(client, "ERROR: Failed to get initial state")
                log("Failed to get initial state, restarting...")
                return
            
            send_status(client, "OK BOOT: System ready", force=True)
            log("Initial state received, starting temperature monitoring...")
            
        except Exception as e:
            log(f"Failed to connect to MQTT broker: {e}")
            return

        # Initialize all tracking variables
        last_temp_update = time.time()
        last_ping = time.time()
        last_timer_check = time.time()
        last_runtime_update = time.time()

        try:
            while True:
                try:
                    # Check WiFi state first
                    if not check_wifi_connection():
                        time.sleep(1)  # Don't spin if WiFi is down
                        continue
                        
                    # Handle MQTT connection state
                    client = handle_mqtt_connection(client)
                    if not client or mqtt_state != config.MQTT_STATE_CONNECTED:
                        time.sleep(1)  # Don't spin if MQTT is down
                        continue
                        
                    try:
                        client.check_msg()
                    except Exception as e:
                        if isinstance(e, MQTTException) and e.args[0] == 1:
                            log("MQTT connection closed by host")
                            set_mqtt_connected(False)
                            client = reconnect_with_backoff(client)
                            if not client:
                                return
                        else:
                            log(f"Main loop error: {e}")
                            set_mqtt_connected(False)
                            time.sleep(1)
                            continue

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
                            send_status(client, "OK")  # Periodic OK status
                            
                        # Always publish every 60 seconds (no log)
                        if current_time - last_temp_update >= 60:
                            client.publish(config.AIO_FEED_TEMP, f"{temperature:.2f}")
                            last_temp_update = current_time
                            
                        update_relay_state(temperature, client)
                        
                        # Update runtime and daily cost
                        update_runtime(client)
                        update_daily_cost(client)
                        
                    except RuntimeError as e:
                        error_msg = f"ERROR: Temperature sensor - {str(e)}"
                        send_status(client, error_msg, force=True)
                        log(f"Error getting temperature: {e}")
                        continue

                    log_wifi_status(client)
                    time.sleep(5)
                    gc.collect()

                    # Timer check and update
                    if timer_end_time > 0:
                        if current_time >= timer_end_time:
                            timer_end_time = 0
                            current_timer_hours = 0
                            relay_state = False
                            client.publish(config.AIO_FEED_RELAY, "OFF")
                            client.publish(config.AIO_FEED_TIMER, "0")
                            log("Timer expired, relay commanded OFF")
                        elif current_time - last_timer_check >= 60:  # Update every minute
                            remaining_hours = (timer_end_time - current_time) / 3600
                            if remaining_hours != current_timer_hours:
                                current_timer_hours = remaining_hours
                                client.publish(config.AIO_FEED_TIMER, f"R:{remaining_hours:.2f}")
                            last_timer_check = current_time

                    # Periodic runtime update (every minute)
                    if current_time - last_runtime_update >= 60:
                        update_runtime(client)
                        last_runtime_update = current_time

                except OSError as e:
                    error_msg = f"ERROR: MQTT connection lost - {str(e)}"
                    send_status(client, error_msg, force=True)
                    log(f"MQTT Error: {e}")
                    set_mqtt_connected(False)
                    time.sleep(1)
                    continue

        except Exception as e:
            log(f"Main loop error: {e}")
            set_mqtt_connected(False)
            time.sleep(1)

if __name__ == "__main__":
    main()
