import time
import gc
import config as config
from umqtt.robust2 import MQTTClient
from mysecrets import AIO_USERNAME, AIO_KEY
from thermo.Log import log
from thermo.TimeManager import TimeManager
from thermo.Temperature import Temperature
from thermo.ThermoState import get_state

temperatures = Temperature()

thermo_state = get_state()

last_mqtt_attempt = 0
timer_end_time = 0
current_timer_hours = 0

MQTT_KEEPALIVE = 60
MQTT_RECONNECT_DELAY = 5
status_update_interval = 300  # 5 minutes for OK updates

# Add these at the module level (near the top with other initializations)
last_status_message = ""  # Track last message to prevent duplicates
last_status_update = 0    # Track when last status was sent

global client

def callback_wrapper(topic, msg, *args):
    """Wrapper for MQTT callback to handle exceptions"""
    try:
        mqtt_callback(topic, msg)
    except Exception as e:
        log(f"Callback error: {type(e).__name__}: {str(e)}")
        all_args = [topic, msg] + list(args)
        log(f"Args received: {all_args}")

def create_mqtt_client():
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

def reconnect_with_backoff(client, max_retries=5):
    """Reconnect to MQTT with exponential backoff"""
    state = get_state()# Get state object
    delay = MQTT_RECONNECT_DELAY
    for attempt in range(max_retries):
        try:
            log(f"Reconnecting to MQTT broker, attempt {attempt + 1}...")
            try:
                client.disconnect()
            except Exception as e:
                log(f"Error during disconnect: {e}")
            
            time.sleep(1)
            gc.collect()

            client = create_mqtt_client()
            client.connect()
            time.sleep(1)
            
            for feed in config.SUBSCRIBE_FEEDS:
                client.subscribe(feed)
                
            log("Reconnected and subscribed to feeds.")
            send_status(client, "OK RECONNECT: MQTT restored")
            state.mqtt_state = config.MQTT_STATE_CONNECTED
            return client
            
        except Exception as e:
            log(f"Reconnection attempt {attempt + 1} failed: {e}")
            send_status(client, f"ERROR: MQTT reconnect failed - attempt {attempt + 1}")
            delay *= 2  # Exponential backoff
            if attempt < max_retries - 1:
                time.sleep(delay)

    log("Failed to reconnect after multiple attempts.")
    state.mqtt_state = config.MQTT_STATE_DISCONNECTED
    return None

def handle_mqtt_connection(client, state):
    """Handle MQTT connection state"""
    thermo_state = get_state()
    
    # If client is None, create a new one
    if client is None:
        log("Creating new MQTT client")
        client = create_mqtt_client()
        client.set_callback(callback_wrapper)
        thermo_state.client = client  # Store in state
        
    if client and not client.is_conn_issue():
        return client
        
    # Connection issue detected
    set_mqtt_connected(False)
    state.mqtt_state = config.MQTT_STATE_DISCONNECTED
    
    try:
        if client:
            client.disconnect()
    except Exception as e:
        log(f"Error during disconnect: {e}")
    
    try:
        # Create new client if needed
        if client is None:
            client = create_mqtt_client()
            client.set_callback(callback_wrapper)
            thermo_state.client = client  # Store in state
            
        # Attempt reconnection
        client.connect()
        set_mqtt_connected(True)
        state.mqtt_state = config.MQTT_STATE_CONNECTED
        thermo_state.client = client  # Ensure state has latest client
        
        # Resubscribe to all topics
        for feed in config.SUBSCRIBE_FEEDS:
            client.subscribe(feed)
            time.sleep(1)
            
        send_status(client, "OK RECONNECT: MQTT restored")
        return client
            
    except Exception as e:
        log(f"Reconnection error: {e}")
        time.sleep(5)
        
    return client

def set_mqtt_connected(connected):
    """Update MQTT connection state"""
    state = get_state()
    if connected:
        state.mqtt_state = config.MQTT_STATE_CONNECTED
    else:
        state.mqtt_state = config.MQTT_STATE_DISCONNECTED
    log(f"MQTT state changed to: {state.mqtt_state}")

def get_initial_state(mqtt_client):
    """Fetch initial state from Adafruit IO"""
    global client
    client = mqtt_client
    
    log("Fetching initial state...")
    max_attempts = 3
    
    # Initialize a dictionary to track the reception of each feed
    feed_received = {
        'relay': False,
        'setpoint': False,
        'cycle_delay': False,
        'temp_diff': False,
        'min_time_on': False
    }
    
    def check_received():
        """Check if all feed values have been received"""
        return all(feed_received.values())
    
    for attempt in range(max_attempts):
        if check_received():
            log("Received all initial values.")
            break
        
        log(f"Starting attempt {attempt + 1} of {max_attempts}")
        
        # Request values for feeds that haven't been received yet
        for feed, received in feed_received.items():
            if not received:
                feed_name = getattr(config, f'AIO_FEED_{feed.upper()}')
                mqtt_client.get(feed_name)
                log(f"Requested value for {feed}")
                time.sleep(1)  # Small delay between requests
        
        # Wait for responses
        timeout = time.time() + 2  # 2 second timeout
        while time.time() < timeout:
            if client:
                client.check_msg()
            time.sleep(0.1)
        
        log(f"Completed attempt {attempt + 1} of {max_attempts}")
        
        if not check_received() and attempt < max_attempts - 1:
            time.sleep(2)  # Wait 2 seconds before next attempt
            log("Retrying initial state fetch...")
    
    if not check_received():
        missing_feeds = [feed for feed, received in feed_received.items() if not received]
        log(f"Failed to receive all values. Missing: {', '.join(missing_feeds)}")
        return False
    
    log("Initial state fetch complete")
    return True

def mqtt_callback(topic, msg):
    """MQTT callback function"""

    # Get a reference to the current client
    current_client = thermo_state.client  # Use the global client
    topic_str = topic.decode() if isinstance(topic, bytes) else topic
    msg_str = msg.decode()
    
    log(f"MQTT message received - Topic: {topic_str}, Message: {msg_str}")
    
    if current_client is None:
        log("Warning: MQTT client is None during callback")
        return
        
    if topic_str == "time/seconds":
        time_manager = TimeManager(client, log)
        if time_manager:
            time_manager.handle_time_message(msg)

    elif topic_str == config.AIO_FEED_RELAY:
        new_relay_state = msg_str.lower() == "on"
        if new_relay_state != thermo_state.relay_state:
            thermo_state.relay_state = new_relay_state
            # Clear timer if relay commanded OFF
            if not thermo_state.relay_state and thermo_state.timer_end_time > 0:
                thermo_state.timer_end_time = 0
                thermo_state.current_timer_hours = 0
                current_client.publish(config.AIO_FEED_TIMER, "0")  # Update timer feed
                log("Timer cancelled due to relay OFF command")
            log(f"Relay command received: {thermo_state.relay_state} (Actual state: {thermo_state.current_relay_state}, min time holding: {thermo_state.current_relay_state and not thermo_state.relay_state})")
        else:
            log(f"Relay command unchanged: command={thermo_state.relay_state}, actual={thermo_state.current_relay_state}")

    elif topic_str == config.AIO_FEED_SETPOINT:
        try:
            new_setpoint = float(msg_str)
            if new_setpoint != thermo_state.setpoint:
                current = thermo_state.setpoint  # Store current value
                thermo_state.setpoint = new_setpoint
                log(f"Setpoint: {current} -> {thermo_state.setpoint}")
            else:
                log(f"Setpoint unchanged: current={thermo_state.setpoint}, received={new_setpoint}")
        except ValueError:
            log("Invalid setpoint value received")
            
    elif topic_str == config.AIO_FEED_CYCLE_DELAY:
        try:
            new_cycle_delay = float(msg_str)
            if new_cycle_delay != thermo_state.cycle_delay:
                if 1 <= new_cycle_delay <= 20:
                    current = thermo_state.cycle_delay  # Store current value
                    thermo_state.cycle_delay = new_cycle_delay
                    log(f"Cycle delay: {current} -> {thermo_state.cycle_delay} minutes")
                else:
                    log(f"Invalid cycle delay value: {new_cycle_delay} (must be between 1-20 minutes)")
            else:
                log(f"Cycle delay unchanged: current={thermo_state.cycle_delay}, received={new_cycle_delay}")
        except ValueError:
            log("Invalid cycle delay value received")
            
    elif topic_str == config.AIO_FEED_TEMP_DIFF:
        try:
            new_temp_diff = float(msg_str)
            if new_temp_diff != thermo_state.temp_diff:
                if 1 <= new_temp_diff <= 5:
                    current = thermo_state.temp_diff  # Store current value
                    thermo_state.temp_diff = new_temp_diff
                    log(f"Temperature differential: {current} -> {thermo_state.temp_diff}°F")
                else:
                    log(f"Invalid temperature differential value: {new_temp_diff} (must be between 1-5°F)")
            else:
                log(f"Temperature differential unchanged: current={thermo_state.temp_diff}, received={new_temp_diff}")
        except ValueError:
            log("Invalid temperature differential value received")
            
    elif topic_str == config.AIO_FEED_MIN_TIME_ON:
        try:
            new_min_time = float(msg_str)
            if new_min_time != thermo_state.min_time_on:
                if 1 <= new_min_time <= 60:
                    current = thermo_state.min_time_on  # Store current value
                    thermo_state.min_time_on = new_min_time
                    log(f"Minimum on-time: {current} -> {thermo_state.min_time_on} minutes")
                else:
                    log(f"Invalid minimum on-time value: {new_min_time} (must be between 1-60 minutes)")
            else:
                log(f"Minimum on-time unchanged: current={thermo_state.min_time_on}, received={new_min_time}")
        except ValueError:
            log("Invalid minimum on-time value received")

    elif topic_str == config.AIO_FEED_HEATER_STATUS:
        try:
            new_status = int(msg_str)
            if new_status != thermo_state.heater_status:
                thermo_state.heater_status = new_status
                log(f"Heater status updated: {thermo_state.heater_status}")
        except ValueError:
            log("Invalid heater status value received")

    elif topic_str == config.AIO_FEED_TIMER:
        try:
            if msg_str.startswith('R:'):  # Handle remaining time updates
                if thermo_state.timer_end_time == 0:  # Only process if we're initializing
                    hours = float(msg_str[2:])  # Skip "R:" prefix
                    if hours > 0:  # If there's time remaining
                        thermo_state.timer_end_time = time.time() + (hours * 3600)  # Convert to seconds
                        thermo_state.current_timer_hours = hours
                        thermo_state.relay_state = True
                        log(f"Timer restored: {hours:.2f} hours remaining")
                return
                
            if msg_str.lower() in ('0', 'off'):
                if thermo_state.timer_end_time > 0:
                    thermo_state.timer_end_time = 0
                    thermo_state.current_timer_hours = 0
                    thermo_state.relay_state = False
                    if current_client:  # Check if client still exists
                        current_client.publish(config.AIO_FEED_RELAY, "OFF")
                    log("Timer cancelled, relay commanded OFF")
            else:
                hours = float(msg_str)
                if 1 <= hours <= 12:  # Between 1 and 12 hours
                    thermo_state.timer_end_time = time.time() + (hours * 3600)  # Convert to seconds
                    thermo_state.current_timer_hours = hours
                    thermo_state.relay_state = True
                    if current_client:  # Check if client still exists
                        current_client.publish(config.AIO_FEED_RELAY, "ON")
                    log(f"Timer set for {hours:.1f} hours, relay commanded ON")
                else:
                    log(f"Invalid timer duration: {hours} (must be between 1-12 hours)")
        except ValueError:
            log(f"Invalid timer format: {msg_str} (use hours between 1-12)")

def send_status(client, message, force=False):
    global last_status_message, last_status_update
    current_time = time.time()
    
    # Don't send duplicate errors within 5 minutes unless forced
    if (message.startswith("ERROR") and 
        last_status_message.startswith("ERROR") and 
        current_time - last_status_update < 300 and 
        not force):
        return
        
    # For OK messages, add current state
    if message.startswith("OK") and not message.startswith("OK BOOT"):
        temp = temperatures.get_valid_temperature()
        state_info = (f"T:{temp:.1f}F S:{thermo_state.setpoint:.1f}F "
                     f"H:{'ON' if thermo_state.current_relay_state else 'OFF'}")
        if thermo_state.timer_end_time > 0:
            hours_left = (thermo_state.timer_end_time - current_time) / 3600
            state_info += f" TMR:{hours_left:.1f}h"
        message = f"{message} - {state_info}"
    
    # Only send if forced, message changed, or interval elapsed for OK messages
    if (force or 
        message != last_status_message or 
        (message.startswith("OK") and current_time - last_status_update >= status_update_interval)):
        
        try:
            # The robust2 library will queue the message if it can't send immediately
            client.publish(config.AIO_FEED_STATUS, message[:125])
            log(f"Status queued/sent: {message}")

            # Update our tracking variables
            last_status_message = message
            last_status_update = current_time
            return True
            
        except Exception as e:
            log(f"Failed to queue status: {type(e).__name__} - {str(e)}")
            return False