import time
#import gc
import config as config
from umqtt.robust2 import MQTTClient
from mysecrets import AIO_USERNAME, AIO_KEY
from thermo.Log import log
from thermo.Temperature import Temperature
from thermo.ThermoState import get_state, format_runtime
import random

temperatures = Temperature()

thermo_state = get_state()

last_mqtt_attempt = 0
timer_end_time = 0
current_timer_hours = 0

MQTT_KEEPALIVE = 600
MQTT_RECONNECT_DELAY = 5
status_update_interval = 300  # 5 minutes for OK updates

# Add these at the module level (near the top with other initializations)
last_status_message = ""  # Track last message to prevent duplicates
last_status_update = 0    # Track when last status was sent

def callback_wrapper(topic, msg, *args):
    """Wrapper for MQTT callback to handle exceptions"""
    try:
        mqtt_callback(topic, msg)
    except Exception as e:
        log(f"Callback error: {type(e).__name__}: {str(e)}")
        all_args = [topic, msg] + list(args)
        log(f"Args received: {all_args}")

def create_mqtt_client():
    """Create MQTT client with appropriate settings"""
    client = MQTTClient(
        client_id=f"pico_{random.randint(0, 1000000)}",  # Add random ID to prevent conflicts
        server=config.AIO_SERVER,
        port=config.AIO_PORT,
        user=AIO_USERNAME,
        password=AIO_KEY,
        keepalive=MQTT_KEEPALIVE,
        socket_timeout=30,
        message_timeout=10
    )
    return client

def handle_mqtt_connection(state):
    """Handle MQTT connection state"""
    
    # If client is None, create a new one
    if state.client is None:
        log("Creating new MQTT client")
        state.client = create_mqtt_client()
        state.client.set_callback(callback_wrapper)
        
    if state.client and not state.client.is_conn_issue():
        log("MQTT connection check - no issues detected.")
        return
    log("MQTT connection check - issues detected.")
    # Connection issue detected
    set_mqtt_connected(False)
    state.mqtt_state = config.MQTT_STATE_DISCONNECTED
    
    try:
        if state.client:
            state.client.disconnect()
    except Exception as e:
        log(f"Error during disconnect: {e}")
    
    try:
        # Create new client if needed
        if state.client is None:
            state.client = create_mqtt_client()
            state.client.set_callback(callback_wrapper)
            
        # Attempt reconnection
        state.client.connect()
        set_mqtt_connected(True)
        state.mqtt_state = config.MQTT_STATE_CONNECTED
        
        # Resubscribe to all topics
        for feed in config.SUBSCRIBE_FEEDS:
            state.client.subscribe(feed)
            time.sleep(0.1)
            
        send_status(state.client, "OK RECONNECT: MQTT restored")
        return
            
    except Exception as e:
        log(f"Reconnection error: {e}")
        time.sleep(5)
        
    return

def set_mqtt_connected(connected):
    """Update MQTT connection state"""
    state = get_state()
    if connected:
        state.mqtt_state = config.MQTT_STATE_CONNECTED
    else:
        state.mqtt_state = config.MQTT_STATE_DISCONNECTED
    log(f"MQTT state changed to: {state.mqtt_state}")

def mqtt_callback(topic, msg):
    """MQTT callback function"""
    topic_str = topic.decode() if isinstance(topic, bytes) else topic
    msg_str = msg.decode()
    
    log(f"MQTT message received - Topic: {topic_str}, Message: {msg_str}")

    if topic_str == config.AIO_FEED_TIMER:        
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
                    if not thermo_state.is_startup:
                        thermo_state.relay_state = False
                        thermo_state.is_startup = False
                        if thermo_state.client:  # Check if client still exists
                            thermo_state.client.publish(config.AIO_FEED_RELAY, "OFF")
                        log("Timer cancelled, relay commanded OFF")
                    else:
                        pass
            else:
                hours = float(msg_str)
                if 1 <= hours <= 12:  # Between 1 and 12 hours
                    thermo_state.timer_end_time = time.time() + (hours * 3600)  # Convert to seconds
                    thermo_state.current_timer_hours = hours
                    thermo_state.relay_state = True
                    if thermo_state.client:  # Check if client still exists
                        thermo_state.client.publish(config.AIO_FEED_RELAY, "ON")
                    log(f"Timer set for {hours:.1f} hours, relay commanded ON")
                else:
                    log(f"Invalid timer duration: {hours} (must be between 1-12 hours)")
        except ValueError:
            log(f"Invalid timer format: {msg_str} (use hours between 1-12)")

    elif topic_str == config.AIO_FEED_RELAY:
        new_relay_state = msg_str.lower() == "on"
        if new_relay_state != thermo_state.relay_state:
            log("About to update relay state")
            thermo_state.relay_state = new_relay_state
            # Clear timer if relay commanded OFF
            if not thermo_state.relay_state and thermo_state.timer_end_time > 0:
                log("About to clear timer")
                thermo_state.timer_end_time = 0
                thermo_state.current_timer_hours = 0
                log("About to publish timer update")
                thermo_state.client.publish(config.AIO_FEED_TIMER, "0")
                log("Timer cancelled due to relay OFF command")
            #log(f"Relay command received: {thermo_state.relay_state} (Actual state: {thermo_state.current_relay_state}, min time holding: {thermo_state.current_relay_state and not thermo_state.relay_state})")
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
                    log(f"Temperature differential: {current} -> {thermo_state.temp_diff} F")
                else:
                    log(f"Invalid temperature differential value: {new_temp_diff} (must be between 1-5 F)")
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

def get_time_for_manager():
    thermo_state.client.publish(f"{AIO_USERNAME}/feeds/time/get", "")

def send_daily_cost(daily_cost):
    thermo_state.client.publish(config.AIO_FEED_DAILY_COST, f"{daily_cost:.2f}")
    log(f"Daily cost posted: ${daily_cost:.2f} ({thermo_state.daily_runtime} minutes)")
    send_status(thermo_state.client, f"OK DAILY: Cost=${daily_cost:.2f} Runtime={format_runtime(thermo_state.daily_runtime)}")
    send_status(thermo_state.client, f"OK RESET: Daily runtime reset from {format_runtime(thermo_state.daily_runtime)} to 0")

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