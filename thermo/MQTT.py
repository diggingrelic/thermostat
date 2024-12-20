import time
import gc
import config as config
from umqtt.simple2 import MQTTClient
from mysecrets import AIO_USERNAME, AIO_KEY
from Log import log
from thermo.TimeManager import TimeManager
from thermo.RelayControl import RelayControl
from thermo.Temperature import Temperature
class ThermoState:
    def __init__(self):
        self.relay_state = False
        self.setpoint = 70.0
        self.cycle_delay = 5.0
        self.temp_diff = 2.0
        self.min_time_on = 5
        self.timer_end_time = 0
        self.current_timer_hours = 0

# Create a module-level instance
thermo_state = ThermoState()

def get_state():
    return thermo_state

relayControl = RelayControl()
temperatures = Temperature()

current_relay_state = relayControl.current_relay_state

last_mqtt_attempt = time.time()
timer_end_time = 0
current_timer_hours = 0

MQTT_KEEPALIVE = 120 
MQTT_RECONNECT_DELAY = 5
status_update_interval = 300  # 5 minutes for OK updates


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
    
def handle_mqtt_connection(client, wifi_state):
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

def mqtt_callback(topic, msg, retained=False, duplicate=False):
    """
    Callback for received subscription messages.
    
    Args:
        topic: The topic the message was received on
        msg: The message content
        retained: Boolean indicating if this was a retained message
        duplicate: Boolean indicating if this was a duplicate delivery
    """
    topic_str = topic.decode() if isinstance(topic, bytes) else topic
    msg_str = msg.decode()
    
    log(f"MQTT message received - Topic: {topic_str}, Message: {msg_str}")
    
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
                client.publish(config.AIO_FEED_TIMER, "0")  # Update timer feed
                log("Timer cancelled due to relay OFF command")
            log(f"Relay command received: {thermo_state.relay_state} (Actual state: {current_relay_state}, min time holding: {current_relay_state and not thermo_state.relay_state})")
        else:
            log(f"Relay command unchanged: command={thermo_state.relay_state}, actual={current_relay_state}")
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
                    client.publish(config.AIO_FEED_RELAY, "OFF")
                    log("Timer cancelled, relay commanded OFF")
            else:
                hours = float(msg_str)
                if 1 <= hours <= 12:  # Between 1 and 12 hours
                    thermo_state.timer_end_time = time.time() + (hours * 3600)  # Convert to seconds
                    thermo_state.current_timer_hours = hours
                    thermo_state.relay_state = True
                    client.publish(config.AIO_FEED_RELAY, "ON")
                    log(f"Timer set for {hours:.1f} hours, relay commanded ON")
                else:
                    log(f"Invalid timer duration: {hours} (must be between 1-12 hours)")
        except ValueError:
            log(f"Invalid timer format: {msg_str} (use hours between 1-12)")

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
        temp = temperatures.get_valid_temperature()
        state_info = (f"T:{temp:.1f}F S:{thermo_state.setpoint:.1f}F "
                     f"H:{'ON' if current_relay_state else 'OFF'}")
        if thermo_state.timer_end_time > 0:
            hours_left = (thermo_state.timer_end_time - current_time) / 3600
            state_info += f" TMR:{hours_left:.1f}h"
        message = f"{message} - {state_info}"
    
    # Only send if forced, message changed, or interval elapsed for OK messages
    if (force or 
        message != last_status_message or 
        (message.startswith("OK") and current_time - last_status_update >= status_update_interval)):
        
        try:
            client.publish(config.AIO_FEED_STATUS, message[:127])  # Limit to 128 chars
            log(f"Status update: {message}")
            last_status_message = message
            last_status_update = current_time
            if config.debug:
                log(f"Status update: {message}")
        except Exception as e:
            log(f"Failed to send status: {e}")
