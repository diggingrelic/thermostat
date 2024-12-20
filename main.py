# type: ignore
import time
import machine
import gc
from umqtt.simple2 import MQTTException
import config
from thermo.TimeManager import TimeManager
from thermo.PinDriveStrength import set_pin_drive_strength
from Log import log
from MQTT import get_state,send_status, set_mqtt_connected, handle_mqtt_connection, reconnect_with_backoff, create_mqtt_client, mqtt_callback
from WiFi import WiFi
from thermo.RelayControl import RelayControl
from thermo.Temperature import Temperature
# Initialize watchdog with 8 second timeout
wdt = machine.WDT(timeout=8000)

# Check reset cause
if machine.reset_cause() == machine.WATCHDOG_RESET:
    log("Watchdog reset occurred")
else:
    log("Normal startup")

# Initialize controllers
led = machine.Pin("LED", machine.Pin.OUT)
relay_pin = machine.Pin(14, machine.Pin.OUT)
set_pin_drive_strength(14, 12)

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
wifi_state = config.WIFI_STATE_DISCONNECTED

ping_interval = 30
mqtt_state = config.MQTT_STATE_DISCONNECTED

# Initialize time management
time_manager = None  # Initialize time_manager as None

temperatures = Temperature()

wifi = WiFi()
relay_control = RelayControl(relay_pin, led)

last_status_update = 0
last_runtime_status = 0  # Track last runtime status message

last_daily_post = 0  # Track when we last posted daily cost

last_wifi_status = None
last_wifi_check = 0

def blink_led(times, delay):
    """Blink onboard LED"""
    for _ in range(times):
        led.on()
        time.sleep(delay)
        led.off()
        time.sleep(delay)

def get_initial_state(client, retries=3):
    """Fetch initial state with retries"""
    state = get_state()  # Get reference to the shared state

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
                if isinstance(state.setpoint, float):
                    received_values.add('setpoint')
                if isinstance(state.cycle_delay, float):
                    received_values.add('cycle_delay')
                if isinstance(state.temp_diff, float):
                    received_values.add('temp_diff')
                if isinstance(state.min_time_on, float):
                    received_values.add('min_time_on')
                if isinstance(state.relay_state, bool):
                    received_values.add('relay')
                if isinstance(state.timer_end_time, (int, float)):
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
    
    if relay_control.current_relay_state and heater_start_time > 0:
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
    machine.wdt.feed()
    state = get_state()  # Get reference to the shared state

    if wifi.connect_to_wifi():
        wifi_state = config.WIFI_STATE_CONNECTED
        blink_led(5, 0.2)
        machine.wdt.feed()

        client = create_mqtt_client()
        client.set_callback(mqtt_callback)
        
        try:
            log("Attempting MQTT connection...")
            client.connect()
            set_mqtt_connected(True)
            log("MQTT connected successfully")
            
            for feed in config.SUBSCRIBE_FEEDS:
                client.subscribe(feed)
                time.sleep(1)
                machine.wdt.feed()
            
            time_manager = TimeManager(client, log) #?????
            
            if not get_initial_state(client):
                send_status(client, "ERROR: Failed to get initial state restarting...")
                time.sleep(5)
                machine.reset()
                return
            
            send_status(client, "OK BOOT: System ready", force=True)
            
        except Exception as e:
            log(f"Failed to connect to MQTT broker: {e}")
            return

        # Initialize all tracking variables
        last_temp_update = time.time()
        last_timer_check = time.time()
        last_runtime_update = time.time()
        timer_end_time = time.time()
        last_logged_temp = None

        try:
            running = True
            while running:
                try:
                    # Check WiFi state first
                    if not wifi.check_wifi_connection():
                        time.sleep(5)
                        machine.wdt.feed()
                        continue
                        
                    # Handle MQTT connection state
                    client = handle_mqtt_connection(client, wifi_state)
                    if not client or mqtt_state != config.MQTT_STATE_CONNECTED:
                        time.sleep(4)
                        machine.wdt.feed()
                        continue
                        
                    try:
                        client.check_msg()
                    except Exception as e:
                        if isinstance(e, MQTTException) and e.args[0] == 1:
                            log("MQTT connection closed by host")
                            set_mqtt_connected(False)
                            try:
                                client.disconnect()
                            except Exception as e:
                                log(f"ERROR: MQTT - {str(e)}")
                                pass

                            client = reconnect_with_backoff(client, max_retries=5)
                            if not client:
                                log("Failed to reconnect MQTT after maximum retries")
                                return
                        else:
                            log(f"Main loop error: {e}")
                            set_mqtt_connected(False)
                            try:
                                client.disconnect()
                            except Exception as e:
                                log(f"ERROR: MQTT - {str(e)}")
                                pass
                            time.sleep(1)

                    current_time = time.time()

                    # Time sync check (only when needed)
                    if time_manager:
                        time_manager.check_sync_needed()

                    try:
                        temperature = temperatures.get_valid_temperature()
                        
                        # Only log if significant change
                        if last_logged_temp is None or abs(temperature - last_logged_temp) > 1:
                            last_logged_temp = temperature
                            send_status(client, f"OK TEMP: {temperature:.2f} °F")
                            
                        # Always publish every 60 seconds to Adafruit IO
                        if current_time - last_temp_update >= 60:
                            client.publish(config.AIO_FEED_TEMP, f"{temperature:.2f}")
                            last_temp_update = current_time
                            
                        relay_control.update_relay_state(temperature, client, state.setpoint, state.temp_diff)
                        
                        # Update runtime and daily cost
                        update_runtime(client)
                        update_daily_cost(client)
                        
                    except RuntimeError as e:
                        error_msg = f"ERROR: Temperature sensor - {str(e)}"
                        send_status(client, error_msg, force=True)
                        continue

                    wifi.log_wifi_status(client)
                    time.sleep(5)
                    gc.collect()

                    # Timer check and update
                    if timer_end_time > 0:
                        if current_time >= timer_end_time:
                            timer_end_time = 0
                            current_timer_hours = 0
                            relay_control.relay_state = False
                            state.relay_state = False
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
                    set_mqtt_connected(False)
                    running = False
                    continue

        except Exception as e:
            log(f"Main loop error: {e}")
            set_mqtt_connected(False)
            time.sleep(1)

if __name__ == "__main__":
    main()
