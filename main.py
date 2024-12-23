# type: ignore
import time
import machine
import gc
#from umqtt.simple2 import MQTTException
import config
from thermo.TimeManager import TimeManager
from thermo.PinDriveStrength import set_pin_drive_strength
from thermo.Log import log
from thermo.ThermoState import get_state, update_runtime, format_runtime
from thermo.MQTT import send_status, set_mqtt_connected, handle_mqtt_connection, create_mqtt_client, callback_wrapper
from thermo.WiFi import WiFi
from thermo.RelayControl import RelayControl
from thermo.Temperature import Temperature

# Check reset cause (using available constants)
reset_cause = machine.reset_cause()
if reset_cause == 3:  # 3 is typically WDT reset
    log("Watchdog reset occurred")
else:
    log("Normal startup")

# Initialize controllers
led = machine.Pin("LED", machine.Pin.OUT)
relay_pin = machine.Pin(14, machine.Pin.OUT)
set_pin_drive_strength(14, 12)

# Initialize runtime tracking
heater_start_time = 0  # Track when heater turns on
total_runtime = 0  # Track accumulated runtime in minutes
daily_runtime = 0  # Track runtime for current day

# Initialize status tracking
last_status_message = ""  # Track last message to prevent duplicates
last_logged_temp = None  # Track last logged temperature
last_logged_bounds = None  # Track last logged temperature bounds

state = get_state()
state.relay_pin = relay_pin
state.led = led
state.wifi_state = config.WIFI_STATE_DISCONNECTED
state.mqtt_state = config.MQTT_STATE_DISCONNECTED
state.timer_end_time = 0  # When timer should finish (0 = no timer)
state.current_timer_hours = 0  # Current timer value for feed updates

# Initialize time management
time_manager = None  # Initialize time_manager as None

# Initialize with no watchdog
wdt = None
wifi = WiFi(wdt, watchdog_enabled=False)

# Try initial connection without watchdog
if not wifi.connect_to_wifi():
    log("Initial WiFi connection failed, restarting...")
    time.sleep(1)
    machine.reset()
else:
    state.wifi_state = config.WIFI_STATE_CONNECTED

# Now enable watchdog after successful connection
wdt = machine.WDT(timeout=8000)
wifi.wdt = wdt  # Update WiFi instance with new WDT
wifi.watchdog_enabled = True

relay_control = RelayControl()

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
    for retry in range(retries):
        log("Fetching initial state...")
        received_values = set()
        
        # Request values
        client.publish(config.AIO_FEED_TIMER + "/get", "")
        time.sleep(1)
        feed_watchdog()
        client.publish(config.AIO_FEED_RELAY + "/get", "")
        time.sleep(.1)
        client.publish(config.AIO_FEED_SETPOINT + "/get", "")
        time.sleep(.1)
        client.publish(config.AIO_FEED_CYCLE_DELAY + "/get", "")
        time.sleep(.1)
        client.publish(config.AIO_FEED_TEMP_DIFF + "/get", "")
        time.sleep(.1)
        client.publish(config.AIO_FEED_MIN_TIME_ON + "/get", "")
        time.sleep(.1)
        
        timeout = time.time() + 5
        while time.time() < timeout:
            try:
                client.check_msg()
                feed_watchdog()
                
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

def feed_watchdog():
    """Safe watchdog feed with enable/disable flag"""
    if wifi.watchdog_enabled and wifi.wdt:
        wifi.wdt.feed()

def main():
    feed_watchdog()

    if state.wifi_state == config.WIFI_STATE_CONNECTED:
        state.client = create_mqtt_client()
        state.client.set_callback(callback_wrapper)
        feed_watchdog()

        try:
            log("Attempting MQTT connection...")
            state.client.connect()
            set_mqtt_connected(True)
            log("MQTT connected successfully")
            
            for feed in config.SUBSCRIBE_FEEDS:
                state.client.subscribe(feed)
                time.sleep(1)
                feed_watchdog()
            
            time_manager = TimeManager(state.client, log) #?????
            
            if not get_initial_state(state.client):
                send_status(state.client, "ERROR: Failed to get initial state restarting...")
                time.sleep(3)
                machine.reset()
                return
            
            send_status(state.client, "OK BOOT: System ready", force=True)
            feed_watchdog()

        except Exception as e:
            log(f"Failed to connect to MQTT broker: {e}")
            return

        # Initialize all tracking variables
        last_temp_update = time.time() - 61
        last_timer_check = time.time()
        last_runtime_update = time.time()
        last_logged_temp = None

        temperatures = Temperature()

        state.is_startup = False
        
        try:
            running = True
            while running:
                try:
                    # Check WiFi state first
                    if not wifi.check_wifi_connection(state.wifi_state):
                        state.wifi_state = config.WIFI_STATE_DISCONNECTED
                        time.sleep(1)
                        feed_watchdog()
                        continue
                        
                    # Handle MQTT connection state
                    client = handle_mqtt_connection(state.client, state)
                    if not client or state.mqtt_state != config.MQTT_STATE_CONNECTED:
                        time.sleep(1)
                        feed_watchdog()
                        continue
                        
                    try:
                        feed_watchdog()
                        client.ping()
                        client.check_msg()
                        client.send_queue()
                    except Exception as e:
                        log(f"Main loop error: {e}")
                        set_mqtt_connected(False)
                        time.sleep(1)
                        continue

                    current_time = time.time()

                    # Time sync check (only when needed)
                    if time_manager:
                        time_manager.check_sync_needed()

                    try:
                        feed_watchdog()
                        temperature = temperatures.get_valid_temperature()
                        
                        # Only log if significant change
                        if last_logged_temp is None or abs(temperature - last_logged_temp) > 1:
                            last_logged_temp = temperature
                            send_status(client, f"OK TEMP: {temperature:.2f} F")
                            
                        # Always publish every 60 seconds to Adafruit IO
                        if current_time - last_temp_update >= 20:
                            client.publish(config.AIO_FEED_TEMP, f"{temperature:.2f}")
                            last_temp_update = current_time
                            
                        relay_control.update_relay_state(temperature, client, state)

                        # Update runtime and daily cost
                        update_runtime(client)
                        update_daily_cost(client)
                        
                    except RuntimeError as e:
                        error_msg = f"ERROR: Temperature sensor - {str(e)}"
                        send_status(client, error_msg, force=True)
                        continue

                    wifi.log_wifi_status(client)
                    feed_watchdog()
                    time.sleep(5)
                    feed_watchdog()
                    gc.collect()

                    # Timer check and update
                    if state.timer_end_time > 0:
                        if current_time >= state.timer_end_time:
                            state.timer_end_time = 0
                            state.current_timer_hours = 0
                            state.relay_state = False
                            client.publish(config.AIO_FEED_RELAY, "OFF")
                            client.publish(config.AIO_FEED_TIMER, "0")
                            log("Timer expired, relay commanded OFF")
                        elif current_time - last_timer_check >= 60:  # Update every minute
                            remaining_hours = (state.timer_end_time - current_time) / 3600
                            if remaining_hours != state.current_timer_hours:
                                state.current_timer_hours = remaining_hours
                                client.publish(config.AIO_FEED_TIMER, f"R:{remaining_hours:.2f}")
                            last_timer_check = current_time

                    # Periodic runtime update (every minute)
                    if current_time - last_runtime_update >= 60:
                        if update_runtime():  # If runtime was updated
                            # Check if it's time for 15-minute status
                            if current_time - state.last_runtime_status >= 900:  # 15 minutes = 900 seconds
                                runtime_msg = format_runtime(state.daily_runtime)
                                send_status(client, f"OK RUNTIME: {runtime_msg}")
                                state.last_runtime_status = current_time
                        last_runtime_update = current_time

                    feed_watchdog()

                except Exception as e:
                    log(f"Main loop error: {e}")
                    set_mqtt_connected(False)
                    running = False
                    continue

        except Exception as e:
            log(f"Main loop error: {e}")
            set_mqtt_connected(False)
            time.sleep(1)

if __name__ == "__main__":
    main()



