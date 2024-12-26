from config import MQTT_STATE_DISCONNECTED
import time
from thermo.TimeManager import TimeManager

# TODO: Remove is_startup flag

class ThermoState:
    def __init__(self):
        self.relay_state = False
        self.setpoint = 70.0
        self.cycle_delay = 5.0
        self.temp_diff = 2.0
        self.min_time_on = 5
        self.timer_end_time = 0
        self.current_timer_hours = 0
        self.relay_pin = 15
        self.led = 16
        self.wifi_state = None
        self.mqtt_state = MQTT_STATE_DISCONNECTED
        self.client = None
        self.current_relay_state = False
        self.heater_status = 0
        self.heater_start_time = 0
        self.last_runtime_update = 0
        self.last_runtime_status = 0
        self.last_relay_change = time.time() - 3600
        self.heater_start_time = 0
        self.is_startup = True
        self.last_sync = 0
        self.timeManager = TimeManager()
        # For CostCalculator
        self.daily_runtime = 0
        self.last_daily_post = 0
        self.daily_runtime_seconds = 0  # Track in seconds instead of minutes

# Create a module-level instance
thermo_state = ThermoState()

def get_state():
    return thermo_state

def set_client(new_client):
    """Update the client reference"""
    thermo_state.client = new_client

def format_runtime(seconds):
    """Convert seconds to display format"""
    minutes = seconds / 60
    hours = int(minutes // 60)
    mins = int(minutes % 60)
    if hours > 0:
        return f"{hours}h {mins}m"
    return f"{mins}m"

def update_runtime(force_update=False):
    """Update daily runtime tracking in seconds"""
    current_time = time.time()

    if thermo_state.current_relay_state and thermo_state.heater_start_time > 0:
        if thermo_state.last_runtime_update == 0:
            thermo_state.last_runtime_update = thermo_state.heater_start_time
            return False

        # Calculate the current runtime since the heater was turned on
        current_runtime = current_time - thermo_state.heater_start_time 

        # Only update if forced OR the current runtime has increased
        if force_update or current_runtime > thermo_state.daily_runtime_seconds:
            thermo_state.daily_runtime_seconds = current_runtime 
            return True
    
    return False

def get_runtime_minutes():
    """Convert runtime seconds to minutes for display/calculations"""
    return thermo_state.daily_runtime_seconds / 60
