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

# Create a module-level instance
thermo_state = ThermoState()

def get_state():
    return thermo_state

def set_client(new_client):
    """Update the client reference"""
    thermo_state.client = new_client

def format_runtime(minutes):
    """Convert total minutes to 'X hours & Y min' format"""
    hours = int(minutes // 60)
    mins = int(minutes % 60)
    if hours > 0:
        return f"{hours}h {mins}m"
    return f"{mins}m"

def update_runtime(force_update=False):
    """Update runtime without MQTT dependency"""
    current_time = time.time()
    
    if thermo_state.current_relay_state and thermo_state.heater_start_time > 0:
        if thermo_state.last_runtime_update == 0:
            thermo_state.last_runtime_update = thermo_state.heater_start_time
            
        minutes_since_update = (current_time - thermo_state.last_runtime_update) / 60
        
        if force_update or minutes_since_update >= 1:
            thermo_state.daily_runtime += minutes_since_update
            thermo_state.last_runtime_update = current_time
            return True  # Indicate an update occurred
    
    return False
