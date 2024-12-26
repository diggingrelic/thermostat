import time
import config
from thermo.MQTT import send_daily_cost
from thermo.ThermoState import get_state

class CostCalculator:
    def __init__(self):
        pass

    def update_daily_cost(self, force=False):
        """Update daily cost at 10:45 PM or when forced"""
        current_time = time.localtime()
        
        # Check if it's 10:45 PM (22:45) or if force update requested
        if force or (current_time[3] == 22 and current_time[4] == 45):
            state = get_state()
            # Only post once per day
            if state.last_daily_post != current_time[7]:  # current_time[7] is day of year
                # Convert runtime from seconds to hours and calculate cost
                daily_cost = (state.daily_runtime_seconds / 3600) * config.HEATER_COST_PER_HOUR
                send_daily_cost(daily_cost)
                state.last_daily_post = current_time[7]
                state.daily_runtime_seconds = 0  # Reset after posting
