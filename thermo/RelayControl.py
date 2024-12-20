# type: ignore
import time
from Log import log
#from MQTT import send_status
from config import AIO_FEED_HEATER_STATUS

class RelayControl:
    def __init__(self, relay_pin, led):
        # Hardware components
        self.relay_pin = relay_pin
        self.led = led
        
        # Control parameters
        self.setpoint = 40.0  # Default setpoint
        self.temp_diff = 2.0  # Default differential
        self.min_time_on = 15  # Minutes
        self.cycle_delay = 10  # Minutes
        
        # State tracking
        self.relay_state = False  # Desired state
        self.current_relay_state = False  # Actual state
        self.last_relay_change = time.time()
        self.heater_start_time = 0
        self.heater_status = 0
        
        # Runtime tracking
        self.last_runtime_update = 0
        self.last_runtime_status = 0
        self.total_runtime = 0
        
        # Logging
        self.last_logged_bounds = None

    def set_setpoint(self, value):
        """Update temperature setpoint"""
        self.setpoint = float(value)
        
    def set_temp_diff(self, value):
        """Update temperature differential"""
        self.temp_diff = float(value)
        
    def set_min_time(self, value):
        """Update minimum on time"""
        self.min_time_on = float(value)
        
    def set_cycle_delay(self, value):
        """Update cycle delay"""
        self.cycle_delay = float(value)
        
    def command_relay(self, state):
        """Command relay to desired state"""
        self.relay_state = state
        
    def get_status(self):
        """Get current status of relay control"""
        return {
            'relay_state': self.relay_state,
            'current_state': self.current_relay_state,
            'heater_status': self.heater_status,
            'runtime': self.total_runtime,
            'setpoint': self.setpoint,
            'temp_diff': self.temp_diff
        }

    def update_relay_state(self, temperature, client, setpoint, temp_diff):

        current_time = time.time()
        
        # Calculate temperature bounds using differential
        temp_high = self.setpoint + self.temp_diff
        temp_low = self.setpoint - self.temp_diff
        
        # Only log bounds if they've changed
        current_bounds = (temp_low, temp_high, setpoint, temp_diff)
        if current_bounds != self.last_logged_bounds:
            log(f"Control bounds: {temp_low:.1f}°F to {temp_high:.1f}°F (Setpoint: {self.setpoint:.1f}°F ±{self.temp_diff:.1f}°F)")
            self.last_logged_bounds = current_bounds
        
        # Check if relay is currently on
        if self.current_relay_state:
            time_on = current_time - self.heater_start_time
            # If commanded OFF and minimum time met, turn off
            if not self.relay_state and time_on >= (self.min_time_on * 60):
                self.update_runtime(client, force_update=True)  # Update before turning off
                self.current_relay_state = False
                self.last_relay_change = current_time
                self.relay_pin.off()
                self.led.off()
                self.heater_status = 0
                self.heater_start_time = 0  # Reset start time
                self.last_runtime_update = 0  # Reset runtime update tracker
                self.last_runtime_status = 0  # Reset runtime status tracker
                client.publish(AIO_FEED_HEATER_STATUS, "0")
                log("Relay turned OFF (minimum time elapsed)")
                return
            # If minimum time not met, stay on
            elif not self.relay_state and time_on < (self.min_time_on * 60):
                remaining = (self.min_time_on * 60) - time_on
                log(f"Relay commanded OFF but held ON for minimum time: {remaining:.1f} seconds remaining")
                return
        
        # Only check cycle delay if we're trying to turn ON
        if not self.current_relay_state and self.relay_state:
            if current_time - self.last_relay_change < self.cycle_delay * 60:
                remaining = (self.cycle_delay * 60) - (current_time - self.last_relay_change)
                log(f"Cycle delay: {remaining:.1f} seconds remaining")
                return
        
        # Changed: Proper differential logic
        desired_relay_state = self.relay_state and self.setpoint >= 30 and (
            (not self.current_relay_state and temperature <= temp_low) or  # Turn ON below low limit
            (self.current_relay_state and temperature <= temp_high)        # Stay ON until high limit
        )

        if desired_relay_state != self.current_relay_state:
            if self.current_relay_state:
                self.update_runtime(client, force_update=True)  # Update before state change
            
            self.current_relay_state = desired_relay_state
            self.last_relay_change = current_time
            
            if self.current_relay_state:
                self.relay_pin.on()
                self.led.on()
                self.heater_start_time = current_time
                self.heater_status = 1
                client.publish(AIO_FEED_HEATER_STATUS, "1")
                log(f"Relay turned ON (Temperature {temperature:.1f}°F below low limit {temp_low:.1f}°F)")
            else:
                self.relay_pin.off()
                self.led.off()
                self.heater_status = 0
                self.heater_start_time = 0  # Reset start time
                self.last_runtime_update = 0  # Reset runtime update tracker
                self.last_runtime_status = 0  # Reset runtime status tracker
                client.publish(AIO_FEED_HEATER_STATUS, "0")
                log(f"Relay turned OFF (Temperature {temperature:.1f}°F above high limit {temp_high:.1f}°F)")
            
            return self.heater_status, self.heater_start_time, self.last_runtime_update, self.last_runtime_status