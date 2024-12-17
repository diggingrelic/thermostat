# type: ignore
class HeaterController:
    def __init__(self, relay_pin, time_manager, cycle_time=600, temp_differential=2.0):
        """
        Initialize HeaterController with timing controls
        
        Args:
            relay_pin: machine.Pin object for controlling the relay
            time_manager: TimeManager instance for accurate timing
            cycle_time: minimum time (seconds) between cycles (default: 10 minutes)
            temp_differential: temperature differential in °F (default: 2°F)
        """
        self.relay = relay_pin
        self.time_manager = time_manager
        self.cycle_time = cycle_time
        self.temp_differential = temp_differential
        self.last_off_time = 0
        self.last_on_time = 0
        self._is_on = False
        self.min_on_time = 900  # 15 minutes in seconds

    def can_turn_on(self):
        """Check if enough time has passed since last cycle to turn on"""
        if not self.time_manager.has_valid_time():
            return False
        current_time = self.time_manager.get_current_time()
        if not self._is_on and current_time - self.last_off_time < self.cycle_time:
            return False
        return True

    def can_turn_off(self):
        """Check if heater has been on for minimum time"""
        if not self.time_manager.has_valid_time():
            return True  # Allow turn off if time sync is invalid
        if self._is_on:
            elapsed_on_time = self.time_manager.get_current_time() - self.last_on_time
            return elapsed_on_time >= self.min_on_time
        return True

    def should_turn_on(self, current_temp, setpoint):
        """Determine if heater should turn on based on temperature differential"""
        if self._is_on:
            if not self.can_turn_off():
                return True  # Must stay on to meet minimum time
            return current_temp <= (setpoint + self.temp_differential)
        else:
            return current_temp <= (setpoint - self.temp_differential)

    def turn_on(self):
        """Attempt to turn on the heater, respecting cycle time"""
        if self.can_turn_on():
            self.relay.on()
            self._is_on = True
            self.last_on_time = self.time_manager.get_current_time()
            return True
        return False

    def turn_off(self):
        """Turn off the heater if minimum on time has been met"""
        if self.can_turn_off():
            self.relay.off()
            self._is_on = False
            self.last_off_time = self.time_manager.get_current_time()
            return True
        return False

    def update_cycle_time(self, minutes):
        """Update cycle time based on minutes value from feed"""
        try:
            minutes = float(minutes)
            if 5 <= minutes <= 20:
                self.cycle_time = minutes * 60  # Convert minutes to seconds
                return True
            return False
        except (ValueError, TypeError):
            return False

    def update_temp_differential(self, diff):
        """Update temperature differential"""
        try:
            diff = float(diff)
            if 1 <= diff <= 5:
                self.temp_differential = diff
                return True
            return False
        except (ValueError, TypeError):
            return False

    def get_status(self):
        """Get current status of heater controller"""
        current_time = self.time_manager.get_current_time()
        return {
            "is_on": self._is_on,
            "cycle_time": self.cycle_time / 60,  # Convert to minutes
            "temp_differential": self.temp_differential,
            "min_on_time": self.min_on_time / 60,  # Convert to minutes
            "time_since_last_off": current_time - self.last_off_time if self.last_off_time else 0,
            "time_since_last_on": current_time - self.last_on_time if self.last_on_time else 0,
            "can_turn_on": self.can_turn_on(),
            "can_turn_off": self.can_turn_off()
        }