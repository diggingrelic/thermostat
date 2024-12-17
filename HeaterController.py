import time

class HeaterController:
    def __init__(self, relay_pin, cycle_time=600, temp_differential=2.0):  # 600 seconds = 10 minutes
        """
        Initialize HeaterController with timing controls
        
        Args:
            relay_pin: machine.Pin object for controlling the relay
            cycle_time: minimum time (seconds) between cycles (default: 10 minutes)
            temp_differential: temperature differential in degrees Fahrenheit (default: 2°F)
        """
        self.relay = relay_pin
        self.cycle_time = cycle_time
        self.temp_differential = temp_differential
        self.last_off_time = 0
        self.last_on_time = 0
        self._is_on = False
        self._cycle_start = 0  # Track when the current cycle started

    def can_turn_on(self):
        """Check if enough time has passed since last cycle to turn on"""
        current_time = time.time()
        if not self._is_on and current_time - self.last_off_time < self.cycle_time:
            return False
        return True

    def get_remaining_cycle_time(self):
        """Get remaining time in current cycle (seconds)"""
        if not self._is_on:
            elapsed = time.time() - self.last_off_time
            remaining = max(0, self.cycle_time - elapsed)
            return remaining
        return 0

    def update_cycle_time(self, minutes):
        """
        Update cycle time based on minutes value from feed
        Args:
            minutes: time in minutes (5-20)
        Returns:
            bool: True if update was successful
        """
        try:
            minutes = float(minutes)
            if 5 <= minutes <= 20:
                self.cycle_time = minutes * 60  # Convert minutes to seconds
                return True
            return False
        except (ValueError, TypeError):
            return False

    def should_turn_on(self, current_temp, setpoint):
        """Determine if heater should turn on based on temperature differential"""
        if self._is_on:
            # If already on, wait until we're above setpoint + differential
            return current_temp <= (setpoint + self.temp_differential)
        else:
            # If off, wait until we're below setpoint - differential
            return current_temp <= (setpoint - self.temp_differential)

    def turn_on(self):
        """Attempt to turn on the heater, respecting cycle time"""
        if self.can_turn_on():
            self.relay.on()
            self._is_on = True
            self.last_on_time = time.time()
            return True
        return False

    def turn_off(self):
        """Turn off the heater and record the time"""
        self.relay.off()
        self._is_on = False
        self.last_off_time = time.time()

    def is_on(self):
        """Return current state of heater"""
        return self._is_on

    def update_temp_differential(self, diff):
        """
        Update temperature differential
        Args:
            diff: temperature differential in °F (1-5)
        Returns:
            bool: True if update was successful
        """
        try:
            diff = float(diff)
            if 1 <= diff <= 5:
                self.temp_differential = diff
                return True
            return False
        except (ValueError, TypeError):
            return False