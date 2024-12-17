# type: ignore
class ThermostatController:
    def __init__(self, heater, led, time_manager, setpoint=40.0):
        """
        Initialize ThermostatController
        
        Args:
            heater: HeaterController instance
            led: LED pin for status indication
            time_manager: TimeManager instance for accurate timing
            setpoint: Initial temperature setpoint in °F (default: 40°F)
        """
        self.heater = heater
        self.led = led
        self.time_manager = time_manager
        self.setpoint = setpoint
        self.enabled = False
        self.last_update_time = 0

    def set_enabled(self, enabled):
        """Enable or disable the thermostat"""
        self.enabled = enabled.lower() == "on" if isinstance(enabled, str) else enabled
        if not self.enabled:
            self.heater.turn_off()
            self.led.off()

    def set_setpoint(self, setpoint):
        """Update the temperature setpoint"""
        try:
            new_setpoint = float(setpoint)
            if 35 <= new_setpoint <= 90:  # Reasonable temperature range
                self.setpoint = new_setpoint
                return True
            return False
        except (ValueError, TypeError):
            return False

    def update(self, temperature):
        """
        Update the thermostat state based on current temperature
        
        Args:
            temperature: Current temperature in °F
        Returns:
            str: Status message describing the action taken
        """
        if not self.time_manager.has_valid_time():
            return "Waiting for valid time sync..."

        self.last_update_time = self.time_manager.get_current_time()

        if not self.enabled:
            self.heater.turn_off()
            self.led.off()
            return "Thermostat disabled"

        if not isinstance(temperature, (int, float)):
            return "Invalid temperature reading"

        should_be_on = self.heater.should_turn_on(temperature, self.setpoint)
        if should_be_on:
            if self.heater.turn_on():
                self.led.on()
                return f"Relay turned ON (Temperature {temperature:.1f}°F below setpoint {self.setpoint}°F)"
        else:
            if self.heater.turn_off():
                self.led.off()
                return f"Relay turned OFF (Temperature {temperature:.1f}°F above setpoint {self.setpoint}°F)"

        return None

    def get_status(self):
        """Get current status of thermostat controller"""
        current_time = self.time_manager.get_current_time()
        heater_status = self.heater.get_status()
        
        return {
            "enabled": self.enabled,
            "setpoint": self.setpoint,
            "last_update": current_time - self.last_update_time if self.last_update_time else 0,
            "has_valid_time": self.time_manager.has_valid_time(),
            "heater": heater_status
        }
