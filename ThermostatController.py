class ThermostatController:
    def __init__(self, heater, led, setpoint=40.0):
        self.heater = heater
        self.led = led
        self.setpoint = setpoint
        self.enabled = False  # replaces relay_state

    def set_enabled(self, enabled):
        """Enable or disable the thermostat"""
        self.enabled = enabled.lower() == "on" if isinstance(enabled, str) else enabled

    def set_setpoint(self, setpoint):
        """Update the temperature setpoint"""
        try:
            self.setpoint = float(setpoint)
            return True
        except (ValueError, TypeError):
            return False

    def update(self, temperature):
        """Update the thermostat state based on current temperature"""
        if self.enabled:
            should_be_on = self.heater.should_turn_on(temperature, self.setpoint)
            if should_be_on:
                if self.heater.turn_on():
                    self.led.on()
                    return f"Relay turned ON (Temperature {temperature:.1f}°F below setpoint {self.setpoint}°F)"
            else:
                self.heater.turn_off()
                self.led.off()
                return f"Relay turned OFF (Temperature {temperature:.1f}°F above setpoint {self.setpoint}°F)"
        else:
            self.heater.turn_off()
            self.led.off()
            return None 