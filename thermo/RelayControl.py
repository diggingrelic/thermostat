# type: ignore
import time
from thermo.Log import log
#from MQTT import send_status
from config import AIO_FEED_HEATER_STATUS
from thermo.ThermoState import update_runtime
from thermo.MQTT import send_status

class RelayControl:
    def __init__(self):
        pass

    def update_relay_state(self, temperature, client, state):
        # Calculate temperature bounds using differential
        temp_high = state.setpoint + state.temp_diff
        temp_low = state.setpoint - state.temp_diff
        
        # ON only when commanded ON and below low limit
        desired_relay_state = state.relay_state and temperature <= temp_low
        # Only update if state is actually changing
        if desired_relay_state != state.current_relay_state:
            if desired_relay_state:
                if(temperature < temp_high):
                    if not delay_cycle(state):
                        turn_on_relay(state, client, state.relay_pin, state.led)
                    else:
                        send_status(client, "Relay commanded ON but waiting for cycle delay to complete.")
                else:
                    if min_runtime_passed(state, state.min_time_on):
                        turn_off_relay(state, client, state.relay_pin, state.led)
                        send_status(client, f"Relay commanded ON but temperature {temperature:.1f} F is above high limit {temp_high:.1f} F")
            else:
                if min_runtime_passed(state, state.min_time_on):
                    turn_off_relay(state, client, state.relay_pin, state.led)
                    send_status(client, "Relay commanded OFF")
            return


#Helper functions
def turn_on_relay(state, client, relay_pin, led):
    update_runtime(force_update=True)
    relay_pin.on()
    led.on()
    state.current_relay_state = True
    state.relay_state = True
    state.heater_status = 1
    state.heater_start_time = time.time()
    state.last_runtime_update = time.time()
    state.last_runtime_status = time.time()
    client.publish(AIO_FEED_HEATER_STATUS, "1")
    log("Relay turned ON")

def turn_off_relay(state, client, relay_pin, led):
    relay_pin.off()
    led.off()
    state.current_relay_state = False
    state.relay_state = False
    state.heater_status = 0
    state.heater_start_time = 0
    state.last_runtime_update = 0
    state.last_runtime_status = 0
    state.last_relay_change = time.time()
    client.publish(AIO_FEED_HEATER_STATUS, "0")
    log("Relay turned OFF")

def delay_cycle(state):
    if not state.current_relay_state and state.relay_state:
        if time.time() - state.last_relay_change < state.cycle_delay * 60:
            remaining = (state.cycle_delay * 60) - (time.time() - state.last_relay_change)
            log(f"Cycle delay: {remaining:.1f} seconds remaining")
            return True
        else:
            return False
    else:
        return False
        
def min_runtime_passed(state, min_time_on):
    if not state.relay_state and (time.time() - state.heater_start_time) < (min_time_on * 60):
        remaining = (min_time_on * 60) - (time.time() - state.heater_start_time)
        log(f"Relay commanded OFF but held ON for minimum time: {remaining:.1f} seconds remaining")
        return False
    else:
        return True