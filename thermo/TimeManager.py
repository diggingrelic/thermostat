# type: ignore
import machine
import time
from thermo.Log import log

class TimeManager:
    def __init__(self):
        self.sync_interval = 12 * 60 * 60  # 12 hours in seconds
        self.sync_needed = False

    def handle_time_message(self, msg, last_sync):
        """Handle incoming time message from MQTT Callback"""
        try:
            current_time = int(msg.decode())
            log(f"Received time message: {current_time}")
            
            # Only sync if we haven't synced yet (last_sync = 0) or if 12 hours have passed
            if last_sync == 0 or (current_time - last_sync) >= self.sync_interval:
                last_sync = current_time
                # Convert epoch to tuple
                time_tuple = time.gmtime(current_time)
                machine.RTC().datetime((time_tuple[0], time_tuple[1], time_tuple[2], time_tuple[6] + 1, time_tuple[3], time_tuple[4], time_tuple[5], 0))
                log(f"System time updated from Adafruit: {time_tuple}")
                return last_sync
            else:
                log("Skipping time update - Last sync too recent")
            
            return last_sync  # Always return last_sync value
            
        except Exception as e:
            log(f"Error processing time message: {e}")
            return last_sync  # Return existing last_sync value on error

    def check_sync_needed(self, last_sync):
        """Check if time sync is needed based on current time"""
        current_time = time.time()
        log(f"Time sync check - Last sync: {last_sync}, Current time: {current_time}")
        
        if last_sync == 0 or (current_time - last_sync) >= self.sync_interval:
            self.sync_needed = True
            log("Time sync needed - Requesting update")
            time.sleep(1)
        else:
            log(f"Time sync not needed - Next sync in {self.sync_interval - (current_time - last_sync):.0f} seconds")
        return last_sync
  