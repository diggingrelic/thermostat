# type: ignore
import machine
import time
from thermo.Log import log
import urequests
from mysecrets import AIO_KEY

class TimeManager:
    def __init__(self):
        self.sync_interval = 43200  # 12 hours in seconds
        self.sync_needed = False
        
    def check_sync_needed(self, last_sync):
        """Check if time sync is needed based on current time"""
        current_time = time.time()
        
        if last_sync == 0 or (current_time - last_sync) >= self.sync_interval:
            self.sync_needed = True
            log("Time sync needed - Requesting update")
            try:
                # Make HTTP request instead of MQTT
                response = urequests.get(
                    "https://io.adafruit.com/api/v2/time/seconds",
                    headers={'X-AIO-Key': AIO_KEY}
                )
                if response.status_code == 200:
                    current_time = int(response.text)
                    response.close()
                    last_sync = self.update_system_time(current_time)
                    self.sync_needed = False
                else:
                    log(f"Time sync failed with status: {response.status_code}")
                    response.close()
            except Exception as e:
                log(f"Error during time sync: {e}")
        return last_sync
        
    def update_system_time(self, current_time):
        """Update system time from epoch timestamp"""
        time_tuple = time.gmtime(current_time)
        machine.RTC().datetime((time_tuple[0], time_tuple[1], time_tuple[2], 
                              time_tuple[6] + 1, time_tuple[3], time_tuple[4], 
                              time_tuple[5], 0))
        log(f"System time updated to: {time_tuple}")
        return current_time
  