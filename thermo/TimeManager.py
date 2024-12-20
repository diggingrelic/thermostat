# type: ignore
import machine
import time
from mysecrets import AIO_USERNAME

class TimeManager:
    def __init__(self, client, log_function):
        """
        Initialize TimeManager
        
        Args:
            client: MQTT client for Adafruit IO communication
            log_function: Function to use for logging
        """
        self.client = client
        self.log = log_function
        self.last_sync = 0  # Track last sync time
        self.sync_interval = 12 * 60 * 60  # 12 hours in seconds
        self.check_sync_needed()  # Force sync on initialization

    def handle_time_message(self, msg):
        """Handle incoming time message from Adafruit"""
        try:
            current_time = int(msg.decode())
            
            # Only sync if we haven't synced yet (last_sync = 0) or if 12 hours have passed
            if self.last_sync == 0 or (current_time - self.last_sync) >= self.sync_interval:
                self.last_sync = current_time
                # Convert epoch to tuple
                time_tuple = time.gmtime(current_time)
                machine.RTC().datetime((time_tuple[0], time_tuple[1], time_tuple[2], 
                                      time_tuple[6] + 1, time_tuple[3], time_tuple[4], 
                                      time_tuple[5], 0))
                self.log(f"System time updated from Adafruit: {time_tuple}")
        except Exception as e:
            self.log(f"Error processing time message: {e}")

    def check_sync_needed(self):
        """Check if time sync is needed based on current time"""
        current_time = time.time()
        if self.last_sync == 0 or (current_time - self.last_sync) >= self.sync_interval:
            # Subscribe, get time, then unsubscribe
            #self.client.subscribe("time/seconds")
            self.client.publish(f"{AIO_USERNAME}/feeds/time/get", "")
            # Wait briefly for response
            for _ in range(10):  # Try for 1 second
                self.client.check_msg()
                time.sleep(0.1)

    def get_current_time(self):
        """Get current time as timestamp"""
        return time.time()

    def has_valid_time(self):
        """Check if we have a valid time sync"""
        return self.last_sync != 0

    def get_last_sync_time(self):
        """Get the last time we synced with Adafruit"""
        return self.last_sync
  