# type: ignore
import machine
import time

class TimeManager:
    def __init__(self, mqtt_client, log_function):
        """
        Initialize TimeManager
        
        Args:
            mqtt_client: MQTT client for Adafruit IO communication
            log_function: Function to use for logging
        """
        self.client = mqtt_client
        self.log = log_function
        self.time_feed = "time/seconds"
        self.last_sync = 0
        self.sync_interval = 43200  # 12 hours in seconds
        self._has_synced = False  # Track if we've ever synced

    def subscribe(self):
        """Subscribe to time feed"""
        try:
            self.client.subscribe(self.time_feed)
            return True
        except Exception as e:
            self.log(f"Error subscribing to time feed: {e}")
            return False

    def handle_time_message(self, msg):
        """Handle incoming time message from Adafruit"""
        try:
            if not self._has_synced or (time.time() - self.last_sync >= self.sync_interval):
                msg_str = msg.decode() if isinstance(msg, bytes) else msg
                current_time = int(msg_str)
                try:
                    machine.RTC().datetime(time.localtime(current_time))
                    self.last_sync = current_time
                    self._has_synced = True
                    self.log(f"System time updated from Adafruit: {time.localtime()}")
                    return True
                except Exception as e:
                    self.log(f"Error setting RTC: {e}")
                    return False
            # else:
            #    self.log("Time sync message received but not needed yet")  # Optional debug
            return True  # Message handled successfully even if we didn't need it
        except ValueError as e:
            self.log(f"Invalid time value received: {e}")
            return False

    def sync_time(self):
        """Request time sync from Adafruit"""
        try:
            self.client.publish(self.time_feed + "/get", "")
            return True
        except Exception as e:
            self.log(f"Error requesting time sync: {e}")
            return False

    def check_sync_needed(self):
        """Check if time sync is needed based on sync interval"""
        if not self._has_synced:
            return self.sync_time()
        current_time = time.time()
        if current_time - self.last_sync >= self.sync_interval:
            return self.sync_time()
        return False

    def get_current_time(self):
        """Get current time as timestamp"""
        return time.time()

    def has_valid_time(self):
        """Check if we have a valid time sync"""
        return self._has_synced

    def get_last_sync_time(self):
        """Get the last time we synced with Adafruit"""
        return self.last_sync
  