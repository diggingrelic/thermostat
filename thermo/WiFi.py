# WiFi.py
#
# This module handles the WiFi connection and status.
#
# type: ignore
import network
import time
from thermo.Log import log
from thermo.MQTT import send_status
from config import WIFI_STATE_DISCONNECTED, WIFI_STATE_CONNECTING, WIFI_STATE_CONNECTED, WIFI_RETRY_DELAY
from mysecrets import SSID, PASSWORD

last_wifi_check = 0
last_wifi_status = None

class WiFi:
    def __init__(self, wdt, watchdog_enabled=True):
        self.wlan = network.WLAN(network.STA_IF)
        self.wlan.active(True)
        self.wdt = wdt
        self.watchdog_enabled = watchdog_enabled

    def feed_watchdog(self):
        """Safe watchdog feed with enable/disable flag"""
        if self.watchdog_enabled and self.wdt:
            self.wdt.feed()

    def connect_to_wifi(self, ssid=SSID, password=PASSWORD):
        try:
            wlan = network.WLAN(network.STA_IF)
            wlan.active(True)
            self.feed_watchdog()

            if not wlan.isconnected():
                log("Connecting to Wi-Fi...")
                wlan.connect(ssid, password)
                
                # Wait for connection with conditional watchdog feeds
                for _ in range(20):
                    self.feed_watchdog()
                    if wlan.isconnected():
                        break
                    time.sleep(0.5)

                if not wlan.isconnected():
                    # Retry logic with more frequent feeds
                    for attempt in range(5):
                        self.feed_watchdog()
                        if wlan.isconnected():
                            break
                        wlan.active(False)
                        time.sleep(0.5)
                        self.feed_watchdog()
                        wlan.active(True)
                        wlan.connect(ssid, password)
                        for _ in range(20):  # 10 seconds per attempt
                            self.feed_watchdog()
                            if wlan.isconnected():
                                break
                            time.sleep(0.5)
                        log(f"Attempt {attempt + 1}")

            if wlan.isconnected():
                log("Connected to Wi-Fi")
                log(f"Network Config: {wlan.ifconfig()}")
                self.feed_watchdog()
                return True
            else:
                log("Failed to connect to Wi-Fi")
                return False
                
        except KeyboardInterrupt:
            log("WiFi connection interrupted")
            return False

    def log_wifi_status(self, client):
        """Monitor WiFi status and send meaningful updates"""
        global last_wifi_status, last_wifi_check
        current_time = time.time()
        if current_time - last_wifi_check >= 10:
            wlan = network.WLAN(network.STA_IF)
            current_status = wlan.isconnected()
            if current_status != last_wifi_status:
                if current_status:
                    send_status(client, f"OK RECONNECT: WiFi restored, IP: {wlan.ifconfig()[0]}")
                else:
                    send_status(client, "ERROR: WiFi disconnected")
                last_wifi_status = current_status
            last_wifi_check = current_time

    def check_wifi_connection(self, wifi_state):
        """Monitor WiFi and attempt reconnection"""
        global last_wifi_attempt
        current_time = time.time()
        last_wifi_attempt = 0
        
        wlan = network.WLAN(network.STA_IF)
        if not wlan.isconnected():
            if wifi_state != WIFI_STATE_DISCONNECTED:
                wifi_state = WIFI_STATE_DISCONNECTED
                log("WiFi disconnected")
                
            # Only attempt reconnect after delay
            if current_time - last_wifi_attempt >= WIFI_RETRY_DELAY:
                last_wifi_attempt = current_time
                wifi_state = WIFI_STATE_CONNECTING
                log("Attempting WiFi connection...")
                
                wlan.active(False)
                time.sleep(1)
                self.feed_watchdog()
                wlan.active(True)
                wlan.connect(SSID, PASSWORD)
                
                # Wait briefly for connection
                for _ in range(10):
                    if wlan.isconnected():
                        wifi_state = WIFI_STATE_CONNECTED
                        log(f"WiFi connected, IP: {wlan.ifconfig()[0]}")
                        return True
                    time.sleep(1)
                    self.feed_watchdog()
                
                log("WiFi connection attempt failed")
                return False
                
        else:
            if wifi_state != WIFI_STATE_CONNECTED:
                wifi_state = WIFI_STATE_CONNECTED
                log(f"WiFi connected, IP: {wlan.ifconfig()[0]}")
            return True
