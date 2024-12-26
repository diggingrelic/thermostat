from mysecrets import AIO_USERNAME

# Debug flag for verbose logging
debug = True

# MQTT details
AIO_SERVER = "io.adafruit.com"
AIO_PORT = 1883
AIO_FEED_RELAY = f"{AIO_USERNAME}/feeds/relay"
AIO_FEED_SETPOINT = f"{AIO_USERNAME}/feeds/setpoint"
AIO_FEED_TEMP = f"{AIO_USERNAME}/feeds/temp"
AIO_FEED_CYCLE_DELAY = f"{AIO_USERNAME}/feeds/cycle_delay"
AIO_FEED_TEMP_DIFF = f"{AIO_USERNAME}/feeds/temp_diff"
AIO_FEED_MIN_TIME_ON = f"{AIO_USERNAME}/feeds/min_time_on"
AIO_FEED_HEATER_STATUS = f"{AIO_USERNAME}/feeds/heater_status"
AIO_FEED_TIMER = f"{AIO_USERNAME}/feeds/timer"  # NEW: Timer feed
AIO_FEED_STATUS = f"{AIO_USERNAME}/feeds/status"
AIO_FEED_DAILY_COST = f"{AIO_USERNAME}/feeds/daily_cost"
HEATER_COST_PER_HOUR = 0.90  # Cost in dollars per hour

# MQTT States
MQTT_STATE_DISCONNECTED = 0
MQTT_STATE_CONNECTING = 1
MQTT_STATE_CONNECTED = 2

# WiFi States
WIFI_STATE_DISCONNECTED = 0
WIFI_STATE_CONNECTING = 1
WIFI_STATE_CONNECTED = 2
WIFI_RETRY_DELAY = 10  # seconds

# Feeds to subscribe
SUBSCRIBE_FEEDS = [
    AIO_FEED_RELAY,
    AIO_FEED_SETPOINT,
    AIO_FEED_CYCLE_DELAY,
    AIO_FEED_TEMP_DIFF,
    AIO_FEED_MIN_TIME_ON,
    AIO_FEED_HEATER_STATUS,
    AIO_FEED_TIMER
]
