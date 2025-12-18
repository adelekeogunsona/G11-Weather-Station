import network
import time
import ujson
import gc
import math
from machine import Pin, I2C, WDT
import bme280_float as bme280
from umqtt.simple import MQTTClient

# ==========================================
# CONFIGURATION
# ==========================================
WIFI_SSID = 'WiFiSSID'
WIFI_PASS = 'password'

MQTT_BROKER = 'test.mosquitto.org'
MQTT_PORT   = 1883
MQTT_TOPIC  = b'Group11/WeatherStation'
CLIENT_ID   = 'raspberry_pico_w_001'

PIN_SDA = 0
PIN_SCL = 1
PIN_LED = 15

I2C_FREQ = 400000
I2C_ADDR = 0x77  # BME280 I2C address
PUBLISH_INTERVAL = 30000  # 30s
TEMP_THRESHOLD = 30.0  # Celsius threshold for LED alert
WDT_TIMEOUT = 8000  # Watchdog timeout in milliseconds

# ==========================================
# SYSTEM SETUP
# ==========================================
red_led = Pin(PIN_LED, Pin.OUT)
i2c = I2C(0, sda=Pin(PIN_SDA), scl=Pin(PIN_SCL), freq=I2C_FREQ)
time.sleep(0.5)  # Allow I2C bus to stabilize

wlan = network.WLAN(network.STA_IF)
mqtt_client = None
bme = None

# ==========================================
# HELPER FUNCTIONS
# ==========================================

def init_sensor():
    global bme
    try:
        bme = bme280.BME280(i2c=i2c, address=I2C_ADDR)
        print(f"[INIT] Sensor initialized at {hex(I2C_ADDR)}")
        return True
    except Exception as e:
        print(f"[CRITICAL] Sensor Init Failed: {e}")
        bme = None
        return False

def ensure_wifi(wdt=None):
    """Ensure WiFi connection is active, reconnect if needed."""
    if wlan.isconnected():
        return True

    print(f"[WIFI] Connecting to {WIFI_SSID}...")
    wlan.active(True)
    wlan.connect(WIFI_SSID, WIFI_PASS)

    attempts = 0
    while not wlan.isconnected() and attempts < 10:
        if wdt:
            wdt.feed()  # Prevent watchdog reset during connection
        time.sleep(1)
        attempts += 1

    if wlan.isconnected():
        try:
            wlan.config(pm=0xa11140)  # Disable power saving for stable connection
        except:
            pass
        print(f"[WIFI] Connected: {wlan.ifconfig()[0]}")
        return True

    print("[WIFI] Connection failed.")
    return False

def ensure_mqtt(wdt=None):
    """Ensure MQTT connection is active, reconnect if needed."""
    global mqtt_client
    try:
        if mqtt_client:
            mqtt_client.ping()  # Check if connection is still alive
            return True
    except Exception:
        print("[MQTT] Connection lost. Reconnecting...")
        mqtt_client = None

    if mqtt_client is None:
        if not ensure_wifi(wdt):
            return False
        try:
            client = MQTTClient(
                CLIENT_ID,
                MQTT_BROKER,
                port=MQTT_PORT
            )
            client.connect()
            print("[MQTT] Connected to Broker")
            mqtt_client = client
            return True
        except Exception as e:
            print(f"[MQTT] Connect error: {e}")
            return False
    return False

def calculate_dew_point(T, RH):
    """Calculate dew point using Magnus formula (T in Celsius, RH in %)."""
    a = 17.62
    b = 243.12
    alpha = ((a * T) / (b + T)) + math.log(RH / 100.0)
    dew_point = (b * alpha) / (a - alpha)
    return dew_point

def calculate_heat_index(T_celsius, RH):
    """Calculate heat index using Rothfusz equation (T in Celsius, RH in %)."""
    T_f = (T_celsius * 9/5) + 32

    if T_f < 80:
        return T_celsius  # Heat index only meaningful above 80°F

    hi = 0.5 * (T_f + 61.0 + ((T_f - 68.0) * 1.2) + (RH * 0.094))

    if hi > 80:
        # Full Rothfusz equation for accurate calculation
        hi = -42.379 + 2.04901523 * T_f + 10.14333127 * RH - \
             0.22475541 * T_f * RH - 0.00683783 * T_f * T_f - \
             0.05481717 * RH * RH + 0.00122874 * T_f * T_f * RH + \
             0.00085282 * T_f * RH * RH - 0.00000199 * T_f * T_f * RH * RH

        # Adjustments for extreme conditions
        if (RH < 13) and (T_f >= 80) and (T_f <= 112):
            adj = ((13 - RH) / 4) * math.sqrt((17 - abs(T_f - 95.)) / 17)
            hi -= adj
        elif (RH > 85) and (T_f >= 80) and (T_f <= 87):
            adj = ((RH - 85) / 10) * ((87 - T_f) / 5)
            hi += adj

    hi_celsius = (hi - 32) * 5/9
    return hi_celsius

def get_sensor_data():
    """Read sensor data and calculate derived values, return formatted JSON payload."""
    if not bme:
        return None
    try:
        t, p, h = bme.read_compensated_data()  # Temperature (°C), Pressure (Pa), Humidity (%)
        dew = calculate_dew_point(t, h)
        heat = calculate_heat_index(t, h)
        return {
            'temp_val': t,
            'payload': ujson.dumps({
                "temperature": round(t, 2),
                "pressure": round(p/100, 2),  # Convert Pa to hPa
                "humidity": round(h, 2),
                "dew_point": round(dew, 2),
                "heat_index": round(heat, 2),
                "device_id": CLIENT_ID
            })
        }
    except Exception as e:
        print(f"[SENSOR] Read error: {e}")
        return None

def safe_sleep(seconds, wdt=None):
    """Sleep in 1-second increments while feeding watchdog to prevent reset."""
    for _ in range(seconds):
        if wdt:
            wdt.feed()
        time.sleep(1)

# ==========================================
# MAIN LOOP
# ==========================================
def main():
    """Main application loop: read sensor, monitor connections, publish data."""
    global bme
    print("[SYSTEM] Starting Weather Station...")

    init_sensor()
    ensure_wifi()
    ensure_mqtt()

    wdt = WDT(timeout=WDT_TIMEOUT)
    last_publish = 0

    while True:
        try:
            wdt.feed()  # Reset watchdog timer each loop iteration

            # Reinitialize sensor if not available
            if not bme:
                print("[SENSOR] Attempting to reinitialize...")
                init_sensor()
                if not bme:
                    safe_sleep(5, wdt)
                    continue

            data = get_sensor_data()

            if data is None:
                print("[SENSOR] Read failed, attempting reinit...")
                bme = None
                safe_sleep(2, wdt)
                continue

            # Control LED based on temperature threshold
            red_led.value(1 if data['temp_val'] > TEMP_THRESHOLD else 0)

            if data['temp_val'] > TEMP_THRESHOLD:
                print(f"[ALERT] High Temp: {data['temp_val']:.2f}")

            # Publish data at configured interval
            now = time.ticks_ms()
            if time.ticks_diff(now, last_publish) > PUBLISH_INTERVAL:
                if ensure_mqtt(wdt):
                    mqtt_client.publish(MQTT_TOPIC, data['payload'], retain=True, qos=1)
                    print(f"[PUB] JSON Sent: {data['payload']}")
                    last_publish = now

            gc.collect()  # Free up memory
            time.sleep(1)

        except Exception as e:
            print(f"[LOOP] Error: {e}")
            safe_sleep(5, wdt)

if __name__ == "__main__":
    main()
