import network
import time
import machine
import ujson
import gc
from machine import Pin, I2C, WDT
import bme280_float as bme280
from umqtt.simple import MQTTClient

# ==========================================
# CONFIGURATION
# ==========================================
WIFI_SSID = 'TPADGuest'
WIFI_PASS = 'password'

MQTT_BROKER = 'test.mosquitto.org'
MQTT_PORT   = 1883
MQTT_TOPIC  = b'Group11/WeatherStation'
CLIENT_ID   = 'raspberry_pico_w_001'

PIN_SDA = 0
PIN_SCL = 1
PIN_LED = 15

I2C_FREQ = 400000
I2C_ADDR = 0x77
PUBLISH_INTERVAL = 30000 # 30s
TEMP_THRESHOLD = 30.0
WDT_TIMEOUT = 8000

# ==========================================
# SYSTEM SETUP
# ==========================================
red_led = Pin(PIN_LED, Pin.OUT)
i2c = I2C(0, sda=Pin(PIN_SDA), scl=Pin(PIN_SCL), freq=I2C_FREQ)
time.sleep(0.5)

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
    if wlan.isconnected():
        return True

    print(f"[WIFI] Connecting to {WIFI_SSID}...")
    wlan.active(True)
    wlan.connect(WIFI_SSID, WIFI_PASS)

    attempts = 0
    while not wlan.isconnected() and attempts < 10:
        if wdt:
            wdt.feed()
        time.sleep(1)
        attempts += 1

    if wlan.isconnected():
        try:
            wlan.config(pm=0xa11140) # disable power saving
        except:
            pass
        print(f"[WIFI] Connected: {wlan.ifconfig()[0]}")
        return True

    print("[WIFI] Connection failed.")
    return False

def ensure_mqtt(wdt=None):
    global mqtt_client
    try:
        if mqtt_client:
            mqtt_client.ping()
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

def get_sensor_data():
    if not bme:
        return None
    try:
        t, p, h = bme.read_compensated_data()
        return {
            'temp_val': t,
            'payload': ujson.dumps({
                "temperature": round(t, 2),
                "pressure": round(p/100, 2),
                "humidity": round(h, 2),
                "device_id": CLIENT_ID
            })
        }
    except Exception as e:
        print(f"[SENSOR] Read error: {e}")
        return None

def safe_sleep(seconds, wdt=None):
    for _ in range(seconds):
        if wdt:
            wdt.feed()
        time.sleep(1)

# ==========================================
# MAIN LOOP
# ==========================================
def main():
    global bme
    print("[SYSTEM] Starting Weather Station...")

    init_sensor()
    ensure_wifi()
    ensure_mqtt()

    wdt = WDT(timeout=WDT_TIMEOUT)
    last_publish = 0

    while True:
        try:
            wdt.feed()

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

            red_led.value(1 if data['temp_val'] > TEMP_THRESHOLD else 0)

            if data['temp_val'] > TEMP_THRESHOLD:
                print(f"[ALERT] High Temp: {data['temp_val']:.2f}")

            now = time.ticks_ms()
            if time.ticks_diff(now, last_publish) > PUBLISH_INTERVAL:
                if ensure_mqtt(wdt):
                    mqtt_client.publish(MQTT_TOPIC, data['payload'], retain=True, qos=0)
                    print(f"[PUB] JSON Sent: {data['payload']}")
                    last_publish = now

            gc.collect()
            time.sleep(1)

        except Exception as e:
            print(f"[LOOP] Error: {e}")
            safe_sleep(5, wdt)

if __name__ == "__main__":
    main()
