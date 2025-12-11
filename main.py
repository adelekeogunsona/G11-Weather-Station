import network
import time
import machine
import ubinascii
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
CLIENT_ID   = ubinascii.hexlify(machine.unique_id())

PIN_SDA = 0
PIN_SCL = 1
PIN_LED = 15

I2C_FREQ = 400000
I2C_ADDR = 0x77
PUBLISH_INTERVAL = 5000
TEMP_THRESHOLD = 30.0

# ==========================================
# SYSTEM SETUP
# ==========================================
red_led = Pin(PIN_LED, Pin.OUT)
i2c = I2C(0, sda=Pin(PIN_SDA), scl=Pin(PIN_SCL), freq=I2C_FREQ)
time.sleep(0.5)

bme = None
try:
    bme = bme280.BME280(i2c=i2c, address=I2C_ADDR)
    print(f"[INIT] Sensor initialized at {hex(I2C_ADDR)}")
except Exception as e:
    print(f"[CRITICAL] Sensor Init Failed: {e}")

wlan = network.WLAN(network.STA_IF)
mqtt_client = None

# ==========================================
# HELPER FUNCTIONS
# ==========================================

def ensure_wifi():
    if wlan.isconnected():
        return True

    print(f"[WIFI] Connecting to {WIFI_SSID}...")
    wlan.active(True)
    wlan.connect(WIFI_SSID, WIFI_PASS)

    attempts = 0
    while not wlan.isconnected() and attempts < 10:
        time.sleep(1)
        attempts += 1

    if wlan.isconnected():
        try:
            wlan.config(pm=0xa11140)
        except:
            pass
        print(f"[WIFI] Connected: {wlan.ifconfig()[0]}")
        return True

    print("[WIFI] Connection failed.")
    return False

def ensure_mqtt():
    global mqtt_client
    try:
        if mqtt_client:
            mqtt_client.ping()
            return True
    except OSError:
        print("[MQTT] Connection lost. Reconnecting...")
        mqtt_client = None

    if mqtt_client is None:
        if not ensure_wifi():
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
    try:
        t, p, h = bme.read_compensated_data()
        return {
            'temp_val': t,
            'payload': ujson.dumps({
                "temperature": round(t, 2),
                "pressure": round(p/100, 2),
                "humidity": round(h, 2),
                "device_id": CLIENT_ID.decode()
            })
        }
    except Exception as e:
        print(f"[SENSOR] Read error: {e}")
        return None

# ==========================================
# MAIN LOOP
# ==========================================
def main():
    print("[SYSTEM] Starting Weather Station...")

    wdt = WDT(timeout=8000)
    last_publish = 0

    ensure_wifi()
    ensure_mqtt()

    while True:
        try:
            wdt.feed()

            if not bme:
                print("[ERROR] No sensor.")
                time.sleep(5)
                continue

            data = get_sensor_data()

            if data:
                red_led.value(1 if data['temp_val'] > TEMP_THRESHOLD else 0)

                if data['temp_val'] > TEMP_THRESHOLD:
                    print(f"[ALERT] High Temp: {data['temp_val']:.2f}")

                now = time.ticks_ms()
                if time.ticks_diff(now, last_publish) > PUBLISH_INTERVAL:
                    if ensure_mqtt():
                        mqtt_client.publish(MQTT_TOPIC, data['payload'], retain=True, qos=1)
                        print(f"[PUB] JSON Sent: {data['payload']}")
                        last_publish = now

            gc.collect()
            time.sleep(1)

        except Exception as e:
            print(f"[LOOP] Error: {e}")
            time.sleep(5)

if __name__ == "__main__":
    main()