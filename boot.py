import network
import time
from machine import Pin
from secrets import secrets  # Load secrets from secrets.py

led=machine.Pin("LED",machine.Pin.OUT)
WIFI_SSID = secrets["SSID"]
WIFI_PASSWORD = secrets["PASSWORD"]

# WiFi Connection
def connect_wifi():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    wlan.connect(WIFI_SSID, WIFI_PASSWORD)

    print("Connecting to WiFi", end="")
    for _ in range(10):
        if wlan.isconnected():
            print("\nConnected to WiFi:", WIFI_SSID)
            print("IP Address:", wlan.ifconfig()[0])
            led.on()
            return
        print(".", end="")
        led.toggle()
        time.sleep(1)

    print("\nWiFi connection failed!")
    while True:
        led.toggle()
        time.sleep(0.25)

led.on()
connect_wifi()

