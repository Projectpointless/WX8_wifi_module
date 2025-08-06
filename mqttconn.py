import time
import network
import ujson
import ubinascii
from umqttsimple import MQTTClient
from secrets import secrets  # Load secrets from secrets.py


# Extract WiFi & MQTT details from secrets
#WIFI_SSID = secrets["SSID"]    # moved to boot.py
#WIFI_PASSWORD = secrets["PASSWORD"]

MQTT_BROKER = secrets["MQTTBROKER"]
MQTT_USER = secrets["MQTTUSER"]
MQTT_PASSWORD = secrets["MQTTPW"]

# MQTT Configuration
MQTT_PORT = 1883
MQTT_CLIENT_ID = ubinascii.hexlify(network.WLAN().config('mac'), ':').decode()


def log_error(message):
    """Logs error messages to a file."""
    with open("mqtt_error_log.txt", "a") as log_file:
        log_file.write(f"{time.time()}: {message}\n")

# MQTT Client
def connect_mqtt():
    try:
        client = MQTTClient(MQTT_CLIENT_ID, MQTT_BROKER, MQTT_PORT, MQTT_USER, MQTT_PASSWORD)
        #client.set_callback(mqtt_callback)
        client.connect()
        #client.subscribe(TOPIC_COMMAND)
    #    client.subscribe(MQTT_TOPIC_REQUEST)
        return client
    except Exception as e:
        error_message = f"Connection Error: {e}"
        print(error_message)  # Print (if a terminal is available)
        log_error(error_message)  # Save to file
        time.sleep(1)  # Prevent rapid crash loopsdef main_loop():


def mqtt_subscribe(client, topics=[]):
    if len(topics) >0:
        try:
            for topic in topics:
                client.subscribe(topic)
        except Exception as e:
            error_message = f"Error Subscribing: {e}"
            print(error_message)  # Print (if a terminal is available)
            log_error(error_message)  # Save to file
            time.sleep(1)  # Prevent rapid crash loopsdef main_loop():

def mqtt_set_callback(client, callback_fnc_name):
    client.set_callback(callback_fnc_name)

def mqtt_callback(topic, msg):
    topic = topic.decode()
    msg = msg.decode()
    
    if topic == TOPIC_GO:
        print("MQTT: Go command received")
        send_uart_message(0x78, FUNCTION_BYTES["go"])  # 0x78 is the pump's address

    elif topic == TOPIC_STOP:
        print("MQTT: Stop command received")
        send_uart_message(0x78, FUNCTION_BYTES["stop"])

    elif topic == TOPIC_SPEED:
        try:
            rpm = int(msg)
            if 0 <= rpm <= 65535:
                demand_value = rpm * 4  # Convert RPM to Demand format
                print(f"MQTT: Setting speed to {rpm} RPM ({demand_value} in demand units)")
                demand_lo = demand_value & 0xFF
                demand_hi = (demand_value >> 8) & 0xFF
                send_uart_message(0x78, FUNCTION_BYTES["set_speed"], [0x00, demand_lo, demand_hi])
            else:
                print("Invalid speed value")
        except ValueError:
            print("Invalid speed format")

    elif topic == TOPIC_STATUS:
        print("MQTT: Requesting status")
        send_uart_message(0x78, FUNCTION_BYTES["status"])

    elif topic == TOPIC_SENSOR:
        try:
            data = ujson.loads(msg)
            page = int(data["page"])
            address = int(data["address"])
            if 0 <= page <= 4 and 0 <= address <= 255:
                print(f"MQTT: Requesting sensor data (Page: {page}, Address: {address})")
                send_uart_message(0x78, FUNCTION_BYTES["read_sensor"], [page, address])
            else:
                print("Invalid sensor request values")
        except Exception as e:
            error_message = f"Error in request: {e}"
            print(error_message)  # Print (if a terminal is available)
            log_error(error_message)  # Save to file
            time.sleep(1)  # Prevent rapid crash loopsdef main_loop():

       



#    mqtt_client.publish(MQTT_TOPIC_REQUEST, str(f"0x{source_addr:02X}, {function_name}, Payload: {payload.hex().upper()}"))
#    log_message(f"Request -> Destination: 0x{source_addr:02X}, Function: {function_name}, Payload: {payload.hex().upper()}")



