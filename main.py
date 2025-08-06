### MicroPython Dual UART Packet Parser with Schedule and Command Decoding
### Pi Pico W Version

from machine import UART, Pin
import time
#import struct
import uasyncio as asyncio
import mqttconn
import ujson
import ubinascii

# WR3 Reset Pin (e.g., GPIO2)
wr3_reset = Pin(2, Pin.OUT, value=1)
input_sw = Pin(6, Pin.IN, Pin.PULL_UP)

led=machine.Pin("LED",machine.Pin.OUT)

# MQTT Topics
TOPIC_HEARTBEAT = "sprinkler/heartbeat"
TOPIC_INFO = "sprinkler/info"
TOPIC_SCHEDULE = "sprinkler/schedules"
TOPIC_ACTIVE = "sprinkler/system/active"
TOPIC_MANUAL = "sprinkler/manual_durations"
TOPIC_COMMAND = "sprinkler/system/command"
TOPIC_STATUS = "sprinkler/system/status"
TOPIC_TIME_LEFT = "sprinkler/system/minremaining"



# Heartbeat message to match and respond withh
HEARTBEAT_MSG = b'\x55\xAA\x00\x00\x00\x00\xFF'

### ---- CONFIGURATION ----

# Setup UART0 (pins 0=TX, 1=RX) and UART1 (pins 4=TX, 5=RX), adjust as needed
uart0 = UART(0, baudrate=9600,  rx=Pin(1), rxbuf=128) #tx=Pin(0),
float_rx_pin = Pin(0,Pin.IN)
uart1 = UART(1, baudrate=9600, tx=Pin(4), rx=Pin(5), rxbuf=128)

### ---- YAML ID MAPPING (FRIENDLY NAMES) ----

id_mapping = {
    0x66: "1",
    0x67: "2",
    0x68: "3",
    0x69: "4",
    0x6A: "5",
    0x6B: "6",
    0x6C: "7",
    0x70: "8",
    0x71: "A",
    0x72: "B",
    0x73: "C",
    0x74: "Firmware Version",
    0x75: "System Voltage",
    0x76: "Battery Voltage1",
    0x77: "DPS 119",
    0x78: "8 Bit String",
    0X79: "Minutes Remaining",
    0X7A: "Switch Position",
    0X7B: "Watering"
}

# Create a reverse mapping for easy lookup from "1".."8", "A".."C" to byte value
reverse_id_mapping = {v: k for k, v in id_mapping.items()}

rot_sw_posns = {
    0x01: "Zone 1",
    0x02: "Zone 2",
    0x03: "Zone 3",
    0x04: "Zone 4",
    0x05: "Zone 5",
    0x06: "Zone 6",
    0x07: "Zone 7",
    0x08: "Zone 8",
    0x09: "AP",
    0x0A: "OFF",
    0x0B: "RUN",
    0x0C: "TEST"
}

state_mapping = {
    0: "OFF",
    1: "ON"
}

# Constants
HEADER = b'\x55\xAA'
MIN_PACKET_LEN = 7  # header(2) + addr(1) + type(1) + unknown(1) + len(1) + checksum(1)

# Buffers for UARTs
uart_buffers = {1: b'', 2: b''}

def read_uart_buffer(uart_obj, uart_id):
    global uart_buffers
    if uart_obj.any():
        uart_buffers[uart_id] += uart_obj.read()

    buffer = uart_buffers[uart_id]
    results = []

    while True:
        idx = buffer.find(HEADER)
        if idx == -1:
            # No header found, discard everything
            uart_buffers[uart_id] = b''
            return results

        if len(buffer) < idx + MIN_PACKET_LEN:
            # Not enough data for even the smallest packet
            break

        # Ensure header starts at 0
        if idx > 0:
            buffer = buffer[idx:]

        if len(buffer) < 6:
            break  # Still waiting for at least header + addr + type + unk + len

        payload_len = buffer[5]
        total_len = 6 + payload_len + 1  # +1 for checksum
        
        if len(buffer) < total_len:
            break  # Wait for more data

        packet = buffer[:total_len]
        hexstr = ' '.join(f'{b:02X}' for b in packet)
        buffer = buffer[total_len:]  # Consume this packet

        # Extract fields
        device_addr = packet[2]
        msg_type = packet[3]
        unknown = packet[4]
        payload = packet[6:6+payload_len]
        checksum = packet[-1]

        # Validate checksum
        computed = sum(packet[:-1]) & 0xFF
        valid = (computed == checksum)

        results.append({
            "device_addr": device_addr,
            "msg_type": msg_type,
            "unknown": unknown,
            "payload_len": payload_len,
            "payload": payload,
            "checksum": checksum,
            "valid": valid,
            "raw": hexstr
        })

    uart_buffers[uart_id] = buffer  # Save unprocessed data
    return results

# --- Packet builder using reverse lookup ---
def build_wr3_command(zone_id, state):
    #55 AA 00 06 00 05 66 01 00 01 01 73 - zone 1 on
    # Create a reverse mapping for easy lookup from "1".."8", "A".."C" to byte value
    reverse_state_mapping = {v: k for k, v in state_mapping.items()}
    zone_key = str(zone_id).upper()
    if zone_key not in reverse_id_mapping:
        print("Invalid zone/program ID:", zone_id)
        return None

    zone_byte = reverse_id_mapping[zone_key]
    zone_state = reverse_state_mapping[state]
    header = bytearray([0x55, 0xAA, 0x00, 0x06, 0x00, 0x05])
    command = bytearray([zone_byte, 0x01, 0x00, 0x01, zone_state])
    checksum = (sum(header + command) & 0xFF)
    packet = header + command + bytearray([checksum])
    return packet

# --- MQTT callback handler ---
def mqtt_callback(topic, msg):
    #print('message recvd')
    try:
        topic = topic.decode()
        msg = msg.decode()
        state = msg.upper()
        print(f'message:{topic} {msg}')

        if topic.startswith("sprinkler/zone/") and topic.endswith("/set"):
            zone_id = topic.split("/")[2]  # e.g. "1", "A", "C"
            #state = 1 if msg else 0

            print("MQTT command: zone/program ", zone_id, " state:", state)

            packet = build_wr3_command(zone_id, state)
            if packet: # and wr3_reset.value() == 0:
                enter_active_mode()
                uart0.write(packet)
                print("Packet sent:", ubinascii.hexlify(packet))
                enter_active_mode(False)

    except Exception as e:
        print("MQTT error:", e)



### ---- SCHEDULE MESSAGE PARSER ----

def parse_schedule(payload):
    #if len(payload) < 68:
    #    return "Schedule message too short."
    
    STOP = payload[4]
    
    YY = payload[5]
    MM = payload[6]
    DD = payload[7]
    hh = payload[8]
    mm = payload[9]
    ss = payload[10]
    datetimestr = f"{DD:02d}/{MM:02d}/{YY:02d} {hh:02d}:{mm:02d}:{ss:02d}"
                 
    general_info = {"Stop Watering":STOP, "Date/Time":datetimestr}

    schedule_labels = ['A', 'B', 'C']
    zone_names = [f"Zone{i+1}" for i in range(8)]
    day_names = ['Mon', 'Tue', 'Wed', 'Thu', 'Fri', 'Sat', 'Sun']

    schedules = {}

    for i, label in enumerate(schedule_labels):
        schd_str = f"Schedule {label}"
        base = 16 + i * 4  # updated base offset

        hour_byte = payload[base]
        minute_byte = payload[base + 1]
        zones_byte = payload[base + 2]
        days_byte = payload[base + 3]

        enabled = (hour_byte & 0x80) >> 7
        hour = hour_byte & 0x7F
        minute = minute_byte
        start_time_str = f"{hour:02d}:{minute:02d}"

        # Decode zones from bits
        zone_list = []
        for bit in range(8):
            if (zones_byte >> bit) & 1:
                zone_name = zone_names[bit]
                zone_offset = 28 + bit * 4
                duration = payload[zone_offset + i + 1]  # +1 skips manual
                zone_list.append({
                    "zone": zone_name,
                    "duration": duration
                })

        # Decode days from bits
        day_list = [day_names[bit] for bit in range(7) if (days_byte >> bit) & 1]

        schedules.update({
            schd_str:{
            "enabled": bool(enabled),
            "start_time": start_time_str,
            "zones_raw": zones_byte,
            "zones": zone_list,
            "days_raw": days_byte,
            "days": day_list
        }})

    return general_info, schedules

def parse_manual_durations(payload):
    manual_durations = {}
    for zone in range(8):
        base = 28 + zone * 4
        manual_duration = payload[base]  # manual is always the first of the 4 bytes
        manual_durations[f"Zone{zone+1}"] = manual_duration
    return manual_durations

### ---- COMMAND MESSAGE PARSER ----

def parse_command(payload):
    #if len(payload) < 5:
    #    return "Command payload too short."
    # | 55 AA 03 07 00 05 | 68 01 00 01 00 | 78     
    msg_type = None
    if payload[0] == 0x7A:	#ROTARY DIAL POSITION
        msg_type = "Dial Position"
        zone_id = payload[0:7]
        value = payload[7]
        friendly_name = rot_sw_posns.get(value, f"Unknown Switch {value}")
        print(f"Dial position: {friendly_name}")
    
    elif 0x66 <= payload[0] <= 0x7B:
        msg_type = "Output State"
        zone_id = payload[0]
        data_len = len(payload)
        value = payload[-1]
        if payload[0] == 0x75 or payload[0] == 0x76:
            value = value / 10
        friendly_name = id_mapping.get(zone_id, f"Unknown ID {zone_id}")
        print(f"{friendly_name}: {value}")
    else:
        msg_type = "Unknown"
        zone_id = None
        friendly_name = None
        value = None

    return {
        "type": msg_type,
        "zone_id": zone_id,
        "friendly_name": friendly_name,
        "value": value
    }


def enter_active_mode(active=True):
    """Hold WR3 module in reset to take control of the UART bus."""
    global reset_uart
    reset_uart = True
    uart0.deinit()
    if active:
        uart0.init(baudrate=9600, tx=Pin(0), rx=Pin(1), rxbuf=128) 
        wr3_reset.value(0)  # Active low reset
        print("*** WR3 held in reset, entering active mode ***")
    else:
        uart0.init(baudrate=9600,  rx=Pin(1), rxbuf=128) #tx=Pin(0),
        float_rx_pin = Pin(0,Pin.IN)
        wr3_reset.value(1)  # Active low reset
        print("*** WR3 rebooting, leaving active mode ***")
    time.sleep(0.1)     # Short delay to ensure reset is registered
    
    reset_uart = False

### ---- MAIN LOOP ----
async def listen_for_packets():
    global reset_uart
    reset_uart = False
    print("listening for packets")
    packets0 = None
    packets1 = None
    while True:
        if reset_uart == False:
            packets0 = read_uart_buffer(uart0, 1)
        if wr3_reset.value() == 1:  #i.e. we are passive listening not active on the bus
            packets1 = read_uart_buffer(uart1, 2)
        
        for pkt0 in packets0:
            if pkt0 and pkt0["valid"]:
                #print("[UART0] Raw:", pkt0)
                data = pkt0["raw"]
                line = f"rx: {data}"
                print(line)    
                if pkt0["payload_len"] == 0:
                    if pkt0["msg_type"] == 0x00:
                        device_id = pkt0["device_addr"]
                        print(f"Hearbeat from device:{device_id}")
                        mqtt_client.publish(TOPIC_HEARTBEAT,f"Hearbeat from device:{device_id}")
                elif pkt0["msg_type"] == 0x07 and 0x66 <= pkt0["payload"][0] <= 0x73:  #valve status
                    parsed = parse_command(pkt0["payload"])
                    z = id_mapping.get(parsed["zone_id"])
                    z_state = state_mapping.get(parsed["value"])
                    mqtt_client.publish(f"sprinkler/zone/{z}/state",z_state)
                elif pkt0["msg_type"] == 0x07 and pkt0["payload"][0] == 0x79:
                    parsed = parse_command(pkt0["payload"])
                    mqtt_client.publish(TOPIC_TIME_LEFT,str(parsed["value"]))
                elif pkt0["msg_type"] == 0x07 and 0x74 <= pkt0["payload"][0] <= 0x7B:
                    #print("[UART0] I/O Status Message")
                    parsed = parse_command(pkt0["payload"])
                    mqtt_client.publish(TOPIC_STATUS,ujson.dumps(parsed))
                    #print(parsed)
                elif pkt0["payload"][0] == 0x65:
                    #print("[UART0] Schedule Message")
                    parsed = parse_schedule(pkt0["payload"])
                    print(parsed[1]["Schedule A"])
                    print(parsed[1]["Schedule B"])
                    print(parsed[1]["Schedule C"])
                    mqtt_client.publish(TOPIC_INFO,ujson.dumps(parsed[0]),retain=True)
                    mqtt_client.publish(TOPIC_SCHEDULE,ujson.dumps(parsed[1]),retain=True)
                    manual_dur = parse_manual_durations(pkt0["payload"])
                    print(manual_dur)
                    mqtt_client.publish(TOPIC_MANUAL,ujson.dumps(manual_dur))
            elif pkt0 and not pkt0["valid"]:
                print("[UART0] Raw:", pkt0)
        
        if packets1:
            for pkt1 in packets1:
                if pkt1 and pkt1["valid"]:
                    #print("[UART1] Raw:", pkt1)
                    data = pkt1["raw"]
                    line = f"tx: {data}"
                    print(line)    
                    if pkt1["msg_type"] == 0x00:
                        device_id = pkt1["device_addr"]
                        print(f"Hearbeat from device:{device_id}")
                        mqtt_client.publish(TOPIC_HEARTBEAT,f"Hearbeat from device:{device_id}")
                    #if pkt1["msg_type"] == 0x07:
                    #    print("[UART1] I/O Status Message")
                    #    parsed = parse_command(pkt1["payload"])
                    #    print(parsed)
                    #elif pkt1["payload_len"] >= 68:
                    #    print("[UART1] Schedule Message")
                    #    parsed = parse_schedule(pkt1["payload"])
                    #    print(parsed)
                #elif pkt1 and not pkt1["valid"]:
                    #print("[UART0] Raw:", pkt1)
        
        await asyncio.sleep_ms(5)

# Create an Event object
button_pressed_event = asyncio.Event()

async def button_handler():
    # Configure the button pin to trigger an interrupt on a falling edge (button press)
    input_sw.irq(trigger=Pin.IRQ_FALLING, handler=lambda p: button_pressed_event.set())

    print("Waiting for button press...")
    await button_pressed_event.wait() # Pause until the event is set
    print("Button pressed!")
    enter_active_mode()
    # Turn ON Zone 4 (ID = 0x69)
    time.sleep(1)
    msg = build_valve_command(0x67, on=True)
    uart0.write(msg)

async def heartbeat_sender(uart):
    """Async task: send heartbeat message every 15 seconds."""
    while True:
        if wr3_reset.value() == 0:
            print(">> Sending heartbeat...")
            uart.write(HEARTBEAT_MSG)
        await asyncio.sleep(15)

# Async polling task
async def mqtt_poll():
    while True:
        try:
            mqtt_client.check_msg()  # Non-blocking call
        except Exception as e:
            print("MQTT poll error:", e)
        await asyncio.sleep(0.1)  # Small delay to yield control


def build_valve_command(zone_id, on=True):
    HEADER = [0x55, 0xAA]
    DEVICE_ADDR = 0x00
    MSG_TYPE = 0x06
    UNKNOWN = 0x00

    CMD_ID = 0x01 if on else 0x00
    ARG = [0x00, 0x01]
    FLAG = 0x01

    payload = [zone_id, FLAG] + ARG + [CMD_ID]
    length = len(payload)
    body = [DEVICE_ADDR, MSG_TYPE, UNKNOWN, length] + payload

    checksum = sum(HEADER+body) & 0xFF
    full_msg = HEADER + body + [checksum]
    hexstr = ' '.join(f'{b:02X}' for b in full_msg)
    print(hexstr)
    return bytes(full_msg)

async def main():
    asyncio.create_task(button_handler())
    asyncio.create_task(mqtt_poll())  # Start MQTT polling

    while True:
        await asyncio.gather(listen_for_packets(),heartbeat_sender(uart0))


# Connect to MQTT Broker
mqtt_client = mqttconn.connect_mqtt()
# Set mqtt callback action
mqtt_client.set_callback(mqtt_callback)

# Subscribe to topics
#mqttconn.mqtt_subscribe(mqtt_client,[TOPIC_COMMAND,TOPIC_SCHEDULE])
try:
    mqttconn.mqtt_subscribe(mqtt_client,["sprinkler/zone/+/set"])
except e:
    print(f'error {e} subscribing}')

asyncio.run(main())