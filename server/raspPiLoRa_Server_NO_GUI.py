import serial
import time
import json
import ssl
import threading
import paho.mqtt.client as mqtt

print("Beginning initalization and setup...")

# Status LUT
STATUS_HOT = 0x01
STATUS_COLD = 0x02
STATUS_LOW_PRS = 0x04
STATUS_HIGH_PRS = 0x08
STATUS_HUMID = 0x10
STATUS_DRY = 0x20
STATUS_LOUD = 0x40
STATUS_GAS = 0x80


class Node:
    def __init__(self, ID, temp=None, hum=None, prs=None, sound=None, gas=None, receivedTime=0):
        self.ID = ID
        self.temp = temp if temp is not None else []
        self.hum = hum if hum is not None else []
        self.prs = prs if prs is not None else []
        self.sound = sound if sound is not None else []
        self.gas = gas if gas is not None else []
        self.receivedTime = receivedTime
        self.lastCodes = []


class Threshold:
    def __init__(self, ID, tempUpper, tempLower, humUpper, humLower, prsUpper, prsLower, sound, gasLower):
        self.ID = ID
        self.tempUpper = tempUpper
        self.tempLower = tempLower
        self.humUpper = humUpper
        self.humLower = humLower
        self.prsUpper = prsUpper
        self.prsLower = prsLower
        self.sound = sound
        self.gasLower = gasLower


nodes = []
thresholds = []

serial_lock = threading.Lock()

PORT = "/dev/serial0"
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=0.5)
time.sleep(1.0)

def on_message(client, userdata, msg):
    text = msg.payload.decode()
    print("Received control:", text)

    data = json.loads(text)

    command = data["cmd"]
    allowed_commands = {
        "tempUpper", "tempLower",
        "humUpper", "humLower",
        "prsUpper", "prsLower",
        "sound", "gasLower"
    }

    if command not in allowed_commands:
        print(f"Invalid command: {command}")
        return

    node_id = str(data["ID"])
    value = int(data["val"])

    threshold = next((t for t in thresholds if str(t.ID) == node_id), None)

    if threshold is None:
        print(f"FAILED TO UPDATE THRESHOLD: {node_id}")
        return

    setattr(threshold, command, value)
    print(f"Updated node {node_id}: {command} = {value}")

def add_value(history, value):
    history.append(value)
    if len(history) > 5:
        history.pop(0)


def average(history):
    if len(history) == 0:
        return 0
    return sum(history) / len(history)


def send_lora(dest, msg):
    cmd = f"AT+SEND={dest},{len(msg)},{msg}\r\n"
    with serial_lock:
        ser.write(cmd.encode("ascii"))
        ser.flush()
    print("Sent:", cmd.strip())


def cmd(s):
    print(">>", s)
    with serial_lock:
        ser.reset_input_buffer()
        ser.write((s + "\r\n").encode())
        ser.flush()
        time.sleep(0.3)
        resp = ser.read_all().decode(errors="replace")
    print("<<", repr(resp))


def get_threshold(node_id):
    return next((t for t in thresholds if str(t.ID) == str(node_id)), None)


def verify(message, nodes):
    parts = message.split(',')

    address = parts[0]
    length = int(parts[1])
    data = parts[2]

    node = next((n for n in nodes if str(n.ID) == address), None)

    if node is None:
        node = Node(address)
        threshold = Threshold(address, 50, 10, 80, 20, 110000, 90000, 90, 500000)
        nodes.append(node)
        thresholds.append(threshold)
        print(f"New node added: {address}")

    if len(data) != length:
        return None, None

    return data, node


def parse(data, node):
    data = data.strip('[]')
    fields = data.split(';')

    values = {}
    for field in fields:
        key, value = field.split('=')
        values[key.strip()] = float(value.strip())

    values = {"ID": node.ID, **values}
    payload = json.dumps(values)

    client.publish("wss/data", payload, qos=1)

    add_value(node.temp, values.get('TMP'))
    add_value(node.hum, values.get('HUM'))
    add_value(node.prs, values.get('PRS'))
    add_value(node.sound, values.get('DB'))
    add_value(node.gas, values.get('GAS'))

    node.receivedTime = int(time.time())

    return node

def get_error_codes(node, threshold):
    codes = []
    avg_temp = average(node.temp)
    avg_hum = average(node.hum)
    avg_prs = average(node.prs)
    avg_sound = average(node.sound)
    avg_gas = average(node.gas)

    if avg_temp > threshold.tempUpper:
        codes.append(STATUS_HOT)
    if avg_temp < threshold.tempLower:
        codes.append(STATUS_COLD)

    if avg_prs < threshold.prsLower:
        codes.append(STATUS_LOW_PRS)
    if avg_prs > threshold.prsUpper:
        codes.append(STATUS_HIGH_PRS)

    if avg_hum > threshold.humUpper:
        codes.append(STATUS_HUMID)
    if avg_hum < threshold.humLower:
        codes.append(STATUS_DRY)

    if avg_sound > threshold.sound:
        codes.append(STATUS_LOUD)

    if avg_gas < threshold.gasLower:
        codes.append(STATUS_GAS)

    return codes


def build_error_payload(codes):
    # No errors case
    if len(codes) == 0:
        return "err=0;codes="

    # Convert each code to hex string (e.g., 0x01, 0x02)
    code_strings = []
    for code in codes:
        hex_code = f"0x{code:02x}"
        code_strings.append(hex_code)

    # Join them with commas
    codes_part = ",".join(code_strings)

    # Build final message
    message = f"err={len(codes)};codes={codes_part}"

    return message


def monitor(nodes, thresholds):
    for node in nodes:
        threshold = next((t for t in thresholds if str(t.ID) == str(node.ID)), None)

        if threshold is None:
            continue

        if node.receivedTime == 0:
            continue

        if time.time() - node.receivedTime > 60:
            continue

        codes = get_error_codes(node, threshold)

        if codes != node.lastCodes:
            msg = build_error_payload(codes)
            send_lora(node.ID, msg)
            node.lastCodes = codes.copy()
            print(f"Sent status to node {node.ID}: {msg}")


def rx_worker():
    last_monitor = time.time()

    while True:
        line = ser.readline().decode("utf-8", errors="replace").strip()
        #line = "+RCV=1,46,[TMP=22.5; HUM=40; PRS=101325; DB=55; GAS=300],-70,10"
        if line:
            if line.startswith("+RCV="):
                payload = line.split("=", 1)
                msg = payload[1]
                print("Received:", msg)

                data, node = verify(msg, nodes)
                if node:
                    parse(data, node)
                else:
                    print("Failed verification.")
            else:
                print("Other:", line)

        now = time.time()
        if now >= last_monitor + 30:
            monitor(nodes, thresholds)
            last_monitor = now

        #time.sleep(2.0)


print("Data initialized...")

cmd("AT")
cmd("AT+ADDRESS=2")
cmd("AT+NETWORKID=6")
cmd("AT+BAND=915000000")
cmd("AT+PARAMETER=9,7,1,12")

print("LoRa Initialized")

# MQTT Setup
print("Setting up MQTT client...")
client = mqtt.Client() # Creates client object
client.username_pw_set(username='vana_', password='Fireonfire1024') # Sets username and password

# Enable TLS (REQUIRED)
client.tls_set(cert_reqs=ssl.CERT_REQUIRED)

print("Connecting to MQTT")
client.connect("97c4bdba7f08451a8eaa96095d775e9e.s1.eu.hivemq.cloud", 8883, 60)

# Subscribing to a topic
client.on_message = on_message

client.subscribe("wss/control")
print("Connected to MQTT broker.")

# Begin the client thread
client.loop_start()


print("Setup complete.")

rx_thread = threading.Thread(target=rx_worker, daemon=True)
rx_thread.start()

while True:
    time.sleep(1)
